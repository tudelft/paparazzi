/** @file soda_vehicle_postprocess.cpp @brief EXIF-driven vehicle-hit post-processing for SODA. */
#include "soda_vehicle_postprocess.h"
#include "path_utils.h"

#include <libexif/exif-data.h>
#include <libexif/exif-entry.h>
#include <libexif/exif-tag.h>

#include <jpeglib.h>
#include <setjmp.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include <fcntl.h>
#include <unistd.h>

namespace {

/** Matches image_exif.c's write_tagged_comment(): an 8-byte ASCII-charset prefix precedes
 * the text payload in EXIF_TAG_USER_COMMENT, and the tagged record itself is introduced by
 * "\n<TAG>\n". The entry is zero-filled before being written and sized one byte past the
 * visible text (image_exif.c's replace_tag()/write_tagged_comment()), so a trailing NUL is
 * present in practice, but entry->size (not strlen()) is used regardless, to stay correct
 * even if that trailing byte is ever not zero. */
constexpr size_t kAsciiCommentPrefixSize = 8;
constexpr char kVehicleDetectionMarker[] = "\nAICAM_VEHICLES_V1\n";
/** Same default as the now-removed Python cropper, kept identical so a ported call site
 * produces the same crop given the same box. */
constexpr double kCropMargin = 0.18;
/** Matches the outline width the now-removed Python draw_hits() used (PIL's
 * ImageDraw.rectangle(..., width=4)). There is no text label here -- see
 * draw_box_outline()'s comment for why. */
constexpr int kBoxOutlineThickness = 4;

struct VehicleDetection {
  int count = 0;
  double confidence = 0.0;
  int box_x = 0, box_y = 0, box_w = 0, box_h = 0;
};

struct CropBox {
  int left, top, right, bottom;
};

struct DecodedImage {
  std::vector<unsigned char> pixels;
  unsigned width = 0;
  unsigned height = 0;
};

struct JpegErrorContext {
  struct jpeg_error_mgr manager;
  jmp_buf recovery;
};

void jpeg_error_exit(j_common_ptr jpeg_info)
{
  JpegErrorContext *error = reinterpret_cast<JpegErrorContext *>(jpeg_info->err);
  longjmp(error->recovery, 1);
}

/** @brief Parse an already-decoded EXIF_TAG_USER_COMMENT payload for an AICAM_VEHICLES_V1
 * hit. Kept separate from the EXIF/file I/O in read_vehicle_detection() so the parsing
 * logic is unit-testable against synthetic buffers without needing a real JPEG on disk.
 * @return true only for a genuine hit; false when the marker is absent (every
 * non-aicam-detect image) or the summary after it is "status=ok; count=0" /
 * "status=analysis_failed; ...", both of which fail to match the count>=1 pattern below. */
bool parse_vehicle_summary(const std::string &comment_text, VehicleDetection *out)
{
  size_t marker_pos = comment_text.find(kVehicleDetectionMarker);
  if (marker_pos == std::string::npos) {
    return false;
  }
  std::string summary = comment_text.substr(marker_pos + std::strlen(kVehicleDetectionMarker));

  int count = 0, box_x = 0, box_y = 0, box_w = 0, box_h = 0;
  double confidence = 0.0;
  if (std::sscanf(summary.c_str(),
                   "status=ok; count=%d; label=vehicle; confidence=%lf; box=%d,%d,%d,%d",
                   &count, &confidence, &box_x, &box_y, &box_w, &box_h) != 6
      || count < 1) {
    return false;
  }

  out->count = count;
  out->confidence = confidence;
  out->box_x = box_x;
  out->box_y = box_y;
  out->box_w = box_w;
  out->box_h = box_h;
  return true;
}

/** @brief Read the AICAM_VEHICLES_V1 record catia's image_exif_write_vehicle_detections()
 * writes into EXIF_TAG_USER_COMMENT, if present and reporting count >= 1.
 * @return true only for a genuine hit; false for a read/parse failure, a shot without the
 * marker (every non-aicam-detect image), or a "status=ok; count=0"/"analysis_failed" shot. */
bool read_vehicle_detection(const char *image_path, VehicleDetection *out)
{
  ExifData *exif = exif_data_new_from_file(image_path);
  if (exif == nullptr) {
    return false;
  }

  ExifEntry *entry = exif_content_get_entry(exif->ifd[EXIF_IFD_EXIF], EXIF_TAG_USER_COMMENT);
  if (entry == nullptr || entry->format != EXIF_FORMAT_UNDEFINED
      || entry->size <= kAsciiCommentPrefixSize) {
    exif_data_unref(exif);
    return false;
  }

  std::string text(reinterpret_cast<const char *>(entry->data) + kAsciiCommentPrefixSize,
                    entry->size - kAsciiCommentPrefixSize);
  exif_data_unref(exif);

  return parse_vehicle_summary(text, out);
}

/** @brief Directory of @p image_path, plus "/<name>", created if missing. */
bool sibling_directory(const char *image_path, const char *name, std::string *out)
{
  std::string path(image_path);
  size_t slash = path.find_last_of('/');
  std::string base_dir = (slash == std::string::npos) ? "." : path.substr(0, slash);
  *out = base_dir + "/" + name;
  return catia_ensure_directory(out->c_str()) == 0;
}

std::string basename_of(const char *image_path)
{
  std::string path(image_path);
  size_t slash = path.find_last_of('/');
  return (slash == std::string::npos) ? path : path.substr(slash + 1);
}

/** @brief Copy every byte of @p source_path into a newly created/truncated @p dest_path.
 * @details A standalone copy loop, deliberately not shared with vehicle_detect_pipe.c's
 * own copy_file() -- that function is file-local there, and linking vehicle_detect_pipe.o
 * into soda would couple this build to catia's object graph for ~20 lines of code. */
bool copy_file_bytes(const char *source_path, const char *dest_path)
{
  int source_fd = open(source_path, O_RDONLY);
  if (source_fd < 0) {
    return false;
  }
  int dest_fd = open(dest_path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
  if (dest_fd < 0) {
    close(source_fd);
    return false;
  }

  char buffer[65536];
  ssize_t bytes_read;
  bool ok = true;
  while ((bytes_read = read(source_fd, buffer, sizeof(buffer))) > 0) {
    ssize_t written_total = 0;
    while (written_total < bytes_read) {
      ssize_t written = write(dest_fd, buffer + written_total,
                              static_cast<size_t>(bytes_read - written_total));
      if (written < 0) {
        if (errno == EINTR) {
          continue;
        }
        ok = false;
        break;
      }
      written_total += written;
    }
    if (!ok) {
      break;
    }
  }
  if (bytes_read < 0) {
    ok = false;
  }

  close(source_fd);
  if (close(dest_fd) != 0) {
    ok = false;
  }
  if (!ok) {
    unlink(dest_path);
  }
  return ok;
}

/** @brief Direct C++ port of the removed Python compute_crop_box(): pad each side by
 * `margin` fraction of the box's own size, then clamp to the frame's bounds, guarding a
 * degenerate zero-area result so it's always safely croppable. */
CropBox compute_crop_box(int box_x, int box_y, int box_w, int box_h,
                         unsigned img_w, unsigned img_h, double margin)
{
  double pad_x = box_w * margin;
  double pad_y = box_h * margin;
  int left = std::max(0, static_cast<int>(std::lround(box_x - pad_x)));
  int top = std::max(0, static_cast<int>(std::lround(box_y - pad_y)));
  int right = std::min(static_cast<int>(img_w), static_cast<int>(std::lround(box_x + box_w + pad_x)));
  int bottom = std::min(static_cast<int>(img_h), static_cast<int>(std::lround(box_y + box_h + pad_y)));
  right = std::max(right, left + 1);
  bottom = std::max(bottom, top + 1);
  return CropBox{left, top, right, bottom};
}

/** @brief Decode a JPEG to a contiguous row-major RGB buffer.
 * @details Same libjpeg pattern as image_mock_transform.c's decode_jpeg(), duplicated
 * rather than shared since that file's decode/encode helpers are file-local (static) and
 * out of scope to refactor for this change. */
bool decode_jpeg_rgb(const char *path, DecodedImage *image)
{
  FILE *input = fopen(path, "rb");
  if (input == nullptr) {
    return false;
  }

  struct jpeg_decompress_struct *decoder =
      static_cast<struct jpeg_decompress_struct *>(calloc(1, sizeof(*decoder)));
  if (decoder == nullptr) {
    fclose(input);
    return false;
  }

  JpegErrorContext error;
  decoder->err = jpeg_std_error(&error.manager);
  error.manager.error_exit = jpeg_error_exit;
  if (setjmp(error.recovery)) {
    jpeg_destroy_decompress(decoder);
    free(decoder);
    fclose(input);
    return false;
  }

  jpeg_create_decompress(decoder);
  jpeg_stdio_src(decoder, input);
  if (jpeg_read_header(decoder, TRUE) != JPEG_HEADER_OK) {
    jpeg_destroy_decompress(decoder);
    free(decoder);
    fclose(input);
    return false;
  }
  decoder->out_color_space = JCS_RGB;
  jpeg_start_decompress(decoder);

  image->width = decoder->output_width;
  image->height = decoder->output_height;
  if (decoder->output_components != 3 || image->width < 1 || image->height < 1) {
    jpeg_destroy_decompress(decoder);
    free(decoder);
    fclose(input);
    return false;
  }

  size_t row_size = static_cast<size_t>(image->width) * 3;
  image->pixels.resize(row_size * image->height);

  while (decoder->output_scanline < decoder->output_height) {
    JSAMPROW row[1] = {&image->pixels[static_cast<size_t>(decoder->output_scanline) * row_size]};
    if (jpeg_read_scanlines(decoder, row, 1) != 1) {
      jpeg_destroy_decompress(decoder);
      free(decoder);
      fclose(input);
      return false;
    }
  }

  jpeg_finish_decompress(decoder);
  jpeg_destroy_decompress(decoder);
  free(decoder);
  return fclose(input) == 0;
}

/** @brief Draw a green rectangle outline in place on a decoded RGB buffer.
 * @details vehicle_detect_server.py never draws on the frame it saves to the configured
 * photo directory any more -- that file must stay a clean flight-record image (see
 * vehicle_detect_server.py's module docstring). SODA is the only place that burns a box in
 * now, onto its own copies (vehicles_captured/ and the crop), using the box this process
 * already read back from EXIF. Unlike the removed Python draw_hits(), this draws only the
 * outline, not the "vehicle 0.NN" confidence label -- replicating PIL's text rendering in
 * raw C++ would need a font-rendering dependency for a label that's already in the console
 * log and the CSV row. */
void draw_box_outline(DecodedImage *image, int box_x, int box_y, int box_w, int box_h, int thickness)
{
  int left = box_x;
  int top = box_y;
  int right = box_x + box_w;
  int bottom = box_y + box_h;
  int img_w = static_cast<int>(image->width);
  int img_h = static_cast<int>(image->height);

  auto set_pixel = [&](int x, int y) {
    if (x < 0 || y < 0 || x >= img_w || y >= img_h) {
      return;
    }
    size_t offset = (static_cast<size_t>(y) * image->width + static_cast<size_t>(x)) * 3;
    image->pixels[offset] = 0;
    image->pixels[offset + 1] = 255;
    image->pixels[offset + 2] = 0;
  };

  for (int stroke = 0; stroke < thickness; ++stroke) {
    for (int x = left; x < right; ++x) {
      set_pixel(x, top + stroke);
      set_pixel(x, bottom - 1 - stroke);
    }
    for (int y = top; y < bottom; ++y) {
      set_pixel(left + stroke, y);
      set_pixel(right - 1 - stroke, y);
    }
  }
}

/** @brief Encode the padded crop region of an already-decoded frame to a new JPEG file.
 * @details No separate crop buffer is allocated: since the decoded buffer is row-major RGB
 * at the *full* image's stride, each output scanline's row pointer can point directly into
 * it at the crop's left/top offset -- the encoder only reads `crop_w*3` bytes from wherever
 * that pointer starts. Quality 95, matching image_mock_transform.c's encode_jpeg(). */
bool encode_cropped_jpeg(const char *path, const DecodedImage &image, const CropBox &box)
{
  int crop_w = box.right - box.left;
  int crop_h = box.bottom - box.top;
  if (crop_w <= 0 || crop_h <= 0) {
    return false;
  }

  FILE *output = fopen(path, "wb");
  if (output == nullptr) {
    return false;
  }

  struct jpeg_compress_struct *encoder =
      static_cast<struct jpeg_compress_struct *>(calloc(1, sizeof(*encoder)));
  if (encoder == nullptr) {
    fclose(output);
    return false;
  }

  JpegErrorContext error;
  encoder->err = jpeg_std_error(&error.manager);
  error.manager.error_exit = jpeg_error_exit;
  if (setjmp(error.recovery)) {
    jpeg_destroy_compress(encoder);
    free(encoder);
    fclose(output);
    return false;
  }

  jpeg_create_compress(encoder);
  jpeg_stdio_dest(encoder, output);
  encoder->image_width = static_cast<JDIMENSION>(crop_w);
  encoder->image_height = static_cast<JDIMENSION>(crop_h);
  encoder->input_components = 3;
  encoder->in_color_space = JCS_RGB;
  jpeg_set_defaults(encoder);
  jpeg_set_quality(encoder, 95, TRUE);
  jpeg_start_compress(encoder, TRUE);

  size_t full_row_size = static_cast<size_t>(image.width) * 3;
  while (encoder->next_scanline < encoder->image_height) {
    size_t source_row = static_cast<size_t>(box.top) + encoder->next_scanline;
    const unsigned char *row_start =
        &image.pixels[source_row * full_row_size + static_cast<size_t>(box.left) * 3];
    JSAMPROW row[1] = {const_cast<JSAMPROW>(row_start)};
    if (jpeg_write_scanlines(encoder, row, 1) != 1) {
      jpeg_destroy_compress(encoder);
      free(encoder);
      fclose(output);
      return false;
    }
  }

  jpeg_finish_compress(encoder);
  jpeg_destroy_compress(encoder);
  free(encoder);
  return fclose(output) == 0;
}

/** @brief write() a whole buffer in one syscall, retrying only an EINTR that transferred
 * nothing. Callers rely on the single-syscall property for O_APPEND atomicity -- see
 * append_detection_csv(). */
bool write_all_atomic(int fd, const std::string &data)
{
  ssize_t written;
  do {
    written = write(fd, data.data(), data.size());
  } while (written < 0 && errno == EINTR);
  return written == static_cast<ssize_t>(data.size());
}

/** @brief Append one CSV row for @p detection to @p csv_path, safe against concurrent SODA
 * processes racing on the same file (catia can run multiple worker threads' finish_image()
 * close together, each spawning its own SODA child).
 * @details Header-or-not is decided with O_CREAT|O_EXCL: whichever process wins that race
 * writes header+row together in a single write(); every loser falls back to a plain
 * O_APPEND open and writes only its own row. Each row (and the header+row for the winner)
 * goes out as exactly one write() call, which POSIX guarantees is atomic against other
 * O_APPEND writers on the same file as long as it stays under PIPE_BUF -- comfortably true
 * for these rows. */
bool append_detection_csv(const std::string &csv_path, const std::string &image_basename,
                          const VehicleDetection &detection)
{
  char row_buffer[512];
  int row_length = std::snprintf(row_buffer, sizeof(row_buffer), "%s,%d,%.4f,%d,%d,%d,%d\n",
                                 image_basename.c_str(), detection.count, detection.confidence,
                                 detection.box_x, detection.box_y, detection.box_w, detection.box_h);
  if (row_length <= 0 || static_cast<size_t>(row_length) >= sizeof(row_buffer)) {
    return false;
  }
  std::string row(row_buffer, static_cast<size_t>(row_length));

  int fd = open(csv_path.c_str(), O_WRONLY | O_CREAT | O_EXCL, 0644);
  if (fd >= 0) {
    static const char kHeader[] = "image,count,confidence,box_x,box_y,box_w,box_h\n";
    bool ok = write_all_atomic(fd, std::string(kHeader) + row);
    return close(fd) == 0 && ok;
  }
  if (errno != EEXIST) {
    return false;
  }

  fd = open(csv_path.c_str(), O_WRONLY | O_APPEND);
  if (fd < 0) {
    return false;
  }
  bool ok = write_all_atomic(fd, row);
  return close(fd) == 0 && ok;
}

} // namespace

bool soda_process_vehicle_detection(const char *image_path)
{
  if (image_path == nullptr || image_path[0] == '\0') {
    return true;
  }

  VehicleDetection detection;
  if (!read_vehicle_detection(image_path, &detection)) {
    return true;
  }

  std::string basename = basename_of(image_path);
  bool ok = true;

  // Decode once and draw the box once; both the vehicles_captured/ copy and the crop are
  // regions of this same annotated buffer, so the box appears consistently in both without
  // decoding or drawing twice. The original file at image_path is never touched -- it stays
  // the plain capture vehicle_detect_server.py saved.
  DecodedImage decoded;
  bool decoded_ok = decode_jpeg_rgb(image_path, &decoded);
  if (decoded_ok) {
    draw_box_outline(&decoded, detection.box_x, detection.box_y, detection.box_w,
                     detection.box_h, kBoxOutlineThickness);
  } else {
    std::cerr << "SODA: failed to decode " << image_path << " for annotation" << std::endl;
  }

  std::string hits_dir;
  if (sibling_directory(image_path, "vehicles_captured", &hits_dir)) {
    std::string dest_path = hits_dir + "/" + basename;
    bool saved;
    if (decoded_ok) {
      CropBox full_frame{0, 0, static_cast<int>(decoded.width), static_cast<int>(decoded.height)};
      saved = encode_cropped_jpeg(dest_path.c_str(), decoded, full_frame);
    } else {
      // Annotation failed -- still land an (unannotated) copy so the hit isn't lost entirely.
      saved = copy_file_bytes(image_path, dest_path.c_str());
    }
    if (!saved) {
      std::cerr << "SODA: failed to save annotated copy of " << image_path << " into " << hits_dir << std::endl;
      ok = false;
    }
    std::string csv_path = hits_dir + "/detections.csv";
    if (!append_detection_csv(csv_path, basename, detection)) {
      std::cerr << "SODA: failed to record detection in " << csv_path << std::endl;
      ok = false;
    }
  } else {
    std::cerr << "SODA: failed to prepare vehicles_captured directory for " << image_path << std::endl;
    ok = false;
  }

  std::string crop_dir;
  if (sibling_directory(image_path, "tight_crop_vehicles", &crop_dir)) {
    if (decoded_ok) {
      CropBox box = compute_crop_box(detection.box_x, detection.box_y, detection.box_w,
                                     detection.box_h, decoded.width, decoded.height, kCropMargin);
      std::string stem = basename;
      size_t dot = stem.find_last_of('.');
      if (dot != std::string::npos) {
        stem.erase(dot);
      }
      std::string crop_path = crop_dir + "/" + stem + "_vehicle01.jpg";
      if (!encode_cropped_jpeg(crop_path.c_str(), decoded, box)) {
        std::cerr << "SODA: failed to save crop for " << image_path << std::endl;
        ok = false;
      }
    } else {
      std::cerr << "SODA: failed to crop " << image_path << " (decode already failed)" << std::endl;
      ok = false;
    }
  } else {
    std::cerr << "SODA: failed to prepare tight_crop_vehicles directory for " << image_path << std::endl;
    ok = false;
  }

  std::cerr << "SODA: vehicle detected (count=" << detection.count << ", conf=" << detection.confidence
            << "); post-processed " << image_path << std::endl;

  return ok;
}
