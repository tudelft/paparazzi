#include "image_exif.h"

#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include <libexif/exif-data.h>
#include <libexif/exif-entry.h>
#include <libexif/exif-mem.h>
#include <libexif/exif-tag.h>
#include <libexif/exif-utils.h>

#define ANGLE_BFP_SCALE 4096.0
#define SPEED_BFP_SCALE 524288.0
#define POSITION_BFP_SCALE 256.0
#define RAD_TO_DEG 57.29577951308232
#define EXIF_APP1_MAX_PAYLOAD 65533U
#define ASCII_COMMENT_PREFIX_SIZE 8U

static const ExifByteOrder byte_order = EXIF_BYTE_ORDER_INTEL;

static ExifEntry *replace_tag(ExifData *exif, ExifIfd ifd, ExifTag tag,
                              ExifFormat format, unsigned long components)
{
  ExifEntry *old_entry = exif_content_get_entry(exif->ifd[ifd], tag);
  if (old_entry != NULL) {
    exif_content_remove_entry(exif->ifd[ifd], old_entry);
  }

  size_t component_size = exif_format_get_size(format);
  if (component_size == 0 || components == 0 || components > UINT32_MAX / component_size) {
    return NULL;
  }

  ExifMem *memory = exif_mem_new_default();
  if (memory == NULL) {
    return NULL;
  }

  ExifEntry *entry = exif_entry_new_mem(memory);
  if (entry == NULL) {
    exif_mem_unref(memory);
    return NULL;
  }

  size_t size = component_size * components;
  entry->data = exif_mem_alloc(memory, (ExifLong)size);
  if (entry->data == NULL) {
    exif_entry_unref(entry);
    exif_mem_unref(memory);
    return NULL;
  }

  for (size_t index = 0; index < size; index++) {
    entry->data[index] = 0;
  }
  entry->size = (unsigned int)size;
  entry->tag = tag;
  entry->format = format;
  entry->components = components;
  exif_content_add_entry(exif->ifd[ifd], entry);
  exif_entry_unref(entry);
  exif_mem_unref(memory);
  return exif_content_get_entry(exif->ifd[ifd], tag);
}

static int set_ascii_tag(ExifData *exif, ExifIfd ifd, ExifTag tag, const char *value)
{
  size_t length = strlen(value) + 1;
  ExifEntry *entry = replace_tag(exif, ifd, tag, EXIF_FORMAT_ASCII, length);
  if (entry == NULL) {
    return -1;
  }
  for (size_t index = 0; index < length; index++) {
    entry->data[index] = (unsigned char)value[index];
  }
  return 0;
}

static int set_rational_tag(ExifData *exif, ExifIfd ifd, ExifTag tag,
                            uint32_t numerator, uint32_t denominator)
{
  ExifEntry *entry = replace_tag(exif, ifd, tag, EXIF_FORMAT_RATIONAL, 1);
  if (entry == NULL || denominator == 0) {
    return -1;
  }
  ExifRational value = {numerator, denominator};
  exif_set_rational(entry->data, byte_order, value);
  return 0;
}

static int set_coordinate(ExifData *exif, ExifTag reference_tag, ExifTag coordinate_tag,
                          int32_t coordinate, int32_t limit, char positive, char negative)
{
  if (coordinate < -limit || coordinate > limit) {
    return -1;
  }

  char reference[2] = {coordinate < 0 ? negative : positive, '\0'};
  if (set_ascii_tag(exif, EXIF_IFD_GPS, reference_tag, reference) != 0) {
    return -1;
  }

  ExifEntry *entry = replace_tag(exif, EXIF_IFD_GPS, coordinate_tag, EXIF_FORMAT_RATIONAL, 3);
  if (entry == NULL) {
    return -1;
  }

  uint64_t absolute = coordinate < 0 ? (uint64_t)(-(int64_t)coordinate) : (uint64_t)coordinate;
  uint32_t degrees = (uint32_t)(absolute / 10000000U);
  uint64_t remainder = absolute % 10000000U;
  uint32_t minutes = (uint32_t)((remainder * 60U) / 10000000U);
  remainder = remainder * 60U - (uint64_t)minutes * 10000000U;
  uint32_t seconds_millionths = (uint32_t)((remainder * 60U * 1000000U) / 10000000U);

  ExifRational part = {degrees, 1};
  exif_set_rational(entry->data, byte_order, part);
  part.numerator = minutes;
  exif_set_rational(entry->data + 8, byte_order, part);
  part.numerator = seconds_millionths;
  part.denominator = 1000000U;
  exif_set_rational(entry->data + 16, byte_order, part);
  return 0;
}

static double normalized_degrees(int32_t angle_bfp)
{
  double degrees = angle_bfp / ANGLE_BFP_SCALE * RAD_TO_DEG;
  degrees = fmod(degrees, 360.0);
  return degrees < 0.0 ? degrees + 360.0 : degrees;
}

static uint32_t scaled_unsigned(double value, double scale)
{
  if (!isfinite(value) || value <= 0.0) {
    return 0;
  }
  double scaled = value * scale;
  return scaled >= UINT32_MAX ? UINT32_MAX : (uint32_t)llround(scaled);
}

static int add_shot_tags(ExifData *exif, const union dc_shot_union *shot)
{
  ExifEntry *version = replace_tag(exif, EXIF_IFD_GPS, EXIF_TAG_GPS_VERSION_ID, EXIF_FORMAT_BYTE, 4);
  if (version == NULL) {
    return -1;
  }
  version->data[0] = 2;
  version->data[1] = 3;

  if (set_coordinate(exif, EXIF_TAG_GPS_LATITUDE_REF, EXIF_TAG_GPS_LATITUDE,
                     shot->data.lat, 900000000, 'N', 'S') != 0
      || set_coordinate(exif, EXIF_TAG_GPS_LONGITUDE_REF, EXIF_TAG_GPS_LONGITUDE,
                        shot->data.lon, 1800000000, 'E', 'W') != 0) {
    return -1;
  }

  ExifEntry *altitude_reference = replace_tag(exif, EXIF_IFD_GPS, EXIF_TAG_GPS_ALTITUDE_REF,
                                               EXIF_FORMAT_BYTE, 1);
  if (altitude_reference == NULL) {
    return -1;
  }
  altitude_reference->data[0] = shot->data.alt < 0 ? 1 : 0;
  uint64_t altitude_mm = shot->data.alt < 0 ? (uint64_t)(-(int64_t)shot->data.alt) : (uint64_t)shot->data.alt;
  if (altitude_mm > UINT32_MAX
      || set_rational_tag(exif, EXIF_IFD_GPS, EXIF_TAG_GPS_ALTITUDE, (uint32_t)altitude_mm, 1000) != 0) {
    return -1;
  }

  double speed_m_s = shot->data.vground / SPEED_BFP_SCALE;
  double course_deg = normalized_degrees(shot->data.course);
  double yaw_deg = normalized_degrees(shot->data.psi);
  if (set_ascii_tag(exif, EXIF_IFD_GPS, EXIF_TAG_GPS_SPEED_REF, "K") != 0
      || set_rational_tag(exif, EXIF_IFD_GPS, EXIF_TAG_GPS_SPEED,
                          scaled_unsigned(fabs(speed_m_s) * 3.6, 1000.0), 1000) != 0
      || set_ascii_tag(exif, EXIF_IFD_GPS, EXIF_TAG_GPS_TRACK_REF, "T") != 0
      || set_rational_tag(exif, EXIF_IFD_GPS, EXIF_TAG_GPS_TRACK,
                          scaled_unsigned(course_deg, 10000.0), 10000) != 0
      || set_ascii_tag(exif, EXIF_IFD_GPS, EXIF_TAG_GPS_IMG_DIRECTION_REF, "T") != 0
      || set_rational_tag(exif, EXIF_IFD_GPS, EXIF_TAG_GPS_IMG_DIRECTION,
                          scaled_unsigned(yaw_deg, 10000.0), 10000) != 0) {
    return -1;
  }

  char comment[768];
  int comment_length = snprintf(comment, sizeof(comment),
                                "Paparazzi CATIA MORA; nr=%" PRId32 "; lat_e7deg=%" PRId32
                                "; lon_e7deg=%" PRId32 "; alt_mm=%" PRId32
                                "; phi_bfp=%" PRId32 "; theta_bfp=%" PRId32
                                "; psi_bfp=%" PRId32 "; vground_bfp=%" PRId32
                                "; course_bfp=%" PRId32 "; groundalt_bfp=%" PRId32
                                "; lat_deg=%.7f; lon_deg=%.7f; alt_m=%.3f"
                                "; roll_deg=%.4f; pitch_deg=%.4f; yaw_deg=%.4f"
                                "; speed_m_s=%.4f; course_deg=%.4f; groundalt_m=%.3f",
                                shot->data.nr, shot->data.lat, shot->data.lon, shot->data.alt,
                                shot->data.phi, shot->data.theta, shot->data.psi,
                                shot->data.vground, shot->data.course, shot->data.groundalt,
                                shot->data.lat / 1e7, shot->data.lon / 1e7, shot->data.alt / 1000.0,
                                shot->data.phi / ANGLE_BFP_SCALE * RAD_TO_DEG,
                                shot->data.theta / ANGLE_BFP_SCALE * RAD_TO_DEG,
                                shot->data.psi / ANGLE_BFP_SCALE * RAD_TO_DEG,
                                speed_m_s, course_deg, shot->data.groundalt / POSITION_BFP_SCALE);
  if (comment_length < 0 || (size_t)comment_length >= sizeof(comment)) {
    return -1;
  }
  if (set_ascii_tag(exif, EXIF_IFD_0, EXIF_TAG_IMAGE_DESCRIPTION, comment) != 0
      || set_ascii_tag(exif, EXIF_IFD_0, EXIF_TAG_SOFTWARE, "Paparazzi CATIA") != 0) {
    return -1;
  }

  size_t comment_size = ASCII_COMMENT_PREFIX_SIZE + (size_t)comment_length;
  ExifEntry *user_comment = replace_tag(exif, EXIF_IFD_EXIF, EXIF_TAG_USER_COMMENT,
                                         EXIF_FORMAT_UNDEFINED, comment_size);
  if (user_comment == NULL) {
    return -1;
  }
  const char prefix[ASCII_COMMENT_PREFIX_SIZE] = {'A', 'S', 'C', 'I', 'I', '\0', '\0', '\0'};
  for (size_t index = 0; index < ASCII_COMMENT_PREFIX_SIZE; index++) {
    user_comment->data[index] = (unsigned char)prefix[index];
  }
  for (int index = 0; index < comment_length; index++) {
    user_comment->data[ASCII_COMMENT_PREFIX_SIZE + (size_t)index] = (unsigned char)comment[index];
  }
  return 0;
}

static int write_bytes(FILE *output, const unsigned char *data, size_t size)
{
  return size == 0 || fwrite(data, 1, size, output) == size ? 0 : -1;
}

static int copy_bytes(FILE *input, FILE *output, size_t size)
{
  unsigned char buffer[8192];
  while (size > 0) {
    size_t chunk = size < sizeof(buffer) ? size : sizeof(buffer);
    size_t count = fread(buffer, 1, chunk, input);
    if (count != chunk || write_bytes(output, buffer, count) != 0) {
      return -1;
    }
    size -= count;
  }
  return 0;
}

static int copy_remainder(FILE *input, FILE *output)
{
  unsigned char buffer[8192];
  size_t count;
  while ((count = fread(buffer, 1, sizeof(buffer), input)) > 0) {
    if (write_bytes(output, buffer, count) != 0) {
      return -1;
    }
  }
  return ferror(input) ? -1 : 0;
}

static bool is_standalone_marker(unsigned char marker)
{
  return marker == 0x01 || marker == 0xd8 || marker == 0xd9 || (marker >= 0xd0 && marker <= 0xd7);
}

static int write_exif_segment(FILE *output, const unsigned char *exif_data, unsigned int exif_size)
{
  if (exif_size > EXIF_APP1_MAX_PAYLOAD) {
    errno = EOVERFLOW;
    return -1;
  }
  unsigned int segment_size = exif_size + 2U;
  unsigned char header[4] = {0xff, 0xe1, (unsigned char)(segment_size >> 8), (unsigned char)segment_size};
  return write_bytes(output, header, sizeof(header)) == 0
         && write_bytes(output, exif_data, exif_size) == 0 ? 0 : -1;
}

static int rewrite_jpeg(FILE *input, FILE *output, const unsigned char *exif_data, unsigned int exif_size)
{
  unsigned char soi[2];
  if (fread(soi, 1, sizeof(soi), input) != sizeof(soi) || soi[0] != 0xff || soi[1] != 0xd8
      || write_bytes(output, soi, sizeof(soi)) != 0) {
    errno = EINVAL;
    return -1;
  }

  bool exif_written = false;
  for (;;) {
    int prefix = fgetc(input);
    int marker = fgetc(input);
    if (prefix != 0xff || marker == EOF) {
      errno = EINVAL;
      return -1;
    }

    while (marker == 0xff) {
      marker = fgetc(input);
      if (marker == EOF) {
        errno = EINVAL;
        return -1;
      }
    }

    if (!exif_written && marker != 0xe0) {
      if (write_exif_segment(output, exif_data, exif_size) != 0) {
        return -1;
      }
      exif_written = true;
    }

    unsigned char marker_bytes[2] = {0xff, (unsigned char)marker};
    if (is_standalone_marker((unsigned char)marker)) {
      if (write_bytes(output, marker_bytes, sizeof(marker_bytes)) != 0) {
        return -1;
      }
      if (marker == 0xd9) {
        return 0;
      }
      continue;
    }

    unsigned char length_bytes[2];
    if (fread(length_bytes, 1, sizeof(length_bytes), input) != sizeof(length_bytes)) {
      errno = EINVAL;
      return -1;
    }
    unsigned int segment_size = ((unsigned int)length_bytes[0] << 8) | length_bytes[1];
    if (segment_size < 2) {
      errno = EINVAL;
      return -1;
    }
    size_t payload_size = segment_size - 2U;

    if (marker == 0xe1) {
      unsigned char signature[6];
      if (payload_size < sizeof(signature)
          || fread(signature, 1, sizeof(signature), input) != sizeof(signature)) {
        errno = EINVAL;
        return -1;
      }
      bool is_exif = signature[0] == 'E' && signature[1] == 'x' && signature[2] == 'i'
                     && signature[3] == 'f' && signature[4] == 0 && signature[5] == 0;
      if (is_exif) {
        if (fseek(input, (long)(payload_size - sizeof(signature)), SEEK_CUR) != 0) {
          return -1;
        }
        continue;
      }
      if (write_bytes(output, marker_bytes, sizeof(marker_bytes)) != 0
          || write_bytes(output, length_bytes, sizeof(length_bytes)) != 0
          || write_bytes(output, signature, sizeof(signature)) != 0
          || copy_bytes(input, output, payload_size - sizeof(signature)) != 0) {
        return -1;
      }
    } else {
      if (write_bytes(output, marker_bytes, sizeof(marker_bytes)) != 0
          || write_bytes(output, length_bytes, sizeof(length_bytes)) != 0
          || copy_bytes(input, output, payload_size) != 0) {
        return -1;
      }
    }

    if (marker == 0xda) {
      return copy_remainder(input, output);
    }
  }
}

int image_exif_write(const char *filename, const union dc_shot_union *shot)
{
  if (filename == NULL || shot == NULL) {
    errno = EINVAL;
    return -1;
  }

  ExifData *exif = exif_data_new_from_file(filename);
  if (exif == NULL) {
    exif = exif_data_new();
  }
  if (exif == NULL) {
    fprintf(stderr, "EXIF:\tfailed to allocate metadata for %s\n", filename);
    return -1;
  }

  exif_data_set_option(exif, EXIF_DATA_OPTION_FOLLOW_SPECIFICATION);
  exif_data_set_option(exif, EXIF_DATA_OPTION_DONT_CHANGE_MAKER_NOTE);
  exif_data_set_data_type(exif, EXIF_DATA_TYPE_COMPRESSED);
  exif_data_set_byte_order(exif, byte_order);
  exif_data_fix(exif);
  if (add_shot_tags(exif, shot) != 0) {
    fprintf(stderr, "EXIF:\tfailed to create shot metadata for %s\n", filename);
    exif_data_unref(exif);
    return -1;
  }

  unsigned char *exif_data = NULL;
  unsigned int exif_size = 0;
  exif_data_save_data(exif, &exif_data, &exif_size);
  exif_data_unref(exif);
  if (exif_data == NULL || exif_size == 0 || exif_size > EXIF_APP1_MAX_PAYLOAD) {
    fprintf(stderr, "EXIF:\tmetadata block is invalid or too large for %s\n", filename);
    free(exif_data);
    return -1;
  }

  struct stat file_status;
  FILE *input = fopen(filename, "rb");
  if (input == NULL || stat(filename, &file_status) != 0) {
    fprintf(stderr, "EXIF:\tfailed to open %s: %s\n", filename, strerror(errno));
    if (input != NULL) {
      fclose(input);
    }
    free(exif_data);
    return -1;
  }

  size_t template_size = strlen(filename) + sizeof(".exif.XXXXXX");
  char *temporary_name = malloc(template_size);
  if (temporary_name == NULL) {
    fclose(input);
    free(exif_data);
    return -1;
  }
  int name_length = snprintf(temporary_name, template_size, "%s.exif.XXXXXX", filename);
  int temporary_fd = name_length < 0 || (size_t)name_length >= template_size ? -1 : mkstemp(temporary_name);
  FILE *output = temporary_fd >= 0 ? fdopen(temporary_fd, "wb") : NULL;
  int result = 0;
  if (output == NULL || fchmod(temporary_fd, file_status.st_mode & 0777) != 0
      || rewrite_jpeg(input, output, exif_data, exif_size) != 0
      || fflush(output) != 0 || fsync(temporary_fd) != 0) {
    result = -1;
  }
  if (fclose(input) != 0) {
    result = -1;
  }
  if (output != NULL && fclose(output) != 0) {
    result = -1;
  } else if (output == NULL && temporary_fd >= 0) {
    close(temporary_fd);
  }

  if (result == 0 && rename(temporary_name, filename) != 0) {
    result = -1;
  }
  if (result != 0) {
    int saved_errno = errno;
    unlink(temporary_name);
    fprintf(stderr, "EXIF:\tfailed to update %s: %s\n", filename, strerror(saved_errno));
    errno = saved_errno;
  }

  free(temporary_name);
  free(exif_data);
  return result;
}