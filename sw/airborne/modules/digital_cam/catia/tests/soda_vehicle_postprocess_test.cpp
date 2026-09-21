/*
 * Unit tests for soda_vehicle_postprocess.cpp's internal helpers, pulled in directly
 * (matching this codebase's existing "#include ../foo.c" idiom, e.g.
 * vehicle_detect_save_copy_test.c) so the anonymous-namespace functions are reachable:
 *   - compute_crop_box(): the C++ port of the removed Python cropper's padding math.
 *   - parse_vehicle_summary(): the AICAM_VEHICLES_V1 EXIF-comment parser, exercised
 *     against synthetic buffers so it needs no real JPEG/EXIF I/O to test.
 *   - draw_box_outline(): the in-place pixel annotation SODA now does itself (the source
 *     capture at image_path must stay clean -- see vehicle_detect_server.py's docstring).
 */
#include "../soda_vehicle_postprocess.cpp"

#include <cassert>
#include <iostream>

namespace {

void test_compute_crop_box()
{
  // Centered box, comfortably inside the frame: cross-checked against a real end-to-end
  // run (box=100,150,300,220 on a 1200x899 frame produced a 408x300 crop).
  {
    CropBox box = compute_crop_box(100, 150, 300, 220, 1200, 899, 0.18);
    assert(box.left == 46 && box.top == 110 && box.right == 454 && box.bottom == 410);
  }
  // Box touching the left/top edge: padding clamps to 0 instead of going negative.
  {
    CropBox box = compute_crop_box(0, 0, 100, 80, 1200, 899, 0.18);
    assert(box.left == 0 && box.top == 0 && box.right == 118 && box.bottom == 94);
  }
  // Box touching the right/bottom edge: padding clamps to the frame's own size.
  {
    CropBox box = compute_crop_box(1100, 800, 100, 99, 1200, 899, 0.18);
    assert(box.left == 1082 && box.top == 782 && box.right == 1200 && box.bottom == 899);
  }
  // Degenerate zero-area box: guarded to stay a croppable (>=1x1) region either way.
  {
    CropBox box = compute_crop_box(500, 400, 0, 0, 1200, 899, 0.18);
    assert(box.right == box.left + 1 && box.bottom == box.top + 1);
  }
  std::cout << "compute_crop_box: ok" << std::endl;
}

void test_parse_vehicle_summary()
{
  VehicleDetection detection;

  // A genuine hit: everything after the marker, matching what
  // vehicle_detect_pipe_last_detection_summary() actually produces for count >= 1.
  {
    std::string text = "some flight metadata\nAICAM_VEHICLES_V1\n"
                        "status=ok; count=2; label=vehicle; confidence=0.8734; box=100,150,300,220";
    assert(parse_vehicle_summary(text, &detection));
    assert(detection.count == 2);
    assert(detection.confidence > 0.873 && detection.confidence < 0.874);
    assert(detection.box_x == 100 && detection.box_y == 150);
    assert(detection.box_w == 300 && detection.box_h == 220);
  }

  // "status=ok; count=0" (no vehicle seen this shot) is not a hit.
  {
    std::string text = "some flight metadata\nAICAM_VEHICLES_V1\nstatus=ok; count=0";
    assert(!parse_vehicle_summary(text, &detection));
  }

  // "status=analysis_failed" (detection subsystem failed this shot) is not a hit.
  {
    std::string text = "some flight metadata\nAICAM_VEHICLES_V1\n"
                        "status=analysis_failed; coordinates_omitted=true";
    assert(!parse_vehicle_summary(text, &detection));
  }

  // No marker at all -- every non-aicam-detect shot's EXIF comment (e.g. a plain LWIR
  // hotspot record, or ordinary flight metadata with no analysis tag whatsoever).
  {
    std::string text = "some flight metadata\nLWIR_HOTSPOTS_V1\nstatus=ok; count=1";
    assert(!parse_vehicle_summary(text, &detection));
  }
  {
    std::string text = "some flight metadata, no tagged record at all";
    assert(!parse_vehicle_summary(text, &detection));
  }

  // Regression guard: parse_vehicle_summary() receives a std::string built from
  // entry->size (not strlen()) in read_vehicle_detection(), so it may contain an embedded
  // NUL followed by more bytes (e.g. the real EXIF entry's own zero-filled trailing byte).
  // sscanf() on .c_str() still stops at the first NUL, so this must keep parsing whatever
  // precedes it correctly rather than choking on the trailing bytes.
  {
    std::string text = "some flight metadata\nAICAM_VEHICLES_V1\n"
                        "status=ok; count=1; label=vehicle; confidence=0.5000; box=1,2,3,4";
    text.push_back('\0');
    text.append("trailing garbage after an embedded NUL");
    assert(parse_vehicle_summary(text, &detection));
    assert(detection.count == 1 && detection.box_x == 1 && detection.box_h == 4);
  }

  std::cout << "parse_vehicle_summary: ok" << std::endl;
}

void test_draw_box_outline()
{
  // A 10x10 buffer, all black. Draw a 4x4 box at (2,2) with thickness 1: only the 1px
  // border of [2,6)x[2,6) turns green -- the 2x2 interior (3,3)-(4,4) is untouched, since
  // a 1px stroke from each side doesn't reach the middle of a 4px-wide box. This is
  // deliberately an outline, not a filled rectangle -- see draw_box_outline()'s doc comment.
  DecodedImage image;
  image.width = 10;
  image.height = 10;
  image.pixels.assign(static_cast<size_t>(image.width) * image.height * 3, 0);

  draw_box_outline(&image, 2, 2, 4, 4, 1);

  auto pixel_is_green = [&](int x, int y) {
    size_t offset = (static_cast<size_t>(y) * image.width + static_cast<size_t>(x)) * 3;
    return image.pixels[offset] == 0 && image.pixels[offset + 1] == 255 && image.pixels[offset + 2] == 0;
  };
  auto pixel_is_black = [&](int x, int y) {
    size_t offset = (static_cast<size_t>(y) * image.width + static_cast<size_t>(x)) * 3;
    return image.pixels[offset] == 0 && image.pixels[offset + 1] == 0 && image.pixels[offset + 2] == 0;
  };

  for (int x = 2; x < 6; ++x) {
    assert(pixel_is_green(x, 2) && pixel_is_green(x, 5));  // top and bottom border rows
  }
  for (int y = 2; y < 6; ++y) {
    assert(pixel_is_green(2, y) && pixel_is_green(5, y));  // left and right border columns
  }
  assert(pixel_is_black(3, 3) && pixel_is_black(3, 4) && pixel_is_black(4, 3) && pixel_is_black(4, 4));
  assert(pixel_is_black(1, 1) && pixel_is_black(6, 6) && pixel_is_black(0, 0) && pixel_is_black(9, 9));

  // A thick (2px) outline on a larger box: only the border rows/columns turn green, the
  // interior stays untouched -- proves this draws an outline, not a filled rectangle.
  DecodedImage large;
  large.width = 20;
  large.height = 20;
  large.pixels.assign(static_cast<size_t>(large.width) * large.height * 3, 0);
  draw_box_outline(&large, 5, 5, 10, 10, 2);
  auto large_pixel_is_green = [&](int x, int y) {
    size_t offset = (static_cast<size_t>(y) * large.width + static_cast<size_t>(x)) * 3;
    return large.pixels[offset] == 0 && large.pixels[offset + 1] == 255 && large.pixels[offset + 2] == 0;
  };
  auto large_pixel_is_black = [&](int x, int y) {
    size_t offset = (static_cast<size_t>(y) * large.width + static_cast<size_t>(x)) * 3;
    return large.pixels[offset] == 0 && large.pixels[offset + 1] == 0 && large.pixels[offset + 2] == 0;
  };
  assert(large_pixel_is_green(5, 5) && large_pixel_is_green(6, 5));   // top edge, both stroke rows
  assert(large_pixel_is_green(14, 14) && large_pixel_is_green(13, 14)); // bottom edge, both stroke rows
  assert(large_pixel_is_black(10, 10));                               // interior untouched

  // Box partially off-frame: out-of-bounds pixels are silently clipped, never a crash or
  // wraparound write (this is exactly the case a box touching a frame edge produces).
  DecodedImage edge;
  edge.width = 8;
  edge.height = 8;
  edge.pixels.assign(static_cast<size_t>(edge.width) * edge.height * 3, 0);
  draw_box_outline(&edge, -2, -2, 6, 6, 1);
  auto edge_pixel_is_green = [&](int x, int y) {
    size_t offset = (static_cast<size_t>(y) * edge.width + static_cast<size_t>(x)) * 3;
    return edge.pixels[offset] == 0 && edge.pixels[offset + 1] == 255 && edge.pixels[offset + 2] == 0;
  };
  assert(edge_pixel_is_green(3, 0) && edge_pixel_is_green(0, 3)); // the in-bounds part of the outline still draws

  std::cout << "draw_box_outline: ok" << std::endl;
}

} // namespace

int main()
{
  test_compute_crop_box();
  test_parse_vehicle_summary();
  test_draw_box_outline();
  std::cout << "soda vehicle post-processing unit tests passed" << std::endl;
  return 0;
}
