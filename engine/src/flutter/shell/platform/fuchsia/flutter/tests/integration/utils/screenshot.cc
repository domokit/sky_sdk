// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "screenshot.h"

#include <lib/zx/vmar.h>

#include <map>
#include <ostream>
#include <utility>
#include <vector>

#include "flutter/fml/logging.h"

namespace fuchsia_test_utils {
namespace {
constexpr uint64_t kBytesPerPixel = 4;
}  // namespace

Screenshot::Screenshot(const zx::vmo& screenshot_vmo,
                       uint64_t width,
                       uint64_t height,
                       int rotation)
    : width_(width), height_(height) {
  FML_CHECK(rotation == 0 || rotation == 90 || rotation == 270);
  if (rotation == 90 || rotation == 270) {
    std::swap(width_, height_);
  }
  // Populate |screenshot_| from |screenshot_vmo|.
  uint64_t vmo_size;
  screenshot_vmo.get_prop_content_size(&vmo_size);
  FML_CHECK(vmo_size == kBytesPerPixel * width_ * height_);
  uint8_t* vmo_host = nullptr;
  auto status = zx::vmar::root_self()->map(
      ZX_VM_PERM_READ, /*vmar_offset*/ 0, screenshot_vmo,
      /*vmo_offset*/ 0, vmo_size, reinterpret_cast<uintptr_t*>(&vmo_host));
  FML_CHECK(status == ZX_OK);
  ExtractScreenshotFromVMO(vmo_host);
  // map the pointer.
  uintptr_t address = reinterpret_cast<uintptr_t>(vmo_host);
  status = zx::vmar::root_self()->unmap(address, vmo_size);
  FML_CHECK(status == ZX_OK);
}

std::ostream& operator<<(std::ostream& stream, const Pixel& pixel) {
  return stream << "{Pixel:" << " r:" << static_cast<unsigned int>(pixel.red)
                << " g:" << static_cast<unsigned int>(pixel.green)
                << " b:" << static_cast<unsigned int>(pixel.blue)
                << " a:" << static_cast<unsigned int>(pixel.alpha) << "}";
}

Pixel Screenshot::GetPixelAt(uint64_t x, uint64_t y) const {
  FML_CHECK(x >= 0 && x < width_ && y >= 0 && y < height_)
      << "Index out of bounds";
  return screenshot_[y][x];
}

std::map<Pixel, uint32_t> Screenshot::Histogram() const {
  std::map<Pixel, uint32_t> histogram;
  FML_CHECK(screenshot_.size() == height_ && screenshot_[0].size() == width_);
  for (size_t i = 0; i < height_; i++) {
    for (size_t j = 0; j < width_; j++) {
      histogram[screenshot_[i][j]]++;
    }
  }
  return histogram;
}

void Screenshot::ExtractScreenshotFromVMO(uint8_t* screenshot_vmo) {
  FML_CHECK(screenshot_vmo);
  for (size_t i = 0; i < height_; i++) {
    // The head index of the ith row in the screenshot is |i* width_*
    // KbytesPerPixel|.
    screenshot_.push_back(GetPixelsInRow(screenshot_vmo, i));
  }
}

std::vector<Pixel> Screenshot::GetPixelsInRow(uint8_t* screenshot_vmo,
                                              size_t row_index) {
  std::vector<Pixel> row;
  for (size_t col_idx = 0;
       col_idx < static_cast<size_t>(width_ * kBytesPerPixel);
       col_idx += kBytesPerPixel) {
    // Each row in the screenshot has |kBytesPerPixel * width_| elements.
    // Therefore in order to reach the first pixel of the |row_index| row, we
    // have to jump |row_index * width_ * kBytesPerPixel| positions.
    auto pixel_start_index = row_index * width_ * kBytesPerPixel;
    // Every |kBytesPerPixel| bytes represents the BGRA values of a pixel. Skip
    // |kBytesPerPixel| bytes to get to the BGRA values of the next pixel. Each
    // row in a screenshot has |kBytesPerPixel * width_| bytes of data.
    // Example:-
    // auto data = TakeScreenshot();
    // data[0-3] -> RGBA of pixel 0.
    // data[4-7] -> RGBA pf pixel 1.
    row.emplace_back(screenshot_vmo[pixel_start_index + col_idx],
                     screenshot_vmo[pixel_start_index + col_idx + 1],
                     screenshot_vmo[pixel_start_index + col_idx + 2],
                     screenshot_vmo[pixel_start_index + col_idx + 3]);
  }
  return row;
}

}  // namespace fuchsia_test_utils
