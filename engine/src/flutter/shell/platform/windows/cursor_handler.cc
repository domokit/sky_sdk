// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/windows/cursor_handler.h"

#include <windows.h>

#include "flutter/shell/platform/common/client_wrapper/include/flutter/standard_method_codec.h"
#include "flutter/shell/platform/windows/flutter_windows_engine.h"
#include "flutter/shell/platform/windows/flutter_windows_view.h"

static constexpr char kChannelName[] = "flutter/mousecursor";

static constexpr char kActivateSystemCursorMethod[] = "activateSystemCursor";
static constexpr char kKindKey[] = "kind";

// This method allows creating a custom cursor with rawBGRA buffer, returns a
// string to identify the cursor.
static constexpr char kCreateCustomCursorMethod[] =
    "createCustomCursor/windows";
// A string, the custom cursor's name.
static constexpr char kCustomCursorNameKey[] = "name";
// A list of bytes, the custom cursor's rawBGRA buffer.
static constexpr char kCustomCursorBufferKey[] = "buffer";
// A double, the x coordinate of the custom cursor's hotspot, starting from
// left.
static constexpr char kCustomCursorHotXKey[] = "hotX";
// A double, the y coordinate of the custom cursor's hotspot, starting from top.
static constexpr char kCustomCursorHotYKey[] = "hotY";
// An int value for the width of the custom cursor.
static constexpr char kCustomCursorWidthKey[] = "width";
// An int value for the height of the custom cursor.
static constexpr char kCustomCursorHeightKey[] = "height";

// This method also has an argument `kCustomCursorNameKey` for the name
// of the cursor to activate.
static constexpr char kSetCustomCursorMethod[] = "setCustomCursor/windows";

// This method also has an argument `kCustomCursorNameKey` for the name
// of the cursor to delete.
static constexpr char kDeleteCustomCursorMethod[] =
    "deleteCustomCursor/windows";

// Error codes used for responses.
static constexpr char kCursorError[] = "Cursor error";

namespace flutter {

CursorHandler::CursorHandler(BinaryMessenger* messenger,
                             FlutterWindowsEngine* engine)
    : channel_(std::make_unique<MethodChannel<EncodableValue>>(
          messenger,
          kChannelName,
          &StandardMethodCodec::GetInstance())),
      engine_(engine) {
  channel_->SetMethodCallHandler(
      [this](const MethodCall<EncodableValue>& call,
             std::unique_ptr<MethodResult<EncodableValue>> result) {
        HandleMethodCall(call, std::move(result));
      });
}

void CursorHandler::HandleMethodCall(
    const MethodCall<EncodableValue>& method_call,
    std::unique_ptr<MethodResult<EncodableValue>> result) {
  const std::string& method = method_call.method_name();
  if (method.compare(kActivateSystemCursorMethod) == 0) {
    const auto& arguments = std::get<EncodableMap>(*method_call.arguments());
    auto kind_iter = arguments.find(EncodableValue(std::string(kKindKey)));
    if (kind_iter == arguments.end()) {
      result->Error("Argument error",
                    "Missing argument while trying to activate system cursor");
      return;
    }
    const auto& kind = std::get<std::string>(kind_iter->second);

    // TODO(loicsharma): Remove implicit view assumption.
    // https://github.com/flutter/flutter/issues/142845
    FlutterWindowsView* view = engine_->view(kImplicitViewId);
    if (view == nullptr) {
      result->Error(kCursorError,
                    "Cursor is not available in Windows headless mode");
      return;
    }
    view->UpdateFlutterCursor(kind);
    result->Success();
  } else if (method.compare(kCreateCustomCursorMethod) == 0) {
    const auto& arguments = std::get<EncodableMap>(*method_call.arguments());
    auto name_iter =
        arguments.find(EncodableValue(std::string(kCustomCursorNameKey)));
    if (name_iter == arguments.end()) {
      result->Error(
          "Argument error",
          "Missing argument name while trying to customize system cursor");
      return;
    }
    auto name = std::get<std::string>(name_iter->second);
    auto buffer_iter =
        arguments.find(EncodableValue(std::string(kCustomCursorBufferKey)));
    if (buffer_iter == arguments.end()) {
      result->Error(
          "Argument error",
          "Missing argument buffer while trying to customize system cursor");
      return;
    }
    auto buffer = std::get<std::vector<uint8_t>>(buffer_iter->second);
    auto width_iter =
        arguments.find(EncodableValue(std::string(kCustomCursorWidthKey)));
    if (width_iter == arguments.end()) {
      result->Error(
          "Argument error",
          "Missing argument width while trying to customize system cursor");
      return;
    }
    auto width = std::get<int>(width_iter->second);
    auto height_iter =
        arguments.find(EncodableValue(std::string(kCustomCursorHeightKey)));
    if (height_iter == arguments.end()) {
      result->Error(
          "Argument error",
          "Missing argument height while trying to customize system cursor");
      return;
    }
    auto height = std::get<int>(height_iter->second);
    auto hot_x_iter =
        arguments.find(EncodableValue(std::string(kCustomCursorHotXKey)));
    if (hot_x_iter == arguments.end()) {
      result->Error(
          "Argument error",
          "Missing argument hotX while trying to customize system cursor");
      return;
    }
    auto hot_x = std::get<double>(hot_x_iter->second);
    auto hot_y_iter =
        arguments.find(EncodableValue(std::string(kCustomCursorHotYKey)));
    if (hot_y_iter == arguments.end()) {
      result->Error(
          "Argument error",
          "Missing argument hotY while trying to customize system cursor");
      return;
    }
    auto hot_y = std::get<double>(hot_y_iter->second);
    HCURSOR cursor = GetCursorFromBuffer(buffer, hot_x, hot_y, width, height);
    if (cursor == nullptr) {
      result->Error("Argument error",
                    "Argument must contains a valid rawBGRA bitmap");
      return;
    }
    // Push the cursor into the cache map.
    custom_cursors_.emplace(name, std::move(cursor));
    result->Success(flutter::EncodableValue(std::move(name)));
  } else if (method.compare(kSetCustomCursorMethod) == 0) {
    const auto& arguments = std::get<EncodableMap>(*method_call.arguments());
    auto name_iter =
        arguments.find(EncodableValue(std::string(kCustomCursorNameKey)));
    if (name_iter == arguments.end()) {
      result->Error("Argument error",
                    "Missing argument key while trying to set a custom cursor");
      return;
    }
    auto name = std::get<std::string>(name_iter->second);
    if (custom_cursors_.find(name) == custom_cursors_.end()) {
      result->Error(
          "Argument error",
          "The custom cursor identified by the argument key cannot be found");
      return;
    }
    HCURSOR cursor = custom_cursors_[name];

    // TODO(loicsharma): Remove implicit view assumption.
    // https://github.com/flutter/flutter/issues/142845
    FlutterWindowsView* view = engine_->view(kImplicitViewId);
    if (view == nullptr) {
      result->Error(kCursorError,
                    "Cursor is not available in Windows headless mode");
      return;
    }
    view->SetFlutterCursor(cursor);
    result->Success();
  } else if (method.compare(kDeleteCustomCursorMethod) == 0) {
    const auto& arguments = std::get<EncodableMap>(*method_call.arguments());
    auto name_iter =
        arguments.find(EncodableValue(std::string(kCustomCursorNameKey)));
    if (name_iter == arguments.end()) {
      result->Error(
          "Argument error",
          "Missing argument key while trying to delete a custom cursor");
      return;
    }
    auto name = std::get<std::string>(name_iter->second);
    auto it = custom_cursors_.find(name);
    // If the specified cursor name is not found, the deletion is a noop and
    // returns success.
    if (it != custom_cursors_.end()) {
      DeleteObject(it->second);
      custom_cursors_.erase(it);
    }
    result->Success();
  } else {
    result->NotImplemented();
  }
}

HCURSOR GetCursorFromBuffer(const std::vector<uint8_t>& buffer,
                            double hot_x,
                            double hot_y,
                            int width,
                            int height) {
  HCURSOR cursor = nullptr;
  HDC display_dc = GetDC(NULL);
  // Flutter should returns rawBGRA, which has 8bits * 4channels.
  BITMAPINFO bmi;
  memset(&bmi, 0, sizeof(bmi));
  bmi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
  bmi.bmiHeader.biWidth = width;
  bmi.bmiHeader.biHeight = -height;
  bmi.bmiHeader.biPlanes = 1;
  bmi.bmiHeader.biBitCount = 32;
  bmi.bmiHeader.biCompression = BI_RGB;
  bmi.bmiHeader.biSizeImage = width * height * 4;
  // Create the pixmap DIB section
  uint8_t* pixels = 0;
  HBITMAP bitmap =
      CreateDIBSection(display_dc, &bmi, DIB_RGB_COLORS, (void**)&pixels, 0, 0);
  ReleaseDC(0, display_dc);
  if (!bitmap || !pixels) {
    return nullptr;
  }
  int bytes_per_line = width * 4;
  for (int y = 0; y < height; ++y) {
    memcpy(pixels + y * bytes_per_line, &buffer[bytes_per_line * y],
           bytes_per_line);
  }
  HBITMAP mask;
  GetMaskBitmaps(bitmap, mask);
  ICONINFO icon_info;
  icon_info.fIcon = 0;
  icon_info.xHotspot = hot_x;
  icon_info.yHotspot = hot_y;
  icon_info.hbmMask = mask;
  icon_info.hbmColor = bitmap;
  cursor = CreateIconIndirect(&icon_info);
  DeleteObject(mask);
  DeleteObject(bitmap);
  return cursor;
}

void GetMaskBitmaps(HBITMAP bitmap, HBITMAP& mask_bitmap) {
  HDC h_dc = ::GetDC(NULL);
  HDC h_main_dc = ::CreateCompatibleDC(h_dc);
  HDC h_and_mask_dc = ::CreateCompatibleDC(h_dc);

  // Get the dimensions of the source bitmap
  BITMAP bm;
  ::GetObject(bitmap, sizeof(BITMAP), &bm);
  mask_bitmap = ::CreateCompatibleBitmap(h_dc, bm.bmWidth, bm.bmHeight);

  // Select the bitmaps to DC
  HBITMAP h_old_main_bitmap = (HBITMAP)::SelectObject(h_main_dc, bitmap);
  HBITMAP h_old_and_mask_bitmap =
      (HBITMAP)::SelectObject(h_and_mask_dc, mask_bitmap);

  // Scan each pixel of the souce bitmap and create the masks
  COLORREF main_bit_pixel;
  for (int x = 0; x < bm.bmWidth; ++x) {
    for (int y = 0; y < bm.bmHeight; ++y) {
      main_bit_pixel = ::GetPixel(h_main_dc, x, y);
      if (main_bit_pixel == RGB(0, 0, 0)) {
        ::SetPixel(h_and_mask_dc, x, y, RGB(255, 255, 255));
      } else {
        ::SetPixel(h_and_mask_dc, x, y, RGB(0, 0, 0));
      }
    }
  }
  ::SelectObject(h_main_dc, h_old_main_bitmap);
  ::SelectObject(h_and_mask_dc, h_old_and_mask_bitmap);

  ::DeleteDC(h_and_mask_dc);
  ::DeleteDC(h_main_dc);

  ::ReleaseDC(NULL, h_dc);
}

}  // namespace flutter
