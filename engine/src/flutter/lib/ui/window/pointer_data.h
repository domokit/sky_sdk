// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_UI_WINDOW_POINTER_DATA_H_
#define FLUTTER_LIB_UI_WINDOW_POINTER_DATA_H_

#include <cstdint>

namespace flutter {

// Must match the button constants in events.dart.
enum PointerButtonMouse : int64_t {
  kPointerButtonMousePrimary = 1 << 0,
  kPointerButtonMouseSecondary = 1 << 1,
  kPointerButtonMouseMiddle = 1 << 2,
  kPointerButtonMouseBack = 1 << 3,
  kPointerButtonMouseForward = 1 << 4,
};

enum PointerButtonTouch : int64_t {
  kPointerButtonTouchContact = 1 << 0,
};

enum PointerButtonStylus : int64_t {
  kPointerButtonStylusContact = 1 << 0,
  kPointerButtonStylusPrimary = 1 << 1,
  kPointerButtonStylusSecondary = 1 << 2,
};

// This structure is unpacked by platform_dispatcher.dart.
//
// If this struct changes, update:
//  * kPointerDataFieldCount in pointer_data.cc. (The pointer_data.cc also
//    lists out other locations that must be kept consistent.)
//  * The functions to create simulated data in
//    pointer_data_packet_converter_unittests.cc.
struct alignas(8) PointerData {
  // Must match the PointerChange enum in pointer.dart.
  enum class Change : int64_t {
    kCancel,
    kAdd,
    kRemove,
    kHover,
    kDown,
    kMove,
    kUp,
    kPanZoomStart,
    kPanZoomUpdate,
    kPanZoomEnd,
  };

  // Must match the PointerDeviceKind enum in pointer.dart.
  enum class DeviceKind : int64_t {
    kTouch,
    kMouse,
    kStylus,
    kInvertedStylus,
    kTrackpad,
  };

  // Must match the PointerSignalKind enum in pointer.dart.
  enum class SignalKind : int64_t {
    kNone,
    kScroll,
    kScrollInertiaCancel,
    kScale,
  };

  int64_t embedder_id;
  int64_t time_stamp;
  Change change;
  DeviceKind kind;
  SignalKind signal_kind;
  int64_t device;
  int64_t pointer_identifier;
  double physical_x;
  double physical_y;
  double physical_delta_x;
  double physical_delta_y;
  int64_t buttons;
  int64_t obscured;
  int64_t synthesized;
  double pressure;
  double pressure_min;
  double pressure_max;
  double distance;
  double distance_max;
  double size;
  double radius_major;
  double radius_minor;
  double radius_min;
  double radius_max;
  double orientation;
  double tilt;
  int64_t platformData;
  double scroll_delta_x;
  double scroll_delta_y;
  double pan_x;
  double pan_y;
  double pan_delta_x;
  double pan_delta_y;
  double scale;
  double rotation;
  int64_t view_id;

  void Clear();
};

}  // namespace flutter

#endif  // FLUTTER_LIB_UI_WINDOW_POINTER_DATA_H_
