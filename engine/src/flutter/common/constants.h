// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_COMMON_CONSTANTS_H_
#define FLUTTER_COMMON_CONSTANTS_H_

namespace flutter {
constexpr double kMegaByteSizeInBytes = (1 << 20);

// The ID for the implicit view if the implicit view is enabled.
//
// The implicit view is a compatibility mechanism to help the transition from
// the older single-view APIs to the newer multi-view APIs. The two sets of APIs
// use different models for view management. The implicit view mechanism allows
// single-view APIs to operate a special view as if other views don't exist.
//
// In the regular multi-view model, all views should be created by
// `Shell::AddView` before being used, and removed by `Shell::RemoveView` to
// signify that they are gone. If a view is added or removed, the framework
// (`PlatformDispatcher`) will be notified. New view IDs are always unique,
// never reused. Operating a non-existing view is an error.
//
// The implicit view is another special view in addition to the "regular views"
// as above. The shell starts up having the implicit view, which has a fixed
// view ID of `kFlutterImplicitViewId` and is available throughout the lifetime
// of the shell. `Shell::AddView` or `RemoveView` must not be called for this
// view. Even when the window that shows the view is closed, the framework is
// unaware and might continue rendering into or operating this view.
//
// The single-view APIs, which are APIs that do not specify view IDs, operate
// the implicit view. The multi-view APIs can operate all views, including the
// implicit view if the target ID is `kFlutterImplicitViewId`, unless specified
// otherwise.
constexpr int64_t kFlutterImplicitViewId = 0;
}  // namespace flutter

#endif  // FLUTTER_COMMON_CONSTANTS_H_
