// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BACKEND_GLES_GPU_TRACER_GLES_H_
#define FLUTTER_IMPELLER_RENDERER_BACKEND_GLES_GPU_TRACER_GLES_H_

#include <cstdint>
#include <deque>
#include <thread>

#include "impeller/renderer/backend/gles/proc_table_gles.h"

namespace impeller {

/// @brief Trace GPU execution times using GL_EXT_disjoint_timer_query on GLES.
///
/// Note: there are a substantial number of GPUs where usage of the this API is
/// known to cause crashes. As a result, this functionality is disabled by
/// default and can only be enabled in debug/profile mode via a specific opt-in
/// flag that is exposed in the Android manifest.
///
/// To enable, add the following metadata to the application's Android manifest:
///   <meta-data
///       android:name="io.flutter.embedding.android.EnableOpenGLGPUTracing"
///       android:value="false" />
class GPUTracerGLES {
 public:
  GPUTracerGLES(const ProcTableGLES& gl, bool enable_tracing);

  ~GPUTracerGLES() = default;

  /// @brief Record the thread id of the raster thread.
  void RecordRasterThread();

  /// @brief Record the start of a frame workload, if one hasn't already been
  ///        started.
  void MarkFrameStart(const ProcTableGLES& gl);

  /// @brief Record the end of a frame workload.
  void MarkFrameEnd(const ProcTableGLES& gl);

 private:
  void ProcessQueries(const ProcTableGLES& gl);

  std::deque<uint32_t> pending_traces_;
  std::optional<uint32_t> active_frame_ = std::nullopt;
  std::thread::id raster_thread_;

  bool enabled_ = false;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_BACKEND_GLES_GPU_TRACER_GLES_H_
