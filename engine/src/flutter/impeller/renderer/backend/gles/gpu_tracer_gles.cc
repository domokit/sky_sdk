// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/backend/gles/gpu_tracer_gles.h"
#include <thread>
#include "fml/trace_event.h"

namespace impeller {

GPUTracerGLES::GPUTracerGLES(const ProcTableGLES& gl, bool enable_tracing) {
#ifdef IMPELLER_DEBUG
  auto desc = gl.GetDescription();
  enabled_ =
      enable_tracing && desc->HasExtension("GL_EXT_disjoint_timer_query");
#endif  // IMPELLER_DEBUG
}

void GPUTracerGLES::MarkFrameStart(const ProcTableGLES& gl) {
  if (!enabled_ || active_frame_.has_value() ||
      std::this_thread::get_id() != raster_thread_) {
    return;
  }

  // At the beginning of a frame, check the status of all pending
  // previous queries.
  ProcessQueries(gl);

  uint32_t query = 0;
  gl.GenQueriesEXT(1, &query);
  if (query == 0) {
    return;
  }

  active_frame_ = query;
  gl.BeginQueryEXT(GL_TIME_ELAPSED_EXT, query);
}

void GPUTracerGLES::RecordRasterThread() {
  raster_thread_ = std::this_thread::get_id();
}

void GPUTracerGLES::ProcessQueries(const ProcTableGLES& gl) {
  // For reasons unknown to me, querying the state of more than
  // one query object per frame causes crashes on a Pixel 6 pro.
  // It does not crash on an S10.
  while (!pending_traces_.empty()) {
    auto query = pending_traces_.front();

    // First check if the query is complete without blocking
    // on the result. Incomplete results are left in the pending
    // trace vector and will not be checked again for another
    // frame.
    GLuint available = GL_FALSE;
    gl.GetQueryObjectuivEXT(query, GL_QUERY_RESULT_AVAILABLE_EXT, &available);

    if (available != GL_TRUE) {
      // If a query is not available, then all subsequent queries will be
      // unavailable.
      return;
    }
    // Return the timer resolution in nanoseconds.
    uint64_t duration = 0;
    gl.GetQueryObjectui64vEXT(query, GL_QUERY_RESULT_EXT, &duration);
    auto gpu_ms = duration / 1000000.0;

    FML_TRACE_COUNTER("flutter", "GPUTracer",
                      reinterpret_cast<int64_t>(this),  // Trace Counter ID
                      "FrameTimeMS", gpu_ms);
    gl.DeleteQueriesEXT(1, &query);
    pending_traces_.pop_front();
  }
}

void GPUTracerGLES::MarkFrameEnd(const ProcTableGLES& gl) {
  if (!enabled_ || std::this_thread::get_id() != raster_thread_ ||
      !active_frame_.has_value()) {
    return;
  }

  auto query = active_frame_.value();
  gl.EndQueryEXT(GL_TIME_ELAPSED_EXT);

  pending_traces_.push_back(query);
  active_frame_ = std::nullopt;
}

}  // namespace impeller
