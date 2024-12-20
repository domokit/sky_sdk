// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/testing/testing.h"  // IWYU pragma: keep
#include "gtest/gtest.h"
#include "impeller/renderer/backend/gles/gpu_tracer_gles.h"
#include "impeller/renderer/backend/gles/test/mock_gles.h"

namespace impeller {
namespace testing {

using ::testing::_;

#ifdef IMPELLER_DEBUG
TEST(GPUTracerGLES, CanFormatFramebufferErrorMessage) {
  auto const extensions = std::vector<const char*>{
      "GL_KHR_debug",                 //
      "GL_EXT_disjoint_timer_query",  //
  };
  auto mock_gles_impl = std::make_unique<MockGLESImpl>();

  {
    ::testing::InSequence seq;
    auto gen_queries = [](GLsizei n, GLuint* ids) {
      for (int i = 0; i < n; ++i) {
        ids[i] = i + 1;
      }
    };
    EXPECT_CALL(*mock_gles_impl, GenQueriesEXT(_, _)).WillOnce(gen_queries);
    EXPECT_CALL(*mock_gles_impl, BeginQueryEXT(GL_TIME_ELAPSED_EXT, _));
    EXPECT_CALL(*mock_gles_impl, EndQueryEXT(GL_TIME_ELAPSED_EXT));
    EXPECT_CALL(*mock_gles_impl,
                GetQueryObjectuivEXT(_, GL_QUERY_RESULT_AVAILABLE_EXT, _))
        .WillOnce([](GLuint id, GLenum target, GLuint* result) {
          *result = GL_TRUE;
        });
    EXPECT_CALL(*mock_gles_impl,
                GetQueryObjectui64vEXT(_, GL_QUERY_RESULT_EXT, _))
        .WillOnce([](GLuint id, GLenum target, GLuint64* result) {
          *result = 1000u;
        });
    EXPECT_CALL(*mock_gles_impl, DeleteQueriesEXT(_, _));
    EXPECT_CALL(*mock_gles_impl, GenQueriesEXT(_, _)).WillOnce(gen_queries);
    EXPECT_CALL(*mock_gles_impl, BeginQueryEXT(GL_TIME_ELAPSED_EXT, _));
  }
  std::shared_ptr<MockGLES> mock_gles =
      MockGLES::Init(std::move(mock_gles_impl), extensions);
  auto tracer =
      std::make_shared<GPUTracerGLES>(mock_gles->GetProcTable(), true);
  tracer->RecordRasterThread();
  tracer->MarkFrameStart(mock_gles->GetProcTable());
  tracer->MarkFrameEnd(mock_gles->GetProcTable());
  tracer->MarkFrameStart(mock_gles->GetProcTable());
}

#endif  // IMPELLER_DEBUG

}  // namespace testing
}  // namespace impeller
