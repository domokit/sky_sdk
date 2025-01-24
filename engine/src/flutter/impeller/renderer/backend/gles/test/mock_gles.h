// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BACKEND_GLES_TEST_MOCK_GLES_H_
#define FLUTTER_IMPELLER_RENDERER_BACKEND_GLES_TEST_MOCK_GLES_H_

#include <memory>
#include <optional>

#include "gmock/gmock.h"
#include "impeller/renderer/backend/gles/proc_table_gles.h"

namespace impeller {
namespace testing {

extern const ProcTableGLES::Resolver kMockResolverGLES;

class IMockGLESImpl {
 public:
  virtual ~IMockGLESImpl() = default;
  virtual void DeleteTextures(GLsizei size, const GLuint* queries) {}
  virtual void GenTextures(GLsizei n, GLuint* textures) {}
  virtual void ObjectLabelKHR(GLenum identifier,
                              GLuint name,
                              GLsizei length,
                              const GLchar* label) {}
  virtual void Uniform1fv(GLint location, GLsizei count, const GLfloat* value) {
  }
  virtual void GenQueriesEXT(GLsizei n, GLuint* ids) {}
  virtual void BeginQueryEXT(GLenum target, GLuint id) {}
  virtual void EndQueryEXT(GLuint id) {}
  virtual void GetQueryObjectuivEXT(GLuint id, GLenum target, GLuint* result) {}
  virtual void GetQueryObjectui64vEXT(GLuint id,
                                      GLenum target,
                                      GLuint64* result) {}
  virtual void DeleteQueriesEXT(GLsizei size, const GLuint* queries) {}
  virtual void GenBuffers(GLsizei n, GLuint* buffers) {}
  virtual GLboolean IsTexture(GLuint texture) { return true; }
};

class MockGLESImpl : public IMockGLESImpl {
 public:
  MOCK_METHOD(void,
              DeleteTextures,
              (GLsizei size, const GLuint* queries),
              (override));
  MOCK_METHOD(void, GenTextures, (GLsizei n, GLuint* textures), (override));
  MOCK_METHOD(
      void,
      ObjectLabelKHR,
      (GLenum identifier, GLuint name, GLsizei length, const GLchar* label),
      (override));
  MOCK_METHOD(void,
              Uniform1fv,
              (GLint location, GLsizei count, const GLfloat* value),
              (override));
  MOCK_METHOD(void, GenQueriesEXT, (GLsizei n, GLuint* ids), (override));
  MOCK_METHOD(void, BeginQueryEXT, (GLenum target, GLuint id), (override));
  MOCK_METHOD(void, EndQueryEXT, (GLuint id), (override));
  MOCK_METHOD(void,
              GetQueryObjectuivEXT,
              (GLuint id, GLenum target, GLuint* result),
              (override));
  MOCK_METHOD(void,
              GetQueryObjectui64vEXT,
              (GLuint id, GLenum target, GLuint64* result),
              (override));
  MOCK_METHOD(void,
              DeleteQueriesEXT,
              (GLsizei size, const GLuint* queries),
              (override));
  MOCK_METHOD(void, GenBuffers, (GLsizei n, GLuint* buffers), (override));
  MOCK_METHOD(GLboolean, IsTexture, (GLuint texture), (override));
};

/// @brief      Provides a mocked version of the |ProcTableGLES| class.
///
/// Typically, Open GLES at runtime will be provided the host's GLES bindings
/// (as function pointers). This class maintains a set of function pointers that
/// appear to be GLES functions, but are actually just stubs that record
/// invocations.
///
/// See `README.md` for more information.
class MockGLES final {
 public:
  static std::shared_ptr<MockGLES> Init(
      std::unique_ptr<MockGLESImpl> impl,
      const std::optional<std::vector<const char*>>& extensions = std::nullopt);

  /// @brief      Returns an initialized |MockGLES| instance.
  ///
  /// This method overwrites mocked global GLES function pointers to record
  /// invocations on this instance of |MockGLES|. As such, it should only be
  /// called once per test.
  static std::shared_ptr<MockGLES> Init(
      const std::optional<std::vector<const char*>>& extensions = std::nullopt,
      const char* version_string = "OpenGL ES 3.0",
      ProcTableGLES::Resolver resolver = kMockResolverGLES);

  /// @brief      Returns a configured |ProcTableGLES| instance.
  const ProcTableGLES& GetProcTable() const { return proc_table_; }

  ~MockGLES();

  IMockGLESImpl* GetImpl() { return impl_.get(); }

 private:
  friend void RecordGLCall(const char* name);
  friend void mockGenTextures(GLsizei n, GLuint* textures);

  explicit MockGLES(ProcTableGLES::Resolver resolver = kMockResolverGLES);

  ProcTableGLES proc_table_;
  std::unique_ptr<IMockGLESImpl> impl_;

  MockGLES(const MockGLES&) = delete;

  MockGLES& operator=(const MockGLES&) = delete;
};

}  // namespace testing
}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_BACKEND_GLES_TEST_MOCK_GLES_H_
