// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BACKEND_GLES_DESCRIPTION_GLES_H_
#define FLUTTER_IMPELLER_RENDERER_BACKEND_GLES_DESCRIPTION_GLES_H_

#include <set>
#include <string>

#include "impeller/base/version.h"

namespace impeller {

class ProcTableGLES;

class DescriptionGLES {
 public:
  explicit DescriptionGLES(const ProcTableGLES& gl);

  ~DescriptionGLES();

  bool IsValid() const;

  bool IsES() const;

  std::string GetString() const;

  Version GetGlVersion() const;

  bool HasExtension(const std::string& ext) const;

  /// @brief      Returns whether GLES includes the debug extension.
  bool HasDebugExtension() const;

  bool IsANGLE() const;

 private:
  Version gl_version_;
  Version sl_version_;
  bool is_es_ = true;
  std::string vendor_;
  std::string renderer_;
  std::string gl_version_string_;
  std::string sl_version_string_;
  std::set<std::string> extensions_;
  bool is_angle_ = false;
  bool is_valid_ = false;

  DescriptionGLES(const DescriptionGLES&) = delete;

  DescriptionGLES& operator=(const DescriptionGLES&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_BACKEND_GLES_DESCRIPTION_GLES_H_
