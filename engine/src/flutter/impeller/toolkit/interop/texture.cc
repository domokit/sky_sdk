// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/toolkit/interop/texture.h"

namespace impeller::interop {

Texture::Texture(const Context& context, const TextureDescriptor& descriptor) {
  if (!context.IsValid()) {
    return;
  }
  auto texture =
      context.GetContext()->GetResourceAllocator()->CreateTexture(descriptor);
  if (!texture || !texture->IsValid()) {
    return;
  }
  backend_ = context.GetContext()->GetBackendType();
  texture_ = std::move(texture);
}

Texture::Texture(impeller::Context::BackendType backend,
                 std::shared_ptr<impeller::Texture> texture)
    : backend_(backend), texture_(std::move(texture)) {}

Texture::~Texture() = default;

bool Texture::IsValid() const {
  return !!texture_;
}

bool Texture::SetContents(const uint8_t* contents, uint64_t length) {
  if (!IsValid()) {
    return false;
  }
  return texture_->SetContents(contents, length);
}

bool Texture::SetContents(std::shared_ptr<const fml::Mapping> contents) {
  if (!IsValid()) {
    return false;
  }
  return texture_->SetContents(std::move(contents));
}

sk_sp<DlImageImpeller> Texture::MakeImage() const {
  return DlImageImpeller::Make(texture_);
}

impeller::Context::BackendType Texture::GetBackendType() const {
  return backend_;
}

const std::shared_ptr<impeller::Texture>& Texture::GetTexture() const {
  return texture_;
}

}  // namespace impeller::interop
