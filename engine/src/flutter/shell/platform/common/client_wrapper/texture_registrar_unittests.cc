// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/common/client_wrapper/include/flutter/texture_registrar.h"

#include <map>
#include <memory>
#include <vector>

#include "flutter/fml/synchronization/waitable_event.h"
#include "flutter/shell/platform/common/client_wrapper/include/flutter/plugin_registrar.h"
#include "flutter/shell/platform/common/client_wrapper/testing/stub_flutter_api.h"
#include "gtest/gtest.h"

namespace flutter {

namespace {

// Stub implementation to validate calls to the API.
class TestApi : public testing::StubFlutterApi {
 public:
  struct FakePixelBufferTexture {
    int64_t texture_id;
    int32_t mark_count;
    FlutterDesktopPixelBufferTextureCallback texture_callback;
    void* user_data;
  };

 public:
  int64_t TextureRegistrarRegisterExternalTexture(
      const FlutterDesktopTextureInfo* info) override {
    last_texture_id_++;

    auto texture = std::make_unique<FakePixelBufferTexture>();
    texture->texture_callback = info->pixel_buffer_config.callback;
    texture->user_data = info->pixel_buffer_config.user_data;
    texture->mark_count = 0;
    texture->texture_id = last_texture_id_;

    textures_[last_texture_id_] = std::move(texture);
    return last_texture_id_;
  }

  void TextureRegistrarUnregisterExternalTexture(
      int64_t texture_id,
      void (*callback)(void* user_data),
      void* user_data) override {
    auto it = textures_.find(texture_id);
    if (it != textures_.end()) {
      textures_.erase(it);
    }
    if (callback) {
      callback(user_data);
    }
  }

  bool TextureRegistrarMarkTextureFrameAvailable(int64_t texture_id) override {
    auto it = textures_.find(texture_id);
    if (it != textures_.end()) {
      it->second->mark_count++;
      return true;
    }
    return false;
  }

  FakePixelBufferTexture* GetFakeTexture(int64_t texture_id) {
    auto it = textures_.find(texture_id);
    if (it != textures_.end()) {
      return it->second.get();
    }
    return nullptr;
  }

  int64_t last_texture_id() { return last_texture_id_; }

  size_t textures_size() { return textures_.size(); }

 private:
  int64_t last_texture_id_ = -1;
  std::map<int64_t, std::unique_ptr<FakePixelBufferTexture>> textures_;
};

}  // namespace

// Tests thats textures can be registered and unregistered.
TEST(TextureRegistrarTest, RegisterUnregisterTexture) {
  testing::ScopedStubFlutterApi scoped_api_stub(std::make_unique<TestApi>());
  auto test_api = static_cast<TestApi*>(scoped_api_stub.stub());

  auto dummy_registrar_handle =
      reinterpret_cast<FlutterDesktopPluginRegistrarRef>(1);
  PluginRegistrar registrar(dummy_registrar_handle);
  TextureRegistrar* textures = registrar.texture_registrar();
  ASSERT_NE(textures, nullptr);

  EXPECT_EQ(test_api->last_texture_id(), -1);
  auto texture = test_api->GetFakeTexture(0);
  EXPECT_EQ(texture, nullptr);

  auto pixel_buffer_texture = std::make_unique<TextureVariant>(
      PixelBufferTexture([](size_t width, size_t height) { return nullptr; }));
  int64_t texture_id = textures->RegisterTexture(pixel_buffer_texture.get());
  EXPECT_EQ(test_api->last_texture_id(), texture_id);
  EXPECT_EQ(test_api->textures_size(), static_cast<size_t>(1));

  texture = test_api->GetFakeTexture(texture_id);
  EXPECT_EQ(texture->texture_id, texture_id);
  EXPECT_EQ(texture->user_data,
            std::get_if<PixelBufferTexture>(pixel_buffer_texture.get()));

  textures->MarkTextureFrameAvailable(texture_id);
  textures->MarkTextureFrameAvailable(texture_id);
  bool success = textures->MarkTextureFrameAvailable(texture_id);
  EXPECT_TRUE(success);
  EXPECT_EQ(texture->mark_count, 3);

  fml::AutoResetWaitableEvent unregister_latch;
  textures->UnregisterTexture(texture_id, [&]() { unregister_latch.Signal(); });
  unregister_latch.Wait();

  texture = test_api->GetFakeTexture(texture_id);
  EXPECT_EQ(texture, nullptr);
  EXPECT_EQ(test_api->textures_size(), static_cast<size_t>(0));
}

// Tests that the unregister callback gets also invoked when attempting to
// unregister a texture with an unknown id.
TEST(TextureRegistrarTest, UnregisterInvalidTexture) {
  auto dummy_registrar_handle =
      reinterpret_cast<FlutterDesktopPluginRegistrarRef>(1);
  PluginRegistrar registrar(dummy_registrar_handle);

  TextureRegistrar* textures = registrar.texture_registrar();

  fml::AutoResetWaitableEvent latch;
  textures->UnregisterTexture(42, [&]() { latch.Signal(); });
  latch.Wait();
}

// Tests that claiming a new frame being available for an unknown texture
// returns false.
TEST(TextureRegistrarTest, MarkFrameAvailableInvalidTexture) {
  auto dummy_registrar_handle =
      reinterpret_cast<FlutterDesktopPluginRegistrarRef>(1);
  PluginRegistrar registrar(dummy_registrar_handle);

  TextureRegistrar* textures = registrar.texture_registrar();

  bool success = textures->MarkTextureFrameAvailable(42);
  EXPECT_FALSE(success);
}

}  // namespace flutter
