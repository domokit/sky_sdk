// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/common/graphics/texture.h"

#include <functional>

#include "flutter/flow/testing/mock_texture.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace flutter {
namespace testing {

namespace {
using int_closure = std::function<void(int)>;

struct TestContextListener : public ContextListener {
  TestContextListener(uintptr_t p_id,
                      int_closure p_create,
                      int_closure p_destroy)
      : id(p_id), create(std::move(p_create)), destroy(std::move(p_destroy)) {}

  virtual ~TestContextListener() = default;

  const uintptr_t id;
  int_closure create;
  int_closure destroy;

  void OnGrContextCreated() override { create(id); }

  void OnGrContextDestroyed() override { destroy(id); }
};
}  // namespace

TEST(TextureRegistryTest, UnregisterTextureCallbackTriggered) {
  TextureRegistry registry;
  auto mock_texture1 = std::make_shared<MockTexture>(0);
  auto mock_texture2 = std::make_shared<MockTexture>(1);

  registry.RegisterTexture(mock_texture1);
  registry.RegisterTexture(mock_texture2);
  ASSERT_EQ(registry.GetTexture(0), mock_texture1);
  ASSERT_EQ(registry.GetTexture(1), mock_texture2);
  ASSERT_FALSE(mock_texture1->unregistered());
  ASSERT_FALSE(mock_texture2->unregistered());

  registry.UnregisterTexture(0);
  ASSERT_EQ(registry.GetTexture(0), nullptr);
  ASSERT_TRUE(mock_texture1->unregistered());
  ASSERT_FALSE(mock_texture2->unregistered());

  registry.UnregisterTexture(1);
  ASSERT_EQ(registry.GetTexture(1), nullptr);
  ASSERT_TRUE(mock_texture1->unregistered());
  ASSERT_TRUE(mock_texture2->unregistered());
}

TEST(TextureRegistryTest, GrContextCallbackTriggered) {
  TextureRegistry registry;
  auto mock_texture1 = std::make_shared<MockTexture>(0);
  auto mock_texture2 = std::make_shared<MockTexture>(1);

  registry.RegisterTexture(mock_texture1);
  registry.RegisterTexture(mock_texture2);
  ASSERT_FALSE(mock_texture1->gr_context_created());
  ASSERT_FALSE(mock_texture2->gr_context_created());
  ASSERT_FALSE(mock_texture1->gr_context_destroyed());
  ASSERT_FALSE(mock_texture2->gr_context_destroyed());

  registry.OnGrContextCreated();
  ASSERT_TRUE(mock_texture1->gr_context_created());
  ASSERT_TRUE(mock_texture2->gr_context_created());

  registry.UnregisterTexture(0);
  registry.OnGrContextDestroyed();
  ASSERT_FALSE(mock_texture1->gr_context_destroyed());
  ASSERT_TRUE(mock_texture2->gr_context_created());
}

TEST(TextureRegistryTest, RegisterTextureTwice) {
  TextureRegistry registry;
  auto mock_texture1 = std::make_shared<MockTexture>(0);
  auto mock_texture2 = std::make_shared<MockTexture>(0);

  registry.RegisterTexture(mock_texture1);
  ASSERT_EQ(registry.GetTexture(0), mock_texture1);
  registry.RegisterTexture(mock_texture2);
  ASSERT_EQ(registry.GetTexture(0), mock_texture2);
  ASSERT_FALSE(mock_texture1->unregistered());
  ASSERT_FALSE(mock_texture2->unregistered());

  registry.UnregisterTexture(0);
  ASSERT_EQ(registry.GetTexture(0), nullptr);
  ASSERT_FALSE(mock_texture1->unregistered());
  ASSERT_TRUE(mock_texture2->unregistered());
}

TEST(TextureRegistryTest, ReuseSameTextureSlot) {
  TextureRegistry registry;
  auto mock_texture1 = std::make_shared<MockTexture>(0);
  auto mock_texture2 = std::make_shared<MockTexture>(0);

  registry.RegisterTexture(mock_texture1);
  ASSERT_EQ(registry.GetTexture(0), mock_texture1);

  registry.UnregisterTexture(0);
  ASSERT_EQ(registry.GetTexture(0), nullptr);
  ASSERT_TRUE(mock_texture1->unregistered());
  ASSERT_FALSE(mock_texture2->unregistered());

  registry.RegisterTexture(mock_texture2);
  ASSERT_EQ(registry.GetTexture(0), mock_texture2);

  registry.UnregisterTexture(0);
  ASSERT_EQ(registry.GetTexture(0), nullptr);
  ASSERT_TRUE(mock_texture1->unregistered());
  ASSERT_TRUE(mock_texture2->unregistered());
}

TEST(TextureRegistryTest, CallsOnGrContextCreatedInInsertionOrder) {
  TextureRegistry registry;
  std::vector<int> create_order;
  std::vector<int> destroy_order;
  auto create = [&](int id) { create_order.push_back(id); };
  auto destroy = [&](int id) { destroy_order.push_back(id); };
  auto a = std::make_shared<TestContextListener>(5, create, destroy);
  auto b = std::make_shared<TestContextListener>(4, create, destroy);
  auto c = std::make_shared<TestContextListener>(3, create, destroy);
  registry.RegisterContextListener(a->id, a);
  registry.RegisterContextListener(b->id, b);
  registry.RegisterContextListener(c->id, c);
  registry.OnGrContextDestroyed();
  registry.OnGrContextCreated();

  EXPECT_THAT(create_order, ::testing::ElementsAre(5, 4, 3));
  EXPECT_THAT(destroy_order, ::testing::ElementsAre(5, 4, 3));
}

}  // namespace testing
}  // namespace flutter
