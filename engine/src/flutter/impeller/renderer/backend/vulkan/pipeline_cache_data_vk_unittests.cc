// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/fml/build_config.h"
#include "flutter/fml/file.h"
#include "flutter/testing/testing.h"
#include "impeller/playground/playground_test.h"
#include "impeller/renderer/backend/vulkan/capabilities_vk.h"
#include "impeller/renderer/backend/vulkan/context_vk.h"
#include "impeller/renderer/backend/vulkan/pipeline_cache_data_vk.h"
#include "impeller/renderer/backend/vulkan/surface_context_vk.h"

namespace impeller::testing {

TEST(PipelineCacheDataVKTest, CanTestHeaderCompatibility) {
  {
    PipelineCacheHeaderVK a;
    PipelineCacheHeaderVK b;
    EXPECT_EQ(a.abi, sizeof(void*));
#ifdef FML_ARCH_CPU_64_BITS
    EXPECT_EQ(a.abi, 8u);
#elif FML_ARCH_CPU_32_BITS
    EXPECT_EQ(a.abi, 4u);
#endif
    EXPECT_TRUE(a.IsCompatibleWith(b));
  }
  // Different data sizes don't matter.
  {
    PipelineCacheHeaderVK a;
    PipelineCacheHeaderVK b;
    a.data_size = b.data_size + 100u;
    EXPECT_TRUE(a.IsCompatibleWith(b));
  }
  // Magic, Driver, vendor, ABI, and UUID matter.
  {
    PipelineCacheHeaderVK a;
    PipelineCacheHeaderVK b;
    b.magic = 100;
    EXPECT_FALSE(a.IsCompatibleWith(b));
  }
  {
    PipelineCacheHeaderVK a;
    PipelineCacheHeaderVK b;
    b.driver_version = 100;
    EXPECT_FALSE(a.IsCompatibleWith(b));
  }
  {
    PipelineCacheHeaderVK a;
    PipelineCacheHeaderVK b;
    b.vendor_id = 100;
    EXPECT_FALSE(a.IsCompatibleWith(b));
  }
  {
    PipelineCacheHeaderVK a;
    PipelineCacheHeaderVK b;
    b.device_id = 100;
    EXPECT_FALSE(a.IsCompatibleWith(b));
  }
  {
    PipelineCacheHeaderVK a;
    PipelineCacheHeaderVK b;
    b.abi = a.abi / 2u;
    EXPECT_FALSE(a.IsCompatibleWith(b));
  }
  {
    PipelineCacheHeaderVK a;
    PipelineCacheHeaderVK b;
    for (size_t i = 0; i < VK_UUID_SIZE; i++) {
      b.uuid[i] = a.uuid[i] + 1;
    }
    EXPECT_FALSE(a.IsCompatibleWith(b));
  }
}

TEST(PipelineCacheDataVKTest, CanCreateFromDeviceProperties) {
  vk::PhysicalDeviceProperties props;
  std::array<uint8_t, 16> uuid{
      0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
  };
  props.pipelineCacheUUID = uuid;
  props.deviceID = 10;
  props.vendorID = 11;
  props.driverVersion = 12;
  PipelineCacheHeaderVK header(props, 99);
  EXPECT_EQ(uuid.size(), std::size(header.uuid));
  EXPECT_EQ(props.deviceID, header.device_id);
  EXPECT_EQ(props.vendorID, header.vendor_id);
  EXPECT_EQ(props.driverVersion, header.driver_version);
  for (size_t i = 0; i < uuid.size(); i++) {
    EXPECT_EQ(header.uuid[i], uuid.at(i));
  }
}

using PipelineCacheDataVKPlaygroundTest = PlaygroundTest;
INSTANTIATE_VULKAN_PLAYGROUND_SUITE(PipelineCacheDataVKPlaygroundTest);

TEST_P(PipelineCacheDataVKPlaygroundTest, CanPersistAndRetrievePipelineCache) {
  fml::ScopedTemporaryDirectory temp_dir;
  const auto& surface_context = SurfaceContextVK::Cast(*GetContext());
  const auto& context_vk = ContextVK::Cast(*surface_context.GetParent());
  const auto& caps = CapabilitiesVK::Cast(*context_vk.GetCapabilities());

  {
    auto cache = context_vk.GetDevice().createPipelineCacheUnique({});
    ASSERT_EQ(cache.result, vk::Result::eSuccess);
    ASSERT_FALSE(fml::FileExists(temp_dir.fd(), "flutter.impeller.vkcache"));
    ASSERT_TRUE(PipelineCacheDataPersist(
        temp_dir.fd(), caps.GetPhysicalDeviceProperties(), cache.value));
  }
  ASSERT_TRUE(fml::FileExists(temp_dir.fd(), "flutter.impeller.vkcache"));

  auto mapping = PipelineCacheDataRetrieve(temp_dir.fd(),
                                           caps.GetPhysicalDeviceProperties());
  ASSERT_NE(mapping, nullptr);
  // Assert that the utility has stripped away the cache header giving us clean
  // pipeline cache bootstrap information.
  vk::PipelineCacheHeaderVersionOne vk_cache_header;
  ASSERT_GE(mapping->GetSize(), sizeof(vk_cache_header));
  std::memcpy(&vk_cache_header, mapping->GetMapping(), sizeof(vk_cache_header));
  ASSERT_EQ(vk_cache_header.headerVersion,
            vk::PipelineCacheHeaderVersion::eOne);
}

TEST_P(PipelineCacheDataVKPlaygroundTest,
       IntegrityChecksArePerformedOnPersistedData) {
  fml::ScopedTemporaryDirectory temp_dir;
  const auto& surface_context = SurfaceContextVK::Cast(*GetContext());
  const auto& context_vk = ContextVK::Cast(*surface_context.GetParent());
  const auto& caps = CapabilitiesVK::Cast(*context_vk.GetCapabilities());

  {
    auto cache = context_vk.GetDevice().createPipelineCacheUnique({});
    ASSERT_EQ(cache.result, vk::Result::eSuccess);
    ASSERT_FALSE(fml::FileExists(temp_dir.fd(), "flutter.impeller.vkcache"));
    ASSERT_TRUE(PipelineCacheDataPersist(
        temp_dir.fd(), caps.GetPhysicalDeviceProperties(), cache.value));
  }
  ASSERT_TRUE(fml::FileExists(temp_dir.fd(), "flutter.impeller.vkcache"));
  auto incompatible_caps = caps.GetPhysicalDeviceProperties();
  // Simulate a driver version bump.
  incompatible_caps.driverVersion =
      caps.GetPhysicalDeviceProperties().driverVersion + 1u;
  auto mapping = PipelineCacheDataRetrieve(temp_dir.fd(), incompatible_caps);
  ASSERT_EQ(mapping, nullptr);
}

}  // namespace impeller::testing
