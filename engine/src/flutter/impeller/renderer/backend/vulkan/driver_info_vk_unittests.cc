// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/playground/playground_test.h"
#include "impeller/renderer/backend/vulkan/context_vk.h"
#include "impeller/renderer/backend/vulkan/driver_info_vk.h"
#include "impeller/renderer/backend/vulkan/surface_context_vk.h"
#include "impeller/renderer/backend/vulkan/test/mock_vulkan.h"

namespace impeller::testing {

using DriverInfoVKTest = PlaygroundTest;
INSTANTIATE_VULKAN_PLAYGROUND_SUITE(DriverInfoVKTest);

TEST_P(DriverInfoVKTest, CanQueryDriverInfo) {
  ASSERT_TRUE(GetContext());
  const auto& driver_info =
      SurfaceContextVK::Cast(*GetContext()).GetParent()->GetDriverInfo();
  ASSERT_NE(driver_info, nullptr);
  // 1.1 is the base Impeller version. The driver can't be lower than that.
  ASSERT_TRUE(driver_info->GetAPIVersion().IsAtLeast(Version{1, 1, 0}));
  ASSERT_NE(driver_info->GetVendor(), VendorVK::kUnknown);
  ASSERT_NE(driver_info->GetDeviceType(), DeviceTypeVK::kUnknown);
  ASSERT_NE(driver_info->GetDriverName(), "");
  EXPECT_FALSE(driver_info->IsKnownBadDriver());
}

TEST_P(DriverInfoVKTest, CanDumpToLog) {
  ASSERT_TRUE(GetContext());
  const auto& driver_info =
      SurfaceContextVK::Cast(*GetContext()).GetParent()->GetDriverInfo();
  ASSERT_NE(driver_info, nullptr);
  fml::testing::LogCapture log;
  driver_info->DumpToLog();
  EXPECT_TRUE(log.str().find("Driver Information") != std::string::npos);
}

TEST(DriverInfoVKTest, CanIdentifyBadMaleoonDriver) {
  auto const context =
      MockVulkanContextBuilder()
          .SetPhysicalPropertiesCallback(
              [](VkPhysicalDevice device, VkPhysicalDeviceProperties* prop) {
                prop->vendorID = 0x19E5;  // Huawei
                prop->deviceType = VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU;
              })
          .Build();

  EXPECT_TRUE(context->GetDriverInfo()->IsKnownBadDriver());
}

bool IsBadVersionTest(std::string_view driver_name, bool qc = true) {
  auto const context =
      MockVulkanContextBuilder()
          .SetPhysicalPropertiesCallback(
              [&driver_name, qc](VkPhysicalDevice device,
                                 VkPhysicalDeviceProperties* prop) {
                if (qc) {
                  prop->vendorID = 0x168C;  // Qualcomm
                } else {
                  prop->vendorID = 0x13B5;  // ARM
                }
                driver_name.copy(prop->deviceName, driver_name.size());
                prop->deviceType = VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU;
              })
          .Build();
  return context->GetDriverInfo()->IsKnownBadDriver();
}

bool CanBatchSubmitTest(std::string_view driver_name, bool qc = true) {
  auto const context =
      MockVulkanContextBuilder()
          .SetPhysicalPropertiesCallback(
              [&driver_name, qc](VkPhysicalDevice device,
                                 VkPhysicalDeviceProperties* prop) {
                if (qc) {
                  prop->vendorID = 0x168C;  // Qualcomm
                } else {
                  prop->vendorID = 0x13B5;  // ARM
                }
                driver_name.copy(prop->deviceName, driver_name.size());
                prop->deviceType = VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU;
              })
          .Build();
  return context->GetDriverInfo()->CanBatchSubmitCommandBuffers();
}

TEST(DriverInfoVKTest, CanBatchSubmitCommandBuffers) {
  // Old Adreno no batch submit!
  EXPECT_FALSE(CanBatchSubmitTest("Adreno (TM) 540", true));

  EXPECT_TRUE(CanBatchSubmitTest("Mali-G51", false));
  EXPECT_TRUE(CanBatchSubmitTest("Adreno (TM) 750", true));
}

TEST(DriverInfoVKTest, DriverParsingMali) {
  EXPECT_EQ(GetMaliVersion("Mali-G51-MORE STUFF"), MaliGPU::kG51);
  EXPECT_EQ(GetMaliVersion("Mali-G51"), MaliGPU::kG51);
  EXPECT_EQ(GetMaliVersion("Mali-111111"), MaliGPU::kUnknown);
}

TEST(DriverInfoVKTest, DriverParsingAdreno) {
  EXPECT_EQ(GetAdrenoVersion("Adreno (TM) 540"), AdrenoGPU::kAdreno540);
  EXPECT_EQ(GetAdrenoVersion("Foo Bar"), AdrenoGPU::kUnknown);
}

TEST(DriverInfoVKTest, DisabledDevices) {
  EXPECT_TRUE(IsBadVersionTest("Adreno (TM) 630"));
  EXPECT_TRUE(IsBadVersionTest("Adreno (TM) 620"));
  EXPECT_TRUE(IsBadVersionTest("Adreno (TM) 610"));
  EXPECT_TRUE(IsBadVersionTest("Adreno (TM) 530"));
  EXPECT_TRUE(IsBadVersionTest("Adreno (TM) 512"));
  EXPECT_TRUE(IsBadVersionTest("Adreno (TM) 509"));
  EXPECT_TRUE(IsBadVersionTest("Adreno (TM) 508"));
  EXPECT_TRUE(IsBadVersionTest("Adreno (TM) 506"));
  EXPECT_TRUE(IsBadVersionTest("Adreno (TM) 505"));
  EXPECT_TRUE(IsBadVersionTest("Adreno (TM) 504"));

  EXPECT_FALSE(IsBadVersionTest("Adreno (TM) 640"));
  EXPECT_FALSE(IsBadVersionTest("Adreno (TM) 650"));
}

TEST(DriverInfoVKTest, EnabledDevicesMali) {
  EXPECT_FALSE(IsBadVersionTest("Mali-G52", /*qc=*/false));
  EXPECT_FALSE(IsBadVersionTest("Mali-G54-MORE STUFF", /*qc=*/false));
}

TEST(DriverInfoVKTest, EnabledDevicesAdreno) {
  EXPECT_FALSE(IsBadVersionTest("Adreno (TM) 750"));
  EXPECT_FALSE(IsBadVersionTest("Adreno (TM) 740"));
  EXPECT_FALSE(IsBadVersionTest("Adreno (TM) 732"));
  EXPECT_FALSE(IsBadVersionTest("Adreno (TM) 730"));
  EXPECT_FALSE(IsBadVersionTest("Adreno (TM) 725"));
  EXPECT_FALSE(IsBadVersionTest("Adreno (TM) 720"));
  EXPECT_FALSE(IsBadVersionTest("Adreno (TM) 710"));
  EXPECT_FALSE(IsBadVersionTest("Adreno (TM) 702"));
}

}  // namespace impeller::testing
