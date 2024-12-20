// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_PLAYGROUND_COMPUTE_PLAYGROUND_TEST_H_
#define FLUTTER_IMPELLER_PLAYGROUND_COMPUTE_PLAYGROUND_TEST_H_

#include <memory>

#include "flutter/fml/time/time_delta.h"
#include "flutter/testing/testing.h"
#include "impeller/core/device_buffer.h"
#include "impeller/playground/playground.h"

namespace impeller {

class ComputePlaygroundTest
    : public Playground,
      public ::testing::TestWithParam<PlaygroundBackend> {
 public:
  ComputePlaygroundTest();

  virtual ~ComputePlaygroundTest();

  void SetUp() override;

  void TearDown() override;

  // |Playground|
  std::unique_ptr<fml::Mapping> OpenAssetAsMapping(
      std::string asset_name) const override;

  // |Playground|
  std::string GetWindowTitle() const override;

  template <typename T>
  std::shared_ptr<DeviceBuffer> CreateHostVisibleDeviceBuffer(
      const std::shared_ptr<Context>& context,
      const std::string& label) {
    DeviceBufferDescriptor desc;
    desc.storage_mode = StorageMode::kHostVisible;
    desc.size = sizeof(T);
    auto buffer = context->GetResourceAllocator()->CreateBuffer(desc);
    buffer->SetLabel(label);
    return buffer;
  }

 private:
  fml::TimeDelta start_time_;

  ComputePlaygroundTest(const ComputePlaygroundTest&) = delete;

  ComputePlaygroundTest& operator=(const ComputePlaygroundTest&) = delete;
};

#define INSTANTIATE_COMPUTE_SUITE(playground)                              \
  INSTANTIATE_TEST_SUITE_P(                                                \
      Compute, playground,                                                 \
      ::testing::Values(PlaygroundBackend::kMetal,                         \
                        PlaygroundBackend::kVulkan),                       \
      [](const ::testing::TestParamInfo<ComputePlaygroundTest::ParamType>& \
             info) { return PlaygroundBackendToString(info.param); });

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_PLAYGROUND_COMPUTE_PLAYGROUND_TEST_H_
