// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "png.h"

#include "flutter/lib/ui/painting/image_encoding.h"
#include "flutter/lib/ui/painting/image_encoding_impl.h"

#include "flutter/common/task_runners.h"
#include "flutter/fml/synchronization/waitable_event.h"
#include "flutter/lib/ui/painting/image.h"
#include "flutter/runtime/dart_vm.h"
#include "flutter/shell/common/shell_test.h"
#include "flutter/shell/common/thread_host.h"
#include "flutter/testing/testing.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

#if IMPELLER_SUPPORTS_RENDERING
#include "flutter/lib/ui/painting/image_encoding_impeller.h"
#include "impeller/renderer/testing/mocks.h"
#endif  // IMPELLER_SUPPORTS_RENDERING

// CREATE_NATIVE_ENTRY is leaky by design
// NOLINTBEGIN(clang-analyzer-core.StackAddressEscape)

#pragma GCC diagnostic ignored "-Wunreachable-code"

namespace flutter {
namespace testing {

namespace {
fml::AutoResetWaitableEvent message_latch;

class MockDlImage : public DlImage {
 public:
  MOCK_METHOD(sk_sp<SkImage>, skia_image, (), (const, override));
  MOCK_METHOD(std::shared_ptr<impeller::Texture>,
              impeller_texture,
              (),
              (const, override));
  MOCK_METHOD(bool, isOpaque, (), (const, override));
  MOCK_METHOD(bool, isTextureBacked, (), (const, override));
  MOCK_METHOD(bool, isUIThreadSafe, (), (const, override));
  MOCK_METHOD(SkISize, dimensions, (), (const, override));
  MOCK_METHOD(DlISize, GetSize, (), (const, override));
  MOCK_METHOD(size_t, GetApproximateByteSize, (), (const, override));
};

}  // namespace

class MockSyncSwitch {
 public:
  struct Handlers {
    Handlers& SetIfTrue(const std::function<void()>& handler) {
      true_handler = handler;
      return *this;
    }
    Handlers& SetIfFalse(const std::function<void()>& handler) {
      false_handler = handler;
      return *this;
    }
    std::function<void()> true_handler = [] {};
    std::function<void()> false_handler = [] {};
  };

  MOCK_METHOD(void, Execute, (const Handlers& handlers), (const));
  MOCK_METHOD(void, SetSwitch, (bool value));
};

TEST_F(ShellTest, EncodeImageGivesExternalTypedData) {
  auto native_encode_image = [&](Dart_NativeArguments args) {
    auto image_handle = Dart_GetNativeArgument(args, 0);
    image_handle =
        Dart_GetField(image_handle, Dart_NewStringFromCString("_image"));
    ASSERT_FALSE(Dart_IsError(image_handle)) << Dart_GetError(image_handle);
    ASSERT_FALSE(Dart_IsNull(image_handle));
    auto format_handle = Dart_GetNativeArgument(args, 1);
    auto callback_handle = Dart_GetNativeArgument(args, 2);

    intptr_t peer = 0;
    Dart_Handle result = Dart_GetNativeInstanceField(
        image_handle, tonic::DartWrappable::kPeerIndex, &peer);
    ASSERT_FALSE(Dart_IsError(result));
    CanvasImage* canvas_image = reinterpret_cast<CanvasImage*>(peer);

    int64_t format = -1;
    result = Dart_IntegerToInt64(format_handle, &format);
    ASSERT_FALSE(Dart_IsError(result));

    result = EncodeImage(canvas_image, format, callback_handle);
    ASSERT_TRUE(Dart_IsNull(result));
  };

  auto nativeValidateExternal = [&](Dart_NativeArguments args) {
    auto handle = Dart_GetNativeArgument(args, 0);

    auto typed_data_type = Dart_GetTypeOfExternalTypedData(handle);
    EXPECT_EQ(typed_data_type, Dart_TypedData_kUint8);

    message_latch.Signal();
  };

  Settings settings = CreateSettingsForFixture();
  TaskRunners task_runners("test",                  // label
                           GetCurrentTaskRunner(),  // platform
                           CreateNewThread(),       // raster
                           CreateNewThread(),       // ui
                           CreateNewThread()        // io
  );

  AddNativeCallback("EncodeImage", CREATE_NATIVE_ENTRY(native_encode_image));
  AddNativeCallback("ValidateExternal",
                    CREATE_NATIVE_ENTRY(nativeValidateExternal));

  std::unique_ptr<Shell> shell = CreateShell(settings, task_runners);

  ASSERT_TRUE(shell->IsSetup());
  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("encodeImageProducesExternalUint8List");

  shell->RunEngine(std::move(configuration), [&](auto result) {
    ASSERT_EQ(result, Engine::RunStatus::Success);
  });

  message_latch.Wait();
  DestroyShell(std::move(shell), task_runners);
}

TEST_F(ShellTest, EncodeImageAccessesSyncSwitch) {
  Settings settings = CreateSettingsForFixture();
  TaskRunners task_runners("test",                  // label
                           GetCurrentTaskRunner(),  // platform
                           CreateNewThread(),       // raster
                           CreateNewThread(),       // ui
                           CreateNewThread()        // io
  );

  auto native_encode_image = [&](Dart_NativeArguments args) {
    auto image_handle = Dart_GetNativeArgument(args, 0);
    image_handle =
        Dart_GetField(image_handle, Dart_NewStringFromCString("_image"));
    ASSERT_FALSE(Dart_IsError(image_handle)) << Dart_GetError(image_handle);
    ASSERT_FALSE(Dart_IsNull(image_handle));
    auto format_handle = Dart_GetNativeArgument(args, 1);

    intptr_t peer = 0;
    Dart_Handle result = Dart_GetNativeInstanceField(
        image_handle, tonic::DartWrappable::kPeerIndex, &peer);
    ASSERT_FALSE(Dart_IsError(result));
    CanvasImage* canvas_image = reinterpret_cast<CanvasImage*>(peer);

    int64_t format = -1;
    result = Dart_IntegerToInt64(format_handle, &format);
    ASSERT_FALSE(Dart_IsError(result));

    auto io_manager = UIDartState::Current()->GetIOManager();
    fml::AutoResetWaitableEvent latch;

    task_runners.GetIOTaskRunner()->PostTask([&]() {
      auto is_gpu_disabled_sync_switch =
          std::make_shared<const MockSyncSwitch>();
      EXPECT_CALL(*is_gpu_disabled_sync_switch, Execute)
          .WillOnce([](const MockSyncSwitch::Handlers& handlers) {
            handlers.true_handler();
          });
      ConvertToRasterUsingResourceContext(canvas_image->image()->skia_image(),
                                          io_manager->GetResourceContext(),
                                          is_gpu_disabled_sync_switch);
      latch.Signal();
    });

    latch.Wait();

    message_latch.Signal();
  };

  AddNativeCallback("EncodeImage", CREATE_NATIVE_ENTRY(native_encode_image));

  std::unique_ptr<Shell> shell = CreateShell(settings, task_runners);

  ASSERT_TRUE(shell->IsSetup());
  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("encodeImageProducesExternalUint8List");

  shell->RunEngine(std::move(configuration), [&](auto result) {
    ASSERT_EQ(result, Engine::RunStatus::Success);
  });

  message_latch.Wait();
  DestroyShell(std::move(shell), task_runners);
}

#if IMPELLER_SUPPORTS_RENDERING
using ::impeller::testing::MockAllocator;
using ::impeller::testing::MockBlitPass;
using ::impeller::testing::MockCommandBuffer;
using ::impeller::testing::MockCommandQueue;
using ::impeller::testing::MockDeviceBuffer;
using ::impeller::testing::MockImpellerContext;
using ::impeller::testing::MockTexture;
using ::testing::_;
using ::testing::DoAll;
using ::testing::Invoke;
using ::testing::InvokeArgument;
using ::testing::Return;

namespace {
std::shared_ptr<impeller::Context> MakeConvertDlImageToSkImageContext(
    std::vector<uint8_t>& buffer) {
  auto context = std::make_shared<MockImpellerContext>();
  auto command_buffer = std::make_shared<MockCommandBuffer>(context);
  auto allocator = std::make_shared<MockAllocator>();
  auto blit_pass = std::make_shared<MockBlitPass>();
  auto command_queue = std::make_shared<MockCommandQueue>();
  impeller::DeviceBufferDescriptor device_buffer_desc;
  device_buffer_desc.size = buffer.size();
  auto device_buffer = std::make_shared<MockDeviceBuffer>(device_buffer_desc);
  EXPECT_CALL(*allocator, OnCreateBuffer).WillOnce(Return(device_buffer));
  EXPECT_CALL(*blit_pass, IsValid).WillRepeatedly(Return(true));
  EXPECT_CALL(*command_buffer, IsValid).WillRepeatedly(Return(true));
  EXPECT_CALL(*command_buffer, OnCreateBlitPass).WillOnce(Return(blit_pass));
  EXPECT_CALL(*context, GetResourceAllocator).WillRepeatedly(Return(allocator));
  EXPECT_CALL(*context, CreateCommandBuffer).WillOnce(Return(command_buffer));
  EXPECT_CALL(*device_buffer, OnGetContents).WillOnce(Return(buffer.data()));
  EXPECT_CALL(*command_queue, Submit(_, _))
      .WillRepeatedly(
          DoAll(InvokeArgument<1>(impeller::CommandBuffer::Status::kCompleted),
                Return(fml::Status())));
  EXPECT_CALL(*context, GetCommandQueue).WillRepeatedly(Return(command_queue));
  return context;
}
}  // namespace

TEST_F(ShellTest, EncodeImageRetries) {
#ifndef FML_OS_MACOSX
  GTEST_SKIP() << "Only works on macos currently.";
#endif
  Settings settings = CreateSettingsForFixture();
  settings.enable_impeller = true;
  TaskRunners task_runners("test",                  // label
                           GetCurrentTaskRunner(),  // platform
                           CreateNewThread(),       // raster
                           CreateNewThread(),       // ui
                           CreateNewThread()        // io
  );

  std::unique_ptr<Shell> shell = CreateShell({
      .settings = settings,
      .task_runners = task_runners,
  });

  auto turn_off_gpu = [&](Dart_NativeArguments args) {
    auto handle = Dart_GetNativeArgument(args, 0);
    bool value = true;
    ASSERT_TRUE(Dart_IsBoolean(handle));
    Dart_BooleanValue(handle, &value);
    TurnOffGPU(shell.get(), value);
  };

  AddNativeCallback("TurnOffGPU", CREATE_NATIVE_ENTRY(turn_off_gpu));

  auto validate_not_null = [&](Dart_NativeArguments args) {
    auto handle = Dart_GetNativeArgument(args, 0);
    EXPECT_FALSE(Dart_IsNull(handle));
    message_latch.Signal();
  };

  AddNativeCallback("ValidateNotNull", CREATE_NATIVE_ENTRY(validate_not_null));

  ASSERT_TRUE(shell->IsSetup());
  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("toByteDataRetries");

  shell->RunEngine(std::move(configuration), [&](auto result) {
    ASSERT_EQ(result, Engine::RunStatus::Success);
  });

  message_latch.Wait();
  DestroyShell(std::move(shell), task_runners);
}

TEST_F(ShellTest, EncodeImageRetryOverflows) {
#ifndef FML_OS_MACOSX
  GTEST_SKIP() << "Only works on macos currently.";
#endif
  Settings settings = CreateSettingsForFixture();
  settings.enable_impeller = true;
  TaskRunners task_runners("test",                  // label
                           GetCurrentTaskRunner(),  // platform
                           CreateNewThread(),       // raster
                           CreateNewThread(),       // ui
                           CreateNewThread()        // io
  );

  std::unique_ptr<Shell> shell = CreateShell({
      .settings = settings,
      .task_runners = task_runners,
  });

  auto turn_off_gpu = [&](Dart_NativeArguments args) {
    auto handle = Dart_GetNativeArgument(args, 0);
    bool value = true;
    ASSERT_TRUE(Dart_IsBoolean(handle));
    Dart_BooleanValue(handle, &value);
    TurnOffGPU(shell.get(), value);
  };

  AddNativeCallback("TurnOffGPU", CREATE_NATIVE_ENTRY(turn_off_gpu));

  auto validate_not_null = [&](Dart_NativeArguments args) {
    auto handle = Dart_GetNativeArgument(args, 0);
    EXPECT_FALSE(Dart_IsNull(handle));
    message_latch.Signal();
  };

  AddNativeCallback("ValidateNotNull", CREATE_NATIVE_ENTRY(validate_not_null));

  ASSERT_TRUE(shell->IsSetup());
  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("toByteDataRetryOverflows");

  shell->RunEngine(std::move(configuration), [&](auto result) {
    ASSERT_EQ(result, Engine::RunStatus::Success);
  });

  message_latch.Wait();
  DestroyShell(std::move(shell), task_runners);
}

TEST_F(ShellTest, ToImageRetries) {
#ifndef FML_OS_MACOSX
  GTEST_SKIP() << "Only works on macos currently.";
#endif
  Settings settings = CreateSettingsForFixture();
  settings.enable_impeller = true;
  TaskRunners task_runners("test",                  // label
                           GetCurrentTaskRunner(),  // platform
                           CreateNewThread(),       // raster
                           CreateNewThread(),       // ui
                           CreateNewThread()        // io
  );

  std::unique_ptr<Shell> shell = CreateShell({
      .settings = settings,
      .task_runners = task_runners,
  });

  auto turn_off_gpu = [&](Dart_NativeArguments args) {
    auto handle = Dart_GetNativeArgument(args, 0);
    bool value = true;
    ASSERT_TRUE(Dart_IsBoolean(handle));
    Dart_BooleanValue(handle, &value);
    TurnOffGPU(shell.get(), value);
  };

  AddNativeCallback("TurnOffGPU", CREATE_NATIVE_ENTRY(turn_off_gpu));

  auto validate_not_null = [&](Dart_NativeArguments args) {
    auto handle = Dart_GetNativeArgument(args, 0);
    EXPECT_FALSE(Dart_IsNull(handle));
    message_latch.Signal();
  };

  AddNativeCallback("ValidateNotNull", CREATE_NATIVE_ENTRY(validate_not_null));

  ASSERT_TRUE(shell->IsSetup());
  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("toImageRetries");

  shell->RunEngine(std::move(configuration), [&](auto result) {
    ASSERT_EQ(result, Engine::RunStatus::Success);
  });

  message_latch.Wait();
  DestroyShell(std::move(shell), task_runners);
}
TEST_F(ShellTest, ToImageRetryOverflow) {
#ifndef FML_OS_MACOSX
  GTEST_SKIP() << "Only works on macos currently.";
#endif
  Settings settings = CreateSettingsForFixture();
  settings.enable_impeller = true;
  TaskRunners task_runners("test",                  // label
                           GetCurrentTaskRunner(),  // platform
                           CreateNewThread(),       // raster
                           CreateNewThread(),       // ui
                           CreateNewThread()        // io
  );

  std::unique_ptr<Shell> shell = CreateShell({
      .settings = settings,
      .task_runners = task_runners,
  });

  auto turn_off_gpu = [&](Dart_NativeArguments args) {
    auto handle = Dart_GetNativeArgument(args, 0);
    bool value = true;
    ASSERT_TRUE(Dart_IsBoolean(handle));
    Dart_BooleanValue(handle, &value);
    TurnOffGPU(shell.get(), value);
  };

  AddNativeCallback("TurnOffGPU", CREATE_NATIVE_ENTRY(turn_off_gpu));

  auto validate_not_null = [&](Dart_NativeArguments args) {
    auto handle = Dart_GetNativeArgument(args, 0);
    EXPECT_FALSE(Dart_IsNull(handle));
    message_latch.Signal();
  };

  AddNativeCallback("ValidateNotNull", CREATE_NATIVE_ENTRY(validate_not_null));

  ASSERT_TRUE(shell->IsSetup());
  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("toImageRetryOverflows");

  shell->RunEngine(std::move(configuration), [&](auto result) {
    ASSERT_EQ(result, Engine::RunStatus::Success);
  });

  message_latch.Wait();
  DestroyShell(std::move(shell), task_runners);
}

TEST_F(ShellTest, EncodeImageFailsWithoutGPUImpeller) {
#ifndef FML_OS_MACOSX
  // Only works on macos currently.
  GTEST_SKIP();
#endif
  Settings settings = CreateSettingsForFixture();
  settings.enable_impeller = true;
  TaskRunners task_runners("test",                  // label
                           GetCurrentTaskRunner(),  // platform
                           CreateNewThread(),       // raster
                           CreateNewThread(),       // ui
                           CreateNewThread()        // io
  );

  auto native_validate_error = [&](Dart_NativeArguments args) {
    auto handle = Dart_GetNativeArgument(args, 0);

    EXPECT_FALSE(Dart_IsNull(handle));

    message_latch.Signal();
  };

  AddNativeCallback("ValidateError",
                    CREATE_NATIVE_ENTRY(native_validate_error));

  std::unique_ptr<Shell> shell = CreateShell({
      .settings = settings,
      .task_runners = task_runners,
  });

  auto turn_off_gpu = [&](Dart_NativeArguments args) {
    auto handle = Dart_GetNativeArgument(args, 0);
    bool value = true;
    ASSERT_TRUE(Dart_IsBoolean(handle));
    Dart_BooleanValue(handle, &value);
    TurnOffGPU(shell.get(), true);
  };

  AddNativeCallback("TurnOffGPU", CREATE_NATIVE_ENTRY(turn_off_gpu));

  auto flush_awaiting_tasks = [&](Dart_NativeArguments args) {
    task_runners.GetIOTaskRunner()->PostTask([&] {
      std::shared_ptr<impeller::Context> impeller_context =
          shell->GetIOManager()->GetImpellerContext();
      // This will cause the stored tasks to overflow and start throwing them
      // away.
      for (int i = 0; i < impeller::Context::kMaxTasksAwaitingGPU; i++) {
        impeller_context->StoreTaskForGPU([] {}, [] {});
      }
    });
  };

  AddNativeCallback("FlushGpuAwaitingTasks",
                    CREATE_NATIVE_ENTRY(flush_awaiting_tasks));

  ASSERT_TRUE(shell->IsSetup());
  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("toByteDataWithoutGPU");

  shell->RunEngine(std::move(configuration), [&](auto result) {
    ASSERT_EQ(result, Engine::RunStatus::Success);
  });

  message_latch.Wait();
  DestroyShell(std::move(shell), task_runners);
}

TEST(ImageEncodingImpellerTest, ConvertDlImageToSkImage16Float) {
  sk_sp<MockDlImage> image(new MockDlImage());
  EXPECT_CALL(*image, dimensions)
      .WillRepeatedly(Return(SkISize::Make(100, 100)));
  impeller::TextureDescriptor desc;
  desc.format = impeller::PixelFormat::kR16G16B16A16Float;
  auto texture = std::make_shared<MockTexture>(desc);
  EXPECT_CALL(*image, impeller_texture).WillOnce(Return(texture));
  std::vector<uint8_t> buffer;
  buffer.reserve(100 * 100 * 8);
  auto context = MakeConvertDlImageToSkImageContext(buffer);
  bool did_call = false;
  ImageEncodingImpeller::ConvertDlImageToSkImage(
      image,
      [&did_call](const fml::StatusOr<sk_sp<SkImage>>& image) {
        did_call = true;
        ASSERT_TRUE(image.ok());
        ASSERT_TRUE(image.value());
        EXPECT_EQ(100, image.value()->width());
        EXPECT_EQ(100, image.value()->height());
        EXPECT_EQ(kRGBA_F16_SkColorType, image.value()->colorType());
        EXPECT_EQ(nullptr, image.value()->colorSpace());
      },
      context);
  EXPECT_TRUE(did_call);
}

TEST(ImageEncodingImpellerTest, ConvertDlImageToSkImage10XR) {
  sk_sp<MockDlImage> image(new MockDlImage());
  EXPECT_CALL(*image, dimensions)
      .WillRepeatedly(Return(SkISize::Make(100, 100)));
  impeller::TextureDescriptor desc;
  desc.format = impeller::PixelFormat::kB10G10R10XR;
  auto texture = std::make_shared<MockTexture>(desc);
  EXPECT_CALL(*image, impeller_texture).WillOnce(Return(texture));
  std::vector<uint8_t> buffer;
  buffer.reserve(100 * 100 * 4);
  auto context = MakeConvertDlImageToSkImageContext(buffer);
  bool did_call = false;
  ImageEncodingImpeller::ConvertDlImageToSkImage(
      image,
      [&did_call](const fml::StatusOr<sk_sp<SkImage>>& image) {
        did_call = true;
        ASSERT_TRUE(image.ok());
        ASSERT_TRUE(image.value());
        EXPECT_EQ(100, image.value()->width());
        EXPECT_EQ(100, image.value()->height());
        EXPECT_EQ(kBGR_101010x_XR_SkColorType, image.value()->colorType());
        EXPECT_EQ(nullptr, image.value()->colorSpace());
      },
      context);
  EXPECT_TRUE(did_call);
}

TEST(ImageEncodingImpellerTest, PngEncoding10XR) {
  int width = 100;
  int height = 100;
  SkImageInfo info = SkImageInfo::Make(
      width, height, kBGR_101010x_XR_SkColorType, kUnpremul_SkAlphaType);

  auto surface = SkSurfaces::Raster(info);
  SkCanvas* canvas = surface->getCanvas();

  SkPaint paint;
  paint.setColor(SK_ColorBLUE);
  paint.setAntiAlias(true);

  canvas->clear(SK_ColorWHITE);
  canvas->drawCircle(width / 2, height / 2, 100, paint);

  sk_sp<SkImage> image = surface->makeImageSnapshot();

  fml::StatusOr<sk_sp<SkData>> png = EncodeImage(image, ImageByteFormat::kPNG);
  EXPECT_TRUE(png.ok());
}

namespace {
struct PngMemoryReader {
  const uint8_t* data;
  size_t offset;
  size_t size;
};

void PngMemoryRead(png_structp png_ptr,
                   png_bytep out_bytes,
                   png_size_t byte_count_to_read) {
  PngMemoryReader* memory_reader =
      reinterpret_cast<PngMemoryReader*>(png_get_io_ptr(png_ptr));
  if (memory_reader->offset + byte_count_to_read > memory_reader->size) {
    png_error(png_ptr, "Read error in PngMemoryRead");
  }
  memcpy(out_bytes, memory_reader->data + memory_reader->offset,
         byte_count_to_read);
  memory_reader->offset += byte_count_to_read;
}

fml::StatusOr<std::vector<uint32_t>> ReadPngFromMemory(const uint8_t* png_data,
                                                       size_t png_size) {
  png_structp png =
      png_create_read_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
  if (!png) {
    return fml::Status(fml::StatusCode::kAborted, "unknown");
  }

  png_infop info = png_create_info_struct(png);
  if (!info) {
    png_destroy_read_struct(&png, nullptr, nullptr);
    return fml::Status(fml::StatusCode::kAborted, "unknown");
  }

  fml::ScopedCleanupClosure png_cleanup(
      [&png, &info]() { png_destroy_read_struct(&png, &info, nullptr); });

  if (setjmp(png_jmpbuf(png))) {
    return fml::Status(fml::StatusCode::kAborted, "unknown");
  }

  PngMemoryReader memory_reader = {
      .data = png_data, .offset = 0, .size = png_size};
  png_set_read_fn(png, &memory_reader, PngMemoryRead);

  png_read_info(png, info);

  int width = png_get_image_width(png, info);
  int height = png_get_image_height(png, info);
  png_byte color_type = png_get_color_type(png, info);
  png_byte bit_depth = png_get_bit_depth(png, info);

  if (bit_depth == 16) {
    png_set_strip_16(png);
  }
  if (color_type == PNG_COLOR_TYPE_PALETTE) {
    png_set_palette_to_rgb(png);
  }

  png_read_update_info(png, info);
  std::vector<uint32_t> result(width * height);
  std::vector<png_bytep> row_pointers;
  row_pointers.reserve(height);

  for (int i = 0; i < height; ++i) {
    row_pointers.push_back(
        reinterpret_cast<png_bytep>(result.data() + i * width));
  }

  png_read_image(png, row_pointers.data());

  return result;
}
}  // namespace

TEST(ImageEncodingImpellerTest, PngEncodingBGRA10XR) {
  int width = 100;
  int height = 100;
  SkImageInfo info = SkImageInfo::Make(
      width, height, kBGRA_10101010_XR_SkColorType, kUnpremul_SkAlphaType);

  auto surface = SkSurfaces::Raster(info);
  SkCanvas* canvas = surface->getCanvas();

  SkPaint paint;
  paint.setColor(SK_ColorBLUE);
  paint.setAntiAlias(true);

  canvas->clear(SK_ColorRED);
  canvas->drawCircle(width / 2, height / 2, 25, paint);

  sk_sp<SkImage> image = surface->makeImageSnapshot();

  fml::StatusOr<sk_sp<SkData>> png = EncodeImage(image, ImageByteFormat::kPNG);
  ASSERT_TRUE(png.ok());
  fml::StatusOr<std::vector<uint32_t>> pixels =
      ReadPngFromMemory(png.value()->bytes(), png.value()->size());
  ASSERT_TRUE(pixels.ok());
  EXPECT_EQ(pixels.value()[0], 0xff0000ff);
  int middle = 100 * 50 + 50;
  EXPECT_EQ(pixels.value()[middle], 0xffff0000);
}

#endif  // IMPELLER_SUPPORTS_RENDERING

}  // namespace testing
}  // namespace flutter

// NOLINTEND(clang-analyzer-core.StackAddressEscape)
