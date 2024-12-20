// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_FUCHSIA_DART_PKG_ZIRCON_SDK_EXT_SYSTEM_H_
#define FLUTTER_SHELL_PLATFORM_FUCHSIA_DART_PKG_ZIRCON_SDK_EXT_SYSTEM_H_

#include <zircon/syscalls.h>

#include "handle.h"
#include "handle_disposition.h"
#include "third_party/dart/runtime/include/dart_api.h"
#include "third_party/tonic/dart_library_natives.h"
#include "third_party/tonic/dart_wrappable.h"
#include "third_party/tonic/typed_data/dart_byte_data.h"

namespace zircon {
namespace dart {

class System : public fml::RefCountedThreadSafe<System>,
               public tonic::DartWrappable {
  DEFINE_WRAPPERTYPEINFO();
  FML_FRIEND_REF_COUNTED_THREAD_SAFE(System);
  FML_FRIEND_MAKE_REF_COUNTED(System);

 public:
  static Dart_Handle ChannelCreate(uint32_t options);
  static Dart_Handle ChannelFromFile(std::string path);
  static zx_status_t ChannelWrite(fml::RefPtr<Handle> channel,
                                  const tonic::DartByteData& data,
                                  std::vector<Handle*> handles);
  static zx_status_t ChannelWriteEtc(
      fml::RefPtr<Handle> channel,
      const tonic::DartByteData& data,
      std::vector<HandleDisposition*> handle_dispositions);
  // TODO(ianloic): Add ChannelRead
  static Dart_Handle ChannelQueryAndRead(fml::RefPtr<Handle> channel);
  static Dart_Handle ChannelQueryAndReadEtc(fml::RefPtr<Handle> channel);

  static Dart_Handle EventpairCreate(uint32_t options);

  static Dart_Handle SocketCreate(uint32_t options);
  static Dart_Handle SocketWrite(fml::RefPtr<Handle> socket,
                                 const tonic::DartByteData& data,
                                 int options);
  static Dart_Handle SocketRead(fml::RefPtr<Handle> socket, size_t size);

  static Dart_Handle VmoCreate(uint64_t size, uint32_t options);
  static Dart_Handle VmoFromFile(std::string path);
  static Dart_Handle VmoGetSize(fml::RefPtr<Handle> vmo);
  static zx_status_t VmoSetSize(fml::RefPtr<Handle> vmo, uint64_t size);
  static zx_status_t VmoWrite(fml::RefPtr<Handle> vmo,
                              uint64_t offset,
                              const tonic::DartByteData& data);
  static Dart_Handle VmoRead(fml::RefPtr<Handle> vmo,
                             uint64_t offset,
                             size_t size);

  static Dart_Handle VmoMap(fml::RefPtr<Handle> vmo);

  static uint64_t ClockGetMonotonic();

  static void RegisterNatives(tonic::DartLibraryNatives* natives);

  static zx_status_t ConnectToService(std::string path,
                                      fml::RefPtr<Handle> channel);

 private:
  static void VmoMapFinalizer(void* isolate_callback_data, void* peer);
};

}  // namespace dart
}  // namespace zircon

#endif  // FLUTTER_SHELL_PLATFORM_FUCHSIA_DART_PKG_ZIRCON_SDK_EXT_SYSTEM_H_
