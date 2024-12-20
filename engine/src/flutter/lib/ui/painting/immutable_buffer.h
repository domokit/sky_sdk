// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_UI_PAINTING_IMMUTABLE_BUFFER_H_
#define FLUTTER_LIB_UI_PAINTING_IMMUTABLE_BUFFER_H_

#include <cstdint>

#include "flutter/fml/macros.h"
#include "flutter/lib/ui/dart_wrapper.h"
#include "third_party/skia/include/core/SkData.h"
#include "third_party/tonic/dart_library_natives.h"
#include "third_party/tonic/logging/dart_invoke.h"
#include "third_party/tonic/typed_data/typed_list.h"

namespace flutter {

//------------------------------------------------------------------------------
/// A simple opaque handle to an immutable byte buffer suitable for use
/// internally by the engine.
///
/// This data is not known by the Dart VM.
///
/// It is expected that C++ users of this object will not modify the data
/// argument. No Dart side calls are provided to do so.
class ImmutableBuffer : public RefCountedDartWrappable<ImmutableBuffer> {
 public:
  ~ImmutableBuffer() override;

  /// Initializes a new ImmutableData from a Dart Uint8List.
  ///
  /// `buffer_handle` is the caller that will be registered as the Dart peer of
  /// the native ImmutableBuffer object.
  ///
  /// `data` is a tonic::Uint8List of bytes to copy.
  ///
  /// `callback_handle` is expected to be a void callback to signal when the
  /// copy has completed.
  static Dart_Handle init(Dart_Handle buffer_handle,
                          Dart_Handle data,
                          Dart_Handle callback_handle);

  /// Initializes a new ImmutableData from an asset matching a provided
  /// asset string.
  ///
  /// The zero indexed argument is the caller that will be registered as the
  /// Dart peer of the native ImmutableBuffer object.
  ///
  /// The first indexed argumented is a String corresponding to the asset
  /// to load.
  ///
  /// The second indexed argument is expected to be a void callback to signal
  /// when the copy has completed.
  static Dart_Handle initFromAsset(Dart_Handle buffer_handle,
                                   Dart_Handle asset_name_handle,
                                   Dart_Handle callback_handle);

  /// Initializes a new ImmutableData from an File path.
  ///
  /// The zero indexed argument is the caller that will be registered as the
  /// Dart peer of the native ImmutableBuffer object.
  ///
  /// The first indexed argumented is a String corresponding to the file path
  /// to load.
  ///
  /// The second indexed argument is expected to be a void callback to signal
  /// when the copy has completed.
  static Dart_Handle initFromFile(Dart_Handle buffer_handle,
                                  Dart_Handle file_path_handle,
                                  Dart_Handle callback_handle);

  /// The length of the data in bytes.
  size_t length() const {
    FML_DCHECK(data_);
    return data_->size();
  }

  /// Callers should not modify the returned data. This is not exposed to Dart.
  sk_sp<SkData> data() const { return data_; }

  /// Clears the Dart native fields and removes the reference to the underlying
  /// byte buffer.
  ///
  /// The byte buffer will continue to live if other objects hold a reference to
  /// it.
  void dispose() {
    data_.reset();
    ClearDartWrapper();
  }

 private:
  explicit ImmutableBuffer(sk_sp<SkData> data) : data_(std::move(data)) {}

  sk_sp<SkData> data_;

  static sk_sp<SkData> MakeSkDataWithCopy(const void* data, size_t length);

  DEFINE_WRAPPERTYPEINFO();
  FML_FRIEND_MAKE_REF_COUNTED(ImmutableBuffer);
  FML_DISALLOW_COPY_AND_ASSIGN(ImmutableBuffer);
};

}  // namespace flutter

#endif  // FLUTTER_LIB_UI_PAINTING_IMMUTABLE_BUFFER_H_
