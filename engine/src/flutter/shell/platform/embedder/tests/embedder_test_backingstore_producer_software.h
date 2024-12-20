// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_EMBEDDER_TESTS_EMBEDDER_TEST_BACKINGSTORE_PRODUCER_SOFTWARE_H_
#define FLUTTER_SHELL_PLATFORM_EMBEDDER_TESTS_EMBEDDER_TEST_BACKINGSTORE_PRODUCER_SOFTWARE_H_

#include "flutter/shell/platform/embedder/tests/embedder_test_backingstore_producer.h"

namespace flutter::testing {

class EmbedderTestBackingStoreProducerSoftware
    : public EmbedderTestBackingStoreProducer {
 public:
  EmbedderTestBackingStoreProducerSoftware(
      sk_sp<GrDirectContext> context,
      RenderTargetType type,
      FlutterSoftwarePixelFormat software_pixfmt =
          kFlutterSoftwarePixelFormatNative32);

  virtual ~EmbedderTestBackingStoreProducerSoftware();

  bool Create(const FlutterBackingStoreConfig* config,
              FlutterBackingStore* backing_store_out) override;

  sk_sp<SkSurface> GetSurface(
      const FlutterBackingStore* backing_store) const override;

  sk_sp<SkImage> MakeImageSnapshot(
      const FlutterBackingStore* backing_store) const override;

 private:
  bool CreateSoftware(const FlutterBackingStoreConfig* config,
                      FlutterBackingStore* backing_store_out);

  bool CreateSoftware2(const FlutterBackingStoreConfig* config,
                       FlutterBackingStore* backing_store_out);

  FlutterSoftwarePixelFormat software_pixfmt_;

  FML_DISALLOW_COPY_AND_ASSIGN(EmbedderTestBackingStoreProducerSoftware);
};

}  // namespace flutter::testing

#endif  // FLUTTER_SHELL_PLATFORM_EMBEDDER_TESTS_EMBEDDER_TEST_BACKINGSTORE_PRODUCER_SOFTWARE_H_
