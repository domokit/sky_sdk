// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_TOOLS_FONT_SUBSET_HB_WRAPPERS_H_
#define FLUTTER_TOOLS_FONT_SUBSET_HB_WRAPPERS_H_

#include <hb-subset.h>

#include <memory>

namespace HarfbuzzWrappers {
struct hb_blob_deleter {
  void operator()(hb_blob_t* ptr) { hb_blob_destroy(ptr); }
};

struct hb_face_deleter {
  void operator()(hb_face_t* ptr) { hb_face_destroy(ptr); }
};

struct hb_subset_input_deleter {
  void operator()(hb_subset_input_t* ptr) { hb_subset_input_destroy(ptr); }
};

struct hb_set_deleter {
  void operator()(hb_set_t* ptr) { hb_set_destroy(ptr); }
};

using HbBlobPtr = std::unique_ptr<hb_blob_t, hb_blob_deleter>;
using HbFacePtr = std::unique_ptr<hb_face_t, hb_face_deleter>;
using HbSubsetInputPtr =
    std::unique_ptr<hb_subset_input_t, hb_subset_input_deleter>;
using HbSetPtr = std::unique_ptr<hb_set_t, hb_set_deleter>;

};  // namespace HarfbuzzWrappers

#endif  // FLUTTER_TOOLS_FONT_SUBSET_HB_WRAPPERS_H_
