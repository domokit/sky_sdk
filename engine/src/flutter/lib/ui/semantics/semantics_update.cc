// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/lib/ui/semantics/semantics_update.h"

#include <memory>

#include "flutter/lib/ui/painting/matrix.h"
#include "flutter/lib/ui/semantics/semantics_update_builder.h"
#include "flutter/lib/ui/ui_dart_state.h"
#include "third_party/tonic/converter/dart_converter.h"
#include "third_party/tonic/dart_args.h"
#include "third_party/tonic/dart_binding_macros.h"
#include "third_party/tonic/dart_library_natives.h"

namespace flutter {

IMPLEMENT_WRAPPERTYPEINFO(ui, SemanticsUpdate);

void SemanticsUpdate::create(Dart_Handle semantics_update_handle,
                             SemanticsNodeUpdates nodes,
                             CustomAccessibilityActionUpdates actions) {
  auto semantics_update = fml::MakeRefCounted<SemanticsUpdate>(
      std::move(nodes), std::move(actions));
  semantics_update->AssociateWithDartWrapper(semantics_update_handle);
}

SemanticsUpdate::SemanticsUpdate(SemanticsNodeUpdates nodes,
                                 CustomAccessibilityActionUpdates actions)
    : nodes_(std::move(nodes)), actions_(std::move(actions)) {}

SemanticsUpdate::~SemanticsUpdate() = default;

SemanticsNodeUpdates SemanticsUpdate::takeNodes() {
  return std::move(nodes_);
}

CustomAccessibilityActionUpdates SemanticsUpdate::takeActions() {
  return std::move(actions_);
}

void SemanticsUpdate::dispose() {
  ClearDartWrapper();
}

}  // namespace flutter
