// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/testing/mock_signal_handler.h"

namespace flutter {
namespace testing {

SignalHandler::SignalHandler(gpointer instance,
                             const gchar* name,
                             GCallback callback)
    : instance_(instance) {
  id_ = g_signal_connect_data(instance, name, callback, this, nullptr,
                              G_CONNECT_SWAPPED);
  g_object_add_weak_pointer(G_OBJECT(instance), &instance_);
}

SignalHandler::~SignalHandler() {
  if (instance_) {
    g_signal_handler_disconnect(instance_, id_);
    g_object_remove_weak_pointer(G_OBJECT(instance_), &instance_);
  }
}

}  // namespace testing
}  // namespace flutter
