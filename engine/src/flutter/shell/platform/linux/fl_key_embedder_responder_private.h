// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_FL_KEY_EMBEDDER_RESPONDER_PRIVATE_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_FL_KEY_EMBEDDER_RESPONDER_PRIVATE_H_

#include <gdk/gdk.h>

#include "flutter/shell/platform/linux/fl_keyboard_handler.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_binary_messenger.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_value.h"

/**
 * FlKeyEmbedderCheckedKey:
 *
 * The information for a key that #FlKeyEmbedderResponder should keep state
 * synchronous on. For every record of #FlKeyEmbedderCheckedKey, the responder
 * will check the #GdkEvent::state and the internal state, and synchronize
 * events if they don't match.
 *
 * #FlKeyEmbedderCheckedKey can synchronize pressing states (such as
 * whether ControlLeft is pressed) or lock states (such as whether CapsLock
 * is enabled).
 *
 * #FlKeyEmbedderCheckedKey has a "primary key". For pressing states, the
 * primary key is the left of the modifiers. For lock states, the primary
 * key is the key.
 *
 * #FlKeyEmbedderCheckedKey may also have a "secondary key". It is only
 * available to pressing states, which is the right of the modifiers.
 */
typedef struct {
  // The physical key for the primary key.
  uint64_t primary_physical_key;
  // The logical key for the primary key.
  uint64_t primary_logical_key;
  // The logical key for the secondary key.
  uint64_t secondary_logical_key;
  // Whether this key is CapsLock.  CapsLock uses a different event model in GDK
  // and needs special treatment.
  bool is_caps_lock;
} FlKeyEmbedderCheckedKey;

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_FL_KEY_EMBEDDER_RESPONDER_PRIVATE_H_
