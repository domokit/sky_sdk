// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_FL_KEYBOARD_PENDING_EVENT_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_FL_KEYBOARD_PENDING_EVENT_H_

#include "fl_key_event.h"

G_BEGIN_DECLS

G_DECLARE_FINAL_TYPE(FlKeyboardPendingEvent,
                     fl_keyboard_pending_event,
                     FL,
                     KEYBOARD_PENDING_EVENT,
                     GObject);

FlKeyboardPendingEvent* fl_keyboard_pending_event_new(FlKeyEvent* event);

FlKeyEvent* fl_keyboard_pending_event_get_event(FlKeyboardPendingEvent* event);

uint64_t fl_keyboard_pending_event_get_hash(FlKeyboardPendingEvent* event);

void fl_keyboard_pending_event_mark_embedder_replied(
    FlKeyboardPendingEvent* event,
    gboolean handled);

void fl_keyboard_pending_event_mark_channel_replied(
    FlKeyboardPendingEvent* event,
    gboolean handled);

gboolean fl_keyboard_pending_event_get_any_handled(
    FlKeyboardPendingEvent* event);

gboolean fl_keyboard_pending_event_is_complete(FlKeyboardPendingEvent* event);

G_END_DECLS

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_FL_KEYBOARD_PENDING_EVENT_H_
