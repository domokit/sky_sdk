// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_FL_ACCESSIBLE_TEXT_FIELD_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_FL_ACCESSIBLE_TEXT_FIELD_H_

#include <gtk/gtk.h>

#include "flutter/shell/platform/linux/fl_accessible_node.h"

G_BEGIN_DECLS

G_DECLARE_FINAL_TYPE(FlAccessibleTextField,
                     fl_accessible_text_field,
                     FL,
                     ACCESSIBLE_TEXT_FIELD,
                     FlAccessibleNode);

/**
 * fl_accessible_text_field_new:
 * @engine: the #FlEngine this node came from.
 * @id: the semantics node ID this object represents.
 *
 * Creates a new accessibility object that exposes an editable Flutter text
 * field to ATK.
 *
 * Returns: a new #FlAccessibleNode.
 */
FlAccessibleNode* fl_accessible_text_field_new(FlEngine* engine, int32_t id);

G_END_DECLS

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_FL_ACCESSIBLE_TEXT_FIELD_H_
