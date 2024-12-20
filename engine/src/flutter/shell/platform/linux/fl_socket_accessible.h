// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_FL_SOCKET_ACCESSIBLE_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_FL_SOCKET_ACCESSIBLE_H_

#include <gtk/gtk-a11y.h>

G_BEGIN_DECLS

G_DECLARE_FINAL_TYPE(FlSocketAccessible,
                     fl_socket_accessible,
                     FL,
                     SOCKET_ACCESSIBLE,
                     GtkContainerAccessible);

void fl_socket_accessible_embed(FlSocketAccessible* self, gchar* id);

G_END_DECLS

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_FL_SOCKET_ACCESSIBLE_H_
