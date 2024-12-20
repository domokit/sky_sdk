// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/fl_platform_handler.h"

#include <gtk/gtk.h>
#include <cstring>

#include "flutter/shell/platform/linux/fl_platform_channel.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_method_channel.h"

static constexpr char kInProgressError[] = "In Progress";
static constexpr char kUnknownClipboardFormatError[] =
    "Unknown Clipboard Format";

static constexpr char kTextPlainFormat[] = "text/plain";

static constexpr char kSoundTypeAlert[] = "SystemSoundType.alert";
static constexpr char kSoundTypeClick[] = "SystemSoundType.click";

struct _FlPlatformHandler {
  GObject parent_instance;

  FlPlatformChannel* channel;

  FlMethodCall* exit_application_method_call;

  bool app_initialization_complete;

  GCancellable* cancellable;
};

G_DEFINE_TYPE(FlPlatformHandler, fl_platform_handler, G_TYPE_OBJECT)

// Called when clipboard text received.
static void clipboard_text_cb(GtkClipboard* clipboard,
                              const gchar* text,
                              gpointer user_data) {
  g_autoptr(FlMethodCall) method_call = FL_METHOD_CALL(user_data);
  fl_platform_channel_respond_clipboard_get_data(method_call, text);
}

// Called when clipboard text received during has_strings.
static void clipboard_text_has_strings_cb(GtkClipboard* clipboard,
                                          const gchar* text,
                                          gpointer user_data) {
  g_autoptr(FlMethodCall) method_call = FL_METHOD_CALL(user_data);
  fl_platform_channel_respond_clipboard_has_strings(
      method_call, text != nullptr && strlen(text) > 0);
}

// Called when Flutter wants to copy to the clipboard.
static FlMethodResponse* clipboard_set_data(FlMethodCall* method_call,
                                            const gchar* text,
                                            gpointer user_data) {
  GtkClipboard* clipboard =
      gtk_clipboard_get_default(gdk_display_get_default());
  gtk_clipboard_set_text(clipboard, text, -1);

  return FL_METHOD_RESPONSE(fl_method_success_response_new(nullptr));
}

// Called when Flutter wants to paste from the clipboard.
static FlMethodResponse* clipboard_get_data(FlMethodCall* method_call,
                                            const gchar* format,
                                            gpointer user_data) {
  if (strcmp(format, kTextPlainFormat) != 0) {
    return FL_METHOD_RESPONSE(fl_method_error_response_new(
        kUnknownClipboardFormatError, "GTK clipboard API only supports text",
        nullptr));
  }

  GtkClipboard* clipboard =
      gtk_clipboard_get_default(gdk_display_get_default());
  gtk_clipboard_request_text(clipboard, clipboard_text_cb,
                             g_object_ref(method_call));

  // Will respond later.
  return nullptr;
}

// Called when Flutter wants to know if the content of the clipboard is able to
// be pasted, without actually accessing the clipboard content itself.
static FlMethodResponse* clipboard_has_strings(FlMethodCall* method_call,
                                               gpointer user_data) {
  GtkClipboard* clipboard =
      gtk_clipboard_get_default(gdk_display_get_default());
  gtk_clipboard_request_text(clipboard, clipboard_text_has_strings_cb,
                             g_object_ref(method_call));

  // Will respond later.
  return nullptr;
}

// Quit this application
static void quit_application() {
  GApplication* app = g_application_get_default();
  if (app == nullptr) {
    // Unable to gracefully quit, so just exit the process.
    exit(0);
  }

  // GtkApplication windows contain a reference back to the application.
  // Break them so the application object can cleanup.
  // See https://gitlab.gnome.org/GNOME/gtk/-/issues/6190
  if (GTK_IS_APPLICATION(app)) {
    // List is copied as it will be modified as windows are disconnected from
    // the application.
    g_autoptr(GList) windows =
        g_list_copy(gtk_application_get_windows(GTK_APPLICATION(app)));
    for (GList* link = windows; link != NULL; link = link->next) {
      GtkWidget* window = GTK_WIDGET(link->data);
      gtk_window_set_application(GTK_WINDOW(window), NULL);
    }
  }

  g_application_quit(app);
}

// Handle response of System.requestAppExit.
static void request_app_exit_response_cb(GObject* object,
                                         GAsyncResult* result,
                                         gpointer user_data) {
  FlPlatformHandler* self = FL_PLATFORM_HANDLER(user_data);

  g_autoptr(GError) error = nullptr;
  FlPlatformChannelExitResponse exit_response;
  if (!fl_platform_channel_system_request_app_exit_finish(
          object, result, &exit_response, &error)) {
    if (g_error_matches(error, G_IO_ERROR, G_IO_ERROR_CANCELLED)) {
      return;
    }
    g_warning("Failed to complete System.requestAppExit: %s", error->message);
    quit_application();
    return;
  }

  if (exit_response == FL_PLATFORM_CHANNEL_EXIT_RESPONSE_EXIT) {
    quit_application();
  }

  // If request was due to a request from Flutter, pass result back.
  if (self->exit_application_method_call != nullptr) {
    fl_platform_channel_respond_system_exit_application(
        self->exit_application_method_call, exit_response);
  }
}

// Send a request to Flutter to exit the application, but only if it's ready for
// a request.
static void request_app_exit(FlPlatformHandler* self,
                             FlPlatformChannelExitType type) {
  if (!self->app_initialization_complete ||
      type == FL_PLATFORM_CHANNEL_EXIT_TYPE_REQUIRED) {
    quit_application();
    return;
  }

  fl_platform_channel_system_request_app_exit(
      self->channel, type, self->cancellable, request_app_exit_response_cb,
      self);
}

// Called when the Dart app has finished initialization and is ready to handle
// requests. For the Flutter framework, this means after the ServicesBinding has
// been initialized and it sends a System.initializationComplete message.
static void system_initialization_complete(gpointer user_data) {
  FlPlatformHandler* self = FL_PLATFORM_HANDLER(user_data);
  self->app_initialization_complete = TRUE;
}

// Called when Flutter wants to exit the application.
static FlMethodResponse* system_exit_application(FlMethodCall* method_call,
                                                 FlPlatformChannelExitType type,
                                                 gpointer user_data) {
  FlPlatformHandler* self = FL_PLATFORM_HANDLER(user_data);
  // Save method call to respond to when our request to Flutter completes.
  if (self->exit_application_method_call != nullptr) {
    return FL_METHOD_RESPONSE(fl_method_error_response_new(
        kInProgressError, "Request already in progress", nullptr));
  }
  self->exit_application_method_call =
      FL_METHOD_CALL(g_object_ref(method_call));

  // Requested to immediately quit if the app hasn't yet signaled that it is
  // ready to handle requests, or if the type of exit requested is "required".
  if (!self->app_initialization_complete ||
      type == FL_PLATFORM_CHANNEL_EXIT_TYPE_REQUIRED) {
    quit_application();
    return fl_platform_channel_make_system_request_app_exit_response(
        FL_PLATFORM_CHANNEL_EXIT_RESPONSE_EXIT);
  }

  // Send the request back to Flutter to follow the standard process.
  request_app_exit(self, type);

  // Will respond later.
  return nullptr;
}

// Called when Flutter wants to play a sound.
static void system_sound_play(const gchar* type, gpointer user_data) {
  if (strcmp(type, kSoundTypeAlert) == 0) {
    GdkDisplay* display = gdk_display_get_default();
    if (display != nullptr) {
      gdk_display_beep(display);
    }
  } else if (strcmp(type, kSoundTypeClick) == 0) {
    // We don't make sounds for keyboard on desktops.
  } else {
    g_warning("Ignoring unknown sound type %s in SystemSound.play.\n", type);
  }
}

// Called when Flutter wants to quit the application.
static void system_navigator_pop(gpointer user_data) {
  quit_application();
}

static void fl_platform_handler_dispose(GObject* object) {
  FlPlatformHandler* self = FL_PLATFORM_HANDLER(object);

  g_cancellable_cancel(self->cancellable);

  g_clear_object(&self->channel);
  g_clear_object(&self->exit_application_method_call);
  g_clear_object(&self->cancellable);

  G_OBJECT_CLASS(fl_platform_handler_parent_class)->dispose(object);
}

static void fl_platform_handler_class_init(FlPlatformHandlerClass* klass) {
  G_OBJECT_CLASS(klass)->dispose = fl_platform_handler_dispose;
}

static void fl_platform_handler_init(FlPlatformHandler* self) {
  self->cancellable = g_cancellable_new();
}

static FlPlatformChannelVTable platform_channel_vtable = {
    .clipboard_set_data = clipboard_set_data,
    .clipboard_get_data = clipboard_get_data,
    .clipboard_has_strings = clipboard_has_strings,
    .system_exit_application = system_exit_application,
    .system_initialization_complete = system_initialization_complete,
    .system_sound_play = system_sound_play,
    .system_navigator_pop = system_navigator_pop,
};

FlPlatformHandler* fl_platform_handler_new(FlBinaryMessenger* messenger) {
  g_return_val_if_fail(FL_IS_BINARY_MESSENGER(messenger), nullptr);

  FlPlatformHandler* self = FL_PLATFORM_HANDLER(
      g_object_new(fl_platform_handler_get_type(), nullptr));

  self->channel =
      fl_platform_channel_new(messenger, &platform_channel_vtable, self);
  self->app_initialization_complete = FALSE;

  return self;
}

void fl_platform_handler_request_app_exit(FlPlatformHandler* self) {
  g_return_if_fail(FL_IS_PLATFORM_HANDLER(self));
  // Request a cancellable exit.
  request_app_exit(self, FL_PLATFORM_CHANNEL_EXIT_TYPE_CANCELABLE);
}
