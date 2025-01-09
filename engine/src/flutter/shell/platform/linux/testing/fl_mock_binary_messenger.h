// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_TESTING_FL_MOCK_BINARY_MESSENGER_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_TESTING_FL_MOCK_BINARY_MESSENGER_H_

#include <gio/gio.h>

#include "flutter/shell/platform/linux/public/flutter_linux/fl_message_codec.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_method_codec.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_value.h"

G_BEGIN_DECLS

G_DECLARE_FINAL_TYPE(FlMockBinaryMessenger,
                     fl_mock_binary_messenger,
                     FL,
                     MOCK_BINARY_MESSENGER,
                     GObject)

typedef GBytes* (*FlMockBinaryMessengerChannelHandler)(
    FlMockBinaryMessenger* messenger,
    GBytes* message,
    gpointer user_data);

typedef FlValue* (*FlMockBinaryMessengerMessageChannelHandler)(
    FlMockBinaryMessenger* messenger,
    GTask* task,
    FlValue* message,
    gpointer user_data);

typedef FlMethodResponse* (*FlMockBinaryMessengerMethodChannelHandler)(
    FlMockBinaryMessenger* messenger,
    GTask* task,
    const gchar* name,
    FlValue* args,
    gpointer user_data);

typedef void (*FlMockBinaryMessengerEventChannelHandler)(
    FlMockBinaryMessenger* messenger,
    FlValue* event,
    gpointer user_data);

typedef void (*FlMockBinaryMessengerEventChannelErrorHandler)(
    FlMockBinaryMessenger* messenger,
    const gchar* code,
    const gchar* message,
    FlValue* details,
    gpointer user_data);

typedef void (*FlMockBinaryMessengerCallback)(FlMockBinaryMessenger* messenger,
                                              GBytes* response,
                                              gpointer user_data);

typedef void (*FlMockBinaryMessengerMessageCallback)(
    FlMockBinaryMessenger* messenger,
    FlValue* response,
    gpointer user_data);

typedef void (*FlMockBinaryMessengerMethodCallback)(
    FlMockBinaryMessenger* messenger,
    FlMethodResponse* response,
    gpointer user_data);

FlMockBinaryMessenger* fl_mock_binary_messenger_new();

gboolean fl_mock_binary_messenger_has_handler(FlMockBinaryMessenger* self,
                                              const gchar* channel);

void fl_mock_binary_messenger_set_channel(
    FlMockBinaryMessenger* self,
    const gchar* channel,
    FlMockBinaryMessengerChannelHandler handler,
    gpointer user_data);

void fl_mock_binary_messenger_set_message_channel(
    FlMockBinaryMessenger* self,
    const gchar* channel,
    FlMessageCodec* codec,
    FlMockBinaryMessengerMessageChannelHandler handler,
    gpointer user_data);

void fl_mock_binary_messenger_set_standard_message_channel(
    FlMockBinaryMessenger* self,
    const gchar* channel,
    FlMockBinaryMessengerMessageChannelHandler handler,
    gpointer user_data);

void fl_mock_binary_messenger_set_string_message_channel(
    FlMockBinaryMessenger* self,
    const gchar* channel,
    FlMockBinaryMessengerMessageChannelHandler handler,
    gpointer user_data);

void fl_mock_binary_messenger_set_json_message_channel(
    FlMockBinaryMessenger* self,
    const gchar* channel,
    FlMockBinaryMessengerMessageChannelHandler handler,
    gpointer user_data);

void fl_mock_binary_messenger_message_channel_respond(
    FlMockBinaryMessenger* self,
    GTask* task,
    FlMessageCodec* codec,
    FlValue* value);

void fl_mock_binary_messenger_standard_message_channel_respond(
    FlMockBinaryMessenger* self,
    GTask* task,
    FlValue* value);

void fl_mock_binary_messenger_string_message_channel_respond(
    FlMockBinaryMessenger* self,
    GTask* task,
    FlValue* value);

void fl_mock_binary_messenger_json_message_channel_respond(
    FlMockBinaryMessenger* self,
    GTask* task,
    FlValue* value);

void fl_mock_binary_messenger_set_method_channel(
    FlMockBinaryMessenger* self,
    const gchar* channel,
    FlMethodCodec* codec,
    FlMockBinaryMessengerMethodChannelHandler handler,
    gpointer user_data);

void fl_mock_binary_messenger_set_standard_method_channel(
    FlMockBinaryMessenger* self,
    const gchar* channel,
    FlMockBinaryMessengerMethodChannelHandler handler,
    gpointer user_data);

void fl_mock_binary_messenger_set_json_method_channel(
    FlMockBinaryMessenger* self,
    const gchar* channel,
    FlMockBinaryMessengerMethodChannelHandler handler,
    gpointer user_data);

void fl_mock_binary_messenger_set_event_channel(
    FlMockBinaryMessenger* self,
    const gchar* channel,
    FlMethodCodec* codec,
    FlMockBinaryMessengerEventChannelHandler handler,
    FlMockBinaryMessengerEventChannelErrorHandler error_handler,
    gpointer user_data);

void fl_mock_binary_messenger_set_standard_event_channel(
    FlMockBinaryMessenger* self,
    const gchar* channel,
    FlMockBinaryMessengerEventChannelHandler handler,
    FlMockBinaryMessengerEventChannelErrorHandler error_handler,
    gpointer user_data);

void fl_mock_binary_messenger_set_json_event_channel(
    FlMockBinaryMessenger* self,
    const gchar* channel,
    FlMockBinaryMessengerEventChannelHandler handler,
    FlMockBinaryMessengerEventChannelErrorHandler error_handler,
    gpointer user_data);

void fl_mock_binary_messenger_set_error_channel(FlMockBinaryMessenger* self,
                                                const gchar* channel,
                                                gint code,
                                                const gchar* message);

void fl_mock_binary_messenger_send(FlMockBinaryMessenger* self,
                                   const gchar* channel,
                                   GBytes* message,
                                   FlMockBinaryMessengerCallback callback,
                                   gpointer user_data);

void fl_mock_binary_messenger_send_message(
    FlMockBinaryMessenger* self,
    const gchar* channel,
    FlMessageCodec* codec,
    FlValue* message,
    FlMockBinaryMessengerMessageCallback callback,
    gpointer user_data);

void fl_mock_binary_messenger_send_standard_message(
    FlMockBinaryMessenger* self,
    const gchar* channel,
    FlValue* message,
    FlMockBinaryMessengerMessageCallback callback,
    gpointer user_data);

void fl_mock_binary_messenger_send_json_message(
    FlMockBinaryMessenger* self,
    const gchar* channel,
    FlValue* message,
    FlMockBinaryMessengerMessageCallback callback,
    gpointer user_data);

void fl_mock_binary_messenger_invoke_method(
    FlMockBinaryMessenger* self,
    const gchar* channel,
    FlMethodCodec* codec,
    const char* name,
    FlValue* args,
    FlMockBinaryMessengerMethodCallback callback,
    gpointer user_data);

void fl_mock_binary_messenger_invoke_standard_method(
    FlMockBinaryMessenger* self,
    const gchar* channel,
    const char* name,
    FlValue* args,
    FlMockBinaryMessengerMethodCallback callback,
    gpointer user_data);

void fl_mock_binary_messenger_invoke_json_method(
    FlMockBinaryMessenger* self,
    const gchar* channel,
    const char* name,
    FlValue* args,
    FlMockBinaryMessengerMethodCallback callback,
    gpointer user_data);

G_END_DECLS

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_TESTING_FL_MOCK_BINARY_MESSENGER_H_
