// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/public/flutter_linux/fl_method_response.h"

#include <gmodule.h>

G_DEFINE_QUARK(fl_method_response_error_quark, fl_method_response_error)

struct _FlMethodSuccessResponse {
  FlMethodResponse parent_instance;

  FlValue* result;
};

struct _FlMethodErrorResponse {
  FlMethodResponse parent_instance;

  gchar* code;
  gchar* message;
  FlValue* details;
};

struct _FlMethodNotImplementedResponse {
  FlMethodResponse parent_instance;
};

G_DEFINE_TYPE(FlMethodResponse, fl_method_response, G_TYPE_OBJECT)
G_DEFINE_TYPE(FlMethodSuccessResponse,
              fl_method_success_response,
              fl_method_response_get_type())
G_DEFINE_TYPE(FlMethodErrorResponse,
              fl_method_error_response,
              fl_method_response_get_type())
G_DEFINE_TYPE(FlMethodNotImplementedResponse,
              fl_method_not_implemented_response,
              fl_method_response_get_type())

static void fl_method_response_class_init(FlMethodResponseClass* klass) {}

static void fl_method_response_init(FlMethodResponse* self) {}

static void fl_method_success_response_dispose(GObject* object) {
  FlMethodSuccessResponse* self = FL_METHOD_SUCCESS_RESPONSE(object);

  g_clear_pointer(&self->result, fl_value_unref);

  G_OBJECT_CLASS(fl_method_success_response_parent_class)->dispose(object);
}

static void fl_method_success_response_class_init(
    FlMethodSuccessResponseClass* klass) {
  G_OBJECT_CLASS(klass)->dispose = fl_method_success_response_dispose;
}

static void fl_method_success_response_init(FlMethodSuccessResponse* self) {}

static void fl_method_error_response_dispose(GObject* object) {
  FlMethodErrorResponse* self = FL_METHOD_ERROR_RESPONSE(object);

  g_clear_pointer(&self->code, g_free);
  g_clear_pointer(&self->message, g_free);
  g_clear_pointer(&self->details, fl_value_unref);

  G_OBJECT_CLASS(fl_method_error_response_parent_class)->dispose(object);
}

static void fl_method_error_response_class_init(
    FlMethodErrorResponseClass* klass) {
  G_OBJECT_CLASS(klass)->dispose = fl_method_error_response_dispose;
}

static void fl_method_error_response_init(FlMethodErrorResponse* self) {}

static void fl_method_not_implemented_response_class_init(
    FlMethodNotImplementedResponseClass* klass) {}

static void fl_method_not_implemented_response_init(
    FlMethodNotImplementedResponse* self) {}

G_MODULE_EXPORT FlValue* fl_method_response_get_result(FlMethodResponse* self,
                                                       GError** error) {
  if (FL_IS_METHOD_SUCCESS_RESPONSE(self)) {
    return fl_method_success_response_get_result(
        FL_METHOD_SUCCESS_RESPONSE(self));
  }

  if (FL_IS_METHOD_ERROR_RESPONSE(self)) {
    const gchar* code =
        fl_method_error_response_get_code(FL_METHOD_ERROR_RESPONSE(self));
    const gchar* message =
        fl_method_error_response_get_message(FL_METHOD_ERROR_RESPONSE(self));
    FlValue* details =
        fl_method_error_response_get_details(FL_METHOD_ERROR_RESPONSE(self));
    g_autofree gchar* details_text = nullptr;
    if (details != nullptr) {
      details_text = fl_value_to_string(details);
    }

    g_autoptr(GString) error_message = g_string_new("");
    g_string_append_printf(error_message, "Remote code returned error %s",
                           code);
    if (message != nullptr) {
      g_string_append_printf(error_message, ": %s", message);
    }
    if (details_text != nullptr) {
      g_string_append_printf(error_message, " %s", details_text);
    }
    g_set_error_literal(error, FL_METHOD_RESPONSE_ERROR,
                        FL_METHOD_RESPONSE_ERROR_REMOTE_ERROR,
                        error_message->str);
    return nullptr;
  } else if (FL_IS_METHOD_NOT_IMPLEMENTED_RESPONSE(self)) {
    g_set_error(error, FL_METHOD_RESPONSE_ERROR,
                FL_METHOD_RESPONSE_ERROR_NOT_IMPLEMENTED,
                "Requested method is not implemented");
    return nullptr;
  } else {
    g_set_error(error, FL_METHOD_RESPONSE_ERROR,
                FL_METHOD_RESPONSE_ERROR_FAILED, "Unknown response type");
    return nullptr;
  }
}

G_MODULE_EXPORT FlMethodSuccessResponse* fl_method_success_response_new(
    FlValue* result) {
  FlMethodSuccessResponse* self = FL_METHOD_SUCCESS_RESPONSE(
      g_object_new(fl_method_success_response_get_type(), nullptr));

  if (result != nullptr) {
    self->result = fl_value_ref(result);
  }

  return self;
}

G_MODULE_EXPORT FlValue* fl_method_success_response_get_result(
    FlMethodSuccessResponse* self) {
  g_return_val_if_fail(FL_IS_METHOD_SUCCESS_RESPONSE(self), nullptr);
  return self->result;
}

G_MODULE_EXPORT FlMethodErrorResponse* fl_method_error_response_new(
    const gchar* code,
    const gchar* message,
    FlValue* details) {
  g_return_val_if_fail(code != nullptr, nullptr);

  FlMethodErrorResponse* self = FL_METHOD_ERROR_RESPONSE(
      g_object_new(fl_method_error_response_get_type(), nullptr));

  self->code = g_strdup(code);
  self->message = g_strdup(message);
  self->details = details != nullptr ? fl_value_ref(details) : nullptr;

  return self;
}

G_MODULE_EXPORT const gchar* fl_method_error_response_get_code(
    FlMethodErrorResponse* self) {
  g_return_val_if_fail(FL_IS_METHOD_ERROR_RESPONSE(self), nullptr);
  return self->code;
}

G_MODULE_EXPORT const gchar* fl_method_error_response_get_message(
    FlMethodErrorResponse* self) {
  g_return_val_if_fail(FL_IS_METHOD_ERROR_RESPONSE(self), nullptr);
  return self->message;
}

G_MODULE_EXPORT FlValue* fl_method_error_response_get_details(
    FlMethodErrorResponse* self) {
  g_return_val_if_fail(FL_IS_METHOD_ERROR_RESPONSE(self), nullptr);
  return self->details;
}

G_MODULE_EXPORT FlMethodNotImplementedResponse*
fl_method_not_implemented_response_new() {
  return FL_METHOD_NOT_IMPLEMENTED_RESPONSE(
      g_object_new(fl_method_not_implemented_response_get_type(), nullptr));
}
