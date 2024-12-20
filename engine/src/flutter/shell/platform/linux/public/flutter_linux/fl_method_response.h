// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_PUBLIC_FLUTTER_LINUX_FL_METHOD_RESPONSE_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_PUBLIC_FLUTTER_LINUX_FL_METHOD_RESPONSE_H_

#if !defined(__FLUTTER_LINUX_INSIDE__) && !defined(FLUTTER_LINUX_COMPILATION)
#error "Only <flutter_linux/flutter_linux.h> can be included directly."
#endif

#include <glib-object.h>
#include <gmodule.h>

#include "fl_value.h"

G_BEGIN_DECLS

/**
 * FlMethodResponseError:
 * @FL_METHOD_RESPONSE_ERROR_FAILED: Call failed due to an unspecified error.
 * @FL_METHOD_RESPONSE_ERROR_REMOTE_ERROR: An error was returned by the other
 * side of the channel.
 * @FL_METHOD_RESPONSE_ERROR_NOT_IMPLEMENTED: The requested method is not
 * implemented.
 *
 * Errors set by `fl_method_response_get_result` when the method call response
 * is not #FlMethodSuccessResponse.
 */
#define FL_METHOD_RESPONSE_ERROR fl_method_response_error_quark()

typedef enum {
  FL_METHOD_RESPONSE_ERROR_FAILED,
  FL_METHOD_RESPONSE_ERROR_REMOTE_ERROR,
  FL_METHOD_RESPONSE_ERROR_NOT_IMPLEMENTED,
} FlMethodResponseError;

G_MODULE_EXPORT
GQuark fl_method_response_error_quark(void) G_GNUC_CONST;

G_MODULE_EXPORT
G_DECLARE_DERIVABLE_TYPE(FlMethodResponse,
                         fl_method_response,
                         FL,
                         METHOD_RESPONSE,
                         GObject)

struct _FlMethodResponseClass {
  GObjectClass parent_class;
};

G_MODULE_EXPORT
G_DECLARE_FINAL_TYPE(FlMethodSuccessResponse,
                     fl_method_success_response,
                     FL,
                     METHOD_SUCCESS_RESPONSE,
                     FlMethodResponse)

G_MODULE_EXPORT
G_DECLARE_FINAL_TYPE(FlMethodErrorResponse,
                     fl_method_error_response,
                     FL,
                     METHOD_ERROR_RESPONSE,
                     FlMethodResponse)

G_MODULE_EXPORT
G_DECLARE_FINAL_TYPE(FlMethodNotImplementedResponse,
                     fl_method_not_implemented_response,
                     FL,
                     METHOD_NOT_IMPLEMENTED_RESPONSE,
                     FlMethodResponse)

/**
 * FlMethodResponse:
 *
 * #FlMethodResponse contains the information returned when an #FlMethodChannel
 * method call returns. If you expect the method call to be successful use
 * fl_method_response_get_result(). If you want to handle error cases then you
 * should use code like:
 *
 * |[<!-- language="C" -->
 *   if (FL_IS_METHOD_SUCCESS_RESPONSE (response)) {
 *     FlValue *result =
 *       fl_method_success_response_get_result(
 *         FL_METHOD_SUCCESS_RESPONSE (response));
 *     handle_result (result);
 *   } else if (FL_IS_METHOD_ERROR_RESPONSE (response)) {
 *     FlMethodErrorResponse *error_response =
 *       FL_METHOD_ERROR_RESPONSE (response);
 *     handle_error (fl_method_error_response_get_code (error_response),
 *                   fl_method_error_response_get_message (error_response),
 *                   fl_method_error_response_get_details (error_response));
 *   }
 *   else if (FL_IS_METHOD_NOT_IMPLEMENTED_RESPONSE (response)) {
 *     handle_not_implemented ();
 *   }
 * }
 * ]|
 */

/**
 * FlMethodSuccessResponse:
 *
 * #FlMethodSuccessResponse is the #FlMethodResponse returned when a method call
 * has successfully completed. The result of the method call is obtained using
 * `fl_method_success_response_get_result`.
 */

/**
 * FlMethodErrorResponse:
 *
 * #FlMethodErrorResponse is the #FlMethodResponse returned when a method call
 * results in an error. The error details are obtained using
 * `fl_method_error_response_get_code`, `fl_method_error_response_get_message`
 * and `fl_method_error_response_get_details`.
 */

/**
 * FlMethodNotImplementedResponse:
 *
 * #FlMethodNotImplementedResponse is the #FlMethodResponse returned when a
 * method call is not implemented.
 */

/**
 * fl_method_response_get_result:
 * @response: an #FlMethodResponse.
 * @error: (allow-none): #GError location to store the error occurring, or %NULL
 * to ignore.
 *
 * Gets the result of a method call, or an error if the response wasn't
 * successful.
 *
 * Returns: an #FlValue or %NULL on error.
 */
FlValue* fl_method_response_get_result(FlMethodResponse* response,
                                       GError** error);

/**
 * fl_method_success_response_new:
 * @result: (allow-none): the #FlValue returned by the method call or %NULL.
 *
 * Creates a response to a method call when that method has successfully
 * completed.
 *
 * Returns: a new #FlMethodResponse.
 */
FlMethodSuccessResponse* fl_method_success_response_new(FlValue* result);

/**
 * fl_method_success_response_get_result:
 * @response: an #FlMethodSuccessResponse.
 *
 * Gets the result of the method call.
 *
 * Returns: an #FlValue.
 */
FlValue* fl_method_success_response_get_result(
    FlMethodSuccessResponse* response);

/**
 * fl_method_error_response_new:
 * @result: an #FlValue.
 * @code: an error code.
 * @message: (allow-none): an error message.
 * @details: (allow-none): error details.
 *
 * Creates a response to a method call when that method has returned an error.
 *
 * Returns: a new #FlMethodErrorResponse.
 */
FlMethodErrorResponse* fl_method_error_response_new(const gchar* code,
                                                    const gchar* message,
                                                    FlValue* details);

/**
 * fl_method_error_response_get_code:
 * @response: an #FlMethodErrorResponse.
 *
 * Gets the error code reported.
 *
 * Returns: an error code.
 */
const gchar* fl_method_error_response_get_code(FlMethodErrorResponse* response);

/**
 * fl_method_error_response_get_message:
 * @response: an #FlMethodErrorResponse.
 *
 * Gets the error message reported.
 *
 * Returns: an error message or %NULL if no error message provided.
 */
const gchar* fl_method_error_response_get_message(
    FlMethodErrorResponse* response);

/**
 * fl_method_error_response_get_details:
 * @response: an #FlMethodErrorResponse.
 *
 * Gets the details provided with this error.
 *
 * Returns: an #FlValue or %NULL if no details provided.
 */
FlValue* fl_method_error_response_get_details(FlMethodErrorResponse* response);

/**
 * fl_method_not_implemented_response_new:
 *
 * Creates a response to a method call when that method does not exist.
 *
 * Returns: a new #FlMethodNotImplementedResponse.
 */
FlMethodNotImplementedResponse* fl_method_not_implemented_response_new();

G_END_DECLS

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_PUBLIC_FLUTTER_LINUX_FL_METHOD_RESPONSE_H_
