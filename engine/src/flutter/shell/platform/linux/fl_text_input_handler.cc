// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/fl_text_input_handler.h"

#include <gtk/gtk.h>

#include "flutter/shell/platform/common/text_editing_delta.h"
#include "flutter/shell/platform/common/text_input_model.h"
#include "flutter/shell/platform/linux/fl_text_input_channel.h"

static constexpr char kNewlineInputAction[] = "TextInputAction.newline";

static constexpr int64_t kClientIdUnset = -1;

struct _FlTextInputHandler {
  GObject parent_instance;

  FlTextInputChannel* channel;

  // The widget with input focus.
  GtkWidget* widget;

  // Client ID provided by Flutter to report events with.
  int64_t client_id;

  // Input action to perform when enter pressed.
  gchar* input_action;

  // The type of the input method.
  FlTextInputType input_type;

  // Whether to enable that the engine sends text input updates to the framework
  // as TextEditingDeltas or as one TextEditingValue.
  // For more information on the delta model, see:
  // https://master-api.flutter.dev/flutter/services/TextInputConfiguration/enableDeltaModel.html
  gboolean enable_delta_model;

  // Input method.
  GtkIMContext* im_context;

  flutter::TextInputModel* text_model;

  // A 4x4 matrix that maps from `EditableText` local coordinates to the
  // coordinate system of `PipelineOwner.rootNode`.
  double editabletext_transform[4][4];

  // The smallest rect, in local coordinates, of the text in the composing
  // range, or of the caret in the case where there is no current composing
  // range. This value is updated via `TextInput.setMarkedTextRect` messages
  // over the text input channel.
  GdkRectangle composing_rect;

  GCancellable* cancellable;
};

G_DEFINE_TYPE(FlTextInputHandler, fl_text_input_handler, G_TYPE_OBJECT)

// Called when a response is received from TextInputClient.updateEditingState()
static void update_editing_state_response_cb(GObject* object,
                                             GAsyncResult* result,
                                             gpointer user_data) {
  g_autoptr(GError) error = nullptr;
  if (!fl_text_input_channel_update_editing_state_finish(object, result,
                                                         &error)) {
    if (!g_error_matches(error, G_IO_ERROR, G_IO_ERROR_CANCELLED)) {
      g_warning("Failed to update editing state: %s", error->message);
    }
  }
}

// Called when a response is received from
// TextInputClient.updateEditingStateWithDeltas()
static void update_editing_state_with_deltas_response_cb(GObject* object,
                                                         GAsyncResult* result,
                                                         gpointer user_data) {
  g_autoptr(GError) error = nullptr;
  if (!fl_text_input_channel_update_editing_state_with_deltas_finish(
          object, result, &error)) {
    if (!g_error_matches(error, G_IO_ERROR, G_IO_ERROR_CANCELLED)) {
      g_warning("Failed to update editing state with deltas: %s",
                error->message);
    }
  }
}

// Informs Flutter of text input changes.
static void update_editing_state(FlTextInputHandler* self) {
  int composing_base = -1;
  int composing_extent = -1;
  if (!self->text_model->composing_range().collapsed()) {
    composing_base = self->text_model->composing_range().base();
    composing_extent = self->text_model->composing_range().extent();
  }
  flutter::TextRange selection = self->text_model->selection();
  fl_text_input_channel_update_editing_state(
      self->channel, self->client_id, self->text_model->GetText().c_str(),
      selection.base(), selection.extent(), FL_TEXT_AFFINITY_DOWNSTREAM, FALSE,
      composing_base, composing_extent, self->cancellable,
      update_editing_state_response_cb, self);
}

// Informs Flutter of text input changes by passing just the delta.
static void update_editing_state_with_delta(FlTextInputHandler* self,
                                            flutter::TextEditingDelta* delta) {
  flutter::TextRange selection = self->text_model->selection();
  int composing_base = -1;
  int composing_extent = -1;
  if (!self->text_model->composing_range().collapsed()) {
    composing_base = self->text_model->composing_range().base();
    composing_extent = self->text_model->composing_range().extent();
  }
  fl_text_input_channel_update_editing_state_with_deltas(
      self->channel, self->client_id, delta->old_text().c_str(),
      delta->delta_text().c_str(), delta->delta_start(), delta->delta_end(),
      selection.base(), selection.extent(), FL_TEXT_AFFINITY_DOWNSTREAM, FALSE,
      composing_base, composing_extent, self->cancellable,
      update_editing_state_with_deltas_response_cb, self);
}

// Called when a response is received from TextInputClient.performAction()
static void perform_action_response_cb(GObject* object,
                                       GAsyncResult* result,
                                       gpointer user_data) {
  g_autoptr(GError) error = nullptr;
  if (!fl_text_input_channel_perform_action_finish(object, result, &error)) {
    if (!g_error_matches(error, G_IO_ERROR, G_IO_ERROR_CANCELLED)) {
      g_warning("Failed to perform action: %s", error->message);
    }
  }
}

// Inform Flutter that the input has been activated.
static void perform_action(FlTextInputHandler* self) {
  g_return_if_fail(FL_IS_TEXT_INPUT_HANDLER(self));
  g_return_if_fail(self->client_id != 0);
  g_return_if_fail(self->input_action != nullptr);

  fl_text_input_channel_perform_action(self->channel, self->client_id,
                                       self->input_action, self->cancellable,
                                       perform_action_response_cb, self);
}

// Signal handler for GtkIMContext::preedit-start
static void im_preedit_start_cb(FlTextInputHandler* self) {
  self->text_model->BeginComposing();
}

// Signal handler for GtkIMContext::preedit-changed
static void im_preedit_changed_cb(FlTextInputHandler* self) {
  std::string text_before_change = self->text_model->GetText();
  flutter::TextRange composing_before_change =
      self->text_model->composing_range();
  g_autofree gchar* buf = nullptr;
  gint cursor_offset = 0;
  gtk_im_context_get_preedit_string(self->im_context, &buf, nullptr,
                                    &cursor_offset);
  if (self->text_model->composing()) {
    cursor_offset += self->text_model->composing_range().start();
  } else {
    cursor_offset += self->text_model->selection().start();
  }
  self->text_model->UpdateComposingText(buf);
  self->text_model->SetSelection(flutter::TextRange(cursor_offset));

  if (self->enable_delta_model) {
    std::string text(buf);
    flutter::TextEditingDelta delta = flutter::TextEditingDelta(
        text_before_change, composing_before_change, text);
    update_editing_state_with_delta(self, &delta);
  } else {
    update_editing_state(self);
  }
}

// Signal handler for GtkIMContext::commit
static void im_commit_cb(FlTextInputHandler* self, const gchar* text) {
  std::string text_before_change = self->text_model->GetText();
  flutter::TextRange composing_before_change =
      self->text_model->composing_range();
  flutter::TextRange selection_before_change = self->text_model->selection();
  gboolean was_composing = self->text_model->composing();

  self->text_model->AddText(text);
  if (self->text_model->composing()) {
    self->text_model->CommitComposing();
  }

  if (self->enable_delta_model) {
    flutter::TextRange replace_range =
        was_composing ? composing_before_change : selection_before_change;
    std::unique_ptr<flutter::TextEditingDelta> delta =
        std::make_unique<flutter::TextEditingDelta>(text_before_change,
                                                    replace_range, text);
    update_editing_state_with_delta(self, delta.get());
  } else {
    update_editing_state(self);
  }
}

// Signal handler for GtkIMContext::preedit-end
static void im_preedit_end_cb(FlTextInputHandler* self) {
  self->text_model->EndComposing();
  if (self->enable_delta_model) {
    flutter::TextEditingDelta delta =
        flutter::TextEditingDelta(self->text_model->GetText());
    update_editing_state_with_delta(self, &delta);
  } else {
    update_editing_state(self);
  }
}

// Signal handler for GtkIMContext::retrieve-surrounding
static gboolean im_retrieve_surrounding_cb(FlTextInputHandler* self) {
  auto text = self->text_model->GetText();
  size_t cursor_offset = self->text_model->GetCursorOffset();
  gtk_im_context_set_surrounding(self->im_context, text.c_str(), -1,
                                 cursor_offset);
  return TRUE;
}

// Signal handler for GtkIMContext::delete-surrounding
static gboolean im_delete_surrounding_cb(FlTextInputHandler* self,
                                         gint offset,
                                         gint n_chars) {
  std::string text_before_change = self->text_model->GetText();
  if (self->text_model->DeleteSurrounding(offset, n_chars)) {
    if (self->enable_delta_model) {
      flutter::TextEditingDelta delta = flutter::TextEditingDelta(
          text_before_change, self->text_model->composing_range(),
          self->text_model->GetText());
      update_editing_state_with_delta(self, &delta);
    } else {
      update_editing_state(self);
    }
  }
  return TRUE;
}

// Called when the input method client is set up.
static void set_client(int64_t client_id,
                       const gchar* input_action,
                       gboolean enable_delta_model,
                       FlTextInputType input_type,
                       gpointer user_data) {
  FlTextInputHandler* self = FL_TEXT_INPUT_HANDLER(user_data);

  self->client_id = client_id;
  g_free(self->input_action);
  self->input_action = g_strdup(input_action);
  self->enable_delta_model = enable_delta_model;
  self->input_type = input_type;
}

// Hides the input method.
static void hide(gpointer user_data) {
  FlTextInputHandler* self = FL_TEXT_INPUT_HANDLER(user_data);

  gtk_im_context_focus_out(self->im_context);
}

// Shows the input method.
static void show(gpointer user_data) {
  FlTextInputHandler* self = FL_TEXT_INPUT_HANDLER(user_data);

  if (self->input_type == FL_TEXT_INPUT_TYPE_NONE) {
    hide(user_data);
    return;
  }

  gtk_im_context_focus_in(self->im_context);
}

// Updates the editing state from Flutter.
static void set_editing_state(const gchar* text,
                              int64_t selection_base,
                              int64_t selection_extent,
                              int64_t composing_base,
                              int64_t composing_extent,
                              gpointer user_data) {
  FlTextInputHandler* self = FL_TEXT_INPUT_HANDLER(user_data);

  self->text_model->SetText(text);

  // Flutter uses -1/-1 for invalid; translate that to 0/0 for the model.
  if (selection_base == -1 && selection_extent == -1) {
    selection_base = selection_extent = 0;
  }

  self->text_model->SetText(text);
  self->text_model->SetSelection(
      flutter::TextRange(selection_base, selection_extent));

  if (composing_base == -1 && composing_extent == -1) {
    self->text_model->EndComposing();
  } else {
    size_t composing_start = std::min(composing_base, composing_extent);
    size_t cursor_offset = selection_base - composing_start;
    self->text_model->SetComposingRange(
        flutter::TextRange(composing_base, composing_extent), cursor_offset);
  }
}

// Called when the input method client is complete.
static void clear_client(gpointer user_data) {
  FlTextInputHandler* self = FL_TEXT_INPUT_HANDLER(user_data);
  self->client_id = kClientIdUnset;
}

// Update the IM cursor position.
//
// As text is input by the user, the framework sends two streams of updates
// over the text input channel: updates to the composing rect (cursor rect
// when not in IME composing mode) and updates to the matrix transform from
// local coordinates to Flutter root coordinates. This function is called
// after each of these updates. It transforms the composing rect to GDK window
// coordinates and notifies GTK of the updated cursor position.
static void update_im_cursor_position(FlTextInputHandler* self) {
  // Skip update if not composing to avoid setting to position 0.
  if (!self->text_model->composing()) {
    return;
  }

  // Transform the x, y positions of the cursor from local coordinates to
  // Flutter view coordinates.
  gint x = self->composing_rect.x * self->editabletext_transform[0][0] +
           self->composing_rect.y * self->editabletext_transform[1][0] +
           self->editabletext_transform[3][0] + self->composing_rect.width;
  gint y = self->composing_rect.x * self->editabletext_transform[0][1] +
           self->composing_rect.y * self->editabletext_transform[1][1] +
           self->editabletext_transform[3][1] + self->composing_rect.height;

  // Transform from Flutter view coordinates to GTK window coordinates.
  GdkRectangle preedit_rect = {};
  gtk_widget_translate_coordinates(self->widget,
                                   gtk_widget_get_toplevel(self->widget), x, y,
                                   &preedit_rect.x, &preedit_rect.y);

  // Set the cursor location in window coordinates so that GTK can position
  // any system input method windows.
  gtk_im_context_set_cursor_location(self->im_context, &preedit_rect);
}

// Handles updates to the EditableText size and position from the framework.
//
// On changes to the size or position of the RenderObject underlying the
// EditableText, this update may be triggered. It provides an updated size and
// transform from the local coordinate system of the EditableText to root
// Flutter coordinate system.
static void set_editable_size_and_transform(double* transform,
                                            gpointer user_data) {
  FlTextInputHandler* self = FL_TEXT_INPUT_HANDLER(user_data);

  for (size_t i = 0; i < 16; i++) {
    self->editabletext_transform[i / 4][i % 4] = transform[i];
  }
  update_im_cursor_position(self);
}

// Handles updates to the composing rect from the framework.
//
// On changes to the state of the EditableText in the framework, this update
// may be triggered. It provides an updated rect for the composing region in
// local coordinates of the EditableText. In the case where there is no
// composing region, the cursor rect is sent.
static void set_marked_text_rect(double x,
                                 double y,
                                 double width,
                                 double height,
                                 gpointer user_data) {
  FlTextInputHandler* self = FL_TEXT_INPUT_HANDLER(user_data);

  self->composing_rect.x = x;
  self->composing_rect.y = y;
  self->composing_rect.width = width;
  self->composing_rect.height = height;
  update_im_cursor_position(self);
}

// Disposes of an FlTextInputHandler.
static void fl_text_input_handler_dispose(GObject* object) {
  FlTextInputHandler* self = FL_TEXT_INPUT_HANDLER(object);

  g_cancellable_cancel(self->cancellable);

  g_clear_object(&self->channel);
  g_clear_pointer(&self->input_action, g_free);
  g_clear_object(&self->im_context);
  if (self->text_model != nullptr) {
    delete self->text_model;
    self->text_model = nullptr;
  }
  g_clear_object(&self->cancellable);

  G_OBJECT_CLASS(fl_text_input_handler_parent_class)->dispose(object);
}

// Initializes the FlTextInputHandler class.
static void fl_text_input_handler_class_init(FlTextInputHandlerClass* klass) {
  G_OBJECT_CLASS(klass)->dispose = fl_text_input_handler_dispose;
}

// Initializes an instance of the FlTextInputHandler class.
static void fl_text_input_handler_init(FlTextInputHandler* self) {
  self->client_id = kClientIdUnset;
  self->input_type = FL_TEXT_INPUT_TYPE_TEXT;
  self->text_model = new flutter::TextInputModel();
  self->cancellable = g_cancellable_new();
}

static FlTextInputChannelVTable text_input_vtable = {
    .set_client = set_client,
    .hide = hide,
    .show = show,
    .set_editing_state = set_editing_state,
    .clear_client = clear_client,
    .set_editable_size_and_transform = set_editable_size_and_transform,
    .set_marked_text_rect = set_marked_text_rect,
};

FlTextInputHandler* fl_text_input_handler_new(FlBinaryMessenger* messenger) {
  g_return_val_if_fail(FL_IS_BINARY_MESSENGER(messenger), nullptr);

  FlTextInputHandler* self = FL_TEXT_INPUT_HANDLER(
      g_object_new(fl_text_input_handler_get_type(), nullptr));

  self->channel =
      fl_text_input_channel_new(messenger, &text_input_vtable, self);

  self->im_context = GTK_IM_CONTEXT(gtk_im_multicontext_new());

  // On Wayland, this call sets up the input method so it can be enabled
  // immediately when required. Without it, on-screen keyboard's don't come up
  // the first time a text field is focused.
  gtk_im_context_focus_out(self->im_context);

  g_signal_connect_object(self->im_context, "preedit-start",
                          G_CALLBACK(im_preedit_start_cb), self,
                          G_CONNECT_SWAPPED);
  g_signal_connect_object(self->im_context, "preedit-end",
                          G_CALLBACK(im_preedit_end_cb), self,
                          G_CONNECT_SWAPPED);
  g_signal_connect_object(self->im_context, "preedit-changed",
                          G_CALLBACK(im_preedit_changed_cb), self,
                          G_CONNECT_SWAPPED);
  g_signal_connect_object(self->im_context, "commit", G_CALLBACK(im_commit_cb),
                          self, G_CONNECT_SWAPPED);
  g_signal_connect_object(self->im_context, "retrieve-surrounding",
                          G_CALLBACK(im_retrieve_surrounding_cb), self,
                          G_CONNECT_SWAPPED);
  g_signal_connect_object(self->im_context, "delete-surrounding",
                          G_CALLBACK(im_delete_surrounding_cb), self,
                          G_CONNECT_SWAPPED);

  return self;
}

GtkIMContext* fl_text_input_handler_get_im_context(FlTextInputHandler* self) {
  g_return_val_if_fail(FL_IS_TEXT_INPUT_HANDLER(self), nullptr);
  return self->im_context;
}

void fl_text_input_handler_set_widget(FlTextInputHandler* self,
                                      GtkWidget* widget) {
  g_return_if_fail(FL_IS_TEXT_INPUT_HANDLER(self));
  self->widget = widget;
  gtk_im_context_set_client_window(self->im_context,
                                   gtk_widget_get_window(self->widget));
}

GtkWidget* fl_text_input_handler_get_widget(FlTextInputHandler* self) {
  g_return_val_if_fail(FL_IS_TEXT_INPUT_HANDLER(self), nullptr);
  return self->widget;
}

gboolean fl_text_input_handler_filter_keypress(FlTextInputHandler* self,
                                               FlKeyEvent* event) {
  g_return_val_if_fail(FL_IS_TEXT_INPUT_HANDLER(self), FALSE);

  if (self->client_id == kClientIdUnset) {
    return FALSE;
  }

  if (gtk_im_context_filter_keypress(
          self->im_context,
          reinterpret_cast<GdkEventKey*>(fl_key_event_get_origin(event)))) {
    return TRUE;
  }

  std::string text_before_change = self->text_model->GetText();
  flutter::TextRange selection_before_change = self->text_model->selection();
  std::string text = self->text_model->GetText();

  // Handle the enter/return key.
  gboolean do_action = FALSE;
  // Handle navigation keys.
  gboolean changed = FALSE;
  if (fl_key_event_get_is_press(event)) {
    switch (fl_key_event_get_keyval(event)) {
      case GDK_KEY_End:
      case GDK_KEY_KP_End:
        if (fl_key_event_get_state(event) & GDK_SHIFT_MASK) {
          changed = self->text_model->SelectToEnd();
        } else {
          changed = self->text_model->MoveCursorToEnd();
        }
        break;
      case GDK_KEY_Return:
      case GDK_KEY_KP_Enter:
      case GDK_KEY_ISO_Enter:
        if (self->input_type == FL_TEXT_INPUT_TYPE_MULTILINE &&
            strcmp(self->input_action, kNewlineInputAction) == 0) {
          self->text_model->AddCodePoint('\n');
          text = "\n";
          changed = TRUE;
        }
        do_action = TRUE;
        break;
      case GDK_KEY_Home:
      case GDK_KEY_KP_Home:
        if (fl_key_event_get_state(event) & GDK_SHIFT_MASK) {
          changed = self->text_model->SelectToBeginning();
        } else {
          changed = self->text_model->MoveCursorToBeginning();
        }
        break;
      case GDK_KEY_BackSpace:
      case GDK_KEY_Delete:
      case GDK_KEY_KP_Delete:
      case GDK_KEY_Left:
      case GDK_KEY_KP_Left:
      case GDK_KEY_Right:
      case GDK_KEY_KP_Right:
        // Already handled inside the framework in RenderEditable.
        break;
    }
  }

  if (changed) {
    if (self->enable_delta_model) {
      flutter::TextEditingDelta delta = flutter::TextEditingDelta(
          text_before_change, selection_before_change, text);
      update_editing_state_with_delta(self, &delta);
    } else {
      update_editing_state(self);
    }
  }
  if (do_action) {
    perform_action(self);
  }

  return changed;
}
