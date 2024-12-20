// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/testing/mock_text_input_view_delegate.h"

using namespace flutter::testing;

G_DECLARE_FINAL_TYPE(FlMockTextInputViewDelegate,
                     fl_mock_text_input_view_delegate,
                     FL,
                     MOCK_TEXT_INPUT_VIEW_DELEGATE,
                     GObject)

struct _FlMockTextInputViewDelegate {
  GObject parent_instance;
  MockTextInputViewDelegate* mock;
};

static FlTextInputViewDelegate* fl_mock_text_input_view_delegate_new(
    MockTextInputViewDelegate* mock) {
  FlMockTextInputViewDelegate* self = FL_MOCK_TEXT_INPUT_VIEW_DELEGATE(
      g_object_new(fl_mock_text_input_view_delegate_get_type(), nullptr));
  self->mock = mock;
  return FL_TEXT_INPUT_VIEW_DELEGATE(self);
}

MockTextInputViewDelegate::MockTextInputViewDelegate()
    : instance_(fl_mock_text_input_view_delegate_new(this)) {}

MockTextInputViewDelegate::~MockTextInputViewDelegate() {
  if (FL_IS_TEXT_INPUT_VIEW_DELEGATE(instance_)) {
    g_clear_object(&instance_);
  }
}

MockTextInputViewDelegate::operator FlTextInputViewDelegate*() {
  return instance_;
}

static void fl_mock_text_input_view_delegate_iface_init(
    FlTextInputViewDelegateInterface* iface);

G_DEFINE_TYPE_WITH_CODE(
    FlMockTextInputViewDelegate,
    fl_mock_text_input_view_delegate,
    G_TYPE_OBJECT,
    G_IMPLEMENT_INTERFACE(fl_text_input_view_delegate_get_type(),
                          fl_mock_text_input_view_delegate_iface_init))

static void fl_mock_text_input_view_delegate_class_init(
    FlMockTextInputViewDelegateClass* klass) {}

static void fl_mock_text_input_view_delegate_translate_coordinates(
    FlTextInputViewDelegate* view_delegate,
    gint view_x,
    gint view_y,
    gint* window_x,
    gint* window_y) {
  g_return_if_fail(FL_IS_MOCK_TEXT_INPUT_VIEW_DELEGATE(view_delegate));
  FlMockTextInputViewDelegate* self =
      FL_MOCK_TEXT_INPUT_VIEW_DELEGATE(view_delegate);
  self->mock->fl_text_input_view_delegate_translate_coordinates(
      view_delegate, view_x, view_y, window_x, window_y);
}

static void fl_mock_text_input_view_delegate_iface_init(
    FlTextInputViewDelegateInterface* iface) {
  iface->translate_coordinates =
      fl_mock_text_input_view_delegate_translate_coordinates;
}

static void fl_mock_text_input_view_delegate_init(
    FlMockTextInputViewDelegate* self) {}
