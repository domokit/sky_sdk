// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/fl_key_embedder_responder.h"

#include "gtest/gtest.h"

#include "flutter/shell/platform/embedder/test_utils/key_codes.g.h"
#include "flutter/shell/platform/embedder/test_utils/proc_table_replacement.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_engine.h"
#include "flutter/shell/platform/linux/testing/fl_test.h"

namespace {
constexpr gboolean kRelease = FALSE;
constexpr gboolean kPress = TRUE;

constexpr guint16 kKeyCodeDigit1 = 0x0au;
constexpr guint16 kKeyCodeKeyA = 0x26u;
constexpr guint16 kKeyCodeShiftLeft = 0x32u;
constexpr guint16 kKeyCodeShiftRight = 0x3Eu;
constexpr guint16 kKeyCodeAltLeft = 0x40u;
constexpr guint16 kKeyCodeAltRight = 0x6Cu;
constexpr guint16 kKeyCodeNumpad1 = 0x57u;
constexpr guint16 kKeyCodeNumLock = 0x4Du;
constexpr guint16 kKeyCodeCapsLock = 0x42u;
constexpr guint16 kKeyCodeControlLeft = 0x25u;
constexpr guint16 kKeyCodeControlRight = 0x69u;

using namespace ::flutter::testing::keycodes;
}  // namespace

static void clear_records(GPtrArray* array) {
  g_ptr_array_remove_range(array, 0, array->len);
}

G_DECLARE_FINAL_TYPE(FlKeyEmbedderCallRecord,
                     fl_key_embedder_call_record,
                     FL,
                     KEY_EMBEDDER_CALL_RECORD,
                     GObject);

struct _FlKeyEmbedderCallRecord {
  GObject parent_instance;

  FlutterKeyEvent* event;
  FlutterKeyEventCallback callback;
  gpointer user_data;
};

G_DEFINE_TYPE(FlKeyEmbedderCallRecord,
              fl_key_embedder_call_record,
              G_TYPE_OBJECT)

static void fl_key_embedder_call_record_init(FlKeyEmbedderCallRecord* self) {}

// Dispose method for FlKeyEmbedderCallRecord.
static void fl_key_embedder_call_record_dispose(GObject* object) {
  g_return_if_fail(FL_IS_KEY_EMBEDDER_CALL_RECORD(object));

  FlKeyEmbedderCallRecord* self = FL_KEY_EMBEDDER_CALL_RECORD(object);
  if (self->event != nullptr) {
    g_free(const_cast<char*>(self->event->character));
    g_free(self->event);
  }
  G_OBJECT_CLASS(fl_key_embedder_call_record_parent_class)->dispose(object);
}

// Class Initialization method for FlKeyEmbedderCallRecord class.
static void fl_key_embedder_call_record_class_init(
    FlKeyEmbedderCallRecordClass* klass) {
  G_OBJECT_CLASS(klass)->dispose = fl_key_embedder_call_record_dispose;
}

static FlKeyEmbedderCallRecord* fl_key_embedder_call_record_new(
    const FlutterKeyEvent* event,
    FlutterKeyEventCallback callback,
    gpointer user_data) {
  g_return_val_if_fail(event != nullptr, nullptr);

  FlKeyEmbedderCallRecord* self = FL_KEY_EMBEDDER_CALL_RECORD(
      g_object_new(fl_key_embedder_call_record_get_type(), nullptr));

  FlutterKeyEvent* clone_event = g_new(FlutterKeyEvent, 1);
  *clone_event = *event;
  if (event->character != nullptr) {
    size_t character_length = strlen(event->character);
    char* clone_character = g_new(char, character_length + 1);
    strncpy(clone_character, event->character, character_length + 1);
    clone_event->character = clone_character;
  }
  self->event = clone_event;
  self->callback = callback;
  self->user_data = user_data;

  return self;
}

static void invoke_record_callback(FlKeyEmbedderCallRecord* record,
                                   bool expected_handled) {
  g_return_if_fail(record->callback != nullptr);
  record->callback(expected_handled, record->user_data);
}

// Basic key presses
TEST(FlKeyEmbedderResponderTest, SendKeyEvent) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);
  EXPECT_TRUE(fl_engine_start(engine, nullptr));

  g_autoptr(FlKeyEmbedderResponder) responder =
      fl_key_embedder_responder_new(engine);

  g_autoptr(GPtrArray) call_records =
      g_ptr_array_new_with_free_func(g_object_unref);
  fl_engine_get_embedder_api(engine)->SendKeyEvent = MOCK_ENGINE_PROC(
      SendKeyEvent,
      ([&call_records](auto engine, const FlutterKeyEvent* event,
                       FlutterKeyEventCallback callback, void* user_data) {
        g_ptr_array_add(call_records, fl_key_embedder_call_record_new(
                                          event, callback, user_data));
        return kSuccess;
      }));

  // On a QWERTY keyboard, press key Q (physically key A), and release.
  // Key down
  g_autoptr(FlKeyEvent) event1 =
      fl_key_event_new(12345, kPress, kKeyCodeKeyA, GDK_KEY_a,
                       static_cast<GdkModifierType>(0), 0);
  g_autoptr(GMainLoop) loop1 = g_main_loop_new(nullptr, 0);
  fl_key_embedder_responder_handle_event(
      responder, event1, 0, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_embedder_responder_handle_event_finish(
            FL_KEY_EMBEDDER_RESPONDER(object), result, &handled, nullptr));
        EXPECT_EQ(handled, TRUE);

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop1);

  EXPECT_EQ(call_records->len, 1u);
  FlKeyEmbedderCallRecord* record =
      FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 0));
  EXPECT_EQ(record->event->struct_size, sizeof(FlutterKeyEvent));
  EXPECT_EQ(record->event->timestamp, 12345000);
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(record->event->physical, kPhysicalKeyA);
  EXPECT_EQ(record->event->logical, kLogicalKeyA);
  EXPECT_STREQ(record->event->character, "a");
  EXPECT_EQ(record->event->synthesized, false);

  invoke_record_callback(record, TRUE);
  g_main_loop_run(loop1);
  clear_records(call_records);

  // Key up
  g_autoptr(FlKeyEvent) event2 =
      fl_key_event_new(12346, kRelease, kKeyCodeKeyA, GDK_KEY_a,
                       static_cast<GdkModifierType>(0), 0);
  g_autoptr(GMainLoop) loop2 = g_main_loop_new(nullptr, 0);
  fl_key_embedder_responder_handle_event(
      responder, event2, 0, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_embedder_responder_handle_event_finish(
            FL_KEY_EMBEDDER_RESPONDER(object), result, &handled, nullptr));
        EXPECT_EQ(handled, FALSE);

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop2);

  EXPECT_EQ(call_records->len, 1u);
  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 0));
  EXPECT_EQ(record->event->struct_size, sizeof(FlutterKeyEvent));
  EXPECT_EQ(record->event->timestamp, 12346000);
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(record->event->physical, kPhysicalKeyA);
  EXPECT_EQ(record->event->logical, kLogicalKeyA);
  EXPECT_STREQ(record->event->character, nullptr);
  EXPECT_EQ(record->event->synthesized, false);

  invoke_record_callback(record, FALSE);
  g_main_loop_run(loop2);
  clear_records(call_records);

  // On an AZERTY keyboard, press key Q (physically key A), and release.
  // Key down
  g_autoptr(FlKeyEvent) event3 =
      fl_key_event_new(12347, kPress, kKeyCodeKeyA, GDK_KEY_q,
                       static_cast<GdkModifierType>(0), 0);
  g_autoptr(GMainLoop) loop3 = g_main_loop_new(nullptr, 0);
  fl_key_embedder_responder_handle_event(
      responder, event3, 0, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_embedder_responder_handle_event_finish(
            FL_KEY_EMBEDDER_RESPONDER(object), result, &handled, nullptr));
        EXPECT_EQ(handled, TRUE);

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop3);

  EXPECT_EQ(call_records->len, 1u);
  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 0));
  EXPECT_EQ(record->event->struct_size, sizeof(FlutterKeyEvent));
  EXPECT_EQ(record->event->timestamp, 12347000);
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(record->event->physical, kPhysicalKeyA);
  EXPECT_EQ(record->event->logical, kLogicalKeyQ);
  EXPECT_STREQ(record->event->character, "q");
  EXPECT_EQ(record->event->synthesized, false);

  invoke_record_callback(record, TRUE);
  g_main_loop_run(loop3);
  clear_records(call_records);

  // Key up
  g_autoptr(FlKeyEvent) event4 =
      fl_key_event_new(12348, kRelease, kKeyCodeKeyA, GDK_KEY_q,
                       static_cast<GdkModifierType>(0), 0);
  g_autoptr(GMainLoop) loop4 = g_main_loop_new(nullptr, 0);
  fl_key_embedder_responder_handle_event(
      responder, event4, 0, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_embedder_responder_handle_event_finish(
            FL_KEY_EMBEDDER_RESPONDER(object), result, &handled, nullptr));
        EXPECT_EQ(handled, FALSE);

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop4);

  EXPECT_EQ(call_records->len, 1u);
  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 0));
  EXPECT_EQ(record->event->struct_size, sizeof(FlutterKeyEvent));
  EXPECT_EQ(record->event->timestamp, 12348000);
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(record->event->physical, kPhysicalKeyA);
  EXPECT_EQ(record->event->logical, kLogicalKeyQ);
  EXPECT_STREQ(record->event->character, nullptr);
  EXPECT_EQ(record->event->synthesized, false);

  invoke_record_callback(record, FALSE);
  g_main_loop_run(loop4);
}

// Basic key presses, but uses the specified logical key if it is not 0.
TEST(FlKeyEmbedderResponderTest, UsesSpecifiedLogicalKey) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);
  EXPECT_TRUE(fl_engine_start(engine, nullptr));

  g_autoptr(FlKeyEmbedderResponder) responder =
      fl_key_embedder_responder_new(engine);

  g_autoptr(GPtrArray) call_records =
      g_ptr_array_new_with_free_func(g_object_unref);
  fl_engine_get_embedder_api(engine)->SendKeyEvent = MOCK_ENGINE_PROC(
      SendKeyEvent,
      ([&call_records](auto engine, const FlutterKeyEvent* event,
                       FlutterKeyEventCallback callback, void* user_data) {
        g_ptr_array_add(call_records, fl_key_embedder_call_record_new(
                                          event, callback, user_data));
        return kSuccess;
      }));

  // On an AZERTY keyboard, press physical key 1, and release.
  // Key down
  g_autoptr(FlKeyEvent) event =
      fl_key_event_new(12345, kPress, kKeyCodeDigit1, GDK_KEY_ampersand,
                       static_cast<GdkModifierType>(0), 0);
  g_autoptr(GMainLoop) loop = g_main_loop_new(nullptr, 0);
  fl_key_embedder_responder_handle_event(
      responder, event, kLogicalDigit1, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_embedder_responder_handle_event_finish(
            FL_KEY_EMBEDDER_RESPONDER(object), result, &handled, nullptr));
        EXPECT_EQ(handled, TRUE);

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop);

  EXPECT_EQ(call_records->len, 1u);
  FlKeyEmbedderCallRecord* record =
      FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 0));
  EXPECT_EQ(record->event->struct_size, sizeof(FlutterKeyEvent));
  EXPECT_EQ(record->event->timestamp, 12345000);
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(record->event->physical, kPhysicalDigit1);
  EXPECT_EQ(record->event->logical, kLogicalDigit1);
  EXPECT_STREQ(record->event->character, "&");
  EXPECT_EQ(record->event->synthesized, false);

  invoke_record_callback(record, TRUE);
  g_main_loop_run(loop);
}

// Press Shift, key A, then release Shift, key A.
TEST(FlKeyEmbedderResponderTest, PressShiftDuringLetterKeyTap) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);
  EXPECT_TRUE(fl_engine_start(engine, nullptr));

  g_autoptr(FlKeyEmbedderResponder) responder =
      fl_key_embedder_responder_new(engine);

  g_autoptr(GPtrArray) call_records =
      g_ptr_array_new_with_free_func(g_object_unref);
  fl_engine_get_embedder_api(engine)->SendKeyEvent = MOCK_ENGINE_PROC(
      SendKeyEvent,
      ([&call_records](auto engine, const FlutterKeyEvent* event,
                       FlutterKeyEventCallback callback, void* user_data) {
        g_ptr_array_add(call_records, fl_key_embedder_call_record_new(
                                          event, callback, user_data));
        return kSuccess;
      }));

  // Press shift right
  g_autoptr(FlKeyEvent) event1 =
      fl_key_event_new(101, kPress, kKeyCodeShiftRight, GDK_KEY_Shift_R,
                       static_cast<GdkModifierType>(0), 0);
  g_autoptr(GMainLoop) loop1 = g_main_loop_new(nullptr, 0);
  fl_key_embedder_responder_handle_event(
      responder, event1, 0, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_embedder_responder_handle_event_finish(
            FL_KEY_EMBEDDER_RESPONDER(object), result, &handled, nullptr));
        EXPECT_EQ(handled, TRUE);

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop1);

  EXPECT_EQ(call_records->len, 1u);
  FlKeyEmbedderCallRecord* record =
      FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 0));
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(record->event->physical, kPhysicalShiftRight);
  EXPECT_EQ(record->event->logical, kLogicalShiftRight);
  EXPECT_STREQ(record->event->character, nullptr);
  EXPECT_EQ(record->event->synthesized, false);

  invoke_record_callback(record, TRUE);
  g_main_loop_run(loop1);
  clear_records(call_records);

  // Press key A
  g_autoptr(FlKeyEvent) event2 =
      fl_key_event_new(102, kPress, kKeyCodeKeyA, GDK_KEY_A, GDK_SHIFT_MASK, 0);
  g_autoptr(GMainLoop) loop2 = g_main_loop_new(nullptr, 0);
  fl_key_embedder_responder_handle_event(
      responder, event2, 0, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_embedder_responder_handle_event_finish(
            FL_KEY_EMBEDDER_RESPONDER(object), result, &handled, nullptr));
        EXPECT_EQ(handled, TRUE);

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop2);

  EXPECT_EQ(call_records->len, 1u);
  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 0));
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(record->event->physical, kPhysicalKeyA);
  EXPECT_EQ(record->event->logical, kLogicalKeyA);
  EXPECT_STREQ(record->event->character, "A");
  EXPECT_EQ(record->event->synthesized, false);

  invoke_record_callback(record, TRUE);
  g_main_loop_run(loop2);
  clear_records(call_records);

  // Release shift right
  g_autoptr(FlKeyEvent) event3 = fl_key_event_new(
      103, kRelease, kKeyCodeShiftRight, GDK_KEY_Shift_R, GDK_SHIFT_MASK, 0);
  g_autoptr(GMainLoop) loop3 = g_main_loop_new(nullptr, 0);
  fl_key_embedder_responder_handle_event(
      responder, event3, 0, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_embedder_responder_handle_event_finish(
            FL_KEY_EMBEDDER_RESPONDER(object), result, &handled, nullptr));
        EXPECT_EQ(handled, TRUE);

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop3);

  EXPECT_EQ(call_records->len, 1u);
  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 0));
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(record->event->physical, kPhysicalShiftRight);
  EXPECT_EQ(record->event->logical, kLogicalShiftRight);
  EXPECT_STREQ(record->event->character, nullptr);
  EXPECT_EQ(record->event->synthesized, false);

  invoke_record_callback(record, TRUE);
  g_main_loop_run(loop3);
  clear_records(call_records);

  // Release key A
  g_autoptr(FlKeyEvent) event4 =
      fl_key_event_new(104, kRelease, kKeyCodeKeyA, GDK_KEY_A,
                       static_cast<GdkModifierType>(0), 0);
  g_autoptr(GMainLoop) loop4 = g_main_loop_new(nullptr, 0);
  fl_key_embedder_responder_handle_event(
      responder, event4, 0, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_embedder_responder_handle_event_finish(
            FL_KEY_EMBEDDER_RESPONDER(object), result, &handled, nullptr));
        EXPECT_EQ(handled, TRUE);

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop4);

  EXPECT_EQ(call_records->len, 1u);
  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 0));
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(record->event->physical, kPhysicalKeyA);
  EXPECT_EQ(record->event->logical, kLogicalKeyA);
  EXPECT_STREQ(record->event->character, nullptr);
  EXPECT_EQ(record->event->synthesized, false);

  invoke_record_callback(record, TRUE);
  g_main_loop_run(loop4);
}

// Press or release Numpad 1 between presses/releases of NumLock.
//
// This tests interaction between lock keys and non-lock keys in cases that do
// not have events missed.
//
// This also tests the result of Numpad keys across NumLock taps, which is
// test-worthy because the keyval for the numpad key will change before and
// after the NumLock tap, which should not alter the resulting logical key.
TEST(FlKeyEmbedderResponderTest, TapNumPadKeysBetweenNumLockEvents) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);
  EXPECT_TRUE(fl_engine_start(engine, nullptr));

  g_autoptr(FlKeyEmbedderResponder) responder =
      fl_key_embedder_responder_new(engine);

  g_autoptr(GPtrArray) call_records =
      g_ptr_array_new_with_free_func(g_object_unref);
  fl_engine_get_embedder_api(engine)->SendKeyEvent = MOCK_ENGINE_PROC(
      SendKeyEvent,
      ([&call_records](auto engine, const FlutterKeyEvent* event,
                       FlutterKeyEventCallback callback, void* user_data) {
        g_ptr_array_add(call_records, fl_key_embedder_call_record_new(
                                          event, callback, user_data));
        return kSuccess;
      }));

  // Press Numpad 1 (stage 0)
  g_autoptr(FlKeyEvent) event1 =
      fl_key_event_new(101, kPress, kKeyCodeNumpad1, GDK_KEY_KP_End,
                       static_cast<GdkModifierType>(0), 0);
  g_autoptr(GMainLoop) loop1 = g_main_loop_new(nullptr, 0);
  fl_key_embedder_responder_handle_event(
      responder, event1, 0, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_embedder_responder_handle_event_finish(
            FL_KEY_EMBEDDER_RESPONDER(object), result, &handled, nullptr));
        EXPECT_EQ(handled, TRUE);

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop1);

  EXPECT_EQ(call_records->len, 1u);
  FlKeyEmbedderCallRecord* record =
      FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 0));
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(record->event->physical, kPhysicalNumpad1);
  EXPECT_EQ(record->event->logical, kLogicalNumpad1);
  EXPECT_STREQ(record->event->character, nullptr);  // TODO(chrome-bot):
  EXPECT_EQ(record->event->synthesized, false);

  invoke_record_callback(record, TRUE);
  g_main_loop_run(loop1);
  clear_records(call_records);

  // Press NumLock (stage 0 -> 1)
  g_autoptr(FlKeyEvent) event2 =
      fl_key_event_new(102, kPress, kKeyCodeNumLock, GDK_KEY_Num_Lock,
                       static_cast<GdkModifierType>(0), 0);
  g_autoptr(GMainLoop) loop2 = g_main_loop_new(nullptr, 0);
  fl_key_embedder_responder_handle_event(
      responder, event2, 0, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_embedder_responder_handle_event_finish(
            FL_KEY_EMBEDDER_RESPONDER(object), result, &handled, nullptr));
        EXPECT_EQ(handled, TRUE);

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop2);

  EXPECT_EQ(call_records->len, 1u);
  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 0));
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(record->event->physical, kPhysicalNumLock);
  EXPECT_EQ(record->event->logical, kLogicalNumLock);
  EXPECT_STREQ(record->event->character, nullptr);
  EXPECT_EQ(record->event->synthesized, false);

  invoke_record_callback(record, TRUE);
  g_main_loop_run(loop2);
  clear_records(call_records);

  // Release numpad 1 (stage 1)
  g_autoptr(FlKeyEvent) event3 = fl_key_event_new(
      104, kRelease, kKeyCodeNumpad1, GDK_KEY_KP_1, GDK_MOD2_MASK, 0);
  g_autoptr(GMainLoop) loop3 = g_main_loop_new(nullptr, 0);
  fl_key_embedder_responder_handle_event(
      responder, event3, 0, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_embedder_responder_handle_event_finish(
            FL_KEY_EMBEDDER_RESPONDER(object), result, &handled, nullptr));
        EXPECT_EQ(handled, TRUE);

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop3);

  EXPECT_EQ(call_records->len, 1u);
  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 0));
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(record->event->physical, kPhysicalNumpad1);
  EXPECT_EQ(record->event->logical, kLogicalNumpad1);
  EXPECT_STREQ(record->event->character, nullptr);
  EXPECT_EQ(record->event->synthesized, false);

  invoke_record_callback(record, TRUE);
  g_main_loop_run(loop3);
  clear_records(call_records);

  // Release NumLock (stage 1 -> 2)
  g_autoptr(FlKeyEvent) event4 = fl_key_event_new(
      103, kRelease, kKeyCodeNumLock, GDK_KEY_Num_Lock, GDK_MOD2_MASK, 0);
  g_autoptr(GMainLoop) loop4 = g_main_loop_new(nullptr, 0);
  fl_key_embedder_responder_handle_event(
      responder, event4, 0, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_embedder_responder_handle_event_finish(
            FL_KEY_EMBEDDER_RESPONDER(object), result, &handled, nullptr));
        EXPECT_EQ(handled, TRUE);

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop4);

  EXPECT_EQ(call_records->len, 1u);
  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 0));
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(record->event->physical, kPhysicalNumLock);
  EXPECT_EQ(record->event->logical, kLogicalNumLock);
  EXPECT_STREQ(record->event->character, nullptr);
  EXPECT_EQ(record->event->synthesized, false);

  invoke_record_callback(record, TRUE);
  g_main_loop_run(loop4);
  clear_records(call_records);

  // Press Numpad 1 (stage 2)
  g_autoptr(FlKeyEvent) event5 = fl_key_event_new(
      101, kPress, kKeyCodeNumpad1, GDK_KEY_KP_End, GDK_MOD2_MASK, 0);
  g_autoptr(GMainLoop) loop5 = g_main_loop_new(nullptr, 0);
  fl_key_embedder_responder_handle_event(
      responder, event5, 0, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_embedder_responder_handle_event_finish(
            FL_KEY_EMBEDDER_RESPONDER(object), result, &handled, nullptr));
        EXPECT_EQ(handled, TRUE);

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop5);

  EXPECT_EQ(call_records->len, 1u);
  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 0));
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(record->event->physical, kPhysicalNumpad1);
  EXPECT_EQ(record->event->logical, kLogicalNumpad1);
  EXPECT_STREQ(record->event->character, nullptr);  // TODO(chrome-bot):
  EXPECT_EQ(record->event->synthesized, false);

  invoke_record_callback(record, TRUE);
  g_main_loop_run(loop5);
  clear_records(call_records);

  // Press NumLock (stage 2 -> 3)
  g_autoptr(FlKeyEvent) event6 = fl_key_event_new(
      102, kPress, kKeyCodeNumLock, GDK_KEY_Num_Lock, GDK_MOD2_MASK, 0);
  g_autoptr(GMainLoop) loop6 = g_main_loop_new(nullptr, 0);
  fl_key_embedder_responder_handle_event(
      responder, event6, 0, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_embedder_responder_handle_event_finish(
            FL_KEY_EMBEDDER_RESPONDER(object), result, &handled, nullptr));
        EXPECT_EQ(handled, TRUE);

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop6);

  EXPECT_EQ(call_records->len, 1u);
  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 0));
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(record->event->physical, kPhysicalNumLock);
  EXPECT_EQ(record->event->logical, kLogicalNumLock);
  EXPECT_STREQ(record->event->character, nullptr);
  EXPECT_EQ(record->event->synthesized, false);

  invoke_record_callback(record, TRUE);
  g_main_loop_run(loop6);
  clear_records(call_records);

  // Release numpad 1 (stage 3)
  g_autoptr(FlKeyEvent) event7 = fl_key_event_new(
      104, kRelease, kKeyCodeNumpad1, GDK_KEY_KP_1, GDK_MOD2_MASK, 0);
  g_autoptr(GMainLoop) loop7 = g_main_loop_new(nullptr, 0);
  fl_key_embedder_responder_handle_event(
      responder, event7, 0, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_embedder_responder_handle_event_finish(
            FL_KEY_EMBEDDER_RESPONDER(object), result, &handled, nullptr));
        EXPECT_EQ(handled, TRUE);

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop7);

  EXPECT_EQ(call_records->len, 1u);
  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 0));
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(record->event->physical, kPhysicalNumpad1);
  EXPECT_EQ(record->event->logical, kLogicalNumpad1);
  EXPECT_STREQ(record->event->character, nullptr);
  EXPECT_EQ(record->event->synthesized, false);

  invoke_record_callback(record, TRUE);
  g_main_loop_run(loop7);
  clear_records(call_records);

  // Release NumLock (stage 3 -> 0)
  g_autoptr(FlKeyEvent) event8 = fl_key_event_new(
      103, kRelease, kKeyCodeNumLock, GDK_KEY_Num_Lock, GDK_MOD2_MASK, 0);
  g_autoptr(GMainLoop) loop8 = g_main_loop_new(nullptr, 0);
  fl_key_embedder_responder_handle_event(
      responder, event8, 0, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_embedder_responder_handle_event_finish(
            FL_KEY_EMBEDDER_RESPONDER(object), result, &handled, nullptr));
        EXPECT_EQ(handled, TRUE);

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop8);

  EXPECT_EQ(call_records->len, 1u);
  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 0));
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(record->event->physical, kPhysicalNumLock);
  EXPECT_EQ(record->event->logical, kLogicalNumLock);
  EXPECT_STREQ(record->event->character, nullptr);
  EXPECT_EQ(record->event->synthesized, false);

  invoke_record_callback(record, TRUE);
  g_main_loop_run(loop8);
}

// Press or release digit 1 between presses/releases of Shift.
//
// GTK will change the virtual key during a key tap, and the embedder
// should regularize it.
TEST(FlKeyEmbedderResponderTest, ReleaseShiftKeyBetweenDigitKeyEvents) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);
  EXPECT_TRUE(fl_engine_start(engine, nullptr));

  g_autoptr(FlKeyEmbedderResponder) responder =
      fl_key_embedder_responder_new(engine);

  g_autoptr(GPtrArray) call_records =
      g_ptr_array_new_with_free_func(g_object_unref);
  fl_engine_get_embedder_api(engine)->SendKeyEvent = MOCK_ENGINE_PROC(
      SendKeyEvent,
      ([&call_records](auto engine, const FlutterKeyEvent* event,
                       FlutterKeyEventCallback callback, void* user_data) {
        g_ptr_array_add(call_records, fl_key_embedder_call_record_new(
                                          event, callback, user_data));
        return kSuccess;
      }));

  GdkModifierType state = static_cast<GdkModifierType>(0);

  // Press shift left
  g_autoptr(FlKeyEvent) event1 = fl_key_event_new(
      101, kPress, kKeyCodeShiftLeft, GDK_KEY_Shift_L, state, 0);
  g_autoptr(GMainLoop) loop1 = g_main_loop_new(nullptr, 0);
  fl_key_embedder_responder_handle_event(
      responder, event1, 0, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_embedder_responder_handle_event_finish(
            FL_KEY_EMBEDDER_RESPONDER(object), result, &handled, nullptr));
        EXPECT_EQ(handled, TRUE);

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop1);

  EXPECT_EQ(call_records->len, 1u);
  FlKeyEmbedderCallRecord* record =
      FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 0));
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(record->event->physical, kPhysicalShiftLeft);
  EXPECT_EQ(record->event->logical, kLogicalShiftLeft);
  EXPECT_STREQ(record->event->character, nullptr);
  EXPECT_EQ(record->event->synthesized, false);

  invoke_record_callback(record, TRUE);
  g_main_loop_run(loop1);
  clear_records(call_records);

  state = GDK_SHIFT_MASK;

  // Press digit 1, which is '!' on a US keyboard
  g_autoptr(FlKeyEvent) event2 =
      fl_key_event_new(102, kPress, kKeyCodeDigit1, GDK_KEY_exclam, state, 0);
  g_autoptr(GMainLoop) loop2 = g_main_loop_new(nullptr, 0);
  fl_key_embedder_responder_handle_event(
      responder, event2, 0, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_embedder_responder_handle_event_finish(
            FL_KEY_EMBEDDER_RESPONDER(object), result, &handled, nullptr));
        EXPECT_EQ(handled, TRUE);

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop2);

  EXPECT_EQ(call_records->len, 1u);
  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 0));
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(record->event->physical, kPhysicalDigit1);
  EXPECT_EQ(record->event->logical, kLogicalExclamation);
  EXPECT_STREQ(record->event->character, "!");
  EXPECT_EQ(record->event->synthesized, false);

  invoke_record_callback(record, TRUE);
  g_main_loop_run(loop2);
  clear_records(call_records);

  // Release shift
  g_autoptr(FlKeyEvent) event3 = fl_key_event_new(
      103, kRelease, kKeyCodeShiftLeft, GDK_KEY_Shift_L, state, 0);
  g_autoptr(GMainLoop) loop3 = g_main_loop_new(nullptr, 0);
  fl_key_embedder_responder_handle_event(
      responder, event3, 0, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_embedder_responder_handle_event_finish(
            FL_KEY_EMBEDDER_RESPONDER(object), result, &handled, nullptr));
        EXPECT_EQ(handled, TRUE);

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop3);

  EXPECT_EQ(call_records->len, 1u);
  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 0));
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(record->event->physical, kPhysicalShiftLeft);
  EXPECT_EQ(record->event->logical, kLogicalShiftLeft);
  EXPECT_STREQ(record->event->character, nullptr);
  EXPECT_EQ(record->event->synthesized, false);

  invoke_record_callback(record, TRUE);
  g_main_loop_run(loop3);
  clear_records(call_records);

  state = static_cast<GdkModifierType>(0);

  // Release digit 1, which is "1" because shift has been released.
  g_autoptr(FlKeyEvent) event4 =
      fl_key_event_new(104, kRelease, kKeyCodeDigit1, GDK_KEY_1, state, 0);
  g_autoptr(GMainLoop) loop4 = g_main_loop_new(nullptr, 0);
  fl_key_embedder_responder_handle_event(
      responder, event4, 0, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_embedder_responder_handle_event_finish(
            FL_KEY_EMBEDDER_RESPONDER(object), result, &handled, nullptr));
        EXPECT_EQ(handled, TRUE);

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop4);

  EXPECT_EQ(call_records->len, 1u);
  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 0));
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(record->event->physical, kPhysicalDigit1);
  EXPECT_EQ(record->event->logical, kLogicalExclamation);  // Important
  EXPECT_STREQ(record->event->character, nullptr);
  EXPECT_EQ(record->event->synthesized, false);

  invoke_record_callback(record, TRUE);
  g_main_loop_run(loop4);
}

// Press or release letter key between presses/releases of CapsLock.
//
// This tests interaction between lock keys and non-lock keys in cases that do
// not have events missed.
TEST(FlKeyEmbedderResponderTest, TapLetterKeysBetweenCapsLockEvents) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);
  EXPECT_TRUE(fl_engine_start(engine, nullptr));

  g_autoptr(FlKeyEmbedderResponder) responder =
      fl_key_embedder_responder_new(engine);

  g_autoptr(GPtrArray) call_records =
      g_ptr_array_new_with_free_func(g_object_unref);
  fl_engine_get_embedder_api(engine)->SendKeyEvent = MOCK_ENGINE_PROC(
      SendKeyEvent,
      ([&call_records](auto engine, const FlutterKeyEvent* event,
                       FlutterKeyEventCallback callback, void* user_data) {
        g_ptr_array_add(call_records, fl_key_embedder_call_record_new(
                                          event, callback, user_data));
        return kSuccess;
      }));

  // Press CapsLock (stage 0 -> 1)
  g_autoptr(FlKeyEvent) event1 =
      fl_key_event_new(101, kPress, kKeyCodeCapsLock, GDK_KEY_Caps_Lock,
                       static_cast<GdkModifierType>(0), 0);
  g_autoptr(GMainLoop) loop1 = g_main_loop_new(nullptr, 0);
  fl_key_embedder_responder_handle_event(
      responder, event1, 0, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_embedder_responder_handle_event_finish(
            FL_KEY_EMBEDDER_RESPONDER(object), result, &handled, nullptr));
        EXPECT_EQ(handled, TRUE);

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop1);

  EXPECT_EQ(call_records->len, 1u);
  FlKeyEmbedderCallRecord* record =
      FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 0));
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(record->event->physical, kPhysicalCapsLock);
  EXPECT_EQ(record->event->logical, kLogicalCapsLock);
  EXPECT_STREQ(record->event->character, nullptr);
  EXPECT_EQ(record->event->synthesized, false);

  invoke_record_callback(record, TRUE);
  g_main_loop_run(loop1);
  clear_records(call_records);

  // Press key A (stage 1)
  g_autoptr(FlKeyEvent) event2 =
      fl_key_event_new(102, kPress, kKeyCodeKeyA, GDK_KEY_A, GDK_LOCK_MASK, 0);
  g_autoptr(GMainLoop) loop2 = g_main_loop_new(nullptr, 0);
  fl_key_embedder_responder_handle_event(
      responder, event2, 0, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_embedder_responder_handle_event_finish(
            FL_KEY_EMBEDDER_RESPONDER(object), result, &handled, nullptr));
        EXPECT_EQ(handled, TRUE);

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop2);

  EXPECT_EQ(call_records->len, 1u);
  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 0));
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(record->event->physical, kPhysicalKeyA);
  EXPECT_EQ(record->event->logical, kLogicalKeyA);
  EXPECT_STREQ(record->event->character, "A");
  EXPECT_EQ(record->event->synthesized, false);

  invoke_record_callback(record, TRUE);
  g_main_loop_run(loop2);
  clear_records(call_records);

  // Release CapsLock (stage 1 -> 2)
  g_autoptr(FlKeyEvent) event3 = fl_key_event_new(
      103, kRelease, kKeyCodeCapsLock, GDK_KEY_Caps_Lock, GDK_LOCK_MASK, 0);
  g_autoptr(GMainLoop) loop3 = g_main_loop_new(nullptr, 0);
  fl_key_embedder_responder_handle_event(
      responder, event3, 0, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_embedder_responder_handle_event_finish(
            FL_KEY_EMBEDDER_RESPONDER(object), result, &handled, nullptr));
        EXPECT_EQ(handled, TRUE);

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop3);

  EXPECT_EQ(call_records->len, 1u);
  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 0));
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(record->event->physical, kPhysicalCapsLock);
  EXPECT_EQ(record->event->logical, kLogicalCapsLock);
  EXPECT_STREQ(record->event->character, nullptr);
  EXPECT_EQ(record->event->synthesized, false);

  invoke_record_callback(record, TRUE);
  g_main_loop_run(loop3);
  clear_records(call_records);

  // Release key A (stage 2)
  g_autoptr(FlKeyEvent) event4 = fl_key_event_new(104, kRelease, kKeyCodeKeyA,
                                                  GDK_KEY_A, GDK_LOCK_MASK, 0);
  g_autoptr(GMainLoop) loop4 = g_main_loop_new(nullptr, 0);
  fl_key_embedder_responder_handle_event(
      responder, event4, 0, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_embedder_responder_handle_event_finish(
            FL_KEY_EMBEDDER_RESPONDER(object), result, &handled, nullptr));
        EXPECT_EQ(handled, TRUE);

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop4);

  EXPECT_EQ(call_records->len, 1u);
  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 0));
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(record->event->physical, kPhysicalKeyA);
  EXPECT_EQ(record->event->logical, kLogicalKeyA);
  EXPECT_STREQ(record->event->character, nullptr);
  EXPECT_EQ(record->event->synthesized, false);

  invoke_record_callback(record, TRUE);
  g_main_loop_run(loop4);
  clear_records(call_records);

  // Press CapsLock (stage 2 -> 3)
  g_autoptr(FlKeyEvent) event5 = fl_key_event_new(
      105, kPress, kKeyCodeCapsLock, GDK_KEY_Caps_Lock, GDK_LOCK_MASK, 0);
  g_autoptr(GMainLoop) loop5 = g_main_loop_new(nullptr, 0);
  fl_key_embedder_responder_handle_event(
      responder, event5, 0, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_embedder_responder_handle_event_finish(
            FL_KEY_EMBEDDER_RESPONDER(object), result, &handled, nullptr));
        EXPECT_EQ(handled, TRUE);

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop5);

  EXPECT_EQ(call_records->len, 1u);
  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 0));
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(record->event->physical, kPhysicalCapsLock);
  EXPECT_EQ(record->event->logical, kLogicalCapsLock);
  EXPECT_STREQ(record->event->character, nullptr);
  EXPECT_EQ(record->event->synthesized, false);

  invoke_record_callback(record, TRUE);
  g_main_loop_run(loop5);
  clear_records(call_records);

  // Press key A (stage 3)
  g_autoptr(FlKeyEvent) event6 =
      fl_key_event_new(106, kPress, kKeyCodeKeyA, GDK_KEY_A, GDK_LOCK_MASK, 0);
  g_autoptr(GMainLoop) loop6 = g_main_loop_new(nullptr, 0);
  fl_key_embedder_responder_handle_event(
      responder, event6, 0, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_embedder_responder_handle_event_finish(
            FL_KEY_EMBEDDER_RESPONDER(object), result, &handled, nullptr));
        EXPECT_EQ(handled, TRUE);

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop6);

  EXPECT_EQ(call_records->len, 1u);
  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 0));
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(record->event->physical, kPhysicalKeyA);
  EXPECT_EQ(record->event->logical, kLogicalKeyA);
  EXPECT_STREQ(record->event->character, "A");
  EXPECT_EQ(record->event->synthesized, false);

  invoke_record_callback(record, TRUE);
  g_main_loop_run(loop6);
  clear_records(call_records);

  // Release CapsLock (stage 3 -> 0)
  g_autoptr(FlKeyEvent) event7 = fl_key_event_new(
      107, kRelease, kKeyCodeCapsLock, GDK_KEY_Caps_Lock, GDK_LOCK_MASK, 0);
  g_autoptr(GMainLoop) loop7 = g_main_loop_new(nullptr, 0);
  fl_key_embedder_responder_handle_event(
      responder, event7, 0, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_embedder_responder_handle_event_finish(
            FL_KEY_EMBEDDER_RESPONDER(object), result, &handled, nullptr));
        EXPECT_EQ(handled, TRUE);

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop7);

  EXPECT_EQ(call_records->len, 1u);
  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 0));
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(record->event->physical, kPhysicalCapsLock);
  EXPECT_EQ(record->event->logical, kLogicalCapsLock);
  EXPECT_STREQ(record->event->character, nullptr);
  EXPECT_EQ(record->event->synthesized, false);

  invoke_record_callback(record, TRUE);
  g_main_loop_run(loop7);
  clear_records(call_records);

  // Release key A (stage 0)
  g_autoptr(FlKeyEvent) event8 =
      fl_key_event_new(108, kRelease, kKeyCodeKeyA, GDK_KEY_a,
                       static_cast<GdkModifierType>(0), 0);
  g_autoptr(GMainLoop) loop8 = g_main_loop_new(nullptr, 0);
  fl_key_embedder_responder_handle_event(
      responder, event8, 0, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_embedder_responder_handle_event_finish(
            FL_KEY_EMBEDDER_RESPONDER(object), result, &handled, nullptr));
        EXPECT_EQ(handled, TRUE);

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop8);

  EXPECT_EQ(call_records->len, 1u);
  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 0));
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(record->event->physical, kPhysicalKeyA);
  EXPECT_EQ(record->event->logical, kLogicalKeyA);
  EXPECT_STREQ(record->event->character, nullptr);
  EXPECT_EQ(record->event->synthesized, false);

  invoke_record_callback(record, TRUE);
  g_main_loop_run(loop8);
}

// Press or release letter key between presses/releases of CapsLock, on
// a platform with reversed logic.
//
// This happens when using a Chrome remote desktop on MacOS.
TEST(FlKeyEmbedderResponderTest, TapLetterKeysBetweenCapsLockEventsReversed) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);
  EXPECT_TRUE(fl_engine_start(engine, nullptr));

  g_autoptr(FlKeyEmbedderResponder) responder =
      fl_key_embedder_responder_new(engine);

  g_autoptr(GPtrArray) call_records =
      g_ptr_array_new_with_free_func(g_object_unref);
  fl_engine_get_embedder_api(engine)->SendKeyEvent = MOCK_ENGINE_PROC(
      SendKeyEvent,
      ([&call_records](auto engine, const FlutterKeyEvent* event,
                       FlutterKeyEventCallback callback, void* user_data) {
        g_ptr_array_add(call_records, fl_key_embedder_call_record_new(
                                          event, callback, user_data));
        return kSuccess;
      }));

  // Press key A (stage 0)
  g_autoptr(FlKeyEvent) event1 = fl_key_event_new(
      101, kPress, kKeyCodeKeyA, GDK_KEY_a, static_cast<GdkModifierType>(0), 0);
  g_autoptr(GMainLoop) loop1 = g_main_loop_new(nullptr, 0);
  fl_key_embedder_responder_handle_event(
      responder, event1, 0, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_embedder_responder_handle_event_finish(
            FL_KEY_EMBEDDER_RESPONDER(object), result, &handled, nullptr));
        EXPECT_EQ(handled, TRUE);

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop1);

  EXPECT_EQ(call_records->len, 1u);
  FlKeyEmbedderCallRecord* record =
      FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 0));
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(record->event->physical, kPhysicalKeyA);
  EXPECT_EQ(record->event->logical, kLogicalKeyA);
  EXPECT_STREQ(record->event->character, "a");
  EXPECT_EQ(record->event->synthesized, false);

  invoke_record_callback(record, TRUE);
  g_main_loop_run(loop1);
  clear_records(call_records);

  // Press CapsLock (stage 0 -> 1)
  g_autoptr(FlKeyEvent) event2 = fl_key_event_new(
      102, kPress, kKeyCodeCapsLock, GDK_KEY_Caps_Lock, GDK_LOCK_MASK, 0);
  g_autoptr(GMainLoop) loop2 = g_main_loop_new(nullptr, 0);
  fl_key_embedder_responder_handle_event(
      responder, event2, 0, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_embedder_responder_handle_event_finish(
            FL_KEY_EMBEDDER_RESPONDER(object), result, &handled, nullptr));
        EXPECT_EQ(handled, TRUE);

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop2);

  EXPECT_EQ(call_records->len, 1u);
  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 0));
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(record->event->physical, kPhysicalCapsLock);
  EXPECT_EQ(record->event->logical, kLogicalCapsLock);
  EXPECT_STREQ(record->event->character, nullptr);
  EXPECT_EQ(record->event->synthesized, false);

  invoke_record_callback(record, TRUE);
  g_main_loop_run(loop2);
  clear_records(call_records);

  // Release CapsLock (stage 1 -> 2)
  g_autoptr(FlKeyEvent) event3 = fl_key_event_new(
      103, kRelease, kKeyCodeCapsLock, GDK_KEY_Caps_Lock, GDK_LOCK_MASK, 0);
  g_autoptr(GMainLoop) loop3 = g_main_loop_new(nullptr, 0);
  fl_key_embedder_responder_handle_event(
      responder, event3, 0, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_embedder_responder_handle_event_finish(
            FL_KEY_EMBEDDER_RESPONDER(object), result, &handled, nullptr));
        EXPECT_EQ(handled, TRUE);

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop3);

  EXPECT_EQ(call_records->len, 1u);
  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 0));
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(record->event->physical, kPhysicalCapsLock);
  EXPECT_EQ(record->event->logical, kLogicalCapsLock);
  EXPECT_STREQ(record->event->character, nullptr);
  EXPECT_EQ(record->event->synthesized, false);

  invoke_record_callback(record, TRUE);
  g_main_loop_run(loop3);
  clear_records(call_records);

  // Release key A (stage 2)
  g_autoptr(FlKeyEvent) event4 = fl_key_event_new(104, kRelease, kKeyCodeKeyA,
                                                  GDK_KEY_A, GDK_LOCK_MASK, 0);
  g_autoptr(GMainLoop) loop4 = g_main_loop_new(nullptr, 0);
  fl_key_embedder_responder_handle_event(
      responder, event4, 0, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_embedder_responder_handle_event_finish(
            FL_KEY_EMBEDDER_RESPONDER(object), result, &handled, nullptr));
        EXPECT_EQ(handled, TRUE);

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop4);

  EXPECT_EQ(call_records->len, 1u);
  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 0));
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(record->event->physical, kPhysicalKeyA);
  EXPECT_EQ(record->event->logical, kLogicalKeyA);
  EXPECT_STREQ(record->event->character, nullptr);
  EXPECT_EQ(record->event->synthesized, false);

  invoke_record_callback(record, TRUE);
  g_main_loop_run(loop4);
  clear_records(call_records);

  // Press key A (stage 2)
  g_autoptr(FlKeyEvent) event5 =
      fl_key_event_new(105, kPress, kKeyCodeKeyA, GDK_KEY_A, GDK_LOCK_MASK, 0);
  g_autoptr(GMainLoop) loop5 = g_main_loop_new(nullptr, 0);
  fl_key_embedder_responder_handle_event(
      responder, event5, 0, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_embedder_responder_handle_event_finish(
            FL_KEY_EMBEDDER_RESPONDER(object), result, &handled, nullptr));
        EXPECT_EQ(handled, TRUE);

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop5);

  EXPECT_EQ(call_records->len, 1u);
  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 0));
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(record->event->physical, kPhysicalKeyA);
  EXPECT_EQ(record->event->logical, kLogicalKeyA);
  EXPECT_STREQ(record->event->character, "A");
  EXPECT_EQ(record->event->synthesized, false);

  invoke_record_callback(record, TRUE);
  g_main_loop_run(loop5);
  clear_records(call_records);

  // Press CapsLock (stage 2 -> 3)
  g_autoptr(FlKeyEvent) event6 =
      fl_key_event_new(106, kPress, kKeyCodeCapsLock, GDK_KEY_Caps_Lock,
                       static_cast<GdkModifierType>(0), 0);
  g_autoptr(GMainLoop) loop6 = g_main_loop_new(nullptr, 0);
  fl_key_embedder_responder_handle_event(
      responder, event6, 0, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_embedder_responder_handle_event_finish(
            FL_KEY_EMBEDDER_RESPONDER(object), result, &handled, nullptr));
        EXPECT_EQ(handled, TRUE);

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop6);

  EXPECT_EQ(call_records->len, 1u);
  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 0));
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(record->event->physical, kPhysicalCapsLock);
  EXPECT_EQ(record->event->logical, kLogicalCapsLock);
  EXPECT_STREQ(record->event->character, nullptr);
  EXPECT_EQ(record->event->synthesized, false);

  invoke_record_callback(record, TRUE);
  g_main_loop_run(loop6);
  clear_records(call_records);

  // Release CapsLock (stage 3 -> 0)
  g_autoptr(FlKeyEvent) event7 = fl_key_event_new(
      107, kRelease, kKeyCodeCapsLock, GDK_KEY_Caps_Lock, GDK_LOCK_MASK, 0);
  g_autoptr(GMainLoop) loop7 = g_main_loop_new(nullptr, 0);
  fl_key_embedder_responder_handle_event(
      responder, event7, 0, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_embedder_responder_handle_event_finish(
            FL_KEY_EMBEDDER_RESPONDER(object), result, &handled, nullptr));
        EXPECT_EQ(handled, TRUE);

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop7);

  EXPECT_EQ(call_records->len, 1u);
  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 0));
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(record->event->physical, kPhysicalCapsLock);
  EXPECT_EQ(record->event->logical, kLogicalCapsLock);
  EXPECT_STREQ(record->event->character, nullptr);
  EXPECT_EQ(record->event->synthesized, false);

  invoke_record_callback(record, TRUE);
  g_main_loop_run(loop7);
  clear_records(call_records);

  // Release key A (stage 0)
  g_autoptr(FlKeyEvent) event8 =
      fl_key_event_new(108, kRelease, kKeyCodeKeyA, GDK_KEY_a,
                       static_cast<GdkModifierType>(0), 0);
  g_autoptr(GMainLoop) loop8 = g_main_loop_new(nullptr, 0);
  fl_key_embedder_responder_handle_event(
      responder, event8, 0, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_embedder_responder_handle_event_finish(
            FL_KEY_EMBEDDER_RESPONDER(object), result, &handled, nullptr));
        EXPECT_EQ(handled, TRUE);

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop8);

  EXPECT_EQ(call_records->len, 1u);
  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 0));
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(record->event->physical, kPhysicalKeyA);
  EXPECT_EQ(record->event->logical, kLogicalKeyA);
  EXPECT_STREQ(record->event->character, nullptr);
  EXPECT_EQ(record->event->synthesized, false);

  invoke_record_callback(record, TRUE);
  g_main_loop_run(loop8);
}

TEST(FlKeyEmbedderResponderTest, TurnDuplicateDownEventsToRepeats) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);
  EXPECT_TRUE(fl_engine_start(engine, nullptr));

  g_autoptr(FlKeyEmbedderResponder) responder =
      fl_key_embedder_responder_new(engine);

  g_autoptr(GPtrArray) call_records =
      g_ptr_array_new_with_free_func(g_object_unref);
  fl_engine_get_embedder_api(engine)->SendKeyEvent = MOCK_ENGINE_PROC(
      SendKeyEvent,
      ([&call_records](auto engine, const FlutterKeyEvent* event,
                       FlutterKeyEventCallback callback, void* user_data) {
        g_ptr_array_add(call_records, fl_key_embedder_call_record_new(
                                          event, callback, user_data));
        return kSuccess;
      }));

  // Press KeyA
  g_autoptr(FlKeyEvent) event1 = fl_key_event_new(
      101, kPress, kKeyCodeKeyA, GDK_KEY_a, static_cast<GdkModifierType>(0), 0);
  g_autoptr(GMainLoop) loop1 = g_main_loop_new(nullptr, 0);
  fl_key_embedder_responder_handle_event(
      responder, event1, 0, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_embedder_responder_handle_event_finish(
            FL_KEY_EMBEDDER_RESPONDER(object), result, &handled, nullptr));
        EXPECT_EQ(handled, TRUE);

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop1);

  EXPECT_EQ(call_records->len, 1u);

  FlKeyEmbedderCallRecord* record =
      FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 0));
  invoke_record_callback(record, TRUE);
  g_main_loop_run(loop1);
  clear_records(call_records);

  // Another KeyA down events, which usually means a repeated event.
  g_autoptr(FlKeyEvent) event2 = fl_key_event_new(
      102, kPress, kKeyCodeKeyA, GDK_KEY_a, static_cast<GdkModifierType>(0), 0);
  g_autoptr(GMainLoop) loop2 = g_main_loop_new(nullptr, 0);
  fl_key_embedder_responder_handle_event(
      responder, event2, 0, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_embedder_responder_handle_event_finish(
            FL_KEY_EMBEDDER_RESPONDER(object), result, &handled, nullptr));
        EXPECT_EQ(handled, TRUE);

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop2);

  EXPECT_EQ(call_records->len, 1u);

  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 0));
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeRepeat);
  EXPECT_EQ(record->event->physical, kPhysicalKeyA);
  EXPECT_EQ(record->event->logical, kLogicalKeyA);
  EXPECT_STREQ(record->event->character, "a");
  EXPECT_EQ(record->event->synthesized, false);
  EXPECT_NE(record->callback, nullptr);

  invoke_record_callback(record, TRUE);
  g_main_loop_run(loop2);
  clear_records(call_records);

  // Release KeyA
  g_autoptr(FlKeyEvent) event3 =
      fl_key_event_new(103, kRelease, kKeyCodeKeyA, GDK_KEY_q,
                       static_cast<GdkModifierType>(0), 0);
  g_autoptr(GMainLoop) loop3 = g_main_loop_new(nullptr, 0);
  fl_key_embedder_responder_handle_event(
      responder, event3, 0, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_embedder_responder_handle_event_finish(
            FL_KEY_EMBEDDER_RESPONDER(object), result, &handled, nullptr));
        EXPECT_EQ(handled, TRUE);

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop3);

  EXPECT_EQ(call_records->len, 1u);
  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 0));
  invoke_record_callback(record, TRUE);
  g_main_loop_run(loop3);
}

TEST(FlKeyEmbedderResponderTest, IgnoreAbruptUpEvent) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);
  EXPECT_TRUE(fl_engine_start(engine, nullptr));

  g_autoptr(FlKeyEmbedderResponder) responder =
      fl_key_embedder_responder_new(engine);

  g_autoptr(GPtrArray) call_records =
      g_ptr_array_new_with_free_func(g_object_unref);
  fl_engine_get_embedder_api(engine)->SendKeyEvent = MOCK_ENGINE_PROC(
      SendKeyEvent,
      ([&call_records](auto engine, const FlutterKeyEvent* event,
                       FlutterKeyEventCallback callback, void* user_data) {
        g_ptr_array_add(call_records, fl_key_embedder_call_record_new(
                                          event, callback, user_data));
        return kSuccess;
      }));

  // Release KeyA before it was even pressed.
  g_autoptr(FlKeyEvent) event =
      fl_key_event_new(103, kRelease, kKeyCodeKeyA, GDK_KEY_q,
                       static_cast<GdkModifierType>(0), 0);
  g_autoptr(GMainLoop) loop = g_main_loop_new(nullptr, 0);
  fl_key_embedder_responder_handle_event(
      responder, event, 0, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_embedder_responder_handle_event_finish(
            FL_KEY_EMBEDDER_RESPONDER(object), result, &handled, nullptr));
        EXPECT_EQ(handled, TRUE);

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop);

  EXPECT_EQ(call_records->len, 1u);

  FlKeyEmbedderCallRecord* record =
      FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 0));
  EXPECT_EQ(record->event->physical, 0ull);
  EXPECT_EQ(record->event->logical, 0ull);
  EXPECT_STREQ(record->event->character, nullptr);
  EXPECT_EQ(record->event->synthesized, false);

  invoke_record_callback(record, TRUE);
  g_main_loop_run(loop);
}

// Test if missed modifier keys can be detected and synthesized with state
// information upon events that are for this modifier key.
TEST(FlKeyEmbedderResponderTest, SynthesizeForDesyncPressingStateOnSelfEvents) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);
  EXPECT_TRUE(fl_engine_start(engine, nullptr));

  g_autoptr(FlKeyEmbedderResponder) responder =
      fl_key_embedder_responder_new(engine);

  g_autoptr(GPtrArray) call_records =
      g_ptr_array_new_with_free_func(g_object_unref);
  fl_engine_get_embedder_api(engine)->SendKeyEvent = MOCK_ENGINE_PROC(
      SendKeyEvent,
      ([&call_records](auto engine, const FlutterKeyEvent* event,
                       FlutterKeyEventCallback callback, void* user_data) {
        g_ptr_array_add(call_records, fl_key_embedder_call_record_new(
                                          event, callback, user_data));
        return kSuccess;
      }));

  // Test 1: synthesize key down.

  // A key down of control left is missed.
  GdkModifierType state = GDK_CONTROL_MASK;

  // Send a ControlLeft up
  g_autoptr(FlKeyEvent) event1 = fl_key_event_new(
      101, kRelease, kKeyCodeControlLeft, GDK_KEY_Control_L, state, 0);
  g_autoptr(GMainLoop) loop1 = g_main_loop_new(nullptr, 0);
  fl_key_embedder_responder_handle_event(
      responder, event1, 0, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_embedder_responder_handle_event_finish(
            FL_KEY_EMBEDDER_RESPONDER(object), result, &handled, nullptr));
        EXPECT_EQ(handled, TRUE);

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop1);

  EXPECT_EQ(call_records->len, 2u);
  FlKeyEmbedderCallRecord* record =
      FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 0));
  EXPECT_EQ(record->event->timestamp, 101000);
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(record->event->physical, kPhysicalControlLeft);
  EXPECT_EQ(record->event->logical, kLogicalControlLeft);
  EXPECT_STREQ(record->event->character, nullptr);
  EXPECT_EQ(record->event->synthesized, true);

  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 1));
  EXPECT_EQ(record->event->timestamp, 101000);
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(record->event->physical, kPhysicalControlLeft);
  EXPECT_EQ(record->event->logical, kLogicalControlLeft);
  EXPECT_STREQ(record->event->character, nullptr);
  EXPECT_EQ(record->event->synthesized, false);

  invoke_record_callback(record, TRUE);
  g_main_loop_run(loop1);
  clear_records(call_records);

  // Test 2: synthesize key up.

  // Send a ControlLeft down.
  state = static_cast<GdkModifierType>(0);
  g_autoptr(FlKeyEvent) event2 = fl_key_event_new(
      102, kPress, kKeyCodeControlLeft, GDK_KEY_Control_L, state, 0);
  g_autoptr(GMainLoop) loop2 = g_main_loop_new(nullptr, 0);
  fl_key_embedder_responder_handle_event(
      responder, event2, 0, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_embedder_responder_handle_event_finish(
            FL_KEY_EMBEDDER_RESPONDER(object), result, &handled, nullptr));
        EXPECT_EQ(handled, TRUE);

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop2);
  EXPECT_EQ(call_records->len, 1u);
  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 0));
  invoke_record_callback(record, TRUE);
  g_main_loop_run(loop2);
  clear_records(call_records);

  // A key up of control left is missed.
  state = static_cast<GdkModifierType>(0);

  // Send another ControlLeft down
  g_autoptr(FlKeyEvent) event3 = fl_key_event_new(
      103, kPress, kKeyCodeControlLeft, GDK_KEY_Control_L, state, 0);
  g_autoptr(GMainLoop) loop3 = g_main_loop_new(nullptr, 0);
  fl_key_embedder_responder_handle_event(
      responder, event3, 0, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_embedder_responder_handle_event_finish(
            FL_KEY_EMBEDDER_RESPONDER(object), result, &handled, nullptr));
        EXPECT_EQ(handled, TRUE);

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop3);

  EXPECT_EQ(call_records->len, 2u);
  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 0));
  EXPECT_EQ(record->event->timestamp, 103000);
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(record->event->physical, kPhysicalControlLeft);
  EXPECT_EQ(record->event->logical, kLogicalControlLeft);
  EXPECT_STREQ(record->event->character, nullptr);
  EXPECT_EQ(record->event->synthesized, true);

  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 1));
  EXPECT_EQ(record->event->timestamp, 103000);
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(record->event->physical, kPhysicalControlLeft);
  EXPECT_EQ(record->event->logical, kLogicalControlLeft);
  EXPECT_STREQ(record->event->character, nullptr);
  EXPECT_EQ(record->event->synthesized, false);

  invoke_record_callback(record, TRUE);
  g_main_loop_run(loop3);
  clear_records(call_records);

  // Send a ControlLeft up to clear up state.
  state = GDK_CONTROL_MASK;
  g_autoptr(FlKeyEvent) event4 = fl_key_event_new(
      104, kRelease, kKeyCodeControlLeft, GDK_KEY_Control_L, state, 0);
  g_autoptr(GMainLoop) loop4 = g_main_loop_new(nullptr, 0);
  fl_key_embedder_responder_handle_event(
      responder, event4, 0, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_embedder_responder_handle_event_finish(
            FL_KEY_EMBEDDER_RESPONDER(object), result, &handled, nullptr));
        EXPECT_EQ(handled, TRUE);

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop4);
  EXPECT_EQ(call_records->len, 1u);
  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 0));
  invoke_record_callback(record, TRUE);
  g_main_loop_run(loop4);
  clear_records(call_records);

  // Test 3: synthesize by right modifier.

  // A key down of control right is missed.
  state = GDK_CONTROL_MASK;

  // Send a ControlRight up.
  g_autoptr(FlKeyEvent) event5 = fl_key_event_new(
      105, kRelease, kKeyCodeControlRight, GDK_KEY_Control_R, state, 0);
  g_autoptr(GMainLoop) loop5 = g_main_loop_new(nullptr, 0);
  fl_key_embedder_responder_handle_event(
      responder, event5, 0, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_embedder_responder_handle_event_finish(
            FL_KEY_EMBEDDER_RESPONDER(object), result, &handled, nullptr));
        EXPECT_EQ(handled, TRUE);

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop5);

  // A ControlLeft down is synthesized, with an empty event.
  // Reason: The ControlLeft down is synthesized to synchronize the state
  // showing Control as pressed. The ControlRight event is ignored because
  // the event is considered a duplicate up event.
  EXPECT_EQ(call_records->len, 1u);
  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 0));
  EXPECT_EQ(record->event->timestamp, 105000);
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(record->event->physical, kPhysicalControlLeft);
  EXPECT_EQ(record->event->logical, kLogicalControlLeft);
  EXPECT_STREQ(record->event->character, nullptr);
  EXPECT_EQ(record->event->synthesized, true);

  invoke_record_callback(record, TRUE);
  g_main_loop_run(loop5);
}

// Test if missed modifier keys can be detected and synthesized with state
// information upon events that are not for this modifier key.
TEST(FlKeyEmbedderResponderTest,
     SynthesizeForDesyncPressingStateOnNonSelfEvents) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);
  EXPECT_TRUE(fl_engine_start(engine, nullptr));

  g_autoptr(FlKeyEmbedderResponder) responder =
      fl_key_embedder_responder_new(engine);

  g_autoptr(GPtrArray) call_records =
      g_ptr_array_new_with_free_func(g_object_unref);
  fl_engine_get_embedder_api(engine)->SendKeyEvent = MOCK_ENGINE_PROC(
      SendKeyEvent,
      ([&call_records](auto engine, const FlutterKeyEvent* event,
                       FlutterKeyEventCallback callback, void* user_data) {
        g_ptr_array_add(call_records, fl_key_embedder_call_record_new(
                                          event, callback, user_data));
        return kSuccess;
      }));

  // A key down of control left is missed.
  GdkModifierType state = GDK_CONTROL_MASK;

  // Send a normal event (KeyA down)
  g_autoptr(FlKeyEvent) event1 =
      fl_key_event_new(101, kPress, kKeyCodeKeyA, GDK_KEY_a, state, 0);
  g_autoptr(GMainLoop) loop1 = g_main_loop_new(nullptr, 0);
  fl_key_embedder_responder_handle_event(
      responder, event1, 0, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_embedder_responder_handle_event_finish(
            FL_KEY_EMBEDDER_RESPONDER(object), result, &handled, nullptr));
        EXPECT_EQ(handled, TRUE);

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop1);

  EXPECT_EQ(call_records->len, 2u);
  FlKeyEmbedderCallRecord* record =
      FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 0));
  EXPECT_EQ(record->event->timestamp, 101000);
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(record->event->physical, kPhysicalControlLeft);
  EXPECT_EQ(record->event->logical, kLogicalControlLeft);
  EXPECT_STREQ(record->event->character, nullptr);
  EXPECT_EQ(record->event->synthesized, true);

  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 1));
  EXPECT_EQ(record->event->timestamp, 101000);
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(record->event->physical, kPhysicalKeyA);
  EXPECT_EQ(record->event->logical, kLogicalKeyA);
  EXPECT_STREQ(record->event->character, "a");
  EXPECT_EQ(record->event->synthesized, false);

  invoke_record_callback(record, TRUE);
  g_main_loop_run(loop1);
  clear_records(call_records);

  // A key up of control left is missed.
  state = static_cast<GdkModifierType>(0);

  // Send a normal event (KeyA up)
  g_autoptr(FlKeyEvent) event2 =
      fl_key_event_new(102, kRelease, kKeyCodeKeyA, GDK_KEY_A, state, 0);
  g_autoptr(GMainLoop) loop2 = g_main_loop_new(nullptr, 0);
  fl_key_embedder_responder_handle_event(
      responder, event2, 0, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_embedder_responder_handle_event_finish(
            FL_KEY_EMBEDDER_RESPONDER(object), result, &handled, nullptr));
        EXPECT_EQ(handled, TRUE);

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop2);

  EXPECT_EQ(call_records->len, 2u);
  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 0));
  EXPECT_EQ(record->event->timestamp, 102000);
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(record->event->physical, kPhysicalControlLeft);
  EXPECT_EQ(record->event->logical, kLogicalControlLeft);
  EXPECT_STREQ(record->event->character, nullptr);
  EXPECT_EQ(record->event->synthesized, true);

  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 1));
  EXPECT_EQ(record->event->timestamp, 102000);
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(record->event->physical, kPhysicalKeyA);
  EXPECT_EQ(record->event->logical, kLogicalKeyA);
  EXPECT_STREQ(record->event->character, nullptr);
  EXPECT_EQ(record->event->synthesized, false);

  invoke_record_callback(record, TRUE);
  g_main_loop_run(loop2);
  clear_records(call_records);

  // Test non-default key mapping.

  // Press a key with physical CapsLock and logical ControlLeft.
  state = static_cast<GdkModifierType>(0);

  g_autoptr(FlKeyEvent) event3 = fl_key_event_new(101, kPress, kKeyCodeCapsLock,
                                                  GDK_KEY_Control_L, state, 0);
  g_autoptr(GMainLoop) loop3 = g_main_loop_new(nullptr, 0);
  fl_key_embedder_responder_handle_event(
      responder, event3, 0, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_embedder_responder_handle_event_finish(
            FL_KEY_EMBEDDER_RESPONDER(object), result, &handled, nullptr));
        EXPECT_EQ(handled, TRUE);

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop3);

  EXPECT_EQ(call_records->len, 1u);
  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 0));
  EXPECT_EQ(record->event->timestamp, 101000);
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(record->event->physical, kPhysicalCapsLock);
  EXPECT_EQ(record->event->logical, kLogicalControlLeft);
  EXPECT_STREQ(record->event->character, nullptr);
  EXPECT_EQ(record->event->synthesized, false);

  invoke_record_callback(record, TRUE);
  g_main_loop_run(loop3);
  clear_records(call_records);

  // The key up of the control left press is missed.
  state = static_cast<GdkModifierType>(0);

  // Send a normal event (KeyA down).
  g_autoptr(FlKeyEvent) event4 =
      fl_key_event_new(102, kPress, kKeyCodeKeyA, GDK_KEY_A, state, 0);
  g_autoptr(GMainLoop) loop4 = g_main_loop_new(nullptr, 0);
  fl_key_embedder_responder_handle_event(
      responder, event4, 0, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_embedder_responder_handle_event_finish(
            FL_KEY_EMBEDDER_RESPONDER(object), result, &handled, nullptr));
        EXPECT_EQ(handled, TRUE);

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop4);

  // The synthesized event should have physical CapsLock and logical
  // ControlLeft.
  EXPECT_EQ(call_records->len, 2u);
  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 0));
  EXPECT_EQ(record->event->timestamp, 102000);
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(record->event->physical, kPhysicalCapsLock);
  EXPECT_EQ(record->event->logical, kLogicalControlLeft);
  EXPECT_STREQ(record->event->character, nullptr);
  EXPECT_EQ(record->event->synthesized, true);

  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 1));
  EXPECT_EQ(record->event->timestamp, 102000);
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(record->event->physical, kPhysicalKeyA);
  EXPECT_EQ(record->event->logical, kLogicalKeyA);
  EXPECT_STREQ(record->event->character, "A");
  EXPECT_EQ(record->event->synthesized, false);

  invoke_record_callback(record, TRUE);
  g_main_loop_run(loop4);
}

// Test if missed modifier keys can be detected and synthesized with state
// information upon events that do not have the standard key mapping.
TEST(FlKeyEmbedderResponderTest,
     SynthesizeForDesyncPressingStateOnRemappedEvents) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);
  EXPECT_TRUE(fl_engine_start(engine, nullptr));

  g_autoptr(FlKeyEmbedderResponder) responder =
      fl_key_embedder_responder_new(engine);

  g_autoptr(GPtrArray) call_records =
      g_ptr_array_new_with_free_func(g_object_unref);
  fl_engine_get_embedder_api(engine)->SendKeyEvent = MOCK_ENGINE_PROC(
      SendKeyEvent,
      ([&call_records](auto engine, const FlutterKeyEvent* event,
                       FlutterKeyEventCallback callback, void* user_data) {
        g_ptr_array_add(call_records, fl_key_embedder_call_record_new(
                                          event, callback, user_data));
        return kSuccess;
      }));

  // Press a key with physical CapsLock and logical ControlLeft.
  GdkModifierType state = static_cast<GdkModifierType>(0);

  g_autoptr(FlKeyEvent) event1 = fl_key_event_new(101, kPress, kKeyCodeCapsLock,
                                                  GDK_KEY_Control_L, state, 0);
  g_autoptr(GMainLoop) loop1 = g_main_loop_new(nullptr, 0);
  fl_key_embedder_responder_handle_event(
      responder, event1, 0, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_embedder_responder_handle_event_finish(
            FL_KEY_EMBEDDER_RESPONDER(object), result, &handled, nullptr));
        EXPECT_EQ(handled, TRUE);

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop1);

  EXPECT_EQ(call_records->len, 1u);
  FlKeyEmbedderCallRecord* record =
      FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 0));
  EXPECT_EQ(record->event->timestamp, 101000);
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(record->event->physical, kPhysicalCapsLock);
  EXPECT_EQ(record->event->logical, kLogicalControlLeft);
  EXPECT_STREQ(record->event->character, nullptr);
  EXPECT_EQ(record->event->synthesized, false);

  invoke_record_callback(record, TRUE);
  g_main_loop_run(loop1);
  clear_records(call_records);

  // The key up of the control left press is missed.
  state = static_cast<GdkModifierType>(0);

  // Send a normal event (KeyA down).
  g_autoptr(FlKeyEvent) event2 =
      fl_key_event_new(102, kPress, kKeyCodeKeyA, GDK_KEY_A, state, 0);
  g_autoptr(GMainLoop) loop2 = g_main_loop_new(nullptr, 0);
  fl_key_embedder_responder_handle_event(
      responder, event2, 0, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_embedder_responder_handle_event_finish(
            FL_KEY_EMBEDDER_RESPONDER(object), result, &handled, nullptr));
        EXPECT_EQ(handled, TRUE);

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop2);

  // The synthesized event should have physical CapsLock and logical
  // ControlLeft.
  EXPECT_EQ(call_records->len, 2u);
  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 0));
  EXPECT_EQ(record->event->timestamp, 102000);
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(record->event->physical, kPhysicalCapsLock);
  EXPECT_EQ(record->event->logical, kLogicalControlLeft);
  EXPECT_STREQ(record->event->character, nullptr);
  EXPECT_EQ(record->event->synthesized, true);

  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 1));
  EXPECT_EQ(record->event->timestamp, 102000);
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(record->event->physical, kPhysicalKeyA);
  EXPECT_EQ(record->event->logical, kLogicalKeyA);
  EXPECT_STREQ(record->event->character, "A");
  EXPECT_EQ(record->event->synthesized, false);

  invoke_record_callback(record, TRUE);
  g_main_loop_run(loop2);
}

// Test if missed lock keys can be detected and synthesized with state
// information upon events that are not for this modifier key.
TEST(FlKeyEmbedderResponderTest, SynthesizeForDesyncLockModeOnNonSelfEvents) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);
  EXPECT_TRUE(fl_engine_start(engine, nullptr));

  g_autoptr(FlKeyEmbedderResponder) responder =
      fl_key_embedder_responder_new(engine);

  g_autoptr(GPtrArray) call_records =
      g_ptr_array_new_with_free_func(g_object_unref);
  fl_engine_get_embedder_api(engine)->SendKeyEvent = MOCK_ENGINE_PROC(
      SendKeyEvent,
      ([&call_records](auto engine, const FlutterKeyEvent* event,
                       FlutterKeyEventCallback callback, void* user_data) {
        g_ptr_array_add(call_records, fl_key_embedder_call_record_new(
                                          event, callback, user_data));
        return kSuccess;
      }));

  // The NumLock is desynchronized by being enabled.
  GdkModifierType state = GDK_MOD2_MASK;

  // Send a normal event
  g_autoptr(FlKeyEvent) event1 =
      fl_key_event_new(101, kPress, kKeyCodeKeyA, GDK_KEY_a, state, 0);
  g_autoptr(GMainLoop) loop1 = g_main_loop_new(nullptr, 0);
  fl_key_embedder_responder_handle_event(
      responder, event1, 0, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_embedder_responder_handle_event_finish(
            FL_KEY_EMBEDDER_RESPONDER(object), result, &handled, nullptr));
        EXPECT_EQ(handled, TRUE);

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop1);

  EXPECT_EQ(call_records->len, 2u);
  FlKeyEmbedderCallRecord* record =
      FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 0));
  EXPECT_EQ(record->event->timestamp, 101000);
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(record->event->physical, kPhysicalNumLock);
  EXPECT_EQ(record->event->logical, kLogicalNumLock);
  EXPECT_STREQ(record->event->character, nullptr);
  EXPECT_EQ(record->event->synthesized, true);

  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 1));
  EXPECT_EQ(record->event->timestamp, 101000);
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(record->event->physical, kPhysicalKeyA);
  EXPECT_EQ(record->event->logical, kLogicalKeyA);
  EXPECT_STREQ(record->event->character, "a");
  EXPECT_EQ(record->event->synthesized, false);

  invoke_record_callback(record, TRUE);
  g_main_loop_run(loop1);
  clear_records(call_records);

  // The NumLock is desynchronized by being disabled.
  state = static_cast<GdkModifierType>(0);

  // Release key A
  g_autoptr(FlKeyEvent) event2 =
      fl_key_event_new(102, kRelease, kKeyCodeKeyA, GDK_KEY_A, state, 0);
  g_autoptr(GMainLoop) loop2 = g_main_loop_new(nullptr, 0);
  fl_key_embedder_responder_handle_event(
      responder, event2, 0, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_embedder_responder_handle_event_finish(
            FL_KEY_EMBEDDER_RESPONDER(object), result, &handled, nullptr));
        EXPECT_EQ(handled, TRUE);

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop2);

  EXPECT_EQ(call_records->len, 4u);
  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 0));
  EXPECT_EQ(record->event->timestamp, 102000);
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(record->event->physical, kPhysicalNumLock);
  EXPECT_EQ(record->event->logical, kLogicalNumLock);
  EXPECT_STREQ(record->event->character, nullptr);
  EXPECT_EQ(record->event->synthesized, true);

  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 1));
  EXPECT_EQ(record->event->timestamp, 102000);
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(record->event->physical, kPhysicalNumLock);
  EXPECT_EQ(record->event->logical, kLogicalNumLock);
  EXPECT_STREQ(record->event->character, nullptr);
  EXPECT_EQ(record->event->synthesized, true);

  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 2));
  EXPECT_EQ(record->event->timestamp, 102000);
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(record->event->physical, kPhysicalNumLock);
  EXPECT_EQ(record->event->logical, kLogicalNumLock);
  EXPECT_STREQ(record->event->character, nullptr);
  EXPECT_EQ(record->event->synthesized, true);

  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 3));
  EXPECT_EQ(record->event->timestamp, 102000);
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(record->event->physical, kPhysicalKeyA);
  EXPECT_EQ(record->event->logical, kLogicalKeyA);
  EXPECT_STREQ(record->event->character, nullptr);
  EXPECT_EQ(record->event->synthesized, false);

  invoke_record_callback(record, TRUE);
  g_main_loop_run(loop2);
  clear_records(call_records);

  // Release NumLock. Since the previous event should have synthesized NumLock
  // to be released, this should result in only an empty event.
  g_autoptr(FlKeyEvent) event3 = fl_key_event_new(
      103, kRelease, kKeyCodeNumLock, GDK_KEY_Num_Lock, state, 0);
  g_autoptr(GMainLoop) loop3 = g_main_loop_new(nullptr, 0);
  fl_key_embedder_responder_handle_event(
      responder, event3, 0, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_embedder_responder_handle_event_finish(
            FL_KEY_EMBEDDER_RESPONDER(object), result, &handled, nullptr));
        EXPECT_EQ(handled, TRUE);

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop3);

  EXPECT_EQ(call_records->len, 1u);
  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 0));
  EXPECT_EQ(record->event->physical, 0ull);
  EXPECT_EQ(record->event->logical, 0ull);
  EXPECT_STREQ(record->event->character, nullptr);
  EXPECT_EQ(record->event->synthesized, false);

  invoke_record_callback(record, TRUE);
  g_main_loop_run(loop3);
}

// Test if missed lock keys can be detected and synthesized with state
// information upon events that are for this modifier key.
TEST(FlKeyEmbedderResponderTest, SynthesizeForDesyncLockModeOnSelfEvents) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);
  EXPECT_TRUE(fl_engine_start(engine, nullptr));

  g_autoptr(FlKeyEmbedderResponder) responder =
      fl_key_embedder_responder_new(engine);

  g_autoptr(GPtrArray) call_records =
      g_ptr_array_new_with_free_func(g_object_unref);
  fl_engine_get_embedder_api(engine)->SendKeyEvent = MOCK_ENGINE_PROC(
      SendKeyEvent,
      ([&call_records](auto engine, const FlutterKeyEvent* event,
                       FlutterKeyEventCallback callback, void* user_data) {
        g_ptr_array_add(call_records, fl_key_embedder_call_record_new(
                                          event, callback, user_data));
        return kSuccess;
      }));

  // The NumLock is desynchronized by being enabled.
  GdkModifierType state = GDK_MOD2_MASK;

  // NumLock down
  g_autoptr(FlKeyEvent) event1 = fl_key_event_new(101, kPress, kKeyCodeNumLock,
                                                  GDK_KEY_Num_Lock, state, 0);
  g_autoptr(GMainLoop) loop1 = g_main_loop_new(nullptr, 0);
  fl_key_embedder_responder_handle_event(
      responder, event1, 0, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_embedder_responder_handle_event_finish(
            FL_KEY_EMBEDDER_RESPONDER(object), result, &handled, nullptr));
        EXPECT_EQ(handled, TRUE);

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop1);

  EXPECT_EQ(call_records->len, 3u);
  FlKeyEmbedderCallRecord* record =
      FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 0));
  EXPECT_EQ(record->event->timestamp, 101000);
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(record->event->physical, kPhysicalNumLock);
  EXPECT_EQ(record->event->logical, kLogicalNumLock);
  EXPECT_STREQ(record->event->character, nullptr);
  EXPECT_EQ(record->event->synthesized, true);

  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 1));
  EXPECT_EQ(record->event->timestamp, 101000);
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(record->event->physical, kPhysicalNumLock);
  EXPECT_EQ(record->event->logical, kLogicalNumLock);
  EXPECT_STREQ(record->event->character, nullptr);
  EXPECT_EQ(record->event->synthesized, true);

  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 2));
  EXPECT_EQ(record->event->timestamp, 101000);
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(record->event->physical, kPhysicalNumLock);
  EXPECT_EQ(record->event->logical, kLogicalNumLock);
  EXPECT_STREQ(record->event->character, nullptr);
  EXPECT_EQ(record->event->synthesized, false);

  invoke_record_callback(record, TRUE);
  g_main_loop_run(loop1);
  clear_records(call_records);

  // The NumLock is desynchronized by being enabled in a press event.
  state = GDK_MOD2_MASK;

  // NumLock up
  g_autoptr(FlKeyEvent) event2 = fl_key_event_new(102, kPress, kKeyCodeNumLock,
                                                  GDK_KEY_Num_Lock, state, 0);
  g_autoptr(GMainLoop) loop2 = g_main_loop_new(nullptr, 0);
  fl_key_embedder_responder_handle_event(
      responder, event2, 0, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_embedder_responder_handle_event_finish(
            FL_KEY_EMBEDDER_RESPONDER(object), result, &handled, nullptr));
        EXPECT_EQ(handled, TRUE);

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop2);

  EXPECT_EQ(call_records->len, 4u);
  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 0));
  EXPECT_EQ(record->event->timestamp, 102000);
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(record->event->physical, kPhysicalNumLock);
  EXPECT_EQ(record->event->logical, kLogicalNumLock);
  EXPECT_STREQ(record->event->character, nullptr);
  EXPECT_EQ(record->event->synthesized, true);

  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 1));
  EXPECT_EQ(record->event->timestamp, 102000);
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(record->event->physical, kPhysicalNumLock);
  EXPECT_EQ(record->event->logical, kLogicalNumLock);
  EXPECT_STREQ(record->event->character, nullptr);
  EXPECT_EQ(record->event->synthesized, true);

  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 2));
  EXPECT_EQ(record->event->timestamp, 102000);
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(record->event->physical, kPhysicalNumLock);
  EXPECT_EQ(record->event->logical, kLogicalNumLock);
  EXPECT_STREQ(record->event->character, nullptr);
  EXPECT_EQ(record->event->synthesized, true);

  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 3));
  EXPECT_EQ(record->event->timestamp, 102000);
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(record->event->physical, kPhysicalNumLock);
  EXPECT_EQ(record->event->logical, kLogicalNumLock);
  EXPECT_STREQ(record->event->character, nullptr);
  EXPECT_EQ(record->event->synthesized, false);

  invoke_record_callback(record, TRUE);
  g_main_loop_run(loop2);
}

// Ensures that even if the primary event is ignored (due to duplicate
// key up or down events), key synthesization is still performed.
TEST(FlKeyEmbedderResponderTest, SynthesizationOccursOnIgnoredEvents) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);
  EXPECT_TRUE(fl_engine_start(engine, nullptr));

  g_autoptr(FlKeyEmbedderResponder) responder =
      fl_key_embedder_responder_new(engine);

  g_autoptr(GPtrArray) call_records =
      g_ptr_array_new_with_free_func(g_object_unref);
  fl_engine_get_embedder_api(engine)->SendKeyEvent = MOCK_ENGINE_PROC(
      SendKeyEvent,
      ([&call_records](auto engine, const FlutterKeyEvent* event,
                       FlutterKeyEventCallback callback, void* user_data) {
        g_ptr_array_add(call_records, fl_key_embedder_call_record_new(
                                          event, callback, user_data));
        return kSuccess;
      }));

  // The NumLock is desynchronized by being enabled, and Control is pressed.
  GdkModifierType state =
      static_cast<GdkModifierType>(GDK_MOD2_MASK | GDK_CONTROL_MASK);

  // Send a KeyA up event, which will be ignored.
  g_autoptr(FlKeyEvent) event =
      fl_key_event_new(101, kRelease, kKeyCodeKeyA, GDK_KEY_a, state, 0);
  g_autoptr(GMainLoop) loop = g_main_loop_new(nullptr, 0);
  fl_key_embedder_responder_handle_event(
      responder, event, 0, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        EXPECT_TRUE(fl_key_embedder_responder_handle_event_finish(
            FL_KEY_EMBEDDER_RESPONDER(object), result, &handled, nullptr));
        EXPECT_EQ(handled, TRUE);

        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop);
  g_main_loop_run(loop);

  EXPECT_EQ(call_records->len, 2u);
  FlKeyEmbedderCallRecord* record =
      FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 0));
  EXPECT_EQ(record->event->timestamp, 101000);
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(record->event->physical, kPhysicalNumLock);
  EXPECT_EQ(record->event->logical, kLogicalNumLock);
  EXPECT_STREQ(record->event->character, nullptr);
  EXPECT_EQ(record->event->synthesized, true);

  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 1));
  EXPECT_EQ(record->event->timestamp, 101000);
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(record->event->physical, kPhysicalControlLeft);
  EXPECT_EQ(record->event->logical, kLogicalControlLeft);
  EXPECT_STREQ(record->event->character, nullptr);
  EXPECT_EQ(record->event->synthesized, true);
}

// This test case occurs when the following two cases collide:
//
// 1. When holding shift, AltRight key gives logical GDK_KEY_Meta_R with the
//    state bitmask still MOD3 (Alt).
// 2. When holding AltRight, ShiftLeft key gives logical GDK_KEY_ISO_Next_Group
//    with the state bitmask RESERVED_14.
//
// The resulting event sequence is not perfectly ideal: it had to synthesize
// AltLeft down because the physical AltRight key corresponds to logical
// MetaRight at the moment.
TEST(FlKeyEmbedderResponderTest, HandlesShiftAltVersusGroupNext) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);
  EXPECT_TRUE(fl_engine_start(engine, nullptr));

  g_autoptr(FlKeyEmbedderResponder) responder =
      fl_key_embedder_responder_new(engine);

  g_autoptr(GPtrArray) call_records =
      g_ptr_array_new_with_free_func(g_object_unref);
  fl_engine_get_embedder_api(engine)->SendKeyEvent = MOCK_ENGINE_PROC(
      SendKeyEvent,
      ([&call_records](auto engine, const FlutterKeyEvent* event,
                       FlutterKeyEventCallback callback, void* user_data) {
        g_ptr_array_add(call_records, fl_key_embedder_call_record_new(
                                          event, callback, user_data));
        callback(true, user_data);
        return kSuccess;
      }));

  guint32 now_time = 1;
  // A convenient shorthand to simulate events.
  auto send_key_event = [responder, &now_time](bool is_press, guint keyval,
                                               guint16 keycode,
                                               GdkModifierType state) {
    now_time += 1;
    g_autoptr(FlKeyEvent) event =
        fl_key_event_new(now_time, is_press, keycode, keyval, state, 0);
    g_autoptr(GMainLoop) loop = g_main_loop_new(nullptr, 0);
    fl_key_embedder_responder_handle_event(
        responder, event, 0, nullptr,
        [](GObject* object, GAsyncResult* result, gpointer user_data) {
          gboolean handled;
          EXPECT_TRUE(fl_key_embedder_responder_handle_event_finish(
              FL_KEY_EMBEDDER_RESPONDER(object), result, &handled, nullptr));
          EXPECT_EQ(handled, TRUE);

          g_main_loop_quit(static_cast<GMainLoop*>(user_data));
        },
        loop);
    g_main_loop_run(loop);
  };

  send_key_event(kPress, GDK_KEY_Shift_L, kKeyCodeShiftLeft,
                 GDK_MODIFIER_RESERVED_25_MASK);
  EXPECT_EQ(call_records->len, 1u);
  FlKeyEmbedderCallRecord* record =
      FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 0));
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(record->event->physical, kPhysicalShiftLeft);
  EXPECT_EQ(record->event->logical, kLogicalShiftLeft);
  EXPECT_EQ(record->event->synthesized, false);

  send_key_event(kPress, GDK_KEY_Meta_R, kKeyCodeAltRight,
                 static_cast<GdkModifierType>(GDK_SHIFT_MASK |
                                              GDK_MODIFIER_RESERVED_25_MASK));
  EXPECT_EQ(call_records->len, 2u);
  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 1));
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(record->event->physical, kPhysicalAltRight);
  EXPECT_EQ(record->event->logical, kLogicalMetaRight);
  EXPECT_EQ(record->event->synthesized, false);

  send_key_event(kRelease, GDK_KEY_ISO_Next_Group, kKeyCodeShiftLeft,
                 static_cast<GdkModifierType>(GDK_SHIFT_MASK | GDK_MOD1_MASK |
                                              GDK_MODIFIER_RESERVED_25_MASK));
  EXPECT_EQ(call_records->len, 5u);
  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 2));
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(record->event->physical, kPhysicalAltLeft);
  EXPECT_EQ(record->event->logical, kLogicalAltLeft);
  EXPECT_EQ(record->event->synthesized, true);

  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 3));
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(record->event->physical, kPhysicalAltRight);
  EXPECT_EQ(record->event->logical, kLogicalMetaRight);
  EXPECT_EQ(record->event->synthesized, true);

  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 4));
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(record->event->physical, kPhysicalShiftLeft);
  EXPECT_EQ(record->event->logical, kLogicalShiftLeft);
  EXPECT_EQ(record->event->synthesized, false);

  send_key_event(kPress, GDK_KEY_ISO_Next_Group, kKeyCodeShiftLeft,
                 static_cast<GdkModifierType>(GDK_MOD1_MASK |
                                              GDK_MODIFIER_RESERVED_25_MASK));
  EXPECT_EQ(call_records->len, 6u);
  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 5));
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(record->event->physical, kPhysicalShiftLeft);
  EXPECT_EQ(record->event->logical, kLogicalGroupNext);
  EXPECT_EQ(record->event->synthesized, false);

  send_key_event(kRelease, GDK_KEY_ISO_Level3_Shift, kKeyCodeAltRight,
                 static_cast<GdkModifierType>(GDK_MOD1_MASK |
                                              GDK_MODIFIER_RESERVED_13_MASK |
                                              GDK_MODIFIER_RESERVED_25_MASK));
  EXPECT_EQ(call_records->len, 7u);
  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 6));
  EXPECT_EQ(record->event->physical, 0u);
  EXPECT_EQ(record->event->logical, 0u);

  send_key_event(kRelease, GDK_KEY_Shift_L, kKeyCodeShiftLeft,
                 static_cast<GdkModifierType>(GDK_MODIFIER_RESERVED_13_MASK |
                                              GDK_MODIFIER_RESERVED_25_MASK));
  EXPECT_EQ(call_records->len, 9u);
  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 7));
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(record->event->physical, kPhysicalAltLeft);
  EXPECT_EQ(record->event->logical, kLogicalAltLeft);
  EXPECT_EQ(record->event->synthesized, true);

  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 8));
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(record->event->physical, kPhysicalShiftLeft);
  EXPECT_EQ(record->event->logical, kLogicalGroupNext);
  EXPECT_EQ(record->event->synthesized, false);
}

// Shift + AltLeft results in GDK event whose keyval is MetaLeft but whose
// keycode is either AltLeft or Shift keycode (depending on which one was
// released last). The physical key is usually deduced from the keycode, but in
// this case (Shift + AltLeft) a correction is needed otherwise the physical
// key won't be the MetaLeft one.
// Regression test for https://github.com/flutter/flutter/issues/96082
TEST(FlKeyEmbedderResponderTest, HandlesShiftAltLeftIsMetaLeft) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);
  EXPECT_TRUE(fl_engine_start(engine, nullptr));

  g_autoptr(FlKeyEmbedderResponder) responder =
      fl_key_embedder_responder_new(engine);

  g_autoptr(GPtrArray) call_records =
      g_ptr_array_new_with_free_func(g_object_unref);
  fl_engine_get_embedder_api(engine)->SendKeyEvent = MOCK_ENGINE_PROC(
      SendKeyEvent,
      ([&call_records](auto engine, const FlutterKeyEvent* event,
                       FlutterKeyEventCallback callback, void* user_data) {
        g_ptr_array_add(call_records, fl_key_embedder_call_record_new(
                                          event, callback, user_data));
        callback(true, user_data);
        return kSuccess;
      }));

  guint32 now_time = 1;
  // A convenient shorthand to simulate events.
  auto send_key_event = [responder, &now_time](bool is_press, guint keyval,
                                               guint16 keycode,
                                               GdkModifierType state) {
    now_time += 1;
    g_autoptr(FlKeyEvent) event =
        fl_key_event_new(now_time, is_press, keycode, keyval, state, 0);
    g_autoptr(GMainLoop) loop = g_main_loop_new(nullptr, 0);
    fl_key_embedder_responder_handle_event(
        responder, event, 0, nullptr,
        [](GObject* object, GAsyncResult* result, gpointer user_data) {
          gboolean handled;
          EXPECT_TRUE(fl_key_embedder_responder_handle_event_finish(
              FL_KEY_EMBEDDER_RESPONDER(object), result, &handled, nullptr));
          EXPECT_EQ(handled, TRUE);

          g_main_loop_quit(static_cast<GMainLoop*>(user_data));
        },
        loop);
    g_main_loop_run(loop);
  };

  // ShiftLeft + AltLeft
  send_key_event(kPress, GDK_KEY_Shift_L, kKeyCodeShiftLeft,
                 GDK_MODIFIER_RESERVED_25_MASK);
  EXPECT_EQ(call_records->len, 1u);
  FlKeyEmbedderCallRecord* record =
      FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 0));
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(record->event->physical, kPhysicalShiftLeft);
  EXPECT_EQ(record->event->logical, kLogicalShiftLeft);
  EXPECT_EQ(record->event->synthesized, false);

  send_key_event(kPress, GDK_KEY_Meta_L, kKeyCodeAltLeft,
                 static_cast<GdkModifierType>(GDK_SHIFT_MASK |
                                              GDK_MODIFIER_RESERVED_25_MASK));
  EXPECT_EQ(call_records->len, 2u);
  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 1));
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(record->event->physical, kPhysicalMetaLeft);
  EXPECT_EQ(record->event->logical, kLogicalMetaLeft);
  EXPECT_EQ(record->event->synthesized, false);

  send_key_event(kRelease, GDK_KEY_Meta_L, kKeyCodeAltLeft,
                 static_cast<GdkModifierType>(GDK_MODIFIER_RESERVED_13_MASK |
                                              GDK_MODIFIER_RESERVED_25_MASK));
  send_key_event(kRelease, GDK_KEY_Shift_L, kKeyCodeShiftLeft,
                 GDK_MODIFIER_RESERVED_25_MASK);
  clear_records(call_records);

  // ShiftRight + AltLeft
  send_key_event(kPress, GDK_KEY_Shift_R, kKeyCodeShiftRight,
                 GDK_MODIFIER_RESERVED_25_MASK);
  EXPECT_EQ(call_records->len, 1u);
  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 0));
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(record->event->physical, kPhysicalShiftRight);
  EXPECT_EQ(record->event->logical, kLogicalShiftRight);
  EXPECT_EQ(record->event->synthesized, false);

  send_key_event(kPress, GDK_KEY_Meta_L, kKeyCodeAltLeft,
                 static_cast<GdkModifierType>(GDK_SHIFT_MASK |
                                              GDK_MODIFIER_RESERVED_25_MASK));
  EXPECT_EQ(call_records->len, 2u);
  record = FL_KEY_EMBEDDER_CALL_RECORD(g_ptr_array_index(call_records, 1));
  EXPECT_EQ(record->event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(record->event->physical, kPhysicalMetaLeft);
  EXPECT_EQ(record->event->logical, kLogicalMetaLeft);
  EXPECT_EQ(record->event->synthesized, false);
}
