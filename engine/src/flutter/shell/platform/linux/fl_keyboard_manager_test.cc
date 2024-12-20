// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/fl_keyboard_manager.h"

#include <cstring>
#include <vector>

#include "flutter/shell/platform/embedder/test_utils/key_codes.g.h"
#include "flutter/shell/platform/linux/fl_binary_messenger_private.h"
#include "flutter/shell/platform/linux/fl_engine_private.h"
#include "flutter/shell/platform/linux/fl_method_codec_private.h"
#include "flutter/shell/platform/linux/key_mapping.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_json_message_codec.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_method_codec.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_standard_method_codec.h"
#include "flutter/shell/platform/linux/testing/fl_mock_binary_messenger.h"
#include "flutter/shell/platform/linux/testing/mock_keymap.h"
#include "flutter/testing/testing.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

// Define compound `expect` in macros. If they were defined in functions, the
// stacktrace wouldn't print where the function is called in the unit tests.

#define EXPECT_KEY_EVENT(RECORD, TYPE, PHYSICAL, LOGICAL, CHAR, SYNTHESIZED) \
  EXPECT_EQ((RECORD).type, CallRecord::KEY_CALL_EMBEDDER);                   \
  EXPECT_EQ((RECORD).event->type, (TYPE));                                   \
  EXPECT_EQ((RECORD).event->physical, (PHYSICAL));                           \
  EXPECT_EQ((RECORD).event->logical, (LOGICAL));                             \
  EXPECT_STREQ((RECORD).event->character, (CHAR));                           \
  EXPECT_EQ((RECORD).event->synthesized, (SYNTHESIZED));

#define VERIFY_DOWN(OUT_LOGICAL, OUT_CHAR)                          \
  EXPECT_EQ(call_records[0].type, CallRecord::KEY_CALL_EMBEDDER);   \
  EXPECT_EQ(call_records[0].event->type, kFlutterKeyEventTypeDown); \
  EXPECT_EQ(call_records[0].event->logical, (OUT_LOGICAL));         \
  EXPECT_STREQ(call_records[0].event->character, (OUT_CHAR));       \
  EXPECT_EQ(call_records[0].event->synthesized, false);             \
  call_records.clear()

namespace {
using ::flutter::testing::keycodes::kLogicalAltLeft;
using ::flutter::testing::keycodes::kLogicalBracketLeft;
using ::flutter::testing::keycodes::kLogicalComma;
using ::flutter::testing::keycodes::kLogicalControlLeft;
using ::flutter::testing::keycodes::kLogicalDigit1;
using ::flutter::testing::keycodes::kLogicalKeyA;
using ::flutter::testing::keycodes::kLogicalKeyB;
using ::flutter::testing::keycodes::kLogicalKeyM;
using ::flutter::testing::keycodes::kLogicalKeyQ;
using ::flutter::testing::keycodes::kLogicalMetaLeft;
using ::flutter::testing::keycodes::kLogicalMinus;
using ::flutter::testing::keycodes::kLogicalParenthesisRight;
using ::flutter::testing::keycodes::kLogicalSemicolon;
using ::flutter::testing::keycodes::kLogicalShiftLeft;
using ::flutter::testing::keycodes::kLogicalUnderscore;

using ::flutter::testing::keycodes::kPhysicalAltLeft;
using ::flutter::testing::keycodes::kPhysicalControlLeft;
using ::flutter::testing::keycodes::kPhysicalKeyA;
using ::flutter::testing::keycodes::kPhysicalKeyB;
using ::flutter::testing::keycodes::kPhysicalMetaLeft;
using ::flutter::testing::keycodes::kPhysicalShiftLeft;

// A type that can record all kinds of effects that the keyboard handler
// triggers.
//
// An instance of `CallRecord` might not have all the fields filled.
typedef struct {
  enum {
    KEY_CALL_EMBEDDER,
    KEY_CALL_CHANNEL,
  } type;

  FlutterKeyEventCallback callback;
  void* callback_user_data;
  std::unique_ptr<FlutterKeyEvent> event;
  std::unique_ptr<char[]> event_character;
} CallRecord;

static void call_callback(CallRecord& record, bool handled) {
  if (record.callback != nullptr) {
    record.callback(handled, record.callback_user_data);
  }
}

// Clone a C-string.
//
// Must be deleted by delete[].
char* cloneString(const char* source) {
  if (source == nullptr) {
    return nullptr;
  }
  size_t charLen = strlen(source);
  char* target = new char[charLen + 1];
  strncpy(target, source, charLen + 1);
  return target;
}

constexpr guint16 kKeyCodeKeyA = 0x26u;
constexpr guint16 kKeyCodeKeyB = 0x38u;
constexpr guint16 kKeyCodeKeyM = 0x3au;
constexpr guint16 kKeyCodeDigit1 = 0x0au;
constexpr guint16 kKeyCodeMinus = 0x14u;
constexpr guint16 kKeyCodeSemicolon = 0x2fu;
constexpr guint16 kKeyCodeKeyLeftBracket = 0x22u;

static constexpr char kKeyEventChannelName[] = "flutter/keyevent";

// All key clues for a keyboard layout.
//
// The index is (keyCode * 2 + hasShift), where each value is the character for
// this key (GTK only supports UTF-16.) Since the maximum keycode of interest
// is 128, it has a total of 256 entries..
typedef std::array<uint32_t, 256> MockGroupLayoutData;
typedef std::vector<const MockGroupLayoutData*> MockLayoutData;

extern const MockLayoutData kLayoutUs;
extern const MockLayoutData kLayoutRussian;
extern const MockLayoutData kLayoutFrench;

G_BEGIN_DECLS

G_DECLARE_FINAL_TYPE(FlMockViewDelegate,
                     fl_mock_view_delegate,
                     FL,
                     MOCK_VIEW_DELEGATE,
                     GObject);

G_DECLARE_FINAL_TYPE(FlMockKeyBinaryMessenger,
                     fl_mock_key_binary_messenger,
                     FL,
                     MOCK_KEY_BINARY_MESSENGER,
                     GObject)

G_END_DECLS

/***** FlMockKeyBinaryMessenger *****/
/* Mock a binary messenger that only processes messages from the embedding on
 * the key event channel, and does so according to the callback set by
 * fl_mock_key_binary_messenger_set_callback_handler */

struct _FlMockKeyBinaryMessenger {
  GObject parent_instance;

  void (*callback)(FlutterKeyEventCallback callback,
                   gpointer callback_user_data,
                   gpointer user_data);
  gpointer callback_user_data;
};

static void fl_mock_key_binary_messenger_iface_init(
    FlBinaryMessengerInterface* iface);

G_DEFINE_TYPE_WITH_CODE(
    FlMockKeyBinaryMessenger,
    fl_mock_key_binary_messenger,
    G_TYPE_OBJECT,
    G_IMPLEMENT_INTERFACE(fl_binary_messenger_get_type(),
                          fl_mock_key_binary_messenger_iface_init))

static void fl_mock_key_binary_messenger_init(FlMockKeyBinaryMessenger* self) {}

static void fl_mock_key_binary_messenger_dispose(GObject* object) {
  G_OBJECT_CLASS(fl_mock_key_binary_messenger_parent_class)->dispose(object);
}

static void fl_mock_key_binary_messenger_class_init(
    FlMockKeyBinaryMessengerClass* klass) {
  G_OBJECT_CLASS(klass)->dispose = fl_mock_key_binary_messenger_dispose;
}

static void fl_mock_key_binary_messenger_send_on_channel(
    FlBinaryMessenger* messenger,
    const gchar* channel,
    GBytes* message,
    GCancellable* cancellable,
    GAsyncReadyCallback callback,
    gpointer user_data) {
  FlMockKeyBinaryMessenger* self = FL_MOCK_KEY_BINARY_MESSENGER(messenger);

  if (callback != nullptr) {
    EXPECT_STREQ(channel, kKeyEventChannelName);
    self->callback(
        [](bool handled, gpointer user_data) {
          g_autoptr(GTask) task = static_cast<GTask*>(user_data);
          g_autoptr(FlValue) result = fl_value_new_map();
          fl_value_set_string_take(result, "handled",
                                   fl_value_new_bool(handled));
          g_autoptr(FlJsonMessageCodec) codec = fl_json_message_codec_new();
          g_autoptr(GError) error = nullptr;
          GBytes* message_data = fl_message_codec_encode_message(
              FL_MESSAGE_CODEC(codec), result, &error);

          g_task_return_pointer(
              task, message_data,
              reinterpret_cast<GDestroyNotify>(g_bytes_unref));
        },
        g_task_new(self, cancellable, callback, user_data),
        self->callback_user_data);
  }
}

static GBytes* fl_mock_key_binary_messenger_send_on_channel_finish(
    FlBinaryMessenger* messenger,
    GAsyncResult* result,
    GError** error) {
  return static_cast<GBytes*>(g_task_propagate_pointer(G_TASK(result), error));
}

static void fl_mock_binary_messenger_resize_channel(
    FlBinaryMessenger* messenger,
    const gchar* channel,
    int64_t new_size) {
  // Mock implementation. Do nothing.
}

static void fl_mock_binary_messenger_set_warns_on_channel_overflow(
    FlBinaryMessenger* messenger,
    const gchar* channel,
    bool warns) {
  // Mock implementation. Do nothing.
}

static void fl_mock_binary_messenger_shutdown(FlBinaryMessenger* messenger) {
  // Mock implementation. Do nothing.
}

static void fl_mock_key_binary_messenger_iface_init(
    FlBinaryMessengerInterface* iface) {
  iface->set_message_handler_on_channel =
      [](FlBinaryMessenger* messenger, const gchar* channel,
         FlBinaryMessengerMessageHandler handler, gpointer user_data,
         GDestroyNotify destroy_notify) {
        EXPECT_STREQ(channel, kKeyEventChannelName);
        // No need to mock. The key event channel expects no incoming messages
        // from the framework.
      };
  iface->send_response = [](FlBinaryMessenger* messenger,
                            FlBinaryMessengerResponseHandle* response_handle,
                            GBytes* response, GError** error) -> gboolean {
    // The key event channel expects no incoming messages from the framework,
    // hence no responses either.
    g_return_val_if_reached(TRUE);
    return TRUE;
  };
  iface->send_on_channel = fl_mock_key_binary_messenger_send_on_channel;
  iface->send_on_channel_finish =
      fl_mock_key_binary_messenger_send_on_channel_finish;
  iface->resize_channel = fl_mock_binary_messenger_resize_channel;
  iface->set_warns_on_channel_overflow =
      fl_mock_binary_messenger_set_warns_on_channel_overflow;
  iface->shutdown = fl_mock_binary_messenger_shutdown;
}

static FlMockKeyBinaryMessenger* fl_mock_key_binary_messenger_new() {
  FlMockKeyBinaryMessenger* self = FL_MOCK_KEY_BINARY_MESSENGER(
      g_object_new(fl_mock_key_binary_messenger_get_type(), NULL));

  // Added to stop compiler complaining about an unused function.
  FL_IS_MOCK_KEY_BINARY_MESSENGER(self);

  return self;
}

static void fl_mock_key_binary_messenger_set_callback_handler(
    FlMockKeyBinaryMessenger* self,
    void (*callback)(FlutterKeyEventCallback callback,
                     gpointer callback_user_data,
                     gpointer user_data),
    gpointer user_data) {
  self->callback = callback;
  self->callback_user_data = user_data;
}

/***** FlMockViewDelegate *****/

struct _FlMockViewDelegate {
  GObject parent_instance;
  bool text_filter_result;
};

static void fl_mock_view_keyboard_delegate_iface_init(
    FlKeyboardViewDelegateInterface* iface);

G_DEFINE_TYPE_WITH_CODE(
    FlMockViewDelegate,
    fl_mock_view_delegate,
    G_TYPE_OBJECT,
    G_IMPLEMENT_INTERFACE(fl_keyboard_view_delegate_get_type(),
                          fl_mock_view_keyboard_delegate_iface_init))

static void fl_mock_view_delegate_init(FlMockViewDelegate* self) {}

static void fl_mock_view_delegate_class_init(FlMockViewDelegateClass* klass) {}

static gboolean fl_mock_view_keyboard_text_filter_key_press(
    FlKeyboardViewDelegate* view_delegate,
    FlKeyEvent* event) {
  FlMockViewDelegate* self = FL_MOCK_VIEW_DELEGATE(view_delegate);
  return self->text_filter_result;
}

static void fl_mock_view_keyboard_delegate_iface_init(
    FlKeyboardViewDelegateInterface* iface) {
  iface->text_filter_key_press = fl_mock_view_keyboard_text_filter_key_press;
}

static FlMockViewDelegate* fl_mock_view_delegate_new() {
  FlMockViewDelegate* self = FL_MOCK_VIEW_DELEGATE(
      g_object_new(fl_mock_view_delegate_get_type(), nullptr));

  // Added to stop compiler complaining about an unused function.
  FL_IS_MOCK_VIEW_DELEGATE(self);

  return self;
}

static void fl_mock_view_set_text_filter_result(FlMockViewDelegate* self,
                                                bool result) {
  self->text_filter_result = result;
}

/***** End FlMockViewDelegate *****/

// Block until all GdkMainLoop messages are processed, which is basically used
// only for channel messages.
static void flush_channel_messages() {
  GMainLoop* loop = g_main_loop_new(nullptr, 0);
  g_idle_add(
      [](gpointer data) {
        g_autoptr(GMainLoop) loop = reinterpret_cast<GMainLoop*>(data);
        g_main_loop_quit(loop);
        return FALSE;
      },
      loop);
  g_main_loop_run(loop);
}

// Dispatch each of the given events, expect their results to be false
// (unhandled), and clear the event array.
//
// Returns the number of events redispatched. If any result is unexpected
// (handled), return a minus number `-x` instead, where `x` is the index of
// the first unexpected redispatch.
int redispatch_events_and_clear(FlKeyboardManager* manager, GPtrArray* events) {
  guint event_count = events->len;
  int first_error = -1;
  for (guint event_id = 0; event_id < event_count; event_id += 1) {
    FlKeyEvent* event = FL_KEY_EVENT(g_ptr_array_index(events, event_id));
    bool handled = fl_keyboard_manager_handle_event(manager, event);
    EXPECT_FALSE(handled);
    if (handled) {
      first_error = first_error == -1 ? event_id : first_error;
    }
  }
  g_ptr_array_set_size(events, 0);
  return first_error < 0 ? event_count : -first_error;
}

// Make sure that the keyboard can be disposed without crashes when there are
// unresolved pending events.
TEST(FlKeyboardManagerTest, DisposeWithUnresolvedPends) {
  ::testing::NiceMock<flutter::testing::MockKeymap> mock_keymap;

  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);
  g_autoptr(FlMockViewDelegate) view = fl_mock_view_delegate_new();
  g_autoptr(FlKeyboardManager) manager =
      fl_keyboard_manager_new(engine, FL_KEYBOARD_VIEW_DELEGATE(view));

  g_autoptr(GError) error = nullptr;
  EXPECT_TRUE(fl_engine_start(engine, &error));
  EXPECT_EQ(error, nullptr);

  // Don't handle first event.
  fl_keyboard_manager_set_send_key_event_handler(
      manager,
      [](const FlutterKeyEvent* event, FlutterKeyEventCallback callback,
         void* callback_user_data, gpointer user_data) {},
      nullptr);
  g_autoptr(FlKeyEvent) event1 = fl_key_event_new(
      0, TRUE, kKeyCodeKeyA, GDK_KEY_a, static_cast<GdkModifierType>(0), 0);
  fl_keyboard_manager_handle_event(manager, event1);

  // Handle second event.
  fl_keyboard_manager_set_send_key_event_handler(
      manager,
      [](const FlutterKeyEvent* event, FlutterKeyEventCallback callback,
         void* callback_user_data,
         gpointer user_data) { callback(true, callback_user_data); },
      nullptr);
  g_autoptr(FlKeyEvent) event2 = fl_key_event_new(
      0, FALSE, kKeyCodeKeyA, GDK_KEY_a, static_cast<GdkModifierType>(0), 0);
  fl_keyboard_manager_handle_event(manager, event2);

  // Passes if the cleanup does not crash.
}

TEST(FlKeyboardManagerTest, SingleDelegateWithAsyncResponds) {
  ::testing::NiceMock<flutter::testing::MockKeymap> mock_keymap;
  g_autoptr(FlMockKeyBinaryMessenger) messenger =
      fl_mock_key_binary_messenger_new();
  g_autoptr(FlMockViewDelegate) view = fl_mock_view_delegate_new();
  g_autoptr(FlEngine) engine =
      FL_ENGINE(g_object_new(fl_engine_get_type(), "binary-messenger",
                             FL_BINARY_MESSENGER(messenger), nullptr));
  g_autoptr(FlKeyboardManager) manager =
      fl_keyboard_manager_new(engine, FL_KEYBOARD_VIEW_DELEGATE(view));
  fl_keyboard_manager_set_lookup_key_handler(
      manager,
      [](const GdkKeymapKey* key, gpointer user_data) {
        MockLayoutData* layout_data = static_cast<MockLayoutData*>(user_data);
        guint8 group = static_cast<guint8>(key->group);
        EXPECT_LT(group, layout_data->size());
        const MockGroupLayoutData* group_layout = (*layout_data)[group];
        EXPECT_NE(group_layout, nullptr);
        EXPECT_TRUE(key->level == 0 || key->level == 1);
        bool shift = key->level == 1;
        return (*group_layout)[key->keycode * 2 + shift];
      },
      (gpointer)(&kLayoutUs));

  std::vector<CallRecord> call_records;
  g_autoptr(GPtrArray) redispatched =
      g_ptr_array_new_with_free_func(g_object_unref);

  fl_mock_key_binary_messenger_set_callback_handler(
      messenger,
      [](FlutterKeyEventCallback callback, gpointer callback_user_data,
         gpointer user_data) { callback(false, callback_user_data); },
      nullptr);

  /// Test 1: One event that is handled by the framework
  fl_keyboard_manager_set_send_key_event_handler(
      manager,
      [](const FlutterKeyEvent* event, FlutterKeyEventCallback callback,
         void* callback_user_data, gpointer user_data) {
        std::vector<CallRecord>* call_records =
            static_cast<std::vector<CallRecord>*>(user_data);
        auto new_event = std::make_unique<FlutterKeyEvent>(*event);
        char* new_event_character = cloneString(event->character);
        new_event->character = new_event_character;
        call_records->push_back(CallRecord{
            .type = CallRecord::KEY_CALL_EMBEDDER,
            .callback = callback,
            .callback_user_data = callback_user_data,
            .event = std::move(new_event),
            .event_character = std::unique_ptr<char[]>(new_event_character),
        });
      },
      &call_records);
  fl_keyboard_manager_set_redispatch_handler(
      manager,
      [](FlKeyEvent* event, gpointer user_data) {
        GPtrArray* redispatched = static_cast<GPtrArray*>(user_data);
        g_ptr_array_add(redispatched, g_object_ref(event));
      },
      redispatched);

  // Dispatch a key event
  g_autoptr(FlKeyEvent) event1 = fl_key_event_new(
      0, TRUE, kKeyCodeKeyA, GDK_KEY_a, static_cast<GdkModifierType>(0), 0);
  EXPECT_TRUE(fl_keyboard_manager_handle_event(manager, event1));
  flush_channel_messages();
  EXPECT_EQ(redispatched->len, 0u);
  EXPECT_EQ(call_records.size(), 1u);
  EXPECT_KEY_EVENT(call_records[0], kFlutterKeyEventTypeDown, kPhysicalKeyA,
                   kLogicalKeyA, "a", false);

  call_callback(call_records[0], true);
  flush_channel_messages();
  EXPECT_EQ(redispatched->len, 0u);
  EXPECT_TRUE(fl_keyboard_manager_is_state_clear(manager));
  call_records.clear();

  /// Test 2: Two events that are unhandled by the framework
  g_autoptr(FlKeyEvent) event2 = fl_key_event_new(
      0, FALSE, kKeyCodeKeyA, GDK_KEY_a, static_cast<GdkModifierType>(0), 0);
  EXPECT_TRUE(fl_keyboard_manager_handle_event(manager, event2));
  flush_channel_messages();
  EXPECT_EQ(redispatched->len, 0u);
  EXPECT_EQ(call_records.size(), 1u);
  EXPECT_KEY_EVENT(call_records[0], kFlutterKeyEventTypeUp, kPhysicalKeyA,
                   kLogicalKeyA, nullptr, false);

  // Dispatch another key event
  g_autoptr(FlKeyEvent) event3 = fl_key_event_new(
      0, TRUE, kKeyCodeKeyB, GDK_KEY_b, static_cast<GdkModifierType>(0), 0);
  EXPECT_TRUE(fl_keyboard_manager_handle_event(manager, event3));
  flush_channel_messages();
  EXPECT_EQ(redispatched->len, 0u);
  EXPECT_EQ(call_records.size(), 2u);
  EXPECT_KEY_EVENT(call_records[1], kFlutterKeyEventTypeDown, kPhysicalKeyB,
                   kLogicalKeyB, "b", false);

  // Resolve the second event first to test out-of-order response
  call_callback(call_records[1], false);
  EXPECT_EQ(redispatched->len, 1u);
  EXPECT_EQ(
      fl_key_event_get_keyval(FL_KEY_EVENT(g_ptr_array_index(redispatched, 0))),
      0x62u);
  call_callback(call_records[0], false);
  flush_channel_messages();
  EXPECT_EQ(redispatched->len, 2u);
  EXPECT_EQ(
      fl_key_event_get_keyval(FL_KEY_EVENT(g_ptr_array_index(redispatched, 1))),
      0x61u);

  EXPECT_FALSE(fl_keyboard_manager_is_state_clear(manager));
  call_records.clear();

  // Resolve redispatches
  EXPECT_EQ(redispatch_events_and_clear(manager, redispatched), 2);
  flush_channel_messages();
  EXPECT_EQ(call_records.size(), 0u);
  EXPECT_TRUE(fl_keyboard_manager_is_state_clear(manager));

  /// Test 3: Dispatch the same event again to ensure that prevention from
  /// redispatching only works once.
  g_autoptr(FlKeyEvent) event4 = fl_key_event_new(
      0, FALSE, kKeyCodeKeyA, GDK_KEY_a, static_cast<GdkModifierType>(0), 0);
  EXPECT_TRUE(fl_keyboard_manager_handle_event(manager, event4));
  flush_channel_messages();
  EXPECT_EQ(redispatched->len, 0u);
  EXPECT_EQ(call_records.size(), 1u);

  call_callback(call_records[0], true);
  EXPECT_TRUE(fl_keyboard_manager_is_state_clear(manager));
}

TEST(FlKeyboardManagerTest, SingleDelegateWithSyncResponds) {
  ::testing::NiceMock<flutter::testing::MockKeymap> mock_keymap;
  g_autoptr(FlMockKeyBinaryMessenger) messenger =
      fl_mock_key_binary_messenger_new();
  g_autoptr(FlMockViewDelegate) view = fl_mock_view_delegate_new();
  g_autoptr(FlEngine) engine =
      FL_ENGINE(g_object_new(fl_engine_get_type(), "binary-messenger",
                             FL_BINARY_MESSENGER(messenger), nullptr));
  g_autoptr(FlKeyboardManager) manager =
      fl_keyboard_manager_new(engine, FL_KEYBOARD_VIEW_DELEGATE(view));
  fl_keyboard_manager_set_lookup_key_handler(
      manager,
      [](const GdkKeymapKey* key, gpointer user_data) {
        MockLayoutData* layout_data = static_cast<MockLayoutData*>(user_data);
        guint8 group = static_cast<guint8>(key->group);
        EXPECT_LT(group, layout_data->size());
        const MockGroupLayoutData* group_layout = (*layout_data)[group];
        EXPECT_NE(group_layout, nullptr);
        EXPECT_TRUE(key->level == 0 || key->level == 1);
        bool shift = key->level == 1;
        return (*group_layout)[key->keycode * 2 + shift];
      },
      (gpointer)(&kLayoutUs));

  std::vector<CallRecord> call_records;
  g_autoptr(GPtrArray) redispatched =
      g_ptr_array_new_with_free_func(g_object_unref);

  fl_mock_key_binary_messenger_set_callback_handler(
      messenger,
      [](FlutterKeyEventCallback callback, gpointer callback_user_data,
         gpointer user_data) { callback(false, callback_user_data); },
      nullptr);

  /// Test 1: One event that is handled by the framework
  fl_keyboard_manager_set_send_key_event_handler(
      manager,
      [](const FlutterKeyEvent* event, FlutterKeyEventCallback callback,
         void* callback_user_data, gpointer user_data) {
        std::vector<CallRecord>* call_records =
            static_cast<std::vector<CallRecord>*>(user_data);
        auto new_event = std::make_unique<FlutterKeyEvent>(*event);
        char* new_event_character = cloneString(event->character);
        new_event->character = new_event_character;
        call_records->push_back(CallRecord{
            .type = CallRecord::KEY_CALL_EMBEDDER,
            .callback = callback,
            .callback_user_data = callback_user_data,
            .event = std::move(new_event),
            .event_character = std::unique_ptr<char[]>(new_event_character),
        });
        callback(true, callback_user_data);
      },
      &call_records);
  fl_keyboard_manager_set_redispatch_handler(
      manager,
      [](FlKeyEvent* event, gpointer user_data) {
        GPtrArray* redispatched = static_cast<GPtrArray*>(user_data);
        g_ptr_array_add(redispatched, g_object_ref(event));
      },
      redispatched);

  // Dispatch a key event
  g_autoptr(FlKeyEvent) event1 = fl_key_event_new(
      0, TRUE, kKeyCodeKeyA, GDK_KEY_a, static_cast<GdkModifierType>(0), 0);
  EXPECT_TRUE(fl_keyboard_manager_handle_event(manager, event1));
  flush_channel_messages();
  EXPECT_EQ(call_records.size(), 1u);
  EXPECT_KEY_EVENT(call_records[0], kFlutterKeyEventTypeDown, kPhysicalKeyA,
                   kLogicalKeyA, "a", false);
  EXPECT_EQ(redispatched->len, 0u);
  call_records.clear();

  EXPECT_TRUE(fl_keyboard_manager_is_state_clear(manager));
  g_ptr_array_set_size(redispatched, 0);

  /// Test 2: An event unhandled by the framework
  fl_keyboard_manager_set_send_key_event_handler(
      manager,
      [](const FlutterKeyEvent* event, FlutterKeyEventCallback callback,
         void* callback_user_data, gpointer user_data) {
        std::vector<CallRecord>* call_records =
            static_cast<std::vector<CallRecord>*>(user_data);
        auto new_event = std::make_unique<FlutterKeyEvent>(*event);
        char* new_event_character = cloneString(event->character);
        new_event->character = new_event_character;
        call_records->push_back(CallRecord{
            .type = CallRecord::KEY_CALL_EMBEDDER,
            .callback = callback,
            .callback_user_data = callback_user_data,
            .event = std::move(new_event),
            .event_character = std::unique_ptr<char[]>(new_event_character),
        });
        callback(false, callback_user_data);
      },
      &call_records);
  g_autoptr(FlKeyEvent) event2 = fl_key_event_new(
      0, FALSE, kKeyCodeKeyA, GDK_KEY_a, static_cast<GdkModifierType>(0), 0);
  EXPECT_TRUE(fl_keyboard_manager_handle_event(manager, event2));
  flush_channel_messages();
  EXPECT_EQ(call_records.size(), 1u);
  EXPECT_KEY_EVENT(call_records[0], kFlutterKeyEventTypeUp, kPhysicalKeyA,
                   kLogicalKeyA, nullptr, false);
  EXPECT_EQ(redispatched->len, 1u);
  call_records.clear();

  EXPECT_FALSE(fl_keyboard_manager_is_state_clear(manager));

  EXPECT_EQ(redispatch_events_and_clear(manager, redispatched), 1);
  EXPECT_EQ(call_records.size(), 0u);

  EXPECT_TRUE(fl_keyboard_manager_is_state_clear(manager));
}

TEST(FlKeyboardManagerTest, WithTwoAsyncDelegates) {
  ::testing::NiceMock<flutter::testing::MockKeymap> mock_keymap;
  g_autoptr(FlMockKeyBinaryMessenger) messenger =
      fl_mock_key_binary_messenger_new();
  g_autoptr(FlMockViewDelegate) view = fl_mock_view_delegate_new();
  g_autoptr(FlEngine) engine =
      FL_ENGINE(g_object_new(fl_engine_get_type(), "binary-messenger",
                             FL_BINARY_MESSENGER(messenger), nullptr));
  g_autoptr(FlKeyboardManager) manager =
      fl_keyboard_manager_new(engine, FL_KEYBOARD_VIEW_DELEGATE(view));
  fl_keyboard_manager_set_lookup_key_handler(
      manager,
      [](const GdkKeymapKey* key, gpointer user_data) {
        MockLayoutData* layout_data = static_cast<MockLayoutData*>(user_data);
        guint8 group = static_cast<guint8>(key->group);
        EXPECT_LT(group, layout_data->size());
        const MockGroupLayoutData* group_layout = (*layout_data)[group];
        EXPECT_NE(group_layout, nullptr);
        EXPECT_TRUE(key->level == 0 || key->level == 1);
        bool shift = key->level == 1;
        return (*group_layout)[key->keycode * 2 + shift];
      },
      (gpointer)(&kLayoutUs));

  std::vector<CallRecord> call_records;
  g_autoptr(GPtrArray) redispatched =
      g_ptr_array_new_with_free_func(g_object_unref);

  fl_keyboard_manager_set_send_key_event_handler(
      manager,
      [](const FlutterKeyEvent* event, FlutterKeyEventCallback callback,
         void* callback_user_data, gpointer user_data) {
        std::vector<CallRecord>* call_records =
            static_cast<std::vector<CallRecord>*>(user_data);
        auto new_event = std::make_unique<FlutterKeyEvent>(*event);
        char* new_event_character = cloneString(event->character);
        new_event->character = new_event_character;
        call_records->push_back(CallRecord{
            .type = CallRecord::KEY_CALL_EMBEDDER,
            .callback = callback,
            .callback_user_data = callback_user_data,
            .event = std::move(new_event),
            .event_character = std::unique_ptr<char[]>(new_event_character),
        });
      },
      &call_records);
  fl_mock_key_binary_messenger_set_callback_handler(
      messenger,
      [](FlutterKeyEventCallback callback, gpointer callback_user_data,
         gpointer user_data) {
        std::vector<CallRecord>* call_records =
            static_cast<std::vector<CallRecord>*>(user_data);
        call_records->push_back(CallRecord{
            .type = CallRecord::KEY_CALL_CHANNEL,
            .callback = callback,
            .callback_user_data = callback_user_data,
        });
      },
      &call_records);
  fl_keyboard_manager_set_redispatch_handler(
      manager,
      [](FlKeyEvent* event, gpointer user_data) {
        GPtrArray* redispatched = static_cast<GPtrArray*>(user_data);
        g_ptr_array_add(redispatched, g_object_ref(event));
      },
      redispatched);

  /// Test 1: One delegate responds true, the other false

  g_autoptr(FlKeyEvent) event1 = fl_key_event_new(
      0, TRUE, kKeyCodeKeyA, GDK_KEY_a, static_cast<GdkModifierType>(0), 0);
  EXPECT_TRUE(fl_keyboard_manager_handle_event(manager, event1));

  EXPECT_EQ(redispatched->len, 0u);
  EXPECT_EQ(call_records.size(), 2u);

  EXPECT_EQ(call_records[0].type, CallRecord::KEY_CALL_EMBEDDER);
  EXPECT_EQ(call_records[1].type, CallRecord::KEY_CALL_CHANNEL);

  call_callback(call_records[0], true);
  call_callback(call_records[1], false);
  flush_channel_messages();
  EXPECT_EQ(redispatched->len, 0u);

  EXPECT_TRUE(fl_keyboard_manager_is_state_clear(manager));
  call_records.clear();

  /// Test 2: All delegates respond false
  g_autoptr(FlKeyEvent) event2 = fl_key_event_new(
      0, FALSE, kKeyCodeKeyA, GDK_KEY_a, static_cast<GdkModifierType>(0), 0);
  EXPECT_TRUE(fl_keyboard_manager_handle_event(manager, event2));

  EXPECT_EQ(redispatched->len, 0u);
  EXPECT_EQ(call_records.size(), 2u);

  EXPECT_EQ(call_records[0].type, CallRecord::KEY_CALL_EMBEDDER);
  EXPECT_EQ(call_records[1].type, CallRecord::KEY_CALL_CHANNEL);

  call_callback(call_records[0], false);
  call_callback(call_records[1], false);

  call_records.clear();

  // Resolve redispatch
  flush_channel_messages();
  EXPECT_EQ(redispatched->len, 1u);
  EXPECT_EQ(redispatch_events_and_clear(manager, redispatched), 1);
  EXPECT_EQ(call_records.size(), 0u);

  EXPECT_TRUE(fl_keyboard_manager_is_state_clear(manager));
}

TEST(FlKeyboardManagerTest, TextInputHandlerReturnsFalse) {
  ::testing::NiceMock<flutter::testing::MockKeymap> mock_keymap;

  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();
  fl_mock_binary_messenger_set_json_message_channel(
      messenger, "flutter/keyevent",
      [](FlMockBinaryMessenger* messenger, GTask* task, FlValue* message,
         gpointer user_data) {
        g_autoptr(FlValue) return_value = fl_value_new_map();
        fl_value_set_string_take(return_value, "handled",
                                 fl_value_new_bool(FALSE));
        return fl_value_ref(return_value);
      },
      nullptr);
  g_autoptr(FlEngine) engine =
      FL_ENGINE(g_object_new(fl_engine_get_type(), "binary-messenger",
                             FL_BINARY_MESSENGER(messenger), nullptr));
  g_autoptr(FlMockViewDelegate) view = fl_mock_view_delegate_new();
  g_autoptr(FlKeyboardManager) manager =
      fl_keyboard_manager_new(engine, FL_KEYBOARD_VIEW_DELEGATE(view));

  // Dispatch a key event.
  fl_mock_view_set_text_filter_result(view, FALSE);
  gboolean redispatched = FALSE;
  fl_keyboard_manager_set_redispatch_handler(
      manager,
      [](FlKeyEvent* event, gpointer user_data) {
        gboolean* redispatched = static_cast<gboolean*>(user_data);
        *redispatched = TRUE;
      },
      &redispatched);
  fl_keyboard_manager_set_send_key_event_handler(
      manager,
      [](const FlutterKeyEvent* event, FlutterKeyEventCallback callback,
         void* callback_user_data,
         gpointer user_data) { callback(false, callback_user_data); },
      nullptr);
  g_autoptr(FlKeyEvent) event = fl_key_event_new(
      0, TRUE, kKeyCodeKeyA, GDK_KEY_a, static_cast<GdkModifierType>(0), 0);
  EXPECT_TRUE(fl_keyboard_manager_handle_event(manager, event));
  flush_channel_messages();
  // The event was redispatched because no one handles it.
  EXPECT_TRUE(redispatched);
}

TEST(FlKeyboardManagerTest, TextInputHandlerReturnsTrue) {
  ::testing::NiceMock<flutter::testing::MockKeymap> mock_keymap;

  g_autoptr(FlMockBinaryMessenger) messenger = fl_mock_binary_messenger_new();
  fl_mock_binary_messenger_set_json_message_channel(
      messenger, "flutter/keyevent",
      [](FlMockBinaryMessenger* messenger, GTask* task, FlValue* message,
         gpointer user_data) {
        g_autoptr(FlValue) return_value = fl_value_new_map();
        fl_value_set_string_take(return_value, "handled",
                                 fl_value_new_bool(FALSE));
        return fl_value_ref(return_value);
      },
      nullptr);
  g_autoptr(FlEngine) engine =
      FL_ENGINE(g_object_new(fl_engine_get_type(), "binary-messenger",
                             FL_BINARY_MESSENGER(messenger), nullptr));
  g_autoptr(FlMockViewDelegate) view = fl_mock_view_delegate_new();
  g_autoptr(FlKeyboardManager) manager =
      fl_keyboard_manager_new(engine, FL_KEYBOARD_VIEW_DELEGATE(view));

  // Dispatch a key event.
  fl_mock_view_set_text_filter_result(view, TRUE);
  gboolean redispatched = FALSE;
  fl_keyboard_manager_set_redispatch_handler(
      manager,
      [](FlKeyEvent* event, gpointer user_data) {
        gboolean* redispatched = static_cast<gboolean*>(user_data);
        *redispatched = TRUE;
      },
      &redispatched);
  fl_keyboard_manager_set_send_key_event_handler(
      manager,
      [](const FlutterKeyEvent* event, FlutterKeyEventCallback callback,
         void* callback_user_data,
         gpointer user_data) { callback(false, callback_user_data); },
      nullptr);
  g_autoptr(FlKeyEvent) event = fl_key_event_new(
      0, TRUE, kKeyCodeKeyA, GDK_KEY_a, static_cast<GdkModifierType>(0), 0);
  EXPECT_TRUE(fl_keyboard_manager_handle_event(manager, event));
  flush_channel_messages();
  // The event was not redispatched because handler handles it.
  EXPECT_FALSE(redispatched);
}

TEST(FlKeyboardManagerTest, CorrectLogicalKeyForLayouts) {
  ::testing::NiceMock<flutter::testing::MockKeymap> mock_keymap;

  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);
  g_autoptr(FlMockViewDelegate) view = fl_mock_view_delegate_new();
  g_autoptr(FlKeyboardManager) manager =
      fl_keyboard_manager_new(engine, FL_KEYBOARD_VIEW_DELEGATE(view));

  std::vector<CallRecord> call_records;
  fl_keyboard_manager_set_send_key_event_handler(
      manager,
      [](const FlutterKeyEvent* event, FlutterKeyEventCallback callback,
         void* callback_user_data, gpointer user_data) {
        std::vector<CallRecord>* call_records =
            static_cast<std::vector<CallRecord>*>(user_data);
        auto new_event = std::make_unique<FlutterKeyEvent>(*event);
        char* new_event_character = cloneString(event->character);
        new_event->character = new_event_character;
        call_records->push_back(CallRecord{
            .type = CallRecord::KEY_CALL_EMBEDDER,
            .callback = callback,
            .callback_user_data = callback_user_data,
            .event = std::move(new_event),
            .event_character = std::unique_ptr<char[]>(new_event_character),
        });
      },
      &call_records);
  fl_keyboard_manager_set_lookup_key_handler(
      manager,
      [](const GdkKeymapKey* key, gpointer user_data) {
        MockLayoutData* layout_data = static_cast<MockLayoutData*>(user_data);
        guint8 group = static_cast<guint8>(key->group);
        EXPECT_LT(group, layout_data->size());
        const MockGroupLayoutData* group_layout = (*layout_data)[group];
        EXPECT_NE(group_layout, nullptr);
        EXPECT_TRUE(key->level == 0 || key->level == 1);
        bool shift = key->level == 1;
        return (*group_layout)[key->keycode * 2 + shift];
      },
      (gpointer)(&kLayoutUs));

  auto sendTap = [&](guint8 keycode, guint keyval, guint8 group) {
    g_autoptr(FlKeyEvent) event1 = fl_key_event_new(
        0, TRUE, keycode, keyval, static_cast<GdkModifierType>(0), group);
    fl_keyboard_manager_handle_event(manager, event1);
    g_autoptr(FlKeyEvent) event2 = fl_key_event_new(
        0, FALSE, keycode, keyval, static_cast<GdkModifierType>(0), group);
    fl_keyboard_manager_handle_event(manager, event2);
  };

  /* US keyboard layout */

  sendTap(kKeyCodeKeyA, GDK_KEY_a, 0);  // KeyA
  VERIFY_DOWN(kLogicalKeyA, "a");

  sendTap(kKeyCodeKeyA, GDK_KEY_A, 0);  // Shift-KeyA
  VERIFY_DOWN(kLogicalKeyA, "A");

  sendTap(kKeyCodeDigit1, GDK_KEY_1, 0);  // Digit1
  VERIFY_DOWN(kLogicalDigit1, "1");

  sendTap(kKeyCodeDigit1, GDK_KEY_exclam, 0);  // Shift-Digit1
  VERIFY_DOWN(kLogicalDigit1, "!");

  sendTap(kKeyCodeMinus, GDK_KEY_minus, 0);  // Minus
  VERIFY_DOWN(kLogicalMinus, "-");

  sendTap(kKeyCodeMinus, GDK_KEY_underscore, 0);  // Shift-Minus
  VERIFY_DOWN(kLogicalUnderscore, "_");

  /* French keyboard layout, group 3, which is when the input method is showing
   * "Fr" */

  fl_keyboard_manager_set_lookup_key_handler(
      manager,
      [](const GdkKeymapKey* key, gpointer user_data) {
        MockLayoutData* layout_data = static_cast<MockLayoutData*>(user_data);
        guint8 group = static_cast<guint8>(key->group);
        EXPECT_LT(group, layout_data->size());
        const MockGroupLayoutData* group_layout = (*layout_data)[group];
        EXPECT_NE(group_layout, nullptr);
        EXPECT_TRUE(key->level == 0 || key->level == 1);
        bool shift = key->level == 1;
        return (*group_layout)[key->keycode * 2 + shift];
      },
      (gpointer)(&kLayoutFrench));

  sendTap(kKeyCodeKeyA, GDK_KEY_q, 3);  // KeyA
  VERIFY_DOWN(kLogicalKeyQ, "q");

  sendTap(kKeyCodeKeyA, GDK_KEY_Q, 3);  // Shift-KeyA
  VERIFY_DOWN(kLogicalKeyQ, "Q");

  sendTap(kKeyCodeSemicolon, GDK_KEY_m, 3);  // ; but prints M
  VERIFY_DOWN(kLogicalKeyM, "m");

  sendTap(kKeyCodeKeyM, GDK_KEY_comma, 3);  // M but prints ,
  VERIFY_DOWN(kLogicalComma, ",");

  sendTap(kKeyCodeDigit1, GDK_KEY_ampersand, 3);  // Digit1
  VERIFY_DOWN(kLogicalDigit1, "&");

  sendTap(kKeyCodeDigit1, GDK_KEY_1, 3);  // Shift-Digit1
  VERIFY_DOWN(kLogicalDigit1, "1");

  sendTap(kKeyCodeMinus, GDK_KEY_parenright, 3);  // Minus
  VERIFY_DOWN(kLogicalParenthesisRight, ")");

  sendTap(kKeyCodeMinus, GDK_KEY_degree, 3);  // Shift-Minus
  VERIFY_DOWN(static_cast<uint32_t>(L'°'), "°");

  /* French keyboard layout, group 0, which is pressing the "extra key for
   * triggering input method" key once after switching to French IME. */

  sendTap(kKeyCodeKeyA, GDK_KEY_a, 0);  // KeyA
  VERIFY_DOWN(kLogicalKeyA, "a");

  sendTap(kKeyCodeDigit1, GDK_KEY_1, 0);  // Digit1
  VERIFY_DOWN(kLogicalDigit1, "1");

  /* Russian keyboard layout, group 2 */
  fl_keyboard_manager_set_lookup_key_handler(
      manager,
      [](const GdkKeymapKey* key, gpointer user_data) {
        MockLayoutData* layout_data = static_cast<MockLayoutData*>(user_data);
        guint8 group = static_cast<guint8>(key->group);
        EXPECT_LT(group, layout_data->size());
        const MockGroupLayoutData* group_layout = (*layout_data)[group];
        EXPECT_NE(group_layout, nullptr);
        EXPECT_TRUE(key->level == 0 || key->level == 1);
        bool shift = key->level == 1;
        return (*group_layout)[key->keycode * 2 + shift];
      },
      (gpointer)(&kLayoutRussian));

  sendTap(kKeyCodeKeyA, GDK_KEY_Cyrillic_ef, 2);  // KeyA
  VERIFY_DOWN(kLogicalKeyA, "ф");

  sendTap(kKeyCodeDigit1, GDK_KEY_1, 2);  // Shift-Digit1
  VERIFY_DOWN(kLogicalDigit1, "1");

  sendTap(kKeyCodeKeyLeftBracket, GDK_KEY_Cyrillic_ha, 2);
  VERIFY_DOWN(kLogicalBracketLeft, "х");

  /* Russian keyboard layout, group 0 */
  sendTap(kKeyCodeKeyA, GDK_KEY_a, 0);  // KeyA
  VERIFY_DOWN(kLogicalKeyA, "a");

  sendTap(kKeyCodeKeyLeftBracket, GDK_KEY_bracketleft, 0);
  VERIFY_DOWN(kLogicalBracketLeft, "[");
}

TEST(FlKeyboardManagerTest, SynthesizeModifiersIfNeeded) {
  ::testing::NiceMock<flutter::testing::MockKeymap> mock_keymap;

  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);
  g_autoptr(FlMockViewDelegate) view = fl_mock_view_delegate_new();
  g_autoptr(FlKeyboardManager) manager =
      fl_keyboard_manager_new(engine, FL_KEYBOARD_VIEW_DELEGATE(view));

  std::vector<CallRecord> call_records;
  fl_keyboard_manager_set_send_key_event_handler(
      manager,
      [](const FlutterKeyEvent* event, FlutterKeyEventCallback callback,
         void* callback_user_data, gpointer user_data) {
        std::vector<CallRecord>* call_records =
            static_cast<std::vector<CallRecord>*>(user_data);
        auto new_event = std::make_unique<FlutterKeyEvent>(*event);
        char* new_event_character = cloneString(event->character);
        new_event->character = new_event_character;
        call_records->push_back(CallRecord{
            .type = CallRecord::KEY_CALL_EMBEDDER,
            .callback = callback,
            .callback_user_data = callback_user_data,
            .event = std::move(new_event),
            .event_character = std::unique_ptr<char[]>(new_event_character),
        });
      },
      &call_records);

  auto verifyModifierIsSynthesized = [&](GdkModifierType mask,
                                         uint64_t physical, uint64_t logical) {
    // Modifier is pressed.
    guint state = mask;
    fl_keyboard_manager_sync_modifier_if_needed(manager, state, 1000);
    EXPECT_EQ(call_records.size(), 1u);
    EXPECT_KEY_EVENT(call_records[0], kFlutterKeyEventTypeDown, physical,
                     logical, NULL, true);
    // Modifier is released.
    state = state ^ mask;
    fl_keyboard_manager_sync_modifier_if_needed(manager, state, 1001);
    EXPECT_EQ(call_records.size(), 2u);
    EXPECT_KEY_EVENT(call_records[1], kFlutterKeyEventTypeUp, physical, logical,
                     NULL, true);
    call_records.clear();
  };

  // No modifiers pressed.
  guint state = 0;
  fl_keyboard_manager_sync_modifier_if_needed(manager, state, 1000);
  EXPECT_EQ(call_records.size(), 0u);
  call_records.clear();

  // Press and release each modifier once.
  verifyModifierIsSynthesized(GDK_CONTROL_MASK, kPhysicalControlLeft,
                              kLogicalControlLeft);
  verifyModifierIsSynthesized(GDK_META_MASK, kPhysicalMetaLeft,
                              kLogicalMetaLeft);
  verifyModifierIsSynthesized(GDK_MOD1_MASK, kPhysicalAltLeft, kLogicalAltLeft);
  verifyModifierIsSynthesized(GDK_SHIFT_MASK, kPhysicalShiftLeft,
                              kLogicalShiftLeft);
}

TEST(FlKeyboardManagerTest, GetPressedState) {
  ::testing::NiceMock<flutter::testing::MockKeymap> mock_keymap;

  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);
  g_autoptr(FlMockViewDelegate) view = fl_mock_view_delegate_new();
  g_autoptr(FlKeyboardManager) manager =
      fl_keyboard_manager_new(engine, FL_KEYBOARD_VIEW_DELEGATE(view));

  // Dispatch a key event.
  g_autoptr(FlKeyEvent) event = fl_key_event_new(
      0, TRUE, kKeyCodeKeyA, GDK_KEY_a, static_cast<GdkModifierType>(0), 0);
  fl_keyboard_manager_handle_event(manager, event);

  GHashTable* pressedState = fl_keyboard_manager_get_pressed_state(manager);
  EXPECT_EQ(g_hash_table_size(pressedState), 1u);

  gpointer physical_key =
      g_hash_table_lookup(pressedState, uint64_to_gpointer(kPhysicalKeyA));
  EXPECT_EQ(gpointer_to_uint64(physical_key), kLogicalKeyA);
}

// The following layout data is generated using DEBUG_PRINT_LAYOUT.

const MockGroupLayoutData kLayoutUs0{{
    // +0x0  Shift   +0x1    Shift   +0x2    Shift   +0x3    Shift
    0x0000, 0xffff, 0x0000, 0xffff, 0x0000, 0xffff, 0x0000, 0xffff,  // 0x00
    0x0000, 0xffff, 0x0000, 0xffff, 0x0000, 0xffff, 0x0000, 0xffff,  // 0x04
    0xffff, 0x0031, 0xffff, 0x0031, 0x0031, 0x0021, 0x0032, 0x0040,  // 0x08
    0x0033, 0x0023, 0x0034, 0x0024, 0x0035, 0x0025, 0x0036, 0x005e,  // 0x0c
    0x0037, 0x0026, 0x0038, 0x002a, 0x0039, 0x0028, 0x0030, 0x0029,  // 0x10
    0x002d, 0x005f, 0x003d, 0x002b, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x14
    0x0071, 0x0051, 0x0077, 0x0057, 0x0065, 0x0045, 0x0072, 0x0052,  // 0x18
    0x0074, 0x0054, 0x0079, 0x0059, 0x0075, 0x0055, 0x0069, 0x0049,  // 0x1c
    0x006f, 0x004f, 0x0070, 0x0050, 0x005b, 0x007b, 0x005d, 0x007d,  // 0x20
    0xffff, 0xffff, 0xffff, 0x0061, 0x0061, 0x0041, 0x0073, 0x0053,  // 0x24
    0x0064, 0x0044, 0x0066, 0x0046, 0x0067, 0x0047, 0x0068, 0x0048,  // 0x28
    0x006a, 0x004a, 0x006b, 0x004b, 0x006c, 0x004c, 0x003b, 0x003a,  // 0x2c
    0x0027, 0x0022, 0x0060, 0x007e, 0xffff, 0x005c, 0x005c, 0x007c,  // 0x30
    0x007a, 0x005a, 0x0078, 0x0058, 0x0063, 0x0043, 0x0076, 0x0056,  // 0x34
    0x0062, 0x0042, 0x006e, 0x004e, 0x006d, 0x004d, 0x002c, 0x003c,  // 0x38
    0x002e, 0x003e, 0x002f, 0x003f, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x3c
    0xffff, 0xffff, 0x0020, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x40
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x44
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x48
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x4c
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x50
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x54
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x58
    0xffff, 0xffff, 0x003c, 0x003e, 0x003c, 0x003e, 0xffff, 0xffff,  // 0x5c
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x60
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x64
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x68
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x6c
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x70
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x74
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x78
    0xffff, 0xffff, 0xffff, 0x00b1, 0x00b1, 0xffff, 0xffff, 0xffff,  // 0x7c
}};

const MockGroupLayoutData kLayoutRussian0{
    // +0x0  Shift   +0x1    Shift   +0x2    Shift   +0x3    Shift
    0x0000, 0xffff, 0x0000, 0xffff, 0x0000, 0xffff, 0x0000, 0xffff,  // 0x00
    0x0000, 0xffff, 0x0000, 0xffff, 0x0000, 0xffff, 0x0000, 0xffff,  // 0x04
    0x0000, 0xffff, 0xffff, 0x0031, 0x0031, 0x0021, 0x0032, 0x0040,  // 0x08
    0x0033, 0x0023, 0x0034, 0x0024, 0x0035, 0x0025, 0x0036, 0x005e,  // 0x0c
    0x0037, 0x0026, 0x0038, 0x002a, 0x0039, 0x0028, 0x0030, 0x0029,  // 0x10
    0x002d, 0x005f, 0x003d, 0x002b, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x14
    0x0071, 0x0051, 0x0077, 0x0057, 0x0065, 0x0045, 0x0072, 0x0052,  // 0x18
    0x0074, 0x0054, 0x0079, 0x0059, 0x0075, 0x0055, 0x0069, 0x0049,  // 0x1c
    0x006f, 0x004f, 0x0070, 0x0050, 0x005b, 0x007b, 0x005d, 0x007d,  // 0x20
    0xffff, 0xffff, 0xffff, 0x0061, 0x0061, 0x0041, 0x0073, 0x0053,  // 0x24
    0x0064, 0x0044, 0x0066, 0x0046, 0x0067, 0x0047, 0x0068, 0x0048,  // 0x28
    0x006a, 0x004a, 0x006b, 0x004b, 0x006c, 0x004c, 0x003b, 0x003a,  // 0x2c
    0x0027, 0x0022, 0x0060, 0x007e, 0xffff, 0x005c, 0x005c, 0x007c,  // 0x30
    0x007a, 0x005a, 0x0078, 0x0058, 0x0063, 0x0043, 0x0076, 0x0056,  // 0x34
    0x0062, 0x0042, 0x006e, 0x004e, 0x006d, 0x004d, 0x002c, 0x003c,  // 0x38
    0x002e, 0x003e, 0x002f, 0x003f, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x3c
    0xffff, 0xffff, 0x0020, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x40
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x44
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x48
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x4c
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x50
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x54
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x58
    0xffff, 0xffff, 0x0000, 0xffff, 0x003c, 0x003e, 0xffff, 0xffff,  // 0x5c
    0xffff, 0xffff, 0x0000, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x60
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0x0000, 0xffff,  // 0x64
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x68
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x6c
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x70
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x74
    0x0000, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x78
    0xffff, 0xffff, 0xffff, 0x00b1, 0x00b1, 0xffff, 0xffff, 0xffff,  // 0x7c
};

const MockGroupLayoutData kLayoutRussian2{{
    // +0x0  Shift   +0x1    Shift   +0x2    Shift   +0x3    Shift
    0x0000, 0xffff, 0x0000, 0xffff, 0x0000, 0xffff, 0x0000, 0xffff,  // 0x00
    0x0000, 0xffff, 0x0000, 0xffff, 0x0000, 0xffff, 0x0000, 0xffff,  // 0x04
    0xffff, 0x0031, 0x0021, 0x0000, 0x0031, 0x0021, 0x0032, 0x0022,  // 0x08
    0x0033, 0x06b0, 0x0034, 0x003b, 0x0035, 0x0025, 0x0036, 0x003a,  // 0x0c
    0x0037, 0x003f, 0x0038, 0x002a, 0x0039, 0x0028, 0x0030, 0x0029,  // 0x10
    0x002d, 0x005f, 0x003d, 0x002b, 0x0071, 0x0051, 0x0000, 0x0000,  // 0x14
    0x06ca, 0x06ea, 0x06c3, 0x06e3, 0x06d5, 0x06f5, 0x06cb, 0x06eb,  // 0x18
    0x06c5, 0x06e5, 0x06ce, 0x06ee, 0x06c7, 0x06e7, 0x06db, 0x06fb,  // 0x1c
    0x06dd, 0x06fd, 0x06da, 0x06fa, 0x06c8, 0x06e8, 0x06df, 0x06ff,  // 0x20
    0x0061, 0x0041, 0x0041, 0x0000, 0x06c6, 0x06e6, 0x06d9, 0x06f9,  // 0x24
    0x06d7, 0x06f7, 0x06c1, 0x06e1, 0x06d0, 0x06f0, 0x06d2, 0x06f2,  // 0x28
    0x06cf, 0x06ef, 0x06cc, 0x06ec, 0x06c4, 0x06e4, 0x06d6, 0x06f6,  // 0x2c
    0x06dc, 0x06fc, 0x06a3, 0x06b3, 0x007c, 0x0000, 0x005c, 0x002f,  // 0x30
    0x06d1, 0x06f1, 0x06de, 0x06fe, 0x06d3, 0x06f3, 0x06cd, 0x06ed,  // 0x34
    0x06c9, 0x06e9, 0x06d4, 0x06f4, 0x06d8, 0x06f8, 0x06c2, 0x06e2,  // 0x38
    0x06c0, 0x06e0, 0x002e, 0x002c, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x3c
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x40
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x44
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x48
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x4c
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x50
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x54
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x58
    0xffff, 0xffff, 0x003c, 0x003e, 0x002f, 0x007c, 0xffff, 0xffff,  // 0x5c
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x60
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x64
    0xffff, 0xffff, 0xffff, 0xffff, 0x0000, 0xffff, 0xffff, 0x0000,  // 0x68
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x6c
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x70
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x74
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0x00b1,  // 0x78
    0x00b1, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x7c
}};

const MockGroupLayoutData kLayoutFrench0 = {
    // +0x0  Shift   +0x1    Shift   +0x2    Shift   +0x3    Shift
    0x0000, 0xffff, 0x0000, 0xffff, 0x0000, 0xffff, 0x0000, 0xffff,  // 0x00
    0x0000, 0xffff, 0x0000, 0xffff, 0x0000, 0xffff, 0x0000, 0xffff,  // 0x04
    0x0000, 0xffff, 0xffff, 0x0031, 0x0031, 0x0021, 0x0032, 0x0040,  // 0x08
    0x0033, 0x0023, 0x0034, 0x0024, 0x0035, 0x0025, 0x0036, 0x005e,  // 0x0c
    0x0037, 0x0026, 0x0038, 0x002a, 0x0039, 0x0028, 0x0030, 0x0029,  // 0x10
    0x002d, 0x005f, 0x003d, 0x002b, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x14
    0x0071, 0x0051, 0x0077, 0x0057, 0x0065, 0x0045, 0x0072, 0x0052,  // 0x18
    0x0074, 0x0054, 0x0079, 0x0059, 0x0075, 0x0055, 0x0069, 0x0049,  // 0x1c
    0x006f, 0x004f, 0x0070, 0x0050, 0x005b, 0x007b, 0x005d, 0x007d,  // 0x20
    0xffff, 0xffff, 0xffff, 0x0061, 0x0061, 0x0041, 0x0073, 0x0053,  // 0x24
    0x0064, 0x0044, 0x0066, 0x0046, 0x0067, 0x0047, 0x0068, 0x0048,  // 0x28
    0x006a, 0x004a, 0x006b, 0x004b, 0x006c, 0x004c, 0x003b, 0x003a,  // 0x2c
    0x0027, 0x0022, 0x0060, 0x007e, 0xffff, 0x005c, 0x005c, 0x007c,  // 0x30
    0x007a, 0x005a, 0x0078, 0x0058, 0x0063, 0x0043, 0x0076, 0x0056,  // 0x34
    0x0062, 0x0042, 0x006e, 0x004e, 0x006d, 0x004d, 0x002c, 0x003c,  // 0x38
    0x002e, 0x003e, 0x002f, 0x003f, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x3c
    0xffff, 0xffff, 0x0020, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x40
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x44
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x48
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x4c
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x50
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x54
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x58
    0xffff, 0xffff, 0x0000, 0xffff, 0x003c, 0x003e, 0xffff, 0xffff,  // 0x5c
    0xffff, 0xffff, 0x0000, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x60
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0x0000, 0xffff,  // 0x64
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x68
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x6c
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x70
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x74
    0x0000, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x78
    0xffff, 0xffff, 0xffff, 0x00b1, 0x00b1, 0xffff, 0xffff, 0xffff,  // 0x7c
};

const MockGroupLayoutData kLayoutFrench3 = {
    // +0x0  Shift   +0x1    Shift   +0x2    Shift   +0x3    Shift
    0x0000, 0xffff, 0x0000, 0xffff, 0x0000, 0xffff, 0x0000, 0xffff,  // 0x00
    0x0000, 0xffff, 0x0000, 0xffff, 0x0000, 0xffff, 0x0000, 0xffff,  // 0x04
    0x0000, 0xffff, 0x0000, 0x0000, 0x0026, 0x0031, 0x00e9, 0x0032,  // 0x08
    0x0022, 0x0033, 0x0027, 0x0034, 0x0028, 0x0035, 0x002d, 0x0036,  // 0x0c
    0x00e8, 0x0037, 0x005f, 0x0038, 0x00e7, 0x0039, 0x00e0, 0x0030,  // 0x10
    0x0029, 0x00b0, 0x003d, 0x002b, 0x0000, 0x0000, 0x0061, 0x0041,  // 0x14
    0x0061, 0x0041, 0x007a, 0x005a, 0x0065, 0x0045, 0x0072, 0x0052,  // 0x18
    0x0074, 0x0054, 0x0079, 0x0059, 0x0075, 0x0055, 0x0069, 0x0049,  // 0x1c
    0x006f, 0x004f, 0x0070, 0x0050, 0xffff, 0xffff, 0x0024, 0x00a3,  // 0x20
    0x0041, 0x0000, 0x0000, 0x0000, 0x0071, 0x0051, 0x0073, 0x0053,  // 0x24
    0x0064, 0x0044, 0x0066, 0x0046, 0x0067, 0x0047, 0x0068, 0x0048,  // 0x28
    0x006a, 0x004a, 0x006b, 0x004b, 0x006c, 0x004c, 0x006d, 0x004d,  // 0x2c
    0x00f9, 0x0025, 0x00b2, 0x007e, 0x0000, 0x0000, 0x002a, 0x00b5,  // 0x30
    0x0077, 0x0057, 0x0078, 0x0058, 0x0063, 0x0043, 0x0076, 0x0056,  // 0x34
    0x0062, 0x0042, 0x006e, 0x004e, 0x002c, 0x003f, 0x003b, 0x002e,  // 0x38
    0x003a, 0x002f, 0x0021, 0x00a7, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x3c
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x40
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x44
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x48
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x4c
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x50
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x54
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x58
    0xffff, 0x003c, 0x0000, 0xffff, 0x003c, 0x003e, 0xffff, 0xffff,  // 0x5c
    0xffff, 0xffff, 0x0000, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x60
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0x0000, 0xffff,  // 0x64
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x68
    0xffff, 0x0000, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x6c
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x70
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x74
    0x0000, 0xffff, 0xffff, 0xffff, 0xffff, 0x00b1, 0x00b1, 0xffff,  // 0x78
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,  // 0x7c
};

const MockLayoutData kLayoutUs{&kLayoutUs0};
const MockLayoutData kLayoutRussian{&kLayoutRussian0, nullptr,
                                    &kLayoutRussian2};
const MockLayoutData kLayoutFrench{&kLayoutFrench0, nullptr, nullptr,
                                   &kLayoutFrench3};

}  // namespace
