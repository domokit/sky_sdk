// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// Included first as it collides with the X11 headers.
#include "gtest/gtest.h"

#include "flutter/shell/platform/embedder/test_utils/proc_table_replacement.h"
#include "flutter/shell/platform/linux/fl_accessible_text_field.h"
#include "flutter/shell/platform/linux/fl_engine_private.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_standard_message_codec.h"
#include "flutter/shell/platform/linux/testing/mock_signal_handler.h"

// MOCK_ENGINE_PROC is leaky by design
// NOLINTBEGIN(clang-analyzer-core.StackAddressEscape)

static FlValue* decode_semantic_data(const uint8_t* data, size_t data_length) {
  g_autoptr(GBytes) bytes = g_bytes_new(data, data_length);
  g_autoptr(FlStandardMessageCodec) codec = fl_standard_message_codec_new();
  return fl_message_codec_decode_message(FL_MESSAGE_CODEC(codec), bytes,
                                         nullptr);
}

// Tests that semantic node value updates from Flutter emit AtkText::text-insert
// and AtkText::text-remove signals as expected.
TEST(FlAccessibleTextFieldTest, SetValue) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);
  g_autoptr(FlAccessibleNode) node = fl_accessible_text_field_new(engine, 1);

  // "" -> "Flutter"
  {
    flutter::testing::MockSignalHandler2<int, int> text_inserted(node,
                                                                 "text-insert");
    flutter::testing::MockSignalHandler text_removed(node, "text-remove");

    EXPECT_SIGNAL2(text_inserted, ::testing::Eq(0), ::testing::Eq(7));
    EXPECT_SIGNAL(text_removed).Times(0);

    fl_accessible_node_set_value(node, "Flutter");
  }

  // "Flutter" -> "Flutter"
  {
    flutter::testing::MockSignalHandler text_inserted(node, "text-insert");
    flutter::testing::MockSignalHandler text_removed(node, "text-remove");

    EXPECT_SIGNAL(text_inserted).Times(0);
    EXPECT_SIGNAL(text_removed).Times(0);

    fl_accessible_node_set_value(node, "Flutter");
  }

  // "Flutter" -> "engine"
  {
    flutter::testing::MockSignalHandler2<int, int> text_inserted(node,
                                                                 "text-insert");
    flutter::testing::MockSignalHandler2<int, int> text_removed(node,
                                                                "text-remove");

    EXPECT_SIGNAL2(text_inserted, ::testing::Eq(0), ::testing::Eq(6));
    EXPECT_SIGNAL2(text_removed, ::testing::Eq(0), ::testing::Eq(7));

    fl_accessible_node_set_value(node, "engine");
  }

  // "engine" -> ""
  {
    flutter::testing::MockSignalHandler text_inserted(node, "text-insert");
    flutter::testing::MockSignalHandler2<int, int> text_removed(node,
                                                                "text-remove");

    EXPECT_SIGNAL(text_inserted).Times(0);
    EXPECT_SIGNAL2(text_removed, ::testing::Eq(0), ::testing::Eq(6));

    fl_accessible_node_set_value(node, "");
  }
}

// Tests that semantic node selection updates from Flutter emit
// AtkText::text-selection-changed and AtkText::text-caret-moved signals as
// expected.
TEST(FlAccessibleTextFieldTest, SetTextSelection) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);
  g_autoptr(FlAccessibleNode) node = fl_accessible_text_field_new(engine, 1);

  // [-1,-1] -> [2,3]
  {
    flutter::testing::MockSignalHandler text_selection_changed(
        node, "text-selection-changed");
    flutter::testing::MockSignalHandler1<int> text_caret_moved(
        node, "text-caret-moved");

    EXPECT_SIGNAL(text_selection_changed);
    EXPECT_SIGNAL1(text_caret_moved, ::testing::Eq(3));

    fl_accessible_node_set_text_selection(node, 2, 3);
  }

  // [2,3] -> [3,3]
  {
    flutter::testing::MockSignalHandler text_selection_changed(
        node, "text-selection-changed");
    flutter::testing::MockSignalHandler text_caret_moved(node,
                                                         "text-caret-moved");

    EXPECT_SIGNAL(text_selection_changed);
    EXPECT_SIGNAL(text_caret_moved).Times(0);

    fl_accessible_node_set_text_selection(node, 3, 3);
  }

  // [3,3] -> [3,3]
  {
    flutter::testing::MockSignalHandler text_selection_changed(
        node, "text-selection-changed");
    flutter::testing::MockSignalHandler text_caret_moved(node,
                                                         "text-caret-moved");

    EXPECT_SIGNAL(text_selection_changed).Times(0);
    EXPECT_SIGNAL(text_caret_moved).Times(0);

    fl_accessible_node_set_text_selection(node, 3, 3);
  }

  // [3,3] -> [4,4]
  {
    flutter::testing::MockSignalHandler text_selection_changed(
        node, "text-selection-changed");
    flutter::testing::MockSignalHandler1<int> text_caret_moved(
        node, "text-caret-moved");

    EXPECT_SIGNAL(text_selection_changed).Times(0);
    EXPECT_SIGNAL1(text_caret_moved, ::testing::Eq(4));

    fl_accessible_node_set_text_selection(node, 4, 4);
  }
}

// Tests that fl_accessible_text_field_perform_action() passes the required
// "expandSelection" argument for semantic cursor move actions.
TEST(FlAccessibleTextFieldTest, PerformAction) {
  g_autoptr(GPtrArray) action_datas = g_ptr_array_new_with_free_func(
      reinterpret_cast<GDestroyNotify>(fl_value_unref));

  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);

  g_autoptr(GError) error = nullptr;
  EXPECT_TRUE(fl_engine_start(engine, &error));
  EXPECT_EQ(error, nullptr);

  fl_engine_get_embedder_api(engine)->DispatchSemanticsAction =
      MOCK_ENGINE_PROC(
          DispatchSemanticsAction,
          ([&action_datas](auto engine, uint64_t id,
                           FlutterSemanticsAction action, const uint8_t* data,
                           size_t data_length) {
            g_ptr_array_add(action_datas,
                            decode_semantic_data(data, data_length));
            return kSuccess;
          }));

  g_autoptr(FlAccessibleNode) node = fl_accessible_text_field_new(engine, 1);
  fl_accessible_node_set_actions(
      node, static_cast<FlutterSemanticsAction>(
                kFlutterSemanticsActionMoveCursorForwardByCharacter |
                kFlutterSemanticsActionMoveCursorBackwardByCharacter |
                kFlutterSemanticsActionMoveCursorForwardByWord |
                kFlutterSemanticsActionMoveCursorBackwardByWord));

  g_autoptr(FlValue) expand_selection = fl_value_new_bool(false);

  for (int i = 0; i < 4; ++i) {
    atk_action_do_action(ATK_ACTION(node), i);

    FlValue* data = static_cast<FlValue*>(g_ptr_array_index(action_datas, i));
    EXPECT_NE(data, nullptr);
    EXPECT_TRUE(fl_value_equal(data, expand_selection));
  }
}

// Tests AtkText::get_character_count.
TEST(FlAccessibleTextFieldTest, GetCharacterCount) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);
  g_autoptr(FlAccessibleNode) node = fl_accessible_text_field_new(engine, 1);

  EXPECT_EQ(atk_text_get_character_count(ATK_TEXT(node)), 0);

  fl_accessible_node_set_value(node, "Flutter!");

  EXPECT_EQ(atk_text_get_character_count(ATK_TEXT(node)), 8);
}

// Tests AtkText::get_text.
TEST(FlAccessibleTextFieldTest, GetText) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);
  g_autoptr(FlAccessibleNode) node = fl_accessible_text_field_new(engine, 1);

  g_autofree gchar* empty = atk_text_get_text(ATK_TEXT(node), 0, -1);
  EXPECT_STREQ(empty, "");

  flutter::testing::MockSignalHandler text_inserted(node, "text-insert");
  EXPECT_SIGNAL(text_inserted).Times(1);

  fl_accessible_node_set_value(node, "Flutter!");

  g_autofree gchar* flutter = atk_text_get_text(ATK_TEXT(node), 0, -1);
  EXPECT_STREQ(flutter, "Flutter!");

  g_autofree gchar* tt = atk_text_get_text(ATK_TEXT(node), 3, 5);
  EXPECT_STREQ(tt, "tt");
}

// Tests AtkText::get_caret_offset.
TEST(FlAccessibleTextFieldTest, GetCaretOffset) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);
  g_autoptr(FlAccessibleNode) node = fl_accessible_text_field_new(engine, 1);

  EXPECT_EQ(atk_text_get_caret_offset(ATK_TEXT(node)), -1);

  fl_accessible_node_set_text_selection(node, 1, 2);

  EXPECT_EQ(atk_text_get_caret_offset(ATK_TEXT(node)), 2);
}

// Tests AtkText::set_caret_offset.
TEST(FlAccessibleTextFieldTest, SetCaretOffset) {
  int base = -1;
  int extent = -1;

  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);

  g_autoptr(GError) error = nullptr;
  EXPECT_TRUE(fl_engine_start(engine, &error));
  EXPECT_EQ(error, nullptr);

  fl_engine_get_embedder_api(engine)->DispatchSemanticsAction =
      MOCK_ENGINE_PROC(
          DispatchSemanticsAction,
          ([&base, &extent](auto engine, uint64_t id,
                            FlutterSemanticsAction action, const uint8_t* data,
                            size_t data_length) {
            EXPECT_EQ(action, kFlutterSemanticsActionSetSelection);
            g_autoptr(FlValue) value = decode_semantic_data(data, data_length);
            EXPECT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_MAP);
            base = fl_value_get_int(fl_value_lookup_string(value, "base"));
            extent = fl_value_get_int(fl_value_lookup_string(value, "extent"));
            return kSuccess;
          }));

  g_autoptr(FlAccessibleNode) node = fl_accessible_text_field_new(engine, 1);

  EXPECT_TRUE(atk_text_set_caret_offset(ATK_TEXT(node), 3));
  EXPECT_EQ(base, 3);
  EXPECT_EQ(extent, 3);
}

// Tests AtkText::get_n_selections.
TEST(FlAccessibleTextFieldTest, GetNSelections) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);
  g_autoptr(FlAccessibleNode) node = fl_accessible_text_field_new(engine, 1);

  EXPECT_EQ(atk_text_get_n_selections(ATK_TEXT(node)), 0);

  fl_accessible_node_set_text_selection(node, 1, 2);

  EXPECT_EQ(atk_text_get_n_selections(ATK_TEXT(node)), 1);
}

// Tests AtkText::get_selection.
TEST(FlAccessibleTextFieldTest, GetSelection) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);
  g_autoptr(FlAccessibleNode) node = fl_accessible_text_field_new(engine, 1);

  EXPECT_EQ(atk_text_get_selection(ATK_TEXT(node), 0, nullptr, nullptr),
            nullptr);

  fl_accessible_node_set_value(node, "Flutter");
  fl_accessible_node_set_text_selection(node, 2, 5);

  gint start, end;
  g_autofree gchar* selection =
      atk_text_get_selection(ATK_TEXT(node), 0, &start, &end);
  EXPECT_STREQ(selection, "utt");
  EXPECT_EQ(start, 2);
  EXPECT_EQ(end, 5);

  // reverse
  fl_accessible_node_set_text_selection(node, 5, 2);
  g_autofree gchar* reverse =
      atk_text_get_selection(ATK_TEXT(node), 0, &start, &end);
  EXPECT_STREQ(reverse, "utt");
  EXPECT_EQ(start, 2);
  EXPECT_EQ(end, 5);

  // empty
  fl_accessible_node_set_text_selection(node, 5, 5);
  EXPECT_EQ(atk_text_get_selection(ATK_TEXT(node), 0, &start, &end), nullptr);

  // selection num != 0
  EXPECT_EQ(atk_text_get_selection(ATK_TEXT(node), 1, &start, &end), nullptr);
}

// Tests AtkText::add_selection.
TEST(FlAccessibleTextFieldTest, AddSelection) {
  int base = -1;
  int extent = -1;

  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);

  g_autoptr(GError) error = nullptr;
  EXPECT_TRUE(fl_engine_start(engine, &error));
  EXPECT_EQ(error, nullptr);

  fl_engine_get_embedder_api(engine)->DispatchSemanticsAction =
      MOCK_ENGINE_PROC(
          DispatchSemanticsAction,
          ([&base, &extent](auto engine, uint64_t id,
                            FlutterSemanticsAction action, const uint8_t* data,
                            size_t data_length) {
            EXPECT_EQ(action, kFlutterSemanticsActionSetSelection);
            g_autoptr(FlValue) value = decode_semantic_data(data, data_length);
            EXPECT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_MAP);
            base = fl_value_get_int(fl_value_lookup_string(value, "base"));
            extent = fl_value_get_int(fl_value_lookup_string(value, "extent"));
            return kSuccess;
          }));

  g_autoptr(FlAccessibleNode) node = fl_accessible_text_field_new(engine, 1);

  EXPECT_TRUE(atk_text_add_selection(ATK_TEXT(node), 2, 4));
  EXPECT_EQ(base, 2);
  EXPECT_EQ(extent, 4);

  fl_accessible_node_set_text_selection(node, 2, 4);

  // already has selection
  EXPECT_FALSE(atk_text_add_selection(ATK_TEXT(node), 6, 7));
  EXPECT_EQ(base, 2);
  EXPECT_EQ(extent, 4);
}

// Tests AtkText::remove_selection.
TEST(FlAccessibleTextFieldTest, RemoveSelection) {
  int base = -1;
  int extent = -1;

  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);

  g_autoptr(GError) error = nullptr;
  EXPECT_TRUE(fl_engine_start(engine, &error));
  EXPECT_EQ(error, nullptr);

  fl_engine_get_embedder_api(engine)->DispatchSemanticsAction =
      MOCK_ENGINE_PROC(
          DispatchSemanticsAction,
          ([&base, &extent](auto engine, uint64_t id,
                            FlutterSemanticsAction action, const uint8_t* data,
                            size_t data_length) {
            EXPECT_EQ(action, kFlutterSemanticsActionSetSelection);
            g_autoptr(FlValue) value = decode_semantic_data(data, data_length);
            EXPECT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_MAP);
            base = fl_value_get_int(fl_value_lookup_string(value, "base"));
            extent = fl_value_get_int(fl_value_lookup_string(value, "extent"));
            return kSuccess;
          }));

  g_autoptr(FlAccessibleNode) node = fl_accessible_text_field_new(engine, 1);

  // no selection
  EXPECT_FALSE(atk_text_remove_selection(ATK_TEXT(node), 0));
  EXPECT_EQ(base, -1);
  EXPECT_EQ(extent, -1);

  fl_accessible_node_set_text_selection(node, 2, 4);

  // selection num != 0
  EXPECT_FALSE(atk_text_remove_selection(ATK_TEXT(node), 1));
  EXPECT_EQ(base, -1);
  EXPECT_EQ(extent, -1);

  // ok, collapses selection
  EXPECT_TRUE(atk_text_remove_selection(ATK_TEXT(node), 0));
  EXPECT_EQ(base, 4);
  EXPECT_EQ(extent, 4);
}

// Tests AtkText::set_selection.
TEST(FlAccessibleTextFieldTest, SetSelection) {
  int base = -1;
  int extent = -1;

  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);

  g_autoptr(GError) error = nullptr;
  EXPECT_TRUE(fl_engine_start(engine, &error));
  EXPECT_EQ(error, nullptr);

  fl_engine_get_embedder_api(engine)->DispatchSemanticsAction =
      MOCK_ENGINE_PROC(
          DispatchSemanticsAction,
          ([&base, &extent](auto engine, uint64_t id,
                            FlutterSemanticsAction action, const uint8_t* data,
                            size_t data_length) {
            EXPECT_EQ(action, kFlutterSemanticsActionSetSelection);
            g_autoptr(FlValue) value = decode_semantic_data(data, data_length);
            EXPECT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_MAP);
            base = fl_value_get_int(fl_value_lookup_string(value, "base"));
            extent = fl_value_get_int(fl_value_lookup_string(value, "extent"));
            return kSuccess;
          }));

  g_autoptr(FlAccessibleNode) node = fl_accessible_text_field_new(engine, 1);

  // selection num != 0
  EXPECT_FALSE(atk_text_set_selection(ATK_TEXT(node), 1, 2, 4));
  EXPECT_EQ(base, -1);
  EXPECT_EQ(extent, -1);

  EXPECT_TRUE(atk_text_set_selection(ATK_TEXT(node), 0, 2, 4));
  EXPECT_EQ(base, 2);
  EXPECT_EQ(extent, 4);

  EXPECT_TRUE(atk_text_set_selection(ATK_TEXT(node), 0, 5, 1));
  EXPECT_EQ(base, 5);
  EXPECT_EQ(extent, 1);
}

// Tests AtkEditableText::set_text_contents.
TEST(FlAccessibleTextFieldTest, SetTextContents) {
  g_autofree gchar* text = nullptr;

  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);

  g_autoptr(GError) error = nullptr;
  EXPECT_TRUE(fl_engine_start(engine, &error));
  EXPECT_EQ(error, nullptr);

  fl_engine_get_embedder_api(engine)->DispatchSemanticsAction =
      MOCK_ENGINE_PROC(
          DispatchSemanticsAction,
          ([&text](auto engine, uint64_t id, FlutterSemanticsAction action,
                   const uint8_t* data, size_t data_length) {
            EXPECT_EQ(action, kFlutterSemanticsActionSetText);
            g_autoptr(FlValue) value = decode_semantic_data(data, data_length);
            EXPECT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_STRING);
            text = g_strdup(fl_value_get_string(value));
            return kSuccess;
          }));

  g_autoptr(FlAccessibleNode) node = fl_accessible_text_field_new(engine, 1);

  atk_editable_text_set_text_contents(ATK_EDITABLE_TEXT(node), "Flutter");
  EXPECT_STREQ(text, "Flutter");
}

// Tests AtkEditableText::insert/delete_text.
TEST(FlAccessibleTextFieldTest, InsertDeleteText) {
  g_autofree gchar* text = nullptr;
  int base = -1;
  int extent = -1;

  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);

  g_autoptr(GError) error = nullptr;
  EXPECT_TRUE(fl_engine_start(engine, &error));
  EXPECT_EQ(error, nullptr);

  fl_engine_get_embedder_api(engine)->DispatchSemanticsAction =
      MOCK_ENGINE_PROC(
          DispatchSemanticsAction,
          ([&text, &base, &extent](auto engine, uint64_t id,
                                   FlutterSemanticsAction action,
                                   const uint8_t* data, size_t data_length) {
            EXPECT_THAT(action,
                        ::testing::AnyOf(kFlutterSemanticsActionSetText,
                                         kFlutterSemanticsActionSetSelection));
            if (action == kFlutterSemanticsActionSetText) {
              g_autoptr(FlValue) value =
                  decode_semantic_data(data, data_length);
              EXPECT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_STRING);
              g_free(text);
              text = g_strdup(fl_value_get_string(value));
            } else {
              g_autoptr(FlValue) value =
                  decode_semantic_data(data, data_length);
              EXPECT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_MAP);
              base = fl_value_get_int(fl_value_lookup_string(value, "base"));
              extent =
                  fl_value_get_int(fl_value_lookup_string(value, "extent"));
            }
            return kSuccess;
          }));

  g_autoptr(FlAccessibleNode) node = fl_accessible_text_field_new(engine, 1);
  fl_accessible_node_set_value(node, "Fler");

  gint pos = 2;
  atk_editable_text_insert_text(ATK_EDITABLE_TEXT(node), "utt", 3, &pos);
  EXPECT_EQ(pos, 5);
  EXPECT_STREQ(text, "Flutter");
  EXPECT_EQ(base, pos);
  EXPECT_EQ(extent, pos);

  atk_editable_text_delete_text(ATK_EDITABLE_TEXT(node), 2, 5);
  EXPECT_STREQ(text, "Fler");
  EXPECT_EQ(base, 2);
  EXPECT_EQ(extent, 2);
}

// Tests AtkEditableText::copy/cut/paste_text.
TEST(FlAccessibleTextFieldTest, CopyCutPasteText) {
  int base = -1;
  int extent = -1;
  FlutterSemanticsAction act = kFlutterSemanticsActionCustomAction;

  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);

  g_autoptr(GError) error = nullptr;
  EXPECT_TRUE(fl_engine_start(engine, &error));
  EXPECT_EQ(error, nullptr);

  fl_engine_get_embedder_api(engine)->DispatchSemanticsAction =
      MOCK_ENGINE_PROC(
          DispatchSemanticsAction,
          ([&act, &base, &extent](auto engine, uint64_t id,
                                  FlutterSemanticsAction action,
                                  const uint8_t* data, size_t data_length) {
            EXPECT_THAT(action,
                        ::testing::AnyOf(kFlutterSemanticsActionCut,
                                         kFlutterSemanticsActionCopy,
                                         kFlutterSemanticsActionPaste,
                                         kFlutterSemanticsActionSetSelection));
            act = action;
            if (action == kFlutterSemanticsActionSetSelection) {
              g_autoptr(FlValue) value =
                  decode_semantic_data(data, data_length);
              EXPECT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_MAP);
              base = fl_value_get_int(fl_value_lookup_string(value, "base"));
              extent =
                  fl_value_get_int(fl_value_lookup_string(value, "extent"));
            }
            return kSuccess;
          }));

  g_autoptr(FlAccessibleNode) node = fl_accessible_text_field_new(engine, 1);

  atk_editable_text_copy_text(ATK_EDITABLE_TEXT(node), 2, 5);
  EXPECT_EQ(base, 2);
  EXPECT_EQ(extent, 5);
  EXPECT_EQ(act, kFlutterSemanticsActionCopy);

  atk_editable_text_cut_text(ATK_EDITABLE_TEXT(node), 1, 4);
  EXPECT_EQ(base, 1);
  EXPECT_EQ(extent, 4);
  EXPECT_EQ(act, kFlutterSemanticsActionCut);

  atk_editable_text_paste_text(ATK_EDITABLE_TEXT(node), 3);
  EXPECT_EQ(base, 3);
  EXPECT_EQ(extent, 3);
  EXPECT_EQ(act, kFlutterSemanticsActionPaste);
}

TEST(FlAccessibleTextFieldTest, TextBoundary) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);
  g_autoptr(FlAccessibleNode) node = fl_accessible_text_field_new(engine, 1);

  fl_accessible_node_set_value(node,
                               "Lorem ipsum.\nDolor sit amet. Praesent commodo?"
                               "\n\nPraesent et felis dui.");

  // |Lorem
  gint start_offset = -1, end_offset = -1;
  g_autofree gchar* lorem_char = atk_text_get_string_at_offset(
      ATK_TEXT(node), 0, ATK_TEXT_GRANULARITY_CHAR, &start_offset, &end_offset);
  EXPECT_STREQ(lorem_char, "L");
  EXPECT_EQ(start_offset, 0);
  EXPECT_EQ(end_offset, 1);

  g_autofree gchar* lorem_word = atk_text_get_string_at_offset(
      ATK_TEXT(node), 0, ATK_TEXT_GRANULARITY_WORD, &start_offset, &end_offset);
  EXPECT_STREQ(lorem_word, "Lorem");
  EXPECT_EQ(start_offset, 0);
  EXPECT_EQ(end_offset, 5);

  g_autofree gchar* lorem_sentence = atk_text_get_string_at_offset(
      ATK_TEXT(node), 0, ATK_TEXT_GRANULARITY_SENTENCE, &start_offset,
      &end_offset);
  EXPECT_STREQ(lorem_sentence, "Lorem ipsum.");
  EXPECT_EQ(start_offset, 0);
  EXPECT_EQ(end_offset, 12);

  g_autofree gchar* lorem_line = atk_text_get_string_at_offset(
      ATK_TEXT(node), 0, ATK_TEXT_GRANULARITY_LINE, &start_offset, &end_offset);
  EXPECT_STREQ(lorem_line, "Lorem ipsum.");
  EXPECT_EQ(start_offset, 0);
  EXPECT_EQ(end_offset, 12);

  g_autofree gchar* lorem_paragraph = atk_text_get_string_at_offset(
      ATK_TEXT(node), 0, ATK_TEXT_GRANULARITY_PARAGRAPH, &start_offset,
      &end_offset);
  EXPECT_STREQ(lorem_paragraph,
               "Lorem ipsum.\nDolor sit amet. Praesent commodo?");
  EXPECT_EQ(start_offset, 0);
  EXPECT_EQ(end_offset, 46);

  // Pra|esent
  g_autofree gchar* praesent_char = atk_text_get_string_at_offset(
      ATK_TEXT(node), 32, ATK_TEXT_GRANULARITY_CHAR, &start_offset,
      &end_offset);
  EXPECT_STREQ(praesent_char, "e");
  EXPECT_EQ(start_offset, 32);
  EXPECT_EQ(end_offset, 33);

  g_autofree gchar* praesent_word = atk_text_get_string_at_offset(
      ATK_TEXT(node), 32, ATK_TEXT_GRANULARITY_WORD, &start_offset,
      &end_offset);
  EXPECT_STREQ(praesent_word, "Praesent");
  EXPECT_EQ(start_offset, 29);
  EXPECT_EQ(end_offset, 37);

  g_autofree gchar* praesent_sentence = atk_text_get_string_at_offset(
      ATK_TEXT(node), 32, ATK_TEXT_GRANULARITY_SENTENCE, &start_offset,
      &end_offset);
  EXPECT_STREQ(praesent_sentence, "Praesent commodo?");
  EXPECT_EQ(start_offset, 29);
  EXPECT_EQ(end_offset, 46);

  g_autofree gchar* praesent_line = atk_text_get_string_at_offset(
      ATK_TEXT(node), 32, ATK_TEXT_GRANULARITY_LINE, &start_offset,
      &end_offset);
  EXPECT_STREQ(praesent_line, "Dolor sit amet. Praesent commodo?");
  EXPECT_EQ(start_offset, 13);
  EXPECT_EQ(end_offset, 46);

  g_autofree gchar* praesent_paragraph = atk_text_get_string_at_offset(
      ATK_TEXT(node), 32, ATK_TEXT_GRANULARITY_PARAGRAPH, &start_offset,
      &end_offset);
  EXPECT_STREQ(praesent_paragraph,
               "Lorem ipsum.\nDolor sit amet. Praesent commodo?");
  EXPECT_EQ(start_offset, 0);
  EXPECT_EQ(end_offset, 46);

  // feli|s
  g_autofree gchar* felis_char = atk_text_get_string_at_offset(
      ATK_TEXT(node), 64, ATK_TEXT_GRANULARITY_CHAR, &start_offset,
      &end_offset);
  EXPECT_STREQ(felis_char, "s");
  EXPECT_EQ(start_offset, 64);
  EXPECT_EQ(end_offset, 65);

  g_autofree gchar* felis_word = atk_text_get_string_at_offset(
      ATK_TEXT(node), 64, ATK_TEXT_GRANULARITY_WORD, &start_offset,
      &end_offset);
  EXPECT_STREQ(felis_word, "felis");
  EXPECT_EQ(start_offset, 60);
  EXPECT_EQ(end_offset, 65);

  g_autofree gchar* felis_sentence = atk_text_get_string_at_offset(
      ATK_TEXT(node), 64, ATK_TEXT_GRANULARITY_SENTENCE, &start_offset,
      &end_offset);
  EXPECT_STREQ(felis_sentence, "Praesent et felis dui.");
  EXPECT_EQ(start_offset, 48);
  EXPECT_EQ(end_offset, 70);

  g_autofree gchar* felis_line = atk_text_get_string_at_offset(
      ATK_TEXT(node), 64, ATK_TEXT_GRANULARITY_LINE, &start_offset,
      &end_offset);
  EXPECT_STREQ(felis_line, "Praesent et felis dui.");
  EXPECT_EQ(start_offset, 48);
  EXPECT_EQ(end_offset, 70);

  g_autofree gchar* felis_paragraph = atk_text_get_string_at_offset(
      ATK_TEXT(node), 64, ATK_TEXT_GRANULARITY_PARAGRAPH, &start_offset,
      &end_offset);
  EXPECT_STREQ(felis_paragraph, "\nPraesent et felis dui.");
  EXPECT_EQ(start_offset, 47);
  EXPECT_EQ(end_offset, 70);
}

// NOLINTEND(clang-analyzer-core.StackAddressEscape)
