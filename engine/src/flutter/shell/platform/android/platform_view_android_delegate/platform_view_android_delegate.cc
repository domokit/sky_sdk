// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/android/platform_view_android_delegate/platform_view_android_delegate.h"

#include <utility>

namespace flutter {

void putStringAttributesIntoBuffer(
    const StringAttributes& attributes,
    int32_t* buffer_int32,
    size_t& position,
    std::vector<std::vector<uint8_t>>& string_attribute_args) {
  if (attributes.empty()) {
    buffer_int32[position++] = -1;
    return;
  }
  buffer_int32[position++] = attributes.size();
  for (const auto& attribute : attributes) {
    buffer_int32[position++] = attribute->start;
    buffer_int32[position++] = attribute->end;
    buffer_int32[position++] = static_cast<int32_t>(attribute->type);
    switch (attribute->type) {
      case StringAttributeType::kSpellOut:
        buffer_int32[position++] = -1;
        break;
      case StringAttributeType::kLocale:
        buffer_int32[position++] = string_attribute_args.size();
        std::shared_ptr<LocaleStringAttribute> locale_attribute =
            std::static_pointer_cast<LocaleStringAttribute>(attribute);
        string_attribute_args.push_back(
            {locale_attribute->locale.begin(), locale_attribute->locale.end()});
        break;
    }
  }
}

PlatformViewAndroidDelegate::PlatformViewAndroidDelegate(
    std::shared_ptr<PlatformViewAndroidJNI> jni_facade)
    : jni_facade_(std::move(jni_facade)){};

void PlatformViewAndroidDelegate::UpdateSemantics(
    const flutter::SemanticsNodeUpdates& update,
    const flutter::CustomAccessibilityActionUpdates& actions) {
  constexpr size_t kBytesPerNode = 48 * sizeof(int32_t);
  constexpr size_t kBytesPerChild = sizeof(int32_t);
  constexpr size_t kBytesPerCustomAction = sizeof(int32_t);
  constexpr size_t kBytesPerAction = 4 * sizeof(int32_t);
  constexpr size_t kBytesPerStringAttribute = 4 * sizeof(int32_t);

  {
    size_t num_bytes = 0;
    for (const auto& value : update) {
      num_bytes += kBytesPerNode;
      num_bytes +=
          value.second.childrenInTraversalOrder.size() * kBytesPerChild;
      num_bytes += value.second.childrenInHitTestOrder.size() * kBytesPerChild;
      num_bytes += value.second.customAccessibilityActions.size() *
                   kBytesPerCustomAction;
      num_bytes +=
          value.second.labelAttributes.size() * kBytesPerStringAttribute;
      num_bytes +=
          value.second.valueAttributes.size() * kBytesPerStringAttribute;
      num_bytes += value.second.increasedValueAttributes.size() *
                   kBytesPerStringAttribute;
      num_bytes += value.second.decreasedValueAttributes.size() *
                   kBytesPerStringAttribute;
      num_bytes +=
          value.second.hintAttributes.size() * kBytesPerStringAttribute;
    }
    // The encoding defined here is used in:
    //
    //  * AccessibilityBridge.java
    //  * AccessibilityBridgeTest.java
    //  * accessibility_bridge.mm
    //
    // If any of the encoding structure or length is changed, those locations
    // must be updated (at a minimum).
    std::vector<uint8_t> buffer(num_bytes);
    int32_t* buffer_int32 = reinterpret_cast<int32_t*>(&buffer[0]);
    float* buffer_float32 = reinterpret_cast<float*>(&buffer[0]);

    std::vector<std::string> strings;
    std::vector<std::vector<uint8_t>> string_attribute_args;
    size_t position = 0;
    for (const auto& value : update) {
      // If you edit this code, make sure you update kBytesPerNode
      // and/or kBytesPerChild above to match the number of values you are
      // sending.
      const flutter::SemanticsNode& node = value.second;
      buffer_int32[position++] = node.id;
      buffer_int32[position++] = node.flags;
      buffer_int32[position++] = node.actions;
      buffer_int32[position++] = node.maxValueLength;
      buffer_int32[position++] = node.currentValueLength;
      buffer_int32[position++] = node.textSelectionBase;
      buffer_int32[position++] = node.textSelectionExtent;
      buffer_int32[position++] = node.platformViewId;
      buffer_int32[position++] = node.scrollChildren;
      buffer_int32[position++] = node.scrollIndex;
      buffer_float32[position++] = static_cast<float>(node.scrollPosition);
      buffer_float32[position++] = static_cast<float>(node.scrollExtentMax);
      buffer_float32[position++] = static_cast<float>(node.scrollExtentMin);

      if (node.identifier.empty()) {
        buffer_int32[position++] = -1;
      } else {
        buffer_int32[position++] = strings.size();
        strings.push_back(node.identifier);
      }

      if (node.label.empty()) {
        buffer_int32[position++] = -1;
      } else {
        buffer_int32[position++] = strings.size();
        strings.push_back(node.label);
      }

      putStringAttributesIntoBuffer(node.labelAttributes, buffer_int32,
                                    position, string_attribute_args);
      if (node.value.empty()) {
        buffer_int32[position++] = -1;
      } else {
        buffer_int32[position++] = strings.size();
        strings.push_back(node.value);
      }

      putStringAttributesIntoBuffer(node.valueAttributes, buffer_int32,
                                    position, string_attribute_args);
      if (node.increasedValue.empty()) {
        buffer_int32[position++] = -1;
      } else {
        buffer_int32[position++] = strings.size();
        strings.push_back(node.increasedValue);
      }

      putStringAttributesIntoBuffer(node.increasedValueAttributes, buffer_int32,
                                    position, string_attribute_args);
      if (node.decreasedValue.empty()) {
        buffer_int32[position++] = -1;
      } else {
        buffer_int32[position++] = strings.size();
        strings.push_back(node.decreasedValue);
      }

      putStringAttributesIntoBuffer(node.decreasedValueAttributes, buffer_int32,
                                    position, string_attribute_args);

      if (node.hint.empty()) {
        buffer_int32[position++] = -1;
      } else {
        buffer_int32[position++] = strings.size();
        strings.push_back(node.hint);
      }

      putStringAttributesIntoBuffer(node.hintAttributes, buffer_int32, position,
                                    string_attribute_args);

      if (node.tooltip.empty()) {
        buffer_int32[position++] = -1;
      } else {
        buffer_int32[position++] = strings.size();
        strings.push_back(node.tooltip);
      }

      buffer_int32[position++] = node.textDirection;
      buffer_float32[position++] = node.rect.left();
      buffer_float32[position++] = node.rect.top();
      buffer_float32[position++] = node.rect.right();
      buffer_float32[position++] = node.rect.bottom();
      node.transform.getColMajor(&buffer_float32[position]);
      position += 16;

      buffer_int32[position++] = node.childrenInTraversalOrder.size();
      for (int32_t child : node.childrenInTraversalOrder) {
        buffer_int32[position++] = child;
      }

      for (int32_t child : node.childrenInHitTestOrder) {
        buffer_int32[position++] = child;
      }

      buffer_int32[position++] = node.customAccessibilityActions.size();
      for (int32_t child : node.customAccessibilityActions) {
        buffer_int32[position++] = child;
      }
    }

    // custom accessibility actions.
    size_t num_action_bytes = actions.size() * kBytesPerAction;
    std::vector<uint8_t> actions_buffer(num_action_bytes);
    int32_t* actions_buffer_int32 =
        reinterpret_cast<int32_t*>(&actions_buffer[0]);

    std::vector<std::string> action_strings;
    size_t actions_position = 0;
    for (const auto& value : actions) {
      // If you edit this code, make sure you update kBytesPerAction
      // to match the number of values you are
      // sending.
      const flutter::CustomAccessibilityAction& action = value.second;
      actions_buffer_int32[actions_position++] = action.id;
      actions_buffer_int32[actions_position++] = action.overrideId;
      if (action.label.empty()) {
        actions_buffer_int32[actions_position++] = -1;
      } else {
        actions_buffer_int32[actions_position++] = action_strings.size();
        action_strings.push_back(action.label);
      }
      if (action.hint.empty()) {
        actions_buffer_int32[actions_position++] = -1;
      } else {
        actions_buffer_int32[actions_position++] = action_strings.size();
        action_strings.push_back(action.hint);
      }
    }

    // Calling NewDirectByteBuffer in API level 22 and below with a size of zero
    // will cause a JNI crash.
    if (!actions_buffer.empty()) {
      jni_facade_->FlutterViewUpdateCustomAccessibilityActions(actions_buffer,
                                                               action_strings);
    }

    if (!buffer.empty()) {
      jni_facade_->FlutterViewUpdateSemantics(buffer, strings,
                                              string_attribute_args);
    }
  }
}

}  // namespace flutter
