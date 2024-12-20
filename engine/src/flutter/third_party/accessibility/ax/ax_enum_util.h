// Copyright 2018 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef UI_ACCESSIBILITY_AX_ENUM_UTIL_H_
#define UI_ACCESSIBILITY_AX_ENUM_UTIL_H_

#include <string>

#include "ax_base_export.h"
#include "ax_enums.h"

namespace ui {

// ax::mojom::Event
AX_BASE_EXPORT const char* ToString(ax::mojom::Event event);
AX_BASE_EXPORT ax::mojom::Event ParseEvent(const char* event);

// ax::mojom::Role
AX_BASE_EXPORT const char* ToString(ax::mojom::Role role);
AX_BASE_EXPORT ax::mojom::Role ParseRole(const char* role);

// ax::mojom::State
AX_BASE_EXPORT const char* ToString(ax::mojom::State state);
AX_BASE_EXPORT ax::mojom::State ParseState(const char* state);

// ax::mojom::Action
AX_BASE_EXPORT const char* ToString(ax::mojom::Action action);
AX_BASE_EXPORT ax::mojom::Action ParseAction(const char* action);

// ax::mojom::ActionFlags
AX_BASE_EXPORT const char* ToString(ax::mojom::ActionFlags action_flags);
AX_BASE_EXPORT ax::mojom::ActionFlags ParseActionFlags(
    const char* action_flags);

// ax::mojom::DefaultActionVerb
AX_BASE_EXPORT const char* ToString(
    ax::mojom::DefaultActionVerb default_action_verb);

// Returns a localized string that corresponds to the name of the given action.
AX_BASE_EXPORT std::string ToLocalizedString(
    ax::mojom::DefaultActionVerb action_verb);

AX_BASE_EXPORT ax::mojom::DefaultActionVerb ParseDefaultActionVerb(
    const char* default_action_verb);

// ax::mojom::Mutation
AX_BASE_EXPORT const char* ToString(ax::mojom::Mutation mutation);
AX_BASE_EXPORT ax::mojom::Mutation ParseMutation(const char* mutation);

// ax::mojom::StringAttribute
AX_BASE_EXPORT const char* ToString(
    ax::mojom::StringAttribute string_attribute);
AX_BASE_EXPORT ax::mojom::StringAttribute ParseStringAttribute(
    const char* string_attribute);

// ax::mojom::IntAttribute
AX_BASE_EXPORT const char* ToString(ax::mojom::IntAttribute int_attribute);
AX_BASE_EXPORT ax::mojom::IntAttribute ParseIntAttribute(
    const char* int_attribute);

// ax::mojom::FloatAttribute
AX_BASE_EXPORT const char* ToString(ax::mojom::FloatAttribute float_attribute);
AX_BASE_EXPORT ax::mojom::FloatAttribute ParseFloatAttribute(
    const char* float_attribute);

// ax::mojom::BoolAttribute
AX_BASE_EXPORT const char* ToString(ax::mojom::BoolAttribute bool_attribute);
AX_BASE_EXPORT ax::mojom::BoolAttribute ParseBoolAttribute(
    const char* bool_attribute);

// ax::mojom::IntListAttribute
AX_BASE_EXPORT const char* ToString(
    ax::mojom::IntListAttribute int_list_attribute);
AX_BASE_EXPORT ax::mojom::IntListAttribute ParseIntListAttribute(
    const char* int_list_attribute);

// ax::mojom::StringListAttribute
AX_BASE_EXPORT const char* ToString(
    ax::mojom::StringListAttribute string_list_attribute);
AX_BASE_EXPORT ax::mojom::StringListAttribute ParseStringListAttribute(
    const char* string_list_attribute);

// ax::mojom::ListStyle
AX_BASE_EXPORT const char* ToString(ax::mojom::ListStyle list_style);
AX_BASE_EXPORT ax::mojom::ListStyle ParseListStyle(const char* list_style);

// ax::mojom::MarkerType
AX_BASE_EXPORT const char* ToString(ax::mojom::MarkerType marker_type);
AX_BASE_EXPORT ax::mojom::MarkerType ParseMarkerType(const char* marker_type);

// ax::mojom::MoveDirection
AX_BASE_EXPORT const char* ToString(ax::mojom::MoveDirection move_direction);
AX_BASE_EXPORT ax::mojom::MoveDirection ParseMoveDirection(
    const char* move_direction);

// ax::mojom::Command
AX_BASE_EXPORT const char* ToString(ax::mojom::Command command);
AX_BASE_EXPORT ax::mojom::Command ParseCommand(const char* command);

// ax::mojom::TextBoundary
AX_BASE_EXPORT const char* ToString(ax::mojom::TextBoundary text_boundary);
AX_BASE_EXPORT ax::mojom::TextBoundary ParseTextBoundary(
    const char* text_boundary);

// ax:mojom::TextDecorationStyle
AX_BASE_EXPORT const char* ToString(
    ax::mojom::TextDecorationStyle text_decoration_style);
AX_BASE_EXPORT ax::mojom::TextDecorationStyle ParseTextDecorationStyle(
    const char* text_decoration_style);

// ax::mojom::TextAlign
AX_BASE_EXPORT const char* ToString(ax::mojom::TextAlign text_align);
AX_BASE_EXPORT ax::mojom::TextAlign ParseTextAlign(const char* text_align);

// ax::mojom::WritingDirection
AX_BASE_EXPORT const char* ToString(ax::mojom::WritingDirection text_direction);
AX_BASE_EXPORT ax::mojom::WritingDirection ParseTextDirection(
    const char* text_direction);

// ax::mojom::TextPosition
AX_BASE_EXPORT const char* ToString(ax::mojom::TextPosition text_position);
AX_BASE_EXPORT ax::mojom::TextPosition ParseTextPosition(
    const char* text_position);

// ax::mojom::TextStyle
AX_BASE_EXPORT const char* ToString(ax::mojom::TextStyle text_style);
AX_BASE_EXPORT ax::mojom::TextStyle ParseTextStyle(const char* text_style);

// ax::mojom::AriaCurrentState
AX_BASE_EXPORT const char* ToString(
    ax::mojom::AriaCurrentState aria_current_state);
AX_BASE_EXPORT ax::mojom::AriaCurrentState ParseAriaCurrentState(
    const char* aria_current_state);

// ax::mojom::HasPopup
AX_BASE_EXPORT const char* ToString(ax::mojom::HasPopup has_popup);
AX_BASE_EXPORT ax::mojom::HasPopup ParseHasPopup(const char* has_popup);

// ax::mojom::InvalidState
AX_BASE_EXPORT const char* ToString(ax::mojom::InvalidState invalid_state);
AX_BASE_EXPORT ax::mojom::InvalidState ParseInvalidState(
    const char* invalid_state);

// ax::mojom::Restriction
AX_BASE_EXPORT const char* ToString(ax::mojom::Restriction restriction);
AX_BASE_EXPORT ax::mojom::Restriction ParseRestriction(const char* restriction);

// ax::mojom::CheckedState
AX_BASE_EXPORT const char* ToString(ax::mojom::CheckedState checked_state);
AX_BASE_EXPORT ax::mojom::CheckedState ParseCheckedState(
    const char* checked_state);

// ax::mojom::SortDirection
AX_BASE_EXPORT const char* ToString(ax::mojom::SortDirection sort_direction);
AX_BASE_EXPORT ax::mojom::SortDirection ParseSortDirection(
    const char* sort_direction);

// ax::mojom::NameFrom
AX_BASE_EXPORT const char* ToString(ax::mojom::NameFrom name_from);
AX_BASE_EXPORT ax::mojom::NameFrom ParseNameFrom(const char* name_from);

// ax::mojom::DescriptionFrom
AX_BASE_EXPORT const char* ToString(
    ax::mojom::DescriptionFrom description_from);
AX_BASE_EXPORT ax::mojom::DescriptionFrom ParseDescriptionFrom(
    const char* description_from);

// ax::mojom::EventFrom
AX_BASE_EXPORT const char* ToString(ax::mojom::EventFrom event_from);
AX_BASE_EXPORT ax::mojom::EventFrom ParseEventFrom(const char* event_from);

// ax::mojom::Gesture
AX_BASE_EXPORT const char* ToString(ax::mojom::Gesture gesture);
AX_BASE_EXPORT ax::mojom::Gesture ParseGesture(const char* gesture);

// ax::mojom::TextAffinity
AX_BASE_EXPORT const char* ToString(ax::mojom::TextAffinity text_affinity);
AX_BASE_EXPORT ax::mojom::TextAffinity ParseTextAffinity(
    const char* text_affinity);

// ax::mojom::TreeOrder
AX_BASE_EXPORT const char* ToString(ax::mojom::TreeOrder tree_order);
AX_BASE_EXPORT ax::mojom::TreeOrder ParseTreeOrder(const char* tree_order);

// ax::mojom::ImageAnnotationStatus
AX_BASE_EXPORT const char* ToString(ax::mojom::ImageAnnotationStatus status);
AX_BASE_EXPORT ax::mojom::ImageAnnotationStatus ParseImageAnnotationStatus(
    const char* status);

// ax::mojom::Dropeffect
AX_BASE_EXPORT const char* ToString(ax::mojom::Dropeffect dropeffect);
AX_BASE_EXPORT ax::mojom::Dropeffect ParseDropeffect(const char* dropeffect);

}  // namespace ui

#endif  // UI_ACCESSIBILITY_AX_ENUM_UTIL_H_
