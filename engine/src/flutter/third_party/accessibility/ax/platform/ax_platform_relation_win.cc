// Copyright 2017 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "ui/accessibility/platform/ax_platform_relation_win.h"

#include <wrl/client.h>

#include <algorithm>
#include <vector>

#include "base/lazy_instance.h"
#include "base/strings/string_number_conversions.h"
#include "base/strings/string_util.h"
#include "base/strings/utf_string_conversions.h"
#include "base/win/enum_variant.h"
#include "base/win/scoped_variant.h"
#include "third_party/iaccessible2/ia2_api_all.h"
#include "third_party/skia/include/core/SkColor.h"
#include "ui/accessibility/ax_action_data.h"
#include "ui/accessibility/ax_mode_observer.h"
#include "ui/accessibility/ax_node_data.h"
#include "ui/accessibility/ax_role_properties.h"
#include "ui/accessibility/ax_text_utils.h"
#include "ui/accessibility/ax_tree_data.h"
#include "ui/accessibility/platform/ax_platform_node_base.h"
#include "ui/accessibility/platform/ax_platform_node_delegate.h"
#include "ui/accessibility/platform/ax_unique_id.h"
#include "ui/base/win/atl_module.h"
#include "ui/gfx/geometry/rect_conversions.h"

namespace ui {

AXPlatformRelationWin::AXPlatformRelationWin() {
  win::CreateATLModuleIfNeeded();
}

AXPlatformRelationWin::~AXPlatformRelationWin() {}

base::string16 GetIA2RelationFromIntAttr(ax::mojom::IntAttribute attribute) {
  switch (attribute) {
    case ax::mojom::IntAttribute::kMemberOfId:
      return IA2_RELATION_MEMBER_OF;
    case ax::mojom::IntAttribute::kErrormessageId:
      return IA2_RELATION_ERROR;
    case ax::mojom::IntAttribute::kPopupForId:
      // Map "popup for" to "controlled by".
      // Unlike ATK there is no special IA2 popup-for relationship, but it can
      // be exposed via the controlled by relation, which is also computed for
      // content as the reverse of the controls relationship.
      return IA2_RELATION_CONTROLLED_BY;
    default:
      break;
  }
  return base::string16();
}

base::string16 GetIA2RelationFromIntListAttr(
    ax::mojom::IntListAttribute attribute) {
  switch (attribute) {
    case ax::mojom::IntListAttribute::kControlsIds:
      return IA2_RELATION_CONTROLLER_FOR;
    case ax::mojom::IntListAttribute::kDescribedbyIds:
      return IA2_RELATION_DESCRIBED_BY;
    case ax::mojom::IntListAttribute::kDetailsIds:
      return IA2_RELATION_DETAILS;
    case ax::mojom::IntListAttribute::kFlowtoIds:
      return IA2_RELATION_FLOWS_TO;
    case ax::mojom::IntListAttribute::kLabelledbyIds:
      return IA2_RELATION_LABELLED_BY;
    default:
      break;
  }
  return base::string16();
}

base::string16 GetIA2ReverseRelationFromIntAttr(
    ax::mojom::IntAttribute attribute) {
  switch (attribute) {
    case ax::mojom::IntAttribute::kErrormessageId:
      return IA2_RELATION_ERROR_FOR;
    default:
      break;
  }
  return base::string16();
}

base::string16 GetIA2ReverseRelationFromIntListAttr(
    ax::mojom::IntListAttribute attribute) {
  switch (attribute) {
    case ax::mojom::IntListAttribute::kControlsIds:
      return IA2_RELATION_CONTROLLED_BY;
    case ax::mojom::IntListAttribute::kDescribedbyIds:
      return IA2_RELATION_DESCRIPTION_FOR;
    case ax::mojom::IntListAttribute::kDetailsIds:
      return IA2_RELATION_DETAILS_FOR;
    case ax::mojom::IntListAttribute::kFlowtoIds:
      return IA2_RELATION_FLOWS_FROM;
    case ax::mojom::IntListAttribute::kLabelledbyIds:
      return IA2_RELATION_LABEL_FOR;
    default:
      break;
  }
  return base::string16();
}

// static
int AXPlatformRelationWin::EnumerateRelationships(
    AXPlatformNodeBase* node,
    int desired_index,
    const base::string16& desired_ia2_relation,
    base::string16* out_ia2_relation,
    std::set<AXPlatformNode*>* out_targets) {
  const AXNodeData& node_data = node->GetData();
  AXPlatformNodeDelegate* delegate = node->GetDelegate();

  // The first time this is called, populate vectors with all of the
  // int attributes and intlist attributes that have reverse relations
  // we care about on Windows. Computing these by calling
  // GetIA2ReverseRelationFrom{Int|IntList}Attr on every possible attribute
  // simplifies the work needed to support an additional relation
  // attribute in the future.
  static std::vector<ax::mojom::IntAttribute>
      int_attributes_with_reverse_relations;
  static std::vector<ax::mojom::IntListAttribute>
      intlist_attributes_with_reverse_relations;
  static bool first_time = true;
  if (first_time) {
    for (int32_t attr_index =
             static_cast<int32_t>(ax::mojom::IntAttribute::kNone);
         attr_index <= static_cast<int32_t>(ax::mojom::IntAttribute::kMaxValue);
         ++attr_index) {
      auto attr = static_cast<ax::mojom::IntAttribute>(attr_index);
      if (!GetIA2ReverseRelationFromIntAttr(attr).empty())
        int_attributes_with_reverse_relations.push_back(attr);
    }
    for (int32_t attr_index =
             static_cast<int32_t>(ax::mojom::IntListAttribute::kNone);
         attr_index <=
         static_cast<int32_t>(ax::mojom::IntListAttribute::kMaxValue);
         ++attr_index) {
      auto attr = static_cast<ax::mojom::IntListAttribute>(attr_index);
      if (!GetIA2ReverseRelationFromIntListAttr(attr).empty())
        intlist_attributes_with_reverse_relations.push_back(attr);
    }
    first_time = false;
  }

  // Enumerate all of the relations and reverse relations that
  // are exposed via IAccessible2 on Windows. We do this with a series
  // of loops. Every time we encounter one, we check if the caller
  // requested that particular relation by index, and return it.
  // Otherwise we build up and return the total number of relations found.
  int total_count = 0;

  // Iterate over all int attributes on this node to check the ones
  // that correspond to IAccessible2 relations.
  for (size_t i = 0; i < node_data.int_attributes.size(); ++i) {
    ax::mojom::IntAttribute int_attribute = node_data.int_attributes[i].first;
    base::string16 relation = GetIA2RelationFromIntAttr(int_attribute);
    if (!relation.empty() &&
        (desired_ia2_relation.empty() || desired_ia2_relation == relation)) {
      // Skip reflexive relations
      if (node_data.int_attributes[i].second == node_data.id)
        continue;
      if (desired_index == total_count) {
        *out_ia2_relation = relation;
        out_targets->insert(
            delegate->GetFromNodeID(node_data.int_attributes[i].second));
        return 1;
      }
      total_count++;
    }
  }

  // Iterate over all of the int attributes that have reverse relations
  // in IAccessible2, and query AXTree to see if the reverse relation exists.
  for (ax::mojom::IntAttribute int_attribute :
       int_attributes_with_reverse_relations) {
    base::string16 relation = GetIA2ReverseRelationFromIntAttr(int_attribute);
    std::set<AXPlatformNode*> targets =
        delegate->GetReverseRelations(int_attribute);
    // Erase reflexive relations.
    targets.erase(node);
    if (targets.size()) {
      if (!relation.empty() &&
          (desired_ia2_relation.empty() || desired_ia2_relation == relation)) {
        if (desired_index == total_count) {
          *out_ia2_relation = relation;
          *out_targets = targets;
          return 1;
        }
        total_count++;
      }
    }
  }

  // Iterate over all intlist attributes on this node to check the ones
  // that correspond to IAccessible2 relations.
  for (size_t i = 0; i < node_data.intlist_attributes.size(); ++i) {
    ax::mojom::IntListAttribute intlist_attribute =
        node_data.intlist_attributes[i].first;
    base::string16 relation = GetIA2RelationFromIntListAttr(intlist_attribute);
    if (!relation.empty() &&
        (desired_ia2_relation.empty() || desired_ia2_relation == relation)) {
      if (desired_index == total_count) {
        *out_ia2_relation = relation;
        for (int32_t target_id : node_data.intlist_attributes[i].second) {
          // Skip reflexive relations
          if (target_id == node_data.id)
            continue;
          out_targets->insert(delegate->GetFromNodeID(target_id));
        }
        if (out_targets->size() == 0)
          continue;
        return 1;
      }
      total_count++;
    }
  }

  // Iterate over all of the intlist attributes that have reverse relations
  // in IAccessible2, and query AXTree to see if the reverse relation exists.
  for (ax::mojom::IntListAttribute intlist_attribute :
       intlist_attributes_with_reverse_relations) {
    base::string16 relation =
        GetIA2ReverseRelationFromIntListAttr(intlist_attribute);
    std::set<AXPlatformNode*> targets =
        delegate->GetReverseRelations(intlist_attribute);
    // Erase reflexive relations.
    targets.erase(node);
    if (targets.size()) {
      if (!relation.empty() &&
          (desired_ia2_relation.empty() || desired_ia2_relation == relation)) {
        if (desired_index == total_count) {
          *out_ia2_relation = relation;
          *out_targets = targets;
          return 1;
        }
        total_count++;
      }
    }
  }

  return total_count;
}

void AXPlatformRelationWin::Initialize(const base::string16& type) {
  type_ = type;
}

void AXPlatformRelationWin::Invalidate() {
  targets_.clear();
}

void AXPlatformRelationWin::AddTarget(AXPlatformNodeWin* target) {
  targets_.push_back(target);
}

IFACEMETHODIMP AXPlatformRelationWin::get_relationType(BSTR* relation_type) {
  if (!relation_type)
    return E_INVALIDARG;

  *relation_type = SysAllocString(type_.c_str());
  DCHECK(*relation_type);
  return S_OK;
}

IFACEMETHODIMP AXPlatformRelationWin::get_nTargets(LONG* n_targets) {
  if (!n_targets)
    return E_INVALIDARG;

  *n_targets = static_cast<LONG>(targets_.size());
  return S_OK;
}

IFACEMETHODIMP AXPlatformRelationWin::get_target(LONG target_index,
                                                 IUnknown** target) {
  if (!target)
    return E_INVALIDARG;

  if (target_index < 0 || target_index >= static_cast<LONG>(targets_.size())) {
    return E_INVALIDARG;
  }

  *target = static_cast<IAccessible*>(targets_[target_index].Get());
  (*target)->AddRef();
  return S_OK;
}

IFACEMETHODIMP AXPlatformRelationWin::get_targets(LONG max_targets,
                                                  IUnknown** targets,
                                                  LONG* n_targets) {
  if (!targets || !n_targets)
    return E_INVALIDARG;

  LONG count = static_cast<LONG>(targets_.size());
  if (count > max_targets)
    count = max_targets;

  *n_targets = count;
  if (count == 0)
    return S_FALSE;

  for (LONG i = 0; i < count; ++i) {
    HRESULT result = get_target(i, &targets[i]);
    if (result != S_OK)
      return result;
  }

  return S_OK;
}

IFACEMETHODIMP
AXPlatformRelationWin::get_localizedRelationType(BSTR* relation_type) {
  return E_NOTIMPL;
}

}  // namespace ui
