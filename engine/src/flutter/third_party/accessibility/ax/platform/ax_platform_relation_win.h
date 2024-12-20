// Copyright 2017 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef UI_ACCESSIBILITY_PLATFORM_AX_PLATFORM_RELATION_WIN_H_
#define UI_ACCESSIBILITY_PLATFORM_AX_PLATFORM_RELATION_WIN_H_

#include <oleacc.h>
#include <wrl/client.h>
#include <set>
#include <vector>

#include "base/compiler_specific.h"
#include "base/metrics/histogram_macros.h"
#include "base/observer_list.h"
#include "base/win/atl.h"
#include "third_party/iaccessible2/ia2_api_all.h"
#include "ui/accessibility/ax_export.h"
#include "ui/accessibility/ax_text_utils.h"
#include "ui/accessibility/platform/ax_platform_node_win.h"

namespace ui {

//
// AXPlatformRelationWin
//
// A simple implementation of IAccessibleRelation, used to represent a
// relationship between one accessible node in the tree and
// potentially multiple target nodes. Also contains a utility function
// to compute all of the possible IAccessible2 relations and reverse
// relations given the internal relation id attributes.
class AXPlatformRelationWin : public CComObjectRootEx<CComMultiThreadModel>,
                              public IAccessibleRelation {
 public:
  BEGIN_COM_MAP(AXPlatformRelationWin)
  COM_INTERFACE_ENTRY(IAccessibleRelation)
  END_COM_MAP()

  AXPlatformRelationWin();
  virtual ~AXPlatformRelationWin();

  // This is the main utility function that enumerates all of the possible
  // IAccessible2 relations between one node and any other node in the tree.
  // Forward relations come from the int_attributes and intlist_attributes
  // in node_data. Reverse relations come from querying |delegate| for the
  // reverse relations given |node_data.id|.
  //
  // If you pass -1 for |desired_index| and "" for |desired_ia2_relation|,
  // it will return a count of all relations.
  //
  // If you pass either an index in |desired_index| or a specific relation
  // in |desired_ia2_relation|, the first matching relation will be returned in
  // |out_ia2_relation| and |out_targets| (both of which must not be null),
  // and it will return 1 on success, and 0 if none were found matching that
  // criteria.
  static int EnumerateRelationships(AXPlatformNodeBase* node,
                                    int desired_index,
                                    const base::string16& desired_ia2_relation,
                                    base::string16* out_ia2_relation,
                                    std::set<AXPlatformNode*>* out_targets);

  void Initialize(const base::string16& type);
  void Invalidate();
  void AddTarget(AXPlatformNodeWin* target);

  // IAccessibleRelation methods.
  IFACEMETHODIMP get_relationType(BSTR* relation_type) override;
  IFACEMETHODIMP get_nTargets(LONG* n_targets) override;
  IFACEMETHODIMP get_target(LONG target_index, IUnknown** target) override;
  IFACEMETHODIMP get_targets(LONG max_targets,
                             IUnknown** targets,
                             LONG* n_targets) override;
  IFACEMETHODIMP get_localizedRelationType(BSTR* relation_type) override;

 private:
  base::string16 type_;
  std::vector<Microsoft::WRL::ComPtr<AXPlatformNodeWin>> targets_;
};

}  // namespace ui

#endif  // UI_ACCESSIBILITY_PLATFORM_AX_PLATFORM_RELATION_WIN_H_
