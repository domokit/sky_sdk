// Copyright 2020 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "uia_registrar_win.h"
#include <wrl/implements.h>

namespace ui {

UiaRegistrarWin::UiaRegistrarWin() {
  // Create the registrar object and get the IUIAutomationRegistrar
  // interface pointer.
  Microsoft::WRL::ComPtr<IUIAutomationRegistrar> registrar;
  if (FAILED(CoCreateInstance(CLSID_CUIAutomationRegistrar, nullptr,
                              CLSCTX_INPROC_SERVER, IID_IUIAutomationRegistrar,
                              &registrar)))
    return;

  // Register the custom UIA property that represents the unique id of an UIA
  // element which also matches its corresponding IA2 element's unique id.
  UIAutomationPropertyInfo unique_id_property_info = {
      kUiaPropertyUniqueIdGuid, L"UniqueId", UIAutomationType_String};
  registrar->RegisterProperty(&unique_id_property_info,
                              &uia_unique_id_property_id_);

  // Register the custom UIA event that represents the test end event for the
  // UIA test suite.
  UIAutomationEventInfo test_complete_event_info = {
      kUiaEventTestCompleteSentinelGuid, L"kUiaTestCompleteSentinel"};
  registrar->RegisterEvent(&test_complete_event_info,
                           &uia_test_complete_event_id_);
}

UiaRegistrarWin::~UiaRegistrarWin() = default;

PROPERTYID UiaRegistrarWin::GetUiaUniqueIdPropertyId() const {
  return uia_unique_id_property_id_;
}

EVENTID UiaRegistrarWin::GetUiaTestCompleteEventId() const {
  return uia_test_complete_event_id_;
}

const UiaRegistrarWin& UiaRegistrarWin::GetInstance() {
  static base::NoDestructor<UiaRegistrarWin> instance;
  return *instance;
}

}  // namespace ui
