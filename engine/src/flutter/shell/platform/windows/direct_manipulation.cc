// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/fml/logging.h"

#include <algorithm>

#include "flutter/shell/platform/windows/direct_manipulation.h"
#include "flutter/shell/platform/windows/flutter_window.h"
#include "flutter/shell/platform/windows/window_binding_handler_delegate.h"

#define RETURN_IF_FAILED(operation)            \
  if (FAILED(operation)) {                     \
    FML_LOG(ERROR) << #operation << " failed"; \
    manager_ = nullptr;                        \
    updateManager_ = nullptr;                  \
    viewport_ = nullptr;                       \
    return -1;                                 \
  }

#define WARN_IF_FAILED(operation)              \
  if (FAILED(operation)) {                     \
    FML_LOG(ERROR) << #operation << " failed"; \
  }

namespace flutter {

int32_t DirectManipulationEventHandler::GetDeviceId() {
  return (int32_t) reinterpret_cast<int64_t>(this);
}

STDMETHODIMP DirectManipulationEventHandler::QueryInterface(REFIID iid,
                                                            void** ppv) {
  if ((iid == IID_IUnknown) ||
      (iid == IID_IDirectManipulationViewportEventHandler)) {
    *ppv = static_cast<IDirectManipulationViewportEventHandler*>(this);
    AddRef();
    return S_OK;
  } else if (iid == IID_IDirectManipulationInteractionEventHandler) {
    *ppv = static_cast<IDirectManipulationInteractionEventHandler*>(this);
    AddRef();
    return S_OK;
  }
  return E_NOINTERFACE;
}

DirectManipulationEventHandler::GestureData
DirectManipulationEventHandler::ConvertToGestureData(float transform[6]) {
  // DirectManipulation provides updates with very high precision. If the user
  // holds their fingers steady on a trackpad, DirectManipulation sends
  // jittery updates. This calculation will reduce the precision of the scale
  // value of the event to avoid jitter.
  const int mantissa_bits_chop = 2;
  const float factor = (1 << mantissa_bits_chop) + 1;
  float c = factor * transform[0];
  return GestureData{
      c - (c - transform[0]),  // scale
      transform[4],            // pan_x
      transform[5],            // pan_y
  };
}

HRESULT DirectManipulationEventHandler::OnViewportStatusChanged(
    IDirectManipulationViewport* viewport,
    DIRECTMANIPULATION_STATUS current,
    DIRECTMANIPULATION_STATUS previous) {
  if (during_synthesized_reset_) {
    during_synthesized_reset_ = current != DIRECTMANIPULATION_READY;
    return S_OK;
  }
  during_inertia_ = current == DIRECTMANIPULATION_INERTIA;
  if (current == DIRECTMANIPULATION_RUNNING) {
    IDirectManipulationContent* content;
    HRESULT hr = viewport->GetPrimaryContent(IID_PPV_ARGS(&content));
    if (SUCCEEDED(hr)) {
      float transform[6];
      hr = content->GetContentTransform(transform, ARRAYSIZE(transform));
      if (SUCCEEDED(hr)) {
        initial_gesture_data_ = ConvertToGestureData(transform);
      } else {
        FML_LOG(ERROR) << "GetContentTransform failed";
      }
    } else {
      FML_LOG(ERROR) << "GetPrimaryContent failed";
    }
    if (owner_->binding_handler_delegate) {
      owner_->binding_handler_delegate->OnPointerPanZoomStart(GetDeviceId());
    }
  } else if (previous == DIRECTMANIPULATION_RUNNING) {
    // Reset deltas to ensure only inertia values will be compared later.
    last_pan_delta_x_ = 0.0;
    last_pan_delta_y_ = 0.0;
    if (owner_->binding_handler_delegate) {
      owner_->binding_handler_delegate->OnPointerPanZoomEnd(GetDeviceId());
    }
  } else if (previous == DIRECTMANIPULATION_INERTIA) {
    if (owner_->binding_handler_delegate &&
        (std::max)(std::abs(last_pan_delta_x_), std::abs(last_pan_delta_y_)) >
            0.01) {
      owner_->binding_handler_delegate->OnScrollInertiaCancel(GetDeviceId());
    }
    // Need to reset the content transform to its original position
    // so that we are ready for the next gesture.
    // Use during_synthesized_reset_ flag to prevent sending reset also to the
    // framework.
    during_synthesized_reset_ = true;
    last_pan_x_ = 0.0;
    last_pan_y_ = 0.0;
    last_pan_delta_x_ = 0.0;
    last_pan_delta_y_ = 0.0;
    RECT rect;
    HRESULT hr = viewport->GetViewportRect(&rect);
    if (FAILED(hr)) {
      FML_LOG(ERROR) << "Failed to get the current viewport rect";
      return E_FAIL;
    }
    hr = viewport->ZoomToRect(rect.left, rect.top, rect.right, rect.bottom,
                              false);
    if (FAILED(hr)) {
      FML_LOG(ERROR) << "Failed to reset the gesture using ZoomToRect";
      return E_FAIL;
    }
  }
  return S_OK;
}

HRESULT DirectManipulationEventHandler::OnViewportUpdated(
    IDirectManipulationViewport* viewport) {
  return S_OK;
}

HRESULT DirectManipulationEventHandler::OnContentUpdated(
    IDirectManipulationViewport* viewport,
    IDirectManipulationContent* content) {
  float transform[6];
  HRESULT hr = content->GetContentTransform(transform, ARRAYSIZE(transform));
  if (FAILED(hr)) {
    FML_LOG(ERROR) << "GetContentTransform failed";
    return S_OK;
  }
  if (!during_synthesized_reset_) {
    GestureData data = ConvertToGestureData(transform);
    float scale = data.scale / initial_gesture_data_.scale;
    float pan_x = data.pan_x - initial_gesture_data_.pan_x;
    float pan_y = data.pan_y - initial_gesture_data_.pan_y;
    last_pan_delta_x_ = pan_x - last_pan_x_;
    last_pan_delta_y_ = pan_y - last_pan_y_;
    last_pan_x_ = pan_x;
    last_pan_y_ = pan_y;
    if (owner_->binding_handler_delegate && !during_inertia_) {
      owner_->binding_handler_delegate->OnPointerPanZoomUpdate(
          GetDeviceId(), pan_x, pan_y, scale, 0);
    }
  }
  return S_OK;
}

HRESULT DirectManipulationEventHandler::OnInteraction(
    IDirectManipulationViewport2* viewport,
    DIRECTMANIPULATION_INTERACTION_TYPE interaction) {
  return S_OK;
}

ULONG STDMETHODCALLTYPE DirectManipulationEventHandler::AddRef() {
  RefCountedThreadSafe::AddRef();
  return 0;
}

ULONG STDMETHODCALLTYPE DirectManipulationEventHandler::Release() {
  RefCountedThreadSafe::Release();
  return 0;
}

DirectManipulationOwner::DirectManipulationOwner(FlutterWindow* window)
    : window_(window) {}

int DirectManipulationOwner::Init(unsigned int width, unsigned int height) {
  RETURN_IF_FAILED(CoCreateInstance(CLSID_DirectManipulationManager, nullptr,
                                    CLSCTX_INPROC_SERVER,
                                    IID_IDirectManipulationManager, &manager_));
  RETURN_IF_FAILED(manager_->GetUpdateManager(
      IID_IDirectManipulationUpdateManager, &updateManager_));
  RETURN_IF_FAILED(manager_->CreateViewport(nullptr, window_->GetWindowHandle(),
                                            IID_IDirectManipulationViewport,
                                            &viewport_));
  DIRECTMANIPULATION_CONFIGURATION configuration =
      DIRECTMANIPULATION_CONFIGURATION_INTERACTION |
      DIRECTMANIPULATION_CONFIGURATION_TRANSLATION_X |
      DIRECTMANIPULATION_CONFIGURATION_TRANSLATION_Y |
      DIRECTMANIPULATION_CONFIGURATION_SCALING |
      DIRECTMANIPULATION_CONFIGURATION_TRANSLATION_INERTIA;
  RETURN_IF_FAILED(viewport_->ActivateConfiguration(configuration));
  RETURN_IF_FAILED(viewport_->SetViewportOptions(
      DIRECTMANIPULATION_VIEWPORT_OPTIONS_MANUALUPDATE));
  handler_ = fml::MakeRefCounted<DirectManipulationEventHandler>(this);
  RETURN_IF_FAILED(viewport_->AddEventHandler(
      window_->GetWindowHandle(), handler_.get(), &viewportHandlerCookie_));
  RECT rect = {0, 0, (LONG)width, (LONG)height};
  RETURN_IF_FAILED(viewport_->SetViewportRect(&rect));
  RETURN_IF_FAILED(manager_->Activate(window_->GetWindowHandle()));
  RETURN_IF_FAILED(viewport_->Enable());
  RETURN_IF_FAILED(updateManager_->Update(nullptr));
  return 0;
}

void DirectManipulationOwner::ResizeViewport(unsigned int width,
                                             unsigned int height) {
  if (viewport_) {
    RECT rect = {0, 0, (LONG)width, (LONG)height};
    WARN_IF_FAILED(viewport_->SetViewportRect(&rect));
  }
}

void DirectManipulationOwner::Destroy() {
  if (handler_) {
    handler_->owner_ = nullptr;
  }

  if (viewport_) {
    WARN_IF_FAILED(viewport_->Disable());
    WARN_IF_FAILED(viewport_->Disable());
    WARN_IF_FAILED(viewport_->RemoveEventHandler(viewportHandlerCookie_));
    WARN_IF_FAILED(viewport_->Abandon());
  }

  if (window_ && manager_) {
    WARN_IF_FAILED(manager_->Deactivate(window_->GetWindowHandle()));
  }

  handler_ = nullptr;
  viewport_ = nullptr;
  updateManager_ = nullptr;
  manager_ = nullptr;
  window_ = nullptr;
}

void DirectManipulationOwner::SetContact(UINT contactId) {
  if (viewport_) {
    viewport_->SetContact(contactId);
  }
}

void DirectManipulationOwner::SetBindingHandlerDelegate(
    WindowBindingHandlerDelegate* delegate) {
  binding_handler_delegate = delegate;
}

void DirectManipulationOwner::Update() {
  if (updateManager_) {
    HRESULT hr = updateManager_->Update(nullptr);
    if (FAILED(hr)) {
      FML_LOG(ERROR) << "updateManager_->Update failed";
      auto error = GetLastError();
      FML_LOG(ERROR) << error;
      LPWSTR message = nullptr;
      size_t size = FormatMessageW(
          FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM |
              FORMAT_MESSAGE_IGNORE_INSERTS,
          NULL, error, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
          reinterpret_cast<LPWSTR>(&message), 0, NULL);
      FML_LOG(ERROR) << message;
    }
  }
}

}  // namespace flutter
