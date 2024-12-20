// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/windows/settings_plugin.h"

#include "flutter/shell/platform/common/json_message_codec.h"
#include "flutter/shell/platform/windows/system_utils.h"

namespace flutter {

namespace {
constexpr char kChannelName[] = "flutter/settings";

constexpr char kAlwaysUse24HourFormat[] = "alwaysUse24HourFormat";
constexpr char kTextScaleFactor[] = "textScaleFactor";
constexpr char kPlatformBrightness[] = "platformBrightness";

constexpr char kPlatformBrightnessDark[] = "dark";
constexpr char kPlatformBrightnessLight[] = "light";

constexpr wchar_t kGetPreferredBrightnessRegKey[] =
    L"Software\\Microsoft\\Windows\\CurrentVersion\\Themes\\Personalize";
constexpr wchar_t kGetPreferredBrightnessRegValue[] = L"AppsUseLightTheme";

constexpr wchar_t kGetTextScaleFactorRegKey[] =
    L"Software\\Microsoft\\Accessibility";
constexpr wchar_t kGetTextScaleFactorRegValue[] = L"TextScaleFactor";

// Return an approximation of the apparent luminance of a given color.
int GetLuminance(DWORD color) {
  int r = GetRValue(color);
  int g = GetGValue(color);
  int b = GetBValue(color);
  return (r + r + r + b + (g << 2)) >> 3;
}

// Return kLight if light mode for apps is selected, otherwise return kDark.
SettingsPlugin::PlatformBrightness GetThemeBrightness() {
  DWORD use_light_theme;
  DWORD use_light_theme_size = sizeof(use_light_theme);
  LONG result = RegGetValue(HKEY_CURRENT_USER, kGetPreferredBrightnessRegKey,
                            kGetPreferredBrightnessRegValue, RRF_RT_REG_DWORD,
                            nullptr, &use_light_theme, &use_light_theme_size);

  if (result == 0) {
    return use_light_theme ? SettingsPlugin::PlatformBrightness::kLight
                           : SettingsPlugin::PlatformBrightness::kDark;
  } else {
    // The current OS does not support dark mode. (Older Windows 10 or before
    // Windows 10)
    return SettingsPlugin::PlatformBrightness::kLight;
  }
}
}  // namespace

SettingsPlugin::SettingsPlugin(BinaryMessenger* messenger,
                               TaskRunner* task_runner)
    : channel_(std::make_unique<BasicMessageChannel<rapidjson::Document>>(
          messenger,
          kChannelName,
          &JsonMessageCodec::GetInstance())),
      task_runner_(task_runner) {}

SettingsPlugin::~SettingsPlugin() {
  StopWatching();
}

void SettingsPlugin::SendSettings() {
  rapidjson::Document settings(rapidjson::kObjectType);
  auto& allocator = settings.GetAllocator();
  settings.AddMember(kAlwaysUse24HourFormat, GetAlwaysUse24HourFormat(),
                     allocator);
  settings.AddMember(kTextScaleFactor, GetTextScaleFactor(), allocator);

  if (GetPreferredBrightness() == PlatformBrightness::kDark) {
    settings.AddMember(kPlatformBrightness, kPlatformBrightnessDark, allocator);
  } else {
    settings.AddMember(kPlatformBrightness, kPlatformBrightnessLight,
                       allocator);
  }
  channel_->Send(settings);
}

void SettingsPlugin::StartWatching() {
  RegOpenKeyEx(HKEY_CURRENT_USER, kGetPreferredBrightnessRegKey,
               RRF_RT_REG_DWORD, KEY_NOTIFY, &preferred_brightness_reg_hkey_);
  RegOpenKeyEx(HKEY_CURRENT_USER, kGetTextScaleFactorRegKey, RRF_RT_REG_DWORD,
               KEY_NOTIFY, &text_scale_factor_reg_hkey_);

  // Start watching when the keys exist.
  if (preferred_brightness_reg_hkey_ != nullptr) {
    WatchPreferredBrightnessChanged();
  }
  if (text_scale_factor_reg_hkey_ != nullptr) {
    WatchTextScaleFactorChanged();
  }
}

void SettingsPlugin::StopWatching() {
  preferred_brightness_changed_watcher_ = nullptr;
  text_scale_factor_changed_watcher_ = nullptr;

  if (preferred_brightness_reg_hkey_ != nullptr) {
    RegCloseKey(preferred_brightness_reg_hkey_);
  }
  if (text_scale_factor_reg_hkey_ != nullptr) {
    RegCloseKey(text_scale_factor_reg_hkey_);
  }
}

bool SettingsPlugin::GetAlwaysUse24HourFormat() {
  return Prefer24HourTime(GetUserTimeFormat());
}

float SettingsPlugin::GetTextScaleFactor() {
  DWORD text_scale_factor;
  DWORD text_scale_factor_size = sizeof(text_scale_factor);
  LONG result = RegGetValue(
      HKEY_CURRENT_USER, kGetTextScaleFactorRegKey, kGetTextScaleFactorRegValue,
      RRF_RT_REG_DWORD, nullptr, &text_scale_factor, &text_scale_factor_size);

  if (result == 0) {
    return text_scale_factor / 100.0;
  } else {
    // The current OS does not have text scale factor.
    return 1.0;
  }
}

SettingsPlugin::PlatformBrightness SettingsPlugin::GetPreferredBrightness() {
  if (is_high_contrast_) {
    DWORD window_color = GetSysColor(COLOR_WINDOW);
    int luminance = GetLuminance(window_color);
    return luminance >= 127 ? SettingsPlugin::PlatformBrightness::kLight
                            : SettingsPlugin::PlatformBrightness::kDark;
  } else {
    return GetThemeBrightness();
  }
}

void SettingsPlugin::WatchPreferredBrightnessChanged() {
  preferred_brightness_changed_watcher_ =
      std::make_unique<EventWatcher>([this]() {
        task_runner_->PostTask([this]() {
          SendSettings();
          WatchPreferredBrightnessChanged();
        });
      });

  RegNotifyChangeKeyValue(
      preferred_brightness_reg_hkey_, FALSE, REG_NOTIFY_CHANGE_LAST_SET,
      preferred_brightness_changed_watcher_->GetHandle(), TRUE);
}

void SettingsPlugin::WatchTextScaleFactorChanged() {
  text_scale_factor_changed_watcher_ = std::make_unique<EventWatcher>([this]() {
    task_runner_->PostTask([this]() {
      SendSettings();
      WatchTextScaleFactorChanged();
    });
  });

  RegNotifyChangeKeyValue(
      text_scale_factor_reg_hkey_, FALSE, REG_NOTIFY_CHANGE_LAST_SET,
      text_scale_factor_changed_watcher_->GetHandle(), TRUE);
}

void SettingsPlugin::UpdateHighContrastMode(bool is_high_contrast) {
  is_high_contrast_ = is_high_contrast;
  SendSettings();
}

}  // namespace flutter
