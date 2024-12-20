// Copyright 2017 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef UI_ACCESSIBILITY_AX_MODE_H_
#define UI_ACCESSIBILITY_AX_MODE_H_

#include <cstdint>
#include <ostream>
#include <string>

#include "ax_base_export.h"

namespace ui {

class AX_BASE_EXPORT AXMode {
 public:
  static constexpr uint32_t kFirstModeFlag = 1 << 0;

  // Native accessibility APIs, specific to each platform, are enabled.
  // When this mode is set that indicates the presence of a third-party
  // client accessing Chrome via accessibility APIs. However, unless one
  // of the modes below is set, the contents of web pages will not be
  // accessible.
  static constexpr uint32_t kNativeAPIs = 1 << 0;

  // The renderer process will generate an accessibility tree containing
  // basic information about all nodes, including role, name, value,
  // state, and location. This is the minimum mode required in order for
  // web contents to be accessible, and the remaining modes are meaningless
  // unless this one is set.
  //
  // Note that sometimes this mode will be set when kNativeAPI is not, when the
  // content layer embedder is providing accessibility support via some other
  // mechanism other than what's implemented in content/browser.
  static constexpr uint32_t kWebContents = 1 << 1;

  // The accessibility tree will contain inline text boxes, which are
  // necessary to expose information about line breaks and word boundaries.
  // Without this mode, you can retrieve the plaintext value of a text field
  // but not the information about how it's broken down into lines.
  //
  // Note that when this mode is off it's still possible to request inline
  // text boxes for a specific node on-demand, asynchronously.
  static constexpr uint32_t kInlineTextBoxes = 1 << 2;

  // The accessibility tree will contain extra accessibility
  // attributes typically only needed by screen readers and other
  // assistive technology for blind users. Examples include text style
  // attributes, table cell information, live region properties, range
  // values, and relationship attributes.
  static constexpr uint32_t kScreenReader = 1 << 3;

  // The accessibility tree will contain the HTML tag name and HTML attributes
  // for all accessibility nodes that come from web content.
  static constexpr uint32_t kHTML = 1 << 4;

  // The accessibility tree will contain automatic image annotations.
  static constexpr uint32_t kLabelImages = 1 << 5;

  // The accessibility tree will contain enough information to export
  // an accessible PDF.
  static constexpr uint32_t kPDF = 1 << 6;

  // Update this to include the last supported mode flag. If you add
  // another, be sure to update the stream insertion operator for
  // logging and debugging.
  static constexpr uint32_t kLastModeFlag = 1 << 6;

  constexpr AXMode() : flags_(0) {}
  constexpr AXMode(uint32_t flags) : flags_(flags) {}

  bool has_mode(uint32_t flag) const { return (flags_ & flag) > 0; }

  void set_mode(uint32_t flag, bool value) {
    flags_ = value ? (flags_ | flag) : (flags_ & ~flag);
  }

  uint32_t mode() const { return flags_; }

  bool operator==(AXMode rhs) const { return flags_ == rhs.flags_; }

  bool is_mode_off() const { return flags_ == 0; }

  bool operator!=(AXMode rhs) const { return !(*this == rhs); }

  AXMode& operator|=(const AXMode& rhs) {
    flags_ |= rhs.flags_;
    return *this;
  }

  std::string ToString() const;

 private:
  uint32_t flags_;
};

static constexpr AXMode kAXModeWebContentsOnly(AXMode::kWebContents |
                                               AXMode::kInlineTextBoxes |
                                               AXMode::kScreenReader |
                                               AXMode::kHTML);

static constexpr AXMode kAXModeComplete(AXMode::kNativeAPIs |
                                        AXMode::kWebContents |
                                        AXMode::kInlineTextBoxes |
                                        AXMode::kScreenReader | AXMode::kHTML);

// For debugging, test assertions, etc.
AX_BASE_EXPORT std::ostream& operator<<(std::ostream& stream,
                                        const AXMode& mode);

}  // namespace ui

#endif  // UI_ACCESSIBILITY_AX_MODE_H_
