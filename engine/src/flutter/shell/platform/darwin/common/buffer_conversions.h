// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_COMMON_BUFFER_CONVERSIONS_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_COMMON_BUFFER_CONVERSIONS_H_

#include <Foundation/Foundation.h>

#include <vector>

#include "flutter/fml/mapping.h"

namespace flutter {

fml::MallocMapping CopyNSDataToMapping(NSData* data);

NSData* ConvertMappingToNSData(fml::MallocMapping buffer);

std::unique_ptr<fml::Mapping> ConvertNSDataToMappingPtr(NSData* data);

NSData* CopyMappingPtrToNSData(std::unique_ptr<fml::Mapping> mapping);

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_COMMON_BUFFER_CONVERSIONS_H_
