// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.plugin.platform;

import java.util.HashMap;
import java.util.Map;

public class PlatformViewRegistryImpl implements PlatformViewRegistry {

  PlatformViewRegistryImpl() {
    viewFactories = new HashMap<>();
  }

  // Maps a platform view type id to its factory.
  private final Map<String, PlatformViewFactory> viewFactories;

  @Override
  public boolean registerViewFactory(String viewTypeId, PlatformViewFactory factory) {
    if (viewFactories.containsKey(viewTypeId)) return false;
    viewFactories.put(viewTypeId, factory);
    return true;
  }

  PlatformViewFactory getFactory(String viewTypeId) {
    return viewFactories.get(viewTypeId);
  }
}
