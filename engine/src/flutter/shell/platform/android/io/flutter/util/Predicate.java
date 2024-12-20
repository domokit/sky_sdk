// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.util;

// TODO(dnfield): remove this if/when we can use appcompat to support it.
// java.util.function.Predicate isn't available until API24
public interface Predicate<T> {
  public abstract boolean test(T t);
}
