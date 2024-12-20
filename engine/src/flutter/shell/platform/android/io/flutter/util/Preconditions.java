// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.util;

import androidx.annotation.Nullable;

/**
 * Static convenience methods that help a method or constructor check whether it was invoked
 * correctly (that is, whether its <i>preconditions</i> were met).
 */
public final class Preconditions {
  private Preconditions() {}

  /**
   * Ensures that an object reference passed as a parameter to the calling method is not null.
   *
   * @param reference an object reference
   * @return the non-null reference that was validated
   * @throws NullPointerException if {@code reference} is null
   */
  public static <T> T checkNotNull(T reference) {
    if (reference == null) {
      throw new NullPointerException();
    }
    return reference;
  }

  /**
   * Ensures the truth of an expression involving the state of the calling instance.
   *
   * @param expression a boolean expression that must be checked to be true
   * @throws IllegalStateException if {@code expression} is false
   */
  public static void checkState(boolean expression) {
    if (!expression) {
      throw new IllegalStateException();
    }
  }

  /**
   * Ensures the truth of an expression involving the state of the calling instance.
   *
   * @param expression a boolean expression that must be checked to be true
   * @param errorMessage the exception message to use if the check fails; will be converted to a
   *     string using {@link String#valueOf(Object)}
   * @throws IllegalStateException if {@code expression} is false
   */
  public static void checkState(boolean expression, @Nullable Object errorMessage) {
    if (!expression) {
      throw new IllegalStateException(String.valueOf(errorMessage));
    }
  }
}
