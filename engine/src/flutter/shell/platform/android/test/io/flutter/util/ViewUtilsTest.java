// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.util;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import android.app.Activity;
import android.content.Context;
import android.content.ContextWrapper;
import android.view.View;
import android.view.ViewGroup;
import androidx.test.ext.junit.runners.AndroidJUnit4;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.robolectric.annotation.Config;

@Config(manifest = Config.NONE)
@RunWith(AndroidJUnit4.class)
public class ViewUtilsTest {
  @Test
  public void canGetActivity() {
    // Non activity context returns null
    Context nonActivityContext = mock(Context.class);
    assertEquals(null, ViewUtils.getActivity(nonActivityContext));

    Activity activity = mock(Activity.class);
    assertEquals(activity, ViewUtils.getActivity(activity));

    ContextWrapper wrapper = new ContextWrapper(new ContextWrapper(activity));
    assertEquals(activity, ViewUtils.getActivity(wrapper));
  }

  @Test
  public void childHasFocus_rootHasFocus() {
    final View rootView = mock(View.class);
    when(rootView.hasFocus()).thenReturn(true);
    assertTrue(ViewUtils.childHasFocus(rootView));
  }

  @Test
  public void childHasFocus_rootDoesNotHaveFocus() {
    final View rootView = mock(View.class);
    when(rootView.hasFocus()).thenReturn(false);
    assertFalse(ViewUtils.childHasFocus(rootView));
  }

  @Test
  public void childHasFocus_rootIsNull() {
    assertFalse(ViewUtils.childHasFocus(null));
  }

  @Test
  public void childHasFocus_childHasFocus() {
    final View childView = mock(View.class);
    when(childView.hasFocus()).thenReturn(true);

    final ViewGroup rootView = mock(ViewGroup.class);
    when(rootView.getChildCount()).thenReturn(1);
    when(rootView.getChildAt(0)).thenReturn(childView);

    assertTrue(ViewUtils.childHasFocus(rootView));
  }

  @Test
  public void childHasFocus_childDoesNotHaveFocus() {
    final View childView = mock(View.class);
    when(childView.hasFocus()).thenReturn(false);

    final ViewGroup rootView = mock(ViewGroup.class);
    when(rootView.getChildCount()).thenReturn(1);
    when(rootView.getChildAt(0)).thenReturn(childView);

    assertFalse(ViewUtils.childHasFocus(rootView));
  }
}
