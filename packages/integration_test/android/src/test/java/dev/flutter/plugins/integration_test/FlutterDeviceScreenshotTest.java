// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package dev.flutter.plugins.integration_test;

import org.junit.Test;

import static org.junit.Assert.*;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import org.mockito.MockedStatic;
import org.mockito.Mockito;

import android.app.Activity;
import android.view.View;

import androidx.test.runner.AndroidJUnitRunner;

import io.flutter.embedding.android.FlutterActivity;
import io.flutter.embedding.android.FlutterFragment;
import io.flutter.embedding.android.FlutterFragmentActivity;
import io.flutter.embedding.android.FlutterView;

public class FlutterDeviceScreenshotTest extends AndroidJUnitRunner {
    @Test
    public void getFlutterView_returnsNullForNonFlutterActivity() {
        Activity mockActivity = mock(Activity.class);
        assertNull(FlutterDeviceScreenshot.getFlutterView(mockActivity));
    }

    @Test
    public void getFlutterView_returnsFlutterViewForFlutterActivity() {
        FlutterView mockFlutterView = mock(FlutterView.class);
        FlutterActivity mockFlutterActivity = mock(FlutterActivity.class);

        mockFlutterActivity.flutterViewId = 123;
        when(mockFlutterActivity.findViewById(mockFlutterActivity.flutterViewId))
                .thenReturn(mockFlutterView);

        assertEquals(
                FlutterDeviceScreenshot.getFlutterView(mockFlutterActivity),
                mockFlutterView
        );
    }

    @Test
    public void getFlutterView_returnsFlutterViewForFlutterFragmentActivity() {
        // Mock the static call to View.generateViewId that FlutterFragment.FRAGMENT_CONTAINER_ID needs.
        // For why this test currently doesn't use Robolectric,
        // see https://github.com/flutter/flutter/pull/148803.
        try (MockedStatic<View> mockedStatic = Mockito.mockStatic(View.class)) {
            FlutterView mockFlutterView = mock(FlutterView.class);
            FlutterFragmentActivity mockFlutterFragmentActivity = mock(FlutterFragmentActivity.class);

            when(mockFlutterFragmentActivity.getFlutterFragmentViewId()).thenReturn(123);
            when(mockFlutterFragmentActivity.findViewById(123))
                    .thenReturn(mockFlutterView);

            assertEquals(
                    FlutterDeviceScreenshot.getFlutterView(mockFlutterFragmentActivity),
                    mockFlutterView
            );
        }
    }
}
