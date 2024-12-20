// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.engine.systemchannels;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.annotation.VisibleForTesting;
import io.flutter.Log;
import io.flutter.embedding.engine.dart.DartExecutor;
import io.flutter.plugin.common.JSONMethodCodec;
import io.flutter.plugin.common.MethodCall;
import io.flutter.plugin.common.MethodChannel;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import org.json.JSONException;
import org.json.JSONObject;

/** Sends the platform's locales to Dart. */
public class LocalizationChannel {
  private static final String TAG = "LocalizationChannel";

  @NonNull public final MethodChannel channel;
  @Nullable private LocalizationMessageHandler localizationMessageHandler;

  @NonNull @VisibleForTesting
  public final MethodChannel.MethodCallHandler handler =
      new MethodChannel.MethodCallHandler() {
        @Override
        public void onMethodCall(@NonNull MethodCall call, @NonNull MethodChannel.Result result) {
          if (localizationMessageHandler == null) {
            // If no explicit LocalizationMessageHandler has been registered then we don't
            // need to forward this call to an API. Return.
            return;
          }

          String method = call.method;
          switch (method) {
            case "Localization.getStringResource":
              JSONObject arguments = call.<JSONObject>arguments();
              try {
                String key = arguments.getString("key");
                String localeString = null;
                if (arguments.has("locale")) {
                  localeString = arguments.getString("locale");
                }
                result.success(localizationMessageHandler.getStringResource(key, localeString));
              } catch (JSONException exception) {
                result.error("error", exception.getMessage(), null);
              }
              break;
            default:
              result.notImplemented();
              break;
          }
        }
      };

  public LocalizationChannel(@NonNull DartExecutor dartExecutor) {
    this.channel =
        new MethodChannel(dartExecutor, "flutter/localization", JSONMethodCodec.INSTANCE);
    channel.setMethodCallHandler(handler);
  }

  /**
   * Sets the {@link LocalizationMessageHandler} which receives all events and requests that are
   * parsed from the underlying platform channel.
   */
  public void setLocalizationMessageHandler(
      @Nullable LocalizationMessageHandler localizationMessageHandler) {
    this.localizationMessageHandler = localizationMessageHandler;
  }

  /** Send the given {@code locales} to Dart. */
  public void sendLocales(@NonNull List<Locale> locales) {
    Log.v(TAG, "Sending Locales to Flutter.");
    // Send the user's preferred locales.
    List<String> data = new ArrayList<>();
    for (Locale locale : locales) {
      Log.v(
          TAG,
          "Locale (Language: "
              + locale.getLanguage()
              + ", Country: "
              + locale.getCountry()
              + ", Variant: "
              + locale.getVariant()
              + ")");
      data.add(locale.getLanguage());
      data.add(locale.getCountry());
      data.add(locale.getScript());
      data.add(locale.getVariant());
    }
    channel.invokeMethod("setLocale", data);
  }

  /**
   * Handler that receives platform messages sent from Flutter to Android through a given {@link
   * PlatformChannel}.
   *
   * <p>To register a {@code LocalizationMessageHandler} with a {@link PlatformChannel}, see {@link
   * LocalizationChannel#setLocalizationMessageHandler(LocalizationMessageHandler)}.
   */
  public interface LocalizationMessageHandler {
    /**
     * The Flutter application would like to obtain the string resource of given {@code key} in
     * {@code locale}.
     */
    @NonNull
    String getStringResource(@NonNull String key, @NonNull String locale);
  }
}
