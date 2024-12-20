// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.engine.systemchannels;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import io.flutter.Log;
import io.flutter.embedding.engine.dart.DartExecutor;
import io.flutter.plugin.common.MethodCall;
import io.flutter.plugin.common.MethodChannel;
import io.flutter.plugin.common.StandardMethodCodec;
import java.util.ArrayList;

/**
 * {@link SpellCheckChannel} is a platform channel that is used by the framework to initiate spell
 * check in the embedding and for the embedding to send back the results.
 *
 * <p>When there is new text to be spell checked, the framework will send to the embedding the
 * message {@code SpellCheck.initiateSpellCheck} with the {@code String} locale to spell check with
 * and the {@code String} of text to spell check as arguments. In response, the {@link
 * io.flutter.plugin.editing.SpellCheckPlugin} will make a call to Android's spell check service to
 * fetch spell check results for the specified text.
 *
 * <p>Once the spell check results are received by the {@link
 * io.flutter.plugin.editing.SpellCheckPlugin}, it will send back to the framework the {@code
 * ArrayList<HashMap<String,Object>>} of spell check results (see {@link
 * io.flutter.plugin.editing.SpellCheckPlugin#onGetSentenceSuggestions} for details). The {@link
 * io.flutter.plugin.editing.SpellCheckPlugin} only handles one request to fetch spell check results
 * at a time; see {@link io.flutter.plugin.editing.SpellCheckPlugin#initiateSpellCheck} for details.
 *
 * <p>{@link io.flutter.plugin.editing.SpellCheckPlugin} implements {@link SpellCheckMethodHandler}
 * to initiate spell check. Implement {@link SpellCheckMethodHandler} to respond to spell check
 * requests.
 */
public class SpellCheckChannel {
  private static final String TAG = "SpellCheckChannel";

  public final MethodChannel channel;
  private SpellCheckMethodHandler spellCheckMethodHandler;

  @NonNull
  public final MethodChannel.MethodCallHandler parsingMethodHandler =
      new MethodChannel.MethodCallHandler() {
        @Override
        public void onMethodCall(@NonNull MethodCall call, @NonNull MethodChannel.Result result) {
          if (spellCheckMethodHandler == null) {
            Log.v(
                TAG,
                "No SpellCheckeMethodHandler registered, call not forwarded to spell check API.");
            return;
          }
          String method = call.method;
          Object args = call.arguments;
          Log.v(TAG, "Received '" + method + "' message.");
          switch (method) {
            case "SpellCheck.initiateSpellCheck":
              try {
                final ArrayList<String> argumentList = (ArrayList<String>) args;
                String locale = argumentList.get(0);
                String text = argumentList.get(1);
                spellCheckMethodHandler.initiateSpellCheck(locale, text, result);
              } catch (IllegalStateException exception) {
                result.error("error", exception.getMessage(), null);
              }
              break;
            default:
              result.notImplemented();
              break;
          }
        }
      };

  public SpellCheckChannel(@NonNull DartExecutor dartExecutor) {
    channel = new MethodChannel(dartExecutor, "flutter/spellcheck", StandardMethodCodec.INSTANCE);
    channel.setMethodCallHandler(parsingMethodHandler);
  }

  /**
   * Sets the {@link SpellCheckMethodHandler} which receives all requests to spell check the
   * specified text sent through this channel.
   */
  public void setSpellCheckMethodHandler(
      @Nullable SpellCheckMethodHandler spellCheckMethodHandler) {
    this.spellCheckMethodHandler = spellCheckMethodHandler;
  }

  public interface SpellCheckMethodHandler {
    /**
     * Requests that spell check is initiated for the specified text, which will respond to the
     * {@code result} with either success if spell check results are received or error if the
     * request is skipped.
     */
    void initiateSpellCheck(
        @NonNull String locale, @NonNull String text, @NonNull MethodChannel.Result result);
  }
}
