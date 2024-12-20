package io.flutter.plugin.editing;

import static org.junit.Assert.assertEquals;
import static org.mockito.Mockito.any;
import static org.mockito.Mockito.eq;
import static org.mockito.Mockito.isNull;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.spy;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import android.content.Context;
import android.view.textservice.SentenceSuggestionsInfo;
import android.view.textservice.SpellCheckerSession;
import android.view.textservice.SuggestionsInfo;
import android.view.textservice.TextInfo;
import android.view.textservice.TextServicesManager;
import androidx.test.ext.junit.runners.AndroidJUnit4;
import io.flutter.embedding.engine.dart.DartExecutor;
import io.flutter.embedding.engine.systemchannels.SpellCheckChannel;
import io.flutter.plugin.common.BinaryMessenger;
import io.flutter.plugin.common.MethodCall;
import io.flutter.plugin.common.MethodChannel;
import io.flutter.plugin.common.StandardMethodCodec;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Locale;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.mockito.ArgumentCaptor;

@RunWith(AndroidJUnit4.class)
public class SpellCheckPluginTest {

  private static void sendToBinaryMessageHandler(
      BinaryMessenger.BinaryMessageHandler binaryMessageHandler, String method, Object args) {
    MethodCall methodCall = new MethodCall(method, args);
    ByteBuffer encodedMethodCall = StandardMethodCodec.INSTANCE.encodeMethodCall(methodCall);
    binaryMessageHandler.onMessage(
        (ByteBuffer) encodedMethodCall.flip(), mock(BinaryMessenger.BinaryReply.class));
  }

  @SuppressWarnings("deprecation")
  // setMessageHandler is deprecated.
  @Test
  public void respondsToSpellCheckChannelMessage() {
    ArgumentCaptor<BinaryMessenger.BinaryMessageHandler> binaryMessageHandlerCaptor =
        ArgumentCaptor.forClass(BinaryMessenger.BinaryMessageHandler.class);
    DartExecutor mockBinaryMessenger = mock(DartExecutor.class);
    SpellCheckChannel.SpellCheckMethodHandler mockHandler =
        mock(SpellCheckChannel.SpellCheckMethodHandler.class);
    SpellCheckChannel spellCheckChannel = new SpellCheckChannel(mockBinaryMessenger);

    spellCheckChannel.setSpellCheckMethodHandler(mockHandler);

    verify(mockBinaryMessenger, times(1))
        .setMessageHandler(any(String.class), binaryMessageHandlerCaptor.capture());

    BinaryMessenger.BinaryMessageHandler binaryMessageHandler =
        binaryMessageHandlerCaptor.getValue();

    sendToBinaryMessageHandler(
        binaryMessageHandler,
        "SpellCheck.initiateSpellCheck",
        Arrays.asList("en-US", "Hello, wrold!"));

    verify(mockHandler)
        .initiateSpellCheck(eq("en-US"), eq("Hello, wrold!"), any(MethodChannel.Result.class));
  }

  @Test
  public void initiateSpellCheckPerformsSpellCheckWhenNoResultPending() {
    SpellCheckChannel fakeSpellCheckChannel = mock(SpellCheckChannel.class);
    TextServicesManager fakeTextServicesManager = mock(TextServicesManager.class);
    SpellCheckPlugin spellCheckPlugin =
        spy(new SpellCheckPlugin(fakeTextServicesManager, fakeSpellCheckChannel));
    MethodChannel.Result mockResult = mock(MethodChannel.Result.class);
    SpellCheckerSession fakeSpellCheckerSession = mock(SpellCheckerSession.class);

    when(fakeTextServicesManager.newSpellCheckerSession(
            null, new Locale("en", "US"), spellCheckPlugin, true))
        .thenReturn(fakeSpellCheckerSession);

    spellCheckPlugin.initiateSpellCheck("en-US", "Hello, wrold!", mockResult);

    verify(spellCheckPlugin).performSpellCheck("en-US", "Hello, wrold!");
  }

  @Test
  public void initiateSpellCheckThrowsErrorWhenResultPending() {
    SpellCheckChannel fakeSpellCheckChannel = mock(SpellCheckChannel.class);
    TextServicesManager fakeTextServicesManager = mock(TextServicesManager.class);
    SpellCheckPlugin spellCheckPlugin =
        spy(new SpellCheckPlugin(fakeTextServicesManager, fakeSpellCheckChannel));
    MethodChannel.Result mockPendingResult = mock(MethodChannel.Result.class);
    MethodChannel.Result mockResult = mock(MethodChannel.Result.class);
    spellCheckPlugin.pendingResult = mockPendingResult;

    spellCheckPlugin.initiateSpellCheck("en-US", "Hello, wrold!", mockResult);

    verify(mockResult).error("error", "Previous spell check request still pending.", null);
    verify(spellCheckPlugin, never()).performSpellCheck("en-US", "Hello, wrold!");
  }

  @Test
  public void destroyClosesSpellCheckerSessionAndClearsSpellCheckMethodHandler() {
    Context fakeContext = mock(Context.class);
    SpellCheckChannel fakeSpellCheckChannel = mock(SpellCheckChannel.class);
    TextServicesManager fakeTextServicesManager = mock(TextServicesManager.class);
    when(fakeContext.getSystemService(Context.TEXT_SERVICES_MANAGER_SERVICE))
        .thenReturn(fakeTextServicesManager);
    SpellCheckPlugin spellCheckPlugin =
        spy(new SpellCheckPlugin(fakeTextServicesManager, fakeSpellCheckChannel));
    SpellCheckerSession fakeSpellCheckerSession = mock(SpellCheckerSession.class);

    when(fakeTextServicesManager.newSpellCheckerSession(
            null, new Locale("en", "US"), spellCheckPlugin, true))
        .thenReturn(fakeSpellCheckerSession);

    spellCheckPlugin.performSpellCheck("en-US", "Hello, wrold!");
    spellCheckPlugin.destroy();

    verify(fakeSpellCheckChannel).setSpellCheckMethodHandler(isNull());
    verify(fakeSpellCheckerSession).close();
  }

  @Test
  public void performSpellCheckSendsRequestToAndroidSpellCheckService() {
    Context fakeContext = mock(Context.class);
    SpellCheckChannel fakeSpellCheckChannel = mock(SpellCheckChannel.class);
    TextServicesManager fakeTextServicesManager = mock(TextServicesManager.class);
    when(fakeContext.getSystemService(Context.TEXT_SERVICES_MANAGER_SERVICE))
        .thenReturn(fakeTextServicesManager);
    SpellCheckPlugin spellCheckPlugin =
        spy(new SpellCheckPlugin(fakeTextServicesManager, fakeSpellCheckChannel));
    SpellCheckerSession fakeSpellCheckerSession = mock(SpellCheckerSession.class);
    Locale english_US = new Locale("en", "US");

    when(fakeTextServicesManager.newSpellCheckerSession(null, english_US, spellCheckPlugin, true))
        .thenReturn(fakeSpellCheckerSession);

    int maxSuggestions = 5;
    ArgumentCaptor<TextInfo[]> textInfosCaptor = ArgumentCaptor.forClass(TextInfo[].class);
    ArgumentCaptor<Integer> maxSuggestionsCaptor = ArgumentCaptor.forClass(Integer.class);

    spellCheckPlugin.performSpellCheck("en-US", "Hello, wrold!");

    verify(fakeSpellCheckerSession)
        .getSentenceSuggestions(textInfosCaptor.capture(), maxSuggestionsCaptor.capture());
    assertEquals("Hello, wrold!", textInfosCaptor.getValue()[0].getText());
    assertEquals(Integer.valueOf(maxSuggestions), maxSuggestionsCaptor.getValue());
  }

  @Test
  public void performSpellCheckCreatesNewSpellCheckerSession() {
    Context fakeContext = mock(Context.class);
    SpellCheckChannel fakeSpellCheckChannel = mock(SpellCheckChannel.class);
    TextServicesManager fakeTextServicesManager = mock(TextServicesManager.class);
    when(fakeContext.getSystemService(Context.TEXT_SERVICES_MANAGER_SERVICE))
        .thenReturn(fakeTextServicesManager);
    SpellCheckPlugin spellCheckPlugin =
        spy(new SpellCheckPlugin(fakeTextServicesManager, fakeSpellCheckChannel));
    SpellCheckerSession fakeSpellCheckerSession = mock(SpellCheckerSession.class);
    Locale english_US = new Locale("en", "US");

    when(fakeTextServicesManager.newSpellCheckerSession(null, english_US, spellCheckPlugin, true))
        .thenReturn(fakeSpellCheckerSession);

    spellCheckPlugin.performSpellCheck("en-US", "Hello, worl!");
    spellCheckPlugin.performSpellCheck("en-US", "Hello, world!");

    verify(fakeTextServicesManager, times(1))
        .newSpellCheckerSession(null, english_US, spellCheckPlugin, true);
  }

  @Test
  public void onGetSentenceSuggestionsResultsWithSuccessAndNoResultsProperly() {
    TextServicesManager fakeTextServicesManager = mock(TextServicesManager.class);
    SpellCheckChannel fakeSpellCheckChannel = mock(SpellCheckChannel.class);
    SpellCheckPlugin spellCheckPlugin =
        spy(new SpellCheckPlugin(fakeTextServicesManager, fakeSpellCheckChannel));
    MethodChannel.Result mockResult = mock(MethodChannel.Result.class);
    spellCheckPlugin.pendingResult = mockResult;

    spellCheckPlugin.onGetSentenceSuggestions(new SentenceSuggestionsInfo[] {});

    verify(mockResult).success(new ArrayList<HashMap<String, Object>>());
  }

  @Test
  public void onGetSentenceSuggestionsResultsWithSuccessAndResultsProperly() {
    TextServicesManager fakeTextServicesManager = mock(TextServicesManager.class);
    SpellCheckChannel fakeSpellCheckChannel = mock(SpellCheckChannel.class);
    SpellCheckPlugin spellCheckPlugin =
        spy(new SpellCheckPlugin(fakeTextServicesManager, fakeSpellCheckChannel));
    MethodChannel.Result mockResult = mock(MethodChannel.Result.class);
    spellCheckPlugin.pendingResult = mockResult;

    spellCheckPlugin.onGetSentenceSuggestions(
        new SentenceSuggestionsInfo[] {
          new SentenceSuggestionsInfo(
              (new SuggestionsInfo[] {
                new SuggestionsInfo(
                    SuggestionsInfo.RESULT_ATTR_LOOKS_LIKE_TYPO,
                    new String[] {"world", "word", "old"})
              }),
              new int[] {7},
              new int[] {5})
        });

    ArrayList<HashMap<String, Object>> expectedResults = new ArrayList<HashMap<String, Object>>();
    HashMap<String, Object> expectedResult = new HashMap<String, Object>();

    expectedResult.put(SpellCheckPlugin.START_INDEX_KEY, 7);
    expectedResult.put(SpellCheckPlugin.END_INDEX_KEY, 12);
    expectedResult.put(
        SpellCheckPlugin.SUGGESTIONS_KEY,
        new ArrayList<String>(Arrays.asList("world", "word", "old")));
    expectedResults.add(expectedResult);

    verify(mockResult).success(expectedResults);
  }

  @Test
  public void onGetSentenceSuggestionsResultsWithSuccessAndNoResultsWhenSuggestionsAreInvalid() {
    TextServicesManager fakeTextServicesManager = mock(TextServicesManager.class);
    SpellCheckChannel fakeSpellCheckChannel = mock(SpellCheckChannel.class);
    SpellCheckPlugin spellCheckPlugin =
        spy(new SpellCheckPlugin(fakeTextServicesManager, fakeSpellCheckChannel));
    MethodChannel.Result mockResult = mock(MethodChannel.Result.class);
    spellCheckPlugin.pendingResult = mockResult;

    spellCheckPlugin.onGetSentenceSuggestions(
        new SentenceSuggestionsInfo[] {
          new SentenceSuggestionsInfo(
              (new SuggestionsInfo[] {
                new SuggestionsInfo(
                    SuggestionsInfo.RESULT_ATTR_LOOKS_LIKE_TYPO,
                    // This is the suggestion that may be provided by the Samsung spell checker:
                    new String[] {""})
              }),
              new int[] {7},
              new int[] {5})
        });

    verify(mockResult).success(new ArrayList<HashMap<String, Object>>());
  }

  @Test
  public void onGetSentenceSuggestionsResultsWithSuccessAndNoResultsWhenSuggestionsAreInvalid2() {
    TextServicesManager fakeTextServicesManager = mock(TextServicesManager.class);
    SpellCheckChannel fakeSpellCheckChannel = mock(SpellCheckChannel.class);
    SpellCheckPlugin spellCheckPlugin =
        spy(new SpellCheckPlugin(fakeTextServicesManager, fakeSpellCheckChannel));
    MethodChannel.Result mockResult = mock(MethodChannel.Result.class);
    spellCheckPlugin.pendingResult = mockResult;

    spellCheckPlugin.onGetSentenceSuggestions(
        new SentenceSuggestionsInfo[] {
          // This "suggestion" may be provided by the Samsung spell checker:
          null
        });

    verify(mockResult).success(new ArrayList<HashMap<String, Object>>());
  }
}
