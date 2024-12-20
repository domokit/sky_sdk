package io.flutter.plugin.editing;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.anyInt;
import static org.mockito.ArgumentMatchers.isNull;
import static org.mockito.Mockito.anyString;
import static org.mockito.Mockito.eq;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.spy;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import android.content.ClipDescription;
import android.content.ClipboardManager;
import android.content.ContentResolver;
import android.content.Context;
import android.content.res.AssetManager;
import android.net.Uri;
import android.os.Bundle;
import android.text.InputType;
import android.text.Selection;
import android.text.SpannableStringBuilder;
import android.view.KeyEvent;
import android.view.View;
import android.view.inputmethod.CursorAnchorInfo;
import android.view.inputmethod.EditorInfo;
import android.view.inputmethod.ExtractedText;
import android.view.inputmethod.ExtractedTextRequest;
import android.view.inputmethod.InputConnection;
import android.view.inputmethod.InputContentInfo;
import android.view.inputmethod.InputMethodManager;
import androidx.core.view.inputmethod.InputConnectionCompat;
import androidx.test.core.app.ApplicationProvider;
import androidx.test.ext.junit.runners.AndroidJUnit4;
import com.ibm.icu.lang.UCharacter;
import com.ibm.icu.lang.UProperty;
import io.flutter.embedding.android.KeyboardManager;
import io.flutter.embedding.engine.FlutterJNI;
import io.flutter.embedding.engine.dart.DartExecutor;
import io.flutter.embedding.engine.systemchannels.ScribeChannel;
import io.flutter.embedding.engine.systemchannels.TextInputChannel;
import io.flutter.plugin.common.JSONMethodCodec;
import io.flutter.plugin.common.MethodCall;
import io.flutter.util.FakeKeyEvent;
import java.io.ByteArrayInputStream;
import java.nio.ByteBuffer;
import java.nio.charset.Charset;
import org.json.JSONArray;
import org.json.JSONException;
import org.junit.Before;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.mockito.ArgumentCaptor;
import org.mockito.Mock;
import org.mockito.MockitoAnnotations;
import org.robolectric.Shadows;
import org.robolectric.annotation.Config;
import org.robolectric.annotation.Implementation;
import org.robolectric.annotation.Implements;
import org.robolectric.shadow.api.Shadow;
import org.robolectric.shadows.ShadowContentResolver;
import org.robolectric.shadows.ShadowInputMethodManager;

@Config(
    manifest = Config.NONE,
    shadows = {InputConnectionAdaptorTest.TestImm.class})
@RunWith(AndroidJUnit4.class)
public class InputConnectionAdaptorTest {
  private final Context ctx = ApplicationProvider.getApplicationContext();
  private ContentResolver contentResolver;
  private ShadowContentResolver shadowContentResolver;

  @Mock KeyboardManager mockKeyboardManager;
  // Verifies the method and arguments for a captured method call.
  private void verifyMethodCall(ByteBuffer buffer, String methodName, String[] expectedArgs)
      throws JSONException {
    buffer.rewind();
    MethodCall methodCall = JSONMethodCodec.INSTANCE.decodeMethodCall(buffer);
    assertEquals(methodName, methodCall.method);
    if (expectedArgs != null) {
      JSONArray args = methodCall.arguments();
      assertEquals(expectedArgs.length, args.length());
      for (int i = 0; i < args.length(); i++) {
        assertEquals(expectedArgs[i], args.get(i).toString());
      }
    }
  }

  @Before
  public void setUp() {
    MockitoAnnotations.openMocks(this);
    contentResolver = ctx.getContentResolver();
    shadowContentResolver = Shadows.shadowOf(contentResolver);
  }

  @Test
  public void inputConnectionAdaptor_ReceivesEnter() throws NullPointerException {
    View testView = new View(ctx);
    FlutterJNI mockFlutterJni = mock(FlutterJNI.class);
    DartExecutor dartExecutor = spy(new DartExecutor(mockFlutterJni, mock(AssetManager.class)));
    int inputTargetId = 0;
    TextInputChannel textInputChannel = new TextInputChannel(dartExecutor);
    ScribeChannel scribeChannel = new ScribeChannel(dartExecutor);
    ListenableEditingState mEditable = new ListenableEditingState(null, testView);
    Selection.setSelection(mEditable, 0, 0);
    ListenableEditingState spyEditable = spy(mEditable);
    EditorInfo outAttrs = new EditorInfo();
    outAttrs.inputType = InputType.TYPE_CLASS_TEXT | InputType.TYPE_TEXT_FLAG_MULTI_LINE;

    InputConnectionAdaptor inputConnectionAdaptor =
        new InputConnectionAdaptor(
            testView,
            inputTargetId,
            textInputChannel,
            scribeChannel,
            mockKeyboardManager,
            spyEditable,
            outAttrs);

    // Send an enter key and make sure the Editable received it.
    FakeKeyEvent keyEvent = new FakeKeyEvent(KeyEvent.ACTION_DOWN, KeyEvent.KEYCODE_ENTER, '\n');
    inputConnectionAdaptor.handleKeyEvent(keyEvent);
    verify(spyEditable, times(1)).insert(eq(0), anyString());
  }

  @Test
  public void testPerformContextMenuAction_selectAll() {
    int selStart = 5;
    ListenableEditingState editable = sampleEditable(selStart, selStart);
    InputConnectionAdaptor adaptor = sampleInputConnectionAdaptor(editable);

    boolean didConsume = adaptor.performContextMenuAction(android.R.id.selectAll);

    assertTrue(didConsume);
    assertEquals(0, Selection.getSelectionStart(editable));
    assertEquals(editable.length(), Selection.getSelectionEnd(editable));
  }

  @SuppressWarnings("deprecation")
  // ClipboardManager.hasText is deprecated.
  @Test
  public void testPerformContextMenuAction_cut() {
    ClipboardManager clipboardManager = ctx.getSystemService(ClipboardManager.class);
    int selStart = 6;
    int selEnd = 11;
    ListenableEditingState editable = sampleEditable(selStart, selEnd);
    CharSequence textToBeCut = editable.subSequence(selStart, selEnd);
    InputConnectionAdaptor adaptor = sampleInputConnectionAdaptor(editable);

    boolean didConsume = adaptor.performContextMenuAction(android.R.id.cut);

    assertTrue(didConsume);
    assertTrue(clipboardManager.hasText());
    assertEquals(textToBeCut, clipboardManager.getPrimaryClip().getItemAt(0).getText());
    assertFalse(editable.toString().contains(textToBeCut));
  }

  @SuppressWarnings("deprecation")
  // ClipboardManager.hasText is deprecated.
  @Test
  public void testPerformContextMenuAction_copy() {
    ClipboardManager clipboardManager = ctx.getSystemService(ClipboardManager.class);
    int selStart = 6;
    int selEnd = 11;
    ListenableEditingState editable = sampleEditable(selStart, selEnd);
    InputConnectionAdaptor adaptor = sampleInputConnectionAdaptor(editable);

    assertFalse(clipboardManager.hasText());

    boolean didConsume = adaptor.performContextMenuAction(android.R.id.copy);

    assertTrue(didConsume);
    assertTrue(clipboardManager.hasText());
    assertEquals(
        editable.subSequence(selStart, selEnd),
        clipboardManager.getPrimaryClip().getItemAt(0).getText());
  }

  @SuppressWarnings("deprecation")
  // ClipboardManager.setText is deprecated.
  @Test
  public void testPerformContextMenuAction_paste() {
    ClipboardManager clipboardManager = ctx.getSystemService(ClipboardManager.class);
    String textToBePasted = "deadbeef";
    clipboardManager.setText(textToBePasted);
    ListenableEditingState editable = sampleEditable(0, 0);
    InputConnectionAdaptor adaptor = sampleInputConnectionAdaptor(editable);

    boolean didConsume = adaptor.performContextMenuAction(android.R.id.paste);

    assertTrue(didConsume);
    assertTrue(editable.toString().startsWith(textToBePasted));
  }

  @SuppressWarnings("deprecation")
  // DartExecutor.send is deprecated.
  @Test
  public void testCommitContent() throws JSONException {
    View testView = new View(ctx);
    int client = 0;
    FlutterJNI mockFlutterJNI = mock(FlutterJNI.class);
    DartExecutor dartExecutor = spy(new DartExecutor(mockFlutterJNI, mock(AssetManager.class)));
    TextInputChannel textInputChannel = new TextInputChannel(dartExecutor);
    ScribeChannel scribeChannel = new ScribeChannel(dartExecutor);
    ListenableEditingState editable = sampleEditable(0, 0);
    InputConnectionAdaptor adaptor =
        new InputConnectionAdaptor(
            testView,
            client,
            textInputChannel,
            scribeChannel,
            mockKeyboardManager,
            editable,
            null,
            mockFlutterJNI);

    String uri = "content://mock/uri/test/commitContent";
    Charset charset = Charset.forName("UTF-8");
    String fakeImageData = "fake image data";
    byte[] fakeImageDataBytes = fakeImageData.getBytes(charset);
    shadowContentResolver.registerInputStream(
        Uri.parse(uri), new ByteArrayInputStream(fakeImageDataBytes));

    boolean commitContentSuccess =
        adaptor.commitContent(
            new InputContentInfo(
                Uri.parse(uri),
                new ClipDescription("commitContent test", new String[] {"image/png"})),
            InputConnectionCompat.INPUT_CONTENT_GRANT_READ_URI_PERMISSION,
            null);
    assertTrue(commitContentSuccess);

    ArgumentCaptor<String> channelCaptor = ArgumentCaptor.forClass(String.class);
    ArgumentCaptor<ByteBuffer> bufferCaptor = ArgumentCaptor.forClass(ByteBuffer.class);
    verify(dartExecutor, times(1)).send(channelCaptor.capture(), bufferCaptor.capture(), isNull());
    assertEquals("flutter/textinput", channelCaptor.getValue());

    String fakeImageDataIntString = "";
    for (int i = 0; i < fakeImageDataBytes.length; i++) {
      int byteAsInt = fakeImageDataBytes[i];
      fakeImageDataIntString += byteAsInt;
      if (i < (fakeImageDataBytes.length - 1)) {
        fakeImageDataIntString += ",";
      }
    }
    verifyMethodCall(
        bufferCaptor.getValue(),
        "TextInputClient.performAction",
        new String[] {
          "0",
          "TextInputAction.commitContent",
          "{\"data\":["
              + fakeImageDataIntString
              + "],\"mimeType\":\"image\\/png\",\"uri\":\"content:\\/\\/mock\\/uri\\/test\\/commitContent\"}"
        });
  }

  @SuppressWarnings("deprecation")
  // DartExecutor.send is deprecated.
  @Test
  public void testPerformPrivateCommand_dataIsNull() throws JSONException {
    View testView = new View(ctx);
    int client = 0;
    FlutterJNI mockFlutterJNI = mock(FlutterJNI.class);
    DartExecutor dartExecutor = spy(new DartExecutor(mockFlutterJNI, mock(AssetManager.class)));
    TextInputChannel textInputChannel = new TextInputChannel(dartExecutor);
    ScribeChannel scribeChannel = new ScribeChannel(dartExecutor);
    ListenableEditingState editable = sampleEditable(0, 0);
    InputConnectionAdaptor adaptor =
        new InputConnectionAdaptor(
            testView,
            client,
            textInputChannel,
            scribeChannel,
            mockKeyboardManager,
            editable,
            null,
            mockFlutterJNI);
    adaptor.performPrivateCommand("actionCommand", null);

    ArgumentCaptor<String> channelCaptor = ArgumentCaptor.forClass(String.class);
    ArgumentCaptor<ByteBuffer> bufferCaptor = ArgumentCaptor.forClass(ByteBuffer.class);
    verify(dartExecutor, times(1)).send(channelCaptor.capture(), bufferCaptor.capture(), isNull());
    assertEquals("flutter/textinput", channelCaptor.getValue());
    verifyMethodCall(
        bufferCaptor.getValue(),
        "TextInputClient.performPrivateCommand",
        new String[] {"0", "{\"action\":\"actionCommand\"}"});
  }

  @SuppressWarnings("deprecation")
  // DartExecutor.send is deprecated.
  @Test
  public void testPerformPrivateCommand_dataIsByteArray() throws JSONException {
    View testView = new View(ctx);
    int client = 0;
    FlutterJNI mockFlutterJNI = mock(FlutterJNI.class);
    DartExecutor dartExecutor = spy(new DartExecutor(mockFlutterJNI, mock(AssetManager.class)));
    TextInputChannel textInputChannel = new TextInputChannel(dartExecutor);
    ScribeChannel scribeChannel = new ScribeChannel(dartExecutor);
    ListenableEditingState editable = sampleEditable(0, 0);
    InputConnectionAdaptor adaptor =
        new InputConnectionAdaptor(
            testView,
            client,
            textInputChannel,
            scribeChannel,
            mockKeyboardManager,
            editable,
            null,
            mockFlutterJNI);

    Bundle bundle = new Bundle();
    byte[] buffer = new byte[] {'a', 'b', 'c', 'd'};
    bundle.putByteArray("keyboard_layout", buffer);
    adaptor.performPrivateCommand("actionCommand", bundle);

    ArgumentCaptor<String> channelCaptor = ArgumentCaptor.forClass(String.class);
    ArgumentCaptor<ByteBuffer> bufferCaptor = ArgumentCaptor.forClass(ByteBuffer.class);
    verify(dartExecutor, times(1)).send(channelCaptor.capture(), bufferCaptor.capture(), isNull());
    assertEquals("flutter/textinput", channelCaptor.getValue());
    verifyMethodCall(
        bufferCaptor.getValue(),
        "TextInputClient.performPrivateCommand",
        new String[] {
          "0", "{\"data\":{\"keyboard_layout\":[97,98,99,100]},\"action\":\"actionCommand\"}"
        });
  }

  @SuppressWarnings("deprecation")
  // DartExecutor.send is deprecated.
  @Test
  public void testPerformPrivateCommand_dataIsByte() throws JSONException {
    View testView = new View(ctx);
    int client = 0;
    FlutterJNI mockFlutterJNI = mock(FlutterJNI.class);
    DartExecutor dartExecutor = spy(new DartExecutor(mockFlutterJNI, mock(AssetManager.class)));
    TextInputChannel textInputChannel = new TextInputChannel(dartExecutor);
    ScribeChannel scribeChannel = new ScribeChannel(dartExecutor);
    ListenableEditingState editable = sampleEditable(0, 0);
    InputConnectionAdaptor adaptor =
        new InputConnectionAdaptor(
            testView,
            client,
            textInputChannel,
            scribeChannel,
            mockKeyboardManager,
            editable,
            null,
            mockFlutterJNI);

    Bundle bundle = new Bundle();
    byte b = 3;
    bundle.putByte("keyboard_layout", b);
    adaptor.performPrivateCommand("actionCommand", bundle);

    ArgumentCaptor<String> channelCaptor = ArgumentCaptor.forClass(String.class);
    ArgumentCaptor<ByteBuffer> bufferCaptor = ArgumentCaptor.forClass(ByteBuffer.class);
    verify(dartExecutor, times(1)).send(channelCaptor.capture(), bufferCaptor.capture(), isNull());
    assertEquals("flutter/textinput", channelCaptor.getValue());
    verifyMethodCall(
        bufferCaptor.getValue(),
        "TextInputClient.performPrivateCommand",
        new String[] {"0", "{\"data\":{\"keyboard_layout\":3},\"action\":\"actionCommand\"}"});
  }

  @SuppressWarnings("deprecation")
  // DartExecutor.send is deprecated.
  @Test
  public void testPerformPrivateCommand_dataIsCharArray() throws JSONException {
    View testView = new View(ctx);
    int client = 0;
    FlutterJNI mockFlutterJNI = mock(FlutterJNI.class);
    DartExecutor dartExecutor = spy(new DartExecutor(mockFlutterJNI, mock(AssetManager.class)));
    TextInputChannel textInputChannel = new TextInputChannel(dartExecutor);
    ScribeChannel scribeChannel = new ScribeChannel(dartExecutor);
    ListenableEditingState editable = sampleEditable(0, 0);
    InputConnectionAdaptor adaptor =
        new InputConnectionAdaptor(
            testView,
            client,
            textInputChannel,
            scribeChannel,
            mockKeyboardManager,
            editable,
            null,
            mockFlutterJNI);

    Bundle bundle = new Bundle();
    char[] buffer = new char[] {'a', 'b', 'c', 'd'};
    bundle.putCharArray("keyboard_layout", buffer);
    adaptor.performPrivateCommand("actionCommand", bundle);

    ArgumentCaptor<String> channelCaptor = ArgumentCaptor.forClass(String.class);
    ArgumentCaptor<ByteBuffer> bufferCaptor = ArgumentCaptor.forClass(ByteBuffer.class);
    verify(dartExecutor, times(1)).send(channelCaptor.capture(), bufferCaptor.capture(), isNull());
    assertEquals("flutter/textinput", channelCaptor.getValue());
    verifyMethodCall(
        bufferCaptor.getValue(),
        "TextInputClient.performPrivateCommand",
        new String[] {
          "0",
          "{\"data\":{\"keyboard_layout\":[\"a\",\"b\",\"c\",\"d\"]},\"action\":\"actionCommand\"}"
        });
  }

  @SuppressWarnings("deprecation")
  // DartExecutor.send is deprecated.
  @Test
  public void testPerformPrivateCommand_dataIsChar() throws JSONException {
    View testView = new View(ctx);
    int client = 0;
    FlutterJNI mockFlutterJNI = mock(FlutterJNI.class);
    DartExecutor dartExecutor = spy(new DartExecutor(mockFlutterJNI, mock(AssetManager.class)));
    TextInputChannel textInputChannel = new TextInputChannel(dartExecutor);
    ScribeChannel scribeChannel = new ScribeChannel(dartExecutor);
    ListenableEditingState editable = sampleEditable(0, 0);
    InputConnectionAdaptor adaptor =
        new InputConnectionAdaptor(
            testView,
            client,
            textInputChannel,
            scribeChannel,
            mockKeyboardManager,
            editable,
            null,
            mockFlutterJNI);

    Bundle bundle = new Bundle();
    char b = 'a';
    bundle.putChar("keyboard_layout", b);
    adaptor.performPrivateCommand("actionCommand", bundle);

    ArgumentCaptor<String> channelCaptor = ArgumentCaptor.forClass(String.class);
    ArgumentCaptor<ByteBuffer> bufferCaptor = ArgumentCaptor.forClass(ByteBuffer.class);
    verify(dartExecutor, times(1)).send(channelCaptor.capture(), bufferCaptor.capture(), isNull());
    assertEquals("flutter/textinput", channelCaptor.getValue());
    verifyMethodCall(
        bufferCaptor.getValue(),
        "TextInputClient.performPrivateCommand",
        new String[] {"0", "{\"data\":{\"keyboard_layout\":\"a\"},\"action\":\"actionCommand\"}"});
  }

  @SuppressWarnings("deprecation")
  // DartExecutor.send is deprecated.
  @Test
  public void testPerformPrivateCommand_dataIsCharSequenceArray() throws JSONException {
    View testView = new View(ctx);
    int client = 0;
    FlutterJNI mockFlutterJNI = mock(FlutterJNI.class);
    DartExecutor dartExecutor = spy(new DartExecutor(mockFlutterJNI, mock(AssetManager.class)));
    TextInputChannel textInputChannel = new TextInputChannel(dartExecutor);
    ScribeChannel scribeChannel = new ScribeChannel(dartExecutor);
    ListenableEditingState editable = sampleEditable(0, 0);
    InputConnectionAdaptor adaptor =
        new InputConnectionAdaptor(
            testView,
            client,
            textInputChannel,
            scribeChannel,
            mockKeyboardManager,
            editable,
            null,
            mockFlutterJNI);

    Bundle bundle = new Bundle();
    CharSequence charSequence1 = new StringBuffer("abc");
    CharSequence charSequence2 = new StringBuffer("efg");
    CharSequence[] value = {charSequence1, charSequence2};
    bundle.putCharSequenceArray("keyboard_layout", value);
    adaptor.performPrivateCommand("actionCommand", bundle);

    ArgumentCaptor<String> channelCaptor = ArgumentCaptor.forClass(String.class);
    ArgumentCaptor<ByteBuffer> bufferCaptor = ArgumentCaptor.forClass(ByteBuffer.class);
    verify(dartExecutor, times(1)).send(channelCaptor.capture(), bufferCaptor.capture(), isNull());
    assertEquals("flutter/textinput", channelCaptor.getValue());
    verifyMethodCall(
        bufferCaptor.getValue(),
        "TextInputClient.performPrivateCommand",
        new String[] {
          "0", "{\"data\":{\"keyboard_layout\":[\"abc\",\"efg\"]},\"action\":\"actionCommand\"}"
        });
  }

  @SuppressWarnings("deprecation")
  // DartExecutor.send is deprecated.
  @Test
  public void testPerformPrivateCommand_dataIsCharSequence() throws JSONException {
    View testView = new View(ctx);
    int client = 0;
    FlutterJNI mockFlutterJNI = mock(FlutterJNI.class);
    DartExecutor dartExecutor = spy(new DartExecutor(mockFlutterJNI, mock(AssetManager.class)));
    TextInputChannel textInputChannel = new TextInputChannel(dartExecutor);
    ScribeChannel scribeChannel = new ScribeChannel(dartExecutor);
    ListenableEditingState editable = sampleEditable(0, 0);
    InputConnectionAdaptor adaptor =
        new InputConnectionAdaptor(
            testView,
            client,
            textInputChannel,
            scribeChannel,
            mockKeyboardManager,
            editable,
            null,
            mockFlutterJNI);

    Bundle bundle = new Bundle();
    CharSequence charSequence = new StringBuffer("abc");
    bundle.putCharSequence("keyboard_layout", charSequence);
    adaptor.performPrivateCommand("actionCommand", bundle);

    ArgumentCaptor<String> channelCaptor = ArgumentCaptor.forClass(String.class);
    ArgumentCaptor<ByteBuffer> bufferCaptor = ArgumentCaptor.forClass(ByteBuffer.class);
    verify(dartExecutor, times(1)).send(channelCaptor.capture(), bufferCaptor.capture(), isNull());
    assertEquals("flutter/textinput", channelCaptor.getValue());
    verifyMethodCall(
        bufferCaptor.getValue(),
        "TextInputClient.performPrivateCommand",
        new String[] {
          "0", "{\"data\":{\"keyboard_layout\":\"abc\"},\"action\":\"actionCommand\"}"
        });
  }

  @SuppressWarnings("deprecation")
  // DartExecutor.send is deprecated.
  @Test
  public void testPerformPrivateCommand_dataIsFloat() throws JSONException {
    View testView = new View(ctx);
    int client = 0;
    FlutterJNI mockFlutterJNI = mock(FlutterJNI.class);
    DartExecutor dartExecutor = spy(new DartExecutor(mockFlutterJNI, mock(AssetManager.class)));
    TextInputChannel textInputChannel = new TextInputChannel(dartExecutor);
    ScribeChannel scribeChannel = new ScribeChannel(dartExecutor);
    ListenableEditingState editable = sampleEditable(0, 0);
    InputConnectionAdaptor adaptor =
        new InputConnectionAdaptor(
            testView,
            client,
            textInputChannel,
            scribeChannel,
            mockKeyboardManager,
            editable,
            null,
            mockFlutterJNI);

    Bundle bundle = new Bundle();
    float value = 0.5f;
    bundle.putFloat("keyboard_layout", value);
    adaptor.performPrivateCommand("actionCommand", bundle);

    ArgumentCaptor<String> channelCaptor = ArgumentCaptor.forClass(String.class);
    ArgumentCaptor<ByteBuffer> bufferCaptor = ArgumentCaptor.forClass(ByteBuffer.class);
    verify(dartExecutor, times(1)).send(channelCaptor.capture(), bufferCaptor.capture(), isNull());
    assertEquals("flutter/textinput", channelCaptor.getValue());
    verifyMethodCall(
        bufferCaptor.getValue(),
        "TextInputClient.performPrivateCommand",
        new String[] {"0", "{\"data\":{\"keyboard_layout\":0.5},\"action\":\"actionCommand\"}"});
  }

  @SuppressWarnings("deprecation")
  // DartExecutor.send is deprecated.
  @Test
  public void testPerformPrivateCommand_dataIsFloatArray() throws JSONException {
    View testView = new View(ctx);
    int client = 0;
    FlutterJNI mockFlutterJNI = mock(FlutterJNI.class);
    DartExecutor dartExecutor = spy(new DartExecutor(mockFlutterJNI, mock(AssetManager.class)));
    TextInputChannel textInputChannel = new TextInputChannel(dartExecutor);
    ScribeChannel scribeChannel = new ScribeChannel(dartExecutor);
    ListenableEditingState editable = sampleEditable(0, 0);
    InputConnectionAdaptor adaptor =
        new InputConnectionAdaptor(
            testView,
            client,
            textInputChannel,
            scribeChannel,
            mockKeyboardManager,
            editable,
            null,
            mockFlutterJNI);

    Bundle bundle = new Bundle();
    float[] value = {0.5f, 0.6f};
    bundle.putFloatArray("keyboard_layout", value);
    adaptor.performPrivateCommand("actionCommand", bundle);

    ArgumentCaptor<String> channelCaptor = ArgumentCaptor.forClass(String.class);
    ArgumentCaptor<ByteBuffer> bufferCaptor = ArgumentCaptor.forClass(ByteBuffer.class);
    verify(dartExecutor, times(1)).send(channelCaptor.capture(), bufferCaptor.capture(), isNull());
    assertEquals("flutter/textinput", channelCaptor.getValue());
    verifyMethodCall(
        bufferCaptor.getValue(),
        "TextInputClient.performPrivateCommand",
        new String[] {
          "0", "{\"data\":{\"keyboard_layout\":[0.5,0.6]},\"action\":\"actionCommand\"}"
        });
  }

  @Test
  public void testSendKeyEvent_shiftKeyUpDoesNotCancelSelection() {
    // Regression test for https://github.com/flutter/flutter/issues/101569.
    int selStart = 5;
    int selEnd = 10;
    ListenableEditingState editable = sampleEditable(selStart, selEnd);
    InputConnectionAdaptor adaptor = sampleInputConnectionAdaptor(editable);

    KeyEvent shiftKeyUp = new KeyEvent(KeyEvent.ACTION_UP, KeyEvent.KEYCODE_SHIFT_LEFT);
    boolean didConsume = adaptor.handleKeyEvent(shiftKeyUp);

    assertFalse(didConsume);
    assertEquals(selStart, Selection.getSelectionStart(editable));
    assertEquals(selEnd, Selection.getSelectionEnd(editable));
  }

  @Test
  public void testSendKeyEvent_leftKeyMovesCaretLeft() {
    int selStart = 5;
    ListenableEditingState editable = sampleEditable(selStart, selStart);
    InputConnectionAdaptor adaptor = sampleInputConnectionAdaptor(editable);

    KeyEvent leftKeyDown = new KeyEvent(KeyEvent.ACTION_DOWN, KeyEvent.KEYCODE_DPAD_LEFT);
    boolean didConsume = adaptor.handleKeyEvent(leftKeyDown);

    assertTrue(didConsume);
    assertEquals(selStart - 1, Selection.getSelectionStart(editable));
    assertEquals(selStart - 1, Selection.getSelectionEnd(editable));
  }

  @Test
  public void testSendKeyEvent_leftKeyMovesCaretLeftComplexEmoji() {
    int selStart = 75;
    ListenableEditingState editable = sampleEditable(selStart, selStart, SAMPLE_EMOJI_TEXT);
    InputConnectionAdaptor adaptor = sampleInputConnectionAdaptor(editable);

    KeyEvent downKeyDown = new KeyEvent(KeyEvent.ACTION_DOWN, KeyEvent.KEYCODE_DPAD_LEFT);
    boolean didConsume;

    // Normal Character
    didConsume = adaptor.handleKeyEvent(downKeyDown);
    assertTrue(didConsume);
    assertEquals(Selection.getSelectionStart(editable), 74);

    // Non-Spacing Mark
    didConsume = adaptor.handleKeyEvent(downKeyDown);
    assertTrue(didConsume);
    assertEquals(Selection.getSelectionStart(editable), 73);
    didConsume = adaptor.handleKeyEvent(downKeyDown);
    assertTrue(didConsume);
    assertEquals(Selection.getSelectionStart(editable), 72);

    // Keycap
    didConsume = adaptor.handleKeyEvent(downKeyDown);
    assertTrue(didConsume);
    assertEquals(Selection.getSelectionStart(editable), 69);

    // Keycap with invalid base
    adaptor.setSelection(68, 68);
    didConsume = adaptor.handleKeyEvent(downKeyDown);
    assertTrue(didConsume);
    assertEquals(Selection.getSelectionStart(editable), 66);
    adaptor.setSelection(67, 67);
    didConsume = adaptor.handleKeyEvent(downKeyDown);
    assertTrue(didConsume);
    assertEquals(Selection.getSelectionStart(editable), 66);

    // Zero Width Joiner
    didConsume = adaptor.handleKeyEvent(downKeyDown);
    assertTrue(didConsume);
    assertEquals(Selection.getSelectionStart(editable), 55);

    // Zero Width Joiner with invalid base
    didConsume = adaptor.handleKeyEvent(downKeyDown);
    assertTrue(didConsume);
    assertEquals(Selection.getSelectionStart(editable), 53);
    didConsume = adaptor.handleKeyEvent(downKeyDown);
    assertTrue(didConsume);
    assertEquals(Selection.getSelectionStart(editable), 52);
    didConsume = adaptor.handleKeyEvent(downKeyDown);
    assertTrue(didConsume);
    assertEquals(Selection.getSelectionStart(editable), 51);

    // ----- Start Emoji Tag Sequence with invalid base testing ----
    // Delete base tag
    adaptor.setSelection(39, 39);
    didConsume = adaptor.handleKeyEvent(downKeyDown);
    assertTrue(didConsume);
    assertEquals(Selection.getSelectionStart(editable), 37);

    // Delete the sequence
    adaptor.setSelection(49, 49);
    for (int i = 0; i < 6; i++) {
      didConsume = adaptor.handleKeyEvent(downKeyDown);
      assertTrue(didConsume);
    }
    assertEquals(Selection.getSelectionStart(editable), 37);
    // ----- End Emoji Tag Sequence with invalid base testing ----

    // Emoji Tag Sequence
    didConsume = adaptor.handleKeyEvent(downKeyDown);
    assertTrue(didConsume);
    assertEquals(Selection.getSelectionStart(editable), 23);

    // Variation Selector with invalid base
    adaptor.setSelection(22, 22);
    didConsume = adaptor.handleKeyEvent(downKeyDown);
    assertTrue(didConsume);
    assertEquals(Selection.getSelectionStart(editable), 21);
    adaptor.setSelection(22, 22);
    didConsume = adaptor.handleKeyEvent(downKeyDown);
    assertTrue(didConsume);
    assertEquals(Selection.getSelectionStart(editable), 21);

    // Variation Selector
    didConsume = adaptor.handleKeyEvent(downKeyDown);
    assertTrue(didConsume);
    assertEquals(Selection.getSelectionStart(editable), 19);

    // Emoji Modifier
    didConsume = adaptor.handleKeyEvent(downKeyDown);
    assertTrue(didConsume);
    assertEquals(Selection.getSelectionStart(editable), 16);

    // Emoji Modifier with invalid base
    adaptor.setSelection(14, 14);
    didConsume = adaptor.handleKeyEvent(downKeyDown);
    assertTrue(didConsume);
    assertEquals(Selection.getSelectionStart(editable), 13);
    adaptor.setSelection(14, 14);
    didConsume = adaptor.handleKeyEvent(downKeyDown);
    assertTrue(didConsume);
    assertEquals(Selection.getSelectionStart(editable), 13);

    // Line Feed
    adaptor.setSelection(12, 12);
    didConsume = adaptor.handleKeyEvent(downKeyDown);
    assertTrue(didConsume);
    assertEquals(Selection.getSelectionStart(editable), 11);

    // Carriage Return
    adaptor.setSelection(12, 12);
    didConsume = adaptor.handleKeyEvent(downKeyDown);
    assertTrue(didConsume);
    assertEquals(Selection.getSelectionStart(editable), 11);

    // Carriage Return and Line Feed
    didConsume = adaptor.handleKeyEvent(downKeyDown);
    assertTrue(didConsume);
    assertEquals(Selection.getSelectionStart(editable), 9);

    // Regional Indicator Symbol odd
    didConsume = adaptor.handleKeyEvent(downKeyDown);
    assertTrue(didConsume);
    assertEquals(Selection.getSelectionStart(editable), 7);

    // Regional Indicator Symbol even
    didConsume = adaptor.handleKeyEvent(downKeyDown);
    assertTrue(didConsume);
    assertEquals(Selection.getSelectionStart(editable), 3);

    // Simple Emoji
    didConsume = adaptor.handleKeyEvent(downKeyDown);
    assertTrue(didConsume);
    assertEquals(Selection.getSelectionStart(editable), 1);

    // First CodePoint
    didConsume = adaptor.handleKeyEvent(downKeyDown);
    assertTrue(didConsume);
    assertEquals(Selection.getSelectionStart(editable), 0);
  }

  @Test
  public void testSendKeyEvent_leftKeyExtendsSelectionLeft() {
    int selStart = 5;
    int selEnd = 40;
    ListenableEditingState editable = sampleEditable(selStart, selEnd);
    InputConnectionAdaptor adaptor = sampleInputConnectionAdaptor(editable);

    KeyEvent leftKeyDown = new KeyEvent(KeyEvent.ACTION_DOWN, KeyEvent.KEYCODE_DPAD_LEFT);
    boolean didConsume = adaptor.handleKeyEvent(leftKeyDown);

    assertTrue(didConsume);
    assertEquals(selStart, Selection.getSelectionStart(editable));
    assertEquals(selEnd - 1, Selection.getSelectionEnd(editable));
  }

  @Test
  public void testSendKeyEvent_shiftLeftKeyStartsSelectionLeft() {
    int selStart = 5;
    ListenableEditingState editable = sampleEditable(selStart, selStart);
    InputConnectionAdaptor adaptor = sampleInputConnectionAdaptor(editable);

    KeyEvent shiftLeftKeyDown =
        new KeyEvent(
            0, 0, KeyEvent.ACTION_DOWN, KeyEvent.KEYCODE_DPAD_LEFT, 0, KeyEvent.META_SHIFT_ON);
    boolean didConsume = adaptor.handleKeyEvent(shiftLeftKeyDown);

    assertTrue(didConsume);
    assertEquals(selStart, Selection.getSelectionStart(editable));
    assertEquals(selStart - 1, Selection.getSelectionEnd(editable));
  }

  @Test
  public void testSendKeyEvent_rightKeyMovesCaretRight() {
    int selStart = 5;
    ListenableEditingState editable = sampleEditable(selStart, selStart);
    InputConnectionAdaptor adaptor = sampleInputConnectionAdaptor(editable);

    KeyEvent rightKeyDown = new KeyEvent(KeyEvent.ACTION_DOWN, KeyEvent.KEYCODE_DPAD_RIGHT);
    boolean didConsume = adaptor.handleKeyEvent(rightKeyDown);

    assertTrue(didConsume);
    assertEquals(selStart + 1, Selection.getSelectionStart(editable));
    assertEquals(selStart + 1, Selection.getSelectionEnd(editable));
  }

  @Test
  public void testSendKeyEvent_rightKeyMovesCaretRightComplexRegion() {
    int selStart = 0;
    // Seven region indicator characters. The first six should be considered as
    // three region indicators, and the final seventh character should be
    // considered to be on its own because it has no partner.
    String SAMPLE_REGION_TEXT = "🇷🇷🇷🇷🇷🇷🇷";
    ListenableEditingState editable = sampleEditable(selStart, selStart, SAMPLE_REGION_TEXT);
    InputConnectionAdaptor adaptor = sampleInputConnectionAdaptor(editable);

    KeyEvent downKeyDown = new KeyEvent(KeyEvent.ACTION_DOWN, KeyEvent.KEYCODE_DPAD_RIGHT);
    boolean didConsume;

    // The cursor moves over two region indicators at a time.
    didConsume = adaptor.handleKeyEvent(downKeyDown);
    assertTrue(didConsume);
    assertEquals(Selection.getSelectionStart(editable), 4);
    didConsume = adaptor.handleKeyEvent(downKeyDown);
    assertTrue(didConsume);
    assertEquals(Selection.getSelectionStart(editable), 8);
    didConsume = adaptor.handleKeyEvent(downKeyDown);
    assertTrue(didConsume);
    assertEquals(Selection.getSelectionStart(editable), 12);

    // When there is only one region indicator left with no pair, the cursor
    // moves over that single region indicator.
    didConsume = adaptor.handleKeyEvent(downKeyDown);
    assertTrue(didConsume);
    assertEquals(Selection.getSelectionStart(editable), 14);

    // If the cursor is placed in the middle of a region indicator pair, it
    // moves over only the second half of the pair.
    adaptor.setSelection(6, 6);
    didConsume = adaptor.handleKeyEvent(downKeyDown);
    assertTrue(didConsume);
    assertEquals(Selection.getSelectionStart(editable), 8);
  }

  @Test
  public void testSendKeyEvent_rightKeyMovesCaretRightComplexEmoji() {
    int selStart = 0;
    ListenableEditingState editable = sampleEditable(selStart, selStart, SAMPLE_EMOJI_TEXT);
    InputConnectionAdaptor adaptor = sampleInputConnectionAdaptor(editable);

    KeyEvent downKeyDown = new KeyEvent(KeyEvent.ACTION_DOWN, KeyEvent.KEYCODE_DPAD_RIGHT);
    boolean didConsume;

    // First CodePoint
    didConsume = adaptor.handleKeyEvent(downKeyDown);
    assertTrue(didConsume);
    assertEquals(Selection.getSelectionStart(editable), 1);

    // Simple Emoji
    didConsume = adaptor.handleKeyEvent(downKeyDown);
    assertTrue(didConsume);
    assertEquals(Selection.getSelectionStart(editable), 3);

    // Regional Indicator Symbol even
    didConsume = adaptor.handleKeyEvent(downKeyDown);
    assertTrue(didConsume);
    assertEquals(Selection.getSelectionStart(editable), 7);

    // Regional Indicator Symbol odd
    didConsume = adaptor.handleKeyEvent(downKeyDown);
    assertTrue(didConsume);
    assertEquals(Selection.getSelectionStart(editable), 9);

    // Carriage Return
    didConsume = adaptor.handleKeyEvent(downKeyDown);
    assertTrue(didConsume);
    assertEquals(Selection.getSelectionStart(editable), 10);

    // Line Feed and Carriage Return
    didConsume = adaptor.handleKeyEvent(downKeyDown);
    assertTrue(didConsume);
    assertEquals(Selection.getSelectionStart(editable), 12);

    // Line Feed
    didConsume = adaptor.handleKeyEvent(downKeyDown);
    assertTrue(didConsume);
    assertEquals(Selection.getSelectionStart(editable), 13);

    // Modified Emoji
    didConsume = adaptor.handleKeyEvent(downKeyDown);
    assertTrue(didConsume);
    assertEquals(Selection.getSelectionStart(editable), 16);

    // Emoji Modifier
    adaptor.setSelection(14, 14);
    didConsume = adaptor.handleKeyEvent(downKeyDown);
    assertTrue(didConsume);
    assertEquals(Selection.getSelectionStart(editable), 16);

    // Emoji Modifier with invalid base
    adaptor.setSelection(18, 18);
    didConsume = adaptor.handleKeyEvent(downKeyDown);
    assertTrue(didConsume);
    assertEquals(Selection.getSelectionStart(editable), 19);

    // Variation Selector
    didConsume = adaptor.handleKeyEvent(downKeyDown);
    assertTrue(didConsume);
    assertEquals(Selection.getSelectionStart(editable), 21);

    // Variation Selector with invalid base
    adaptor.setSelection(22, 22);
    didConsume = adaptor.handleKeyEvent(downKeyDown);
    assertTrue(didConsume);
    assertEquals(Selection.getSelectionStart(editable), 23);

    // Emoji Tag Sequence
    for (int i = 0; i < 7; i++) {
      didConsume = adaptor.handleKeyEvent(downKeyDown);
      assertTrue(didConsume);
      assertEquals(Selection.getSelectionStart(editable), 25 + 2 * i);
    }
    assertEquals(Selection.getSelectionStart(editable), 37);

    // ----- Start Emoji Tag Sequence with invalid base testing ----
    // Pass the sequence
    adaptor.setSelection(39, 39);
    for (int i = 0; i < 6; i++) {
      didConsume = adaptor.handleKeyEvent(downKeyDown);
      assertTrue(didConsume);
      assertEquals(Selection.getSelectionStart(editable), 41 + 2 * i);
    }
    assertEquals(Selection.getSelectionStart(editable), 51);
    // ----- End Emoji Tag Sequence with invalid base testing ----

    // Zero Width Joiner with invalid base
    didConsume = adaptor.handleKeyEvent(downKeyDown);
    assertTrue(didConsume);
    assertEquals(Selection.getSelectionStart(editable), 52);
    didConsume = adaptor.handleKeyEvent(downKeyDown);
    assertTrue(didConsume);
    assertEquals(Selection.getSelectionStart(editable), 53);
    didConsume = adaptor.handleKeyEvent(downKeyDown);
    assertTrue(didConsume);
    assertEquals(Selection.getSelectionStart(editable), 55);

    // Zero Width Joiner
    didConsume = adaptor.handleKeyEvent(downKeyDown);
    assertTrue(didConsume);
    assertEquals(Selection.getSelectionStart(editable), 66);

    // Keycap with invalid base
    adaptor.setSelection(67, 67);
    didConsume = adaptor.handleKeyEvent(downKeyDown);
    assertTrue(didConsume);
    assertEquals(Selection.getSelectionStart(editable), 68);
    didConsume = adaptor.handleKeyEvent(downKeyDown);
    assertTrue(didConsume);
    assertEquals(Selection.getSelectionStart(editable), 69);

    // Keycap
    didConsume = adaptor.handleKeyEvent(downKeyDown);
    assertTrue(didConsume);
    assertEquals(Selection.getSelectionStart(editable), 72);

    // Non-Spacing Mark
    didConsume = adaptor.handleKeyEvent(downKeyDown);
    assertTrue(didConsume);
    assertEquals(Selection.getSelectionStart(editable), 73);
    didConsume = adaptor.handleKeyEvent(downKeyDown);
    assertTrue(didConsume);
    assertEquals(Selection.getSelectionStart(editable), 74);

    // Normal Character
    didConsume = adaptor.handleKeyEvent(downKeyDown);
    assertTrue(didConsume);
    assertEquals(Selection.getSelectionStart(editable), 75);
  }

  @Test
  public void testSendKeyEvent_rightKeyExtendsSelectionRight() {
    int selStart = 5;
    int selEnd = 40;
    ListenableEditingState editable = sampleEditable(selStart, selEnd);
    InputConnectionAdaptor adaptor = sampleInputConnectionAdaptor(editable);

    KeyEvent rightKeyDown = new KeyEvent(KeyEvent.ACTION_DOWN, KeyEvent.KEYCODE_DPAD_RIGHT);
    boolean didConsume = adaptor.handleKeyEvent(rightKeyDown);

    assertTrue(didConsume);
    assertEquals(selStart, Selection.getSelectionStart(editable));
    assertEquals(selEnd + 1, Selection.getSelectionEnd(editable));
  }

  @Test
  public void testSendKeyEvent_shiftRightKeyStartsSelectionRight() {
    int selStart = 5;
    ListenableEditingState editable = sampleEditable(selStart, selStart);
    InputConnectionAdaptor adaptor = sampleInputConnectionAdaptor(editable);

    KeyEvent shiftRightKeyDown =
        new KeyEvent(
            0, 0, KeyEvent.ACTION_DOWN, KeyEvent.KEYCODE_DPAD_RIGHT, 0, KeyEvent.META_SHIFT_ON);
    boolean didConsume = adaptor.handleKeyEvent(shiftRightKeyDown);

    assertTrue(didConsume);
    assertEquals(selStart, Selection.getSelectionStart(editable));
    assertEquals(selStart + 1, Selection.getSelectionEnd(editable));
  }

  @Test
  public void testSendKeyEvent_upKeyMovesCaretUp() {
    int selStart = SAMPLE_TEXT.indexOf('\n') + 4;
    ListenableEditingState editable = sampleEditable(selStart, selStart);
    InputConnectionAdaptor adaptor = sampleInputConnectionAdaptor(editable);

    KeyEvent upKeyDown = new KeyEvent(KeyEvent.ACTION_DOWN, KeyEvent.KEYCODE_DPAD_UP);
    boolean didConsume = adaptor.handleKeyEvent(upKeyDown);

    assertTrue(didConsume);
    // Checks the caret moved left (to some previous character). Selection.moveUp() behaves
    // different in tests than on a real device, we can't verify the exact position.
    assertTrue(Selection.getSelectionStart(editable) < selStart);
  }

  @Test
  public void testSendKeyEvent_downKeyMovesCaretDown() {
    int selStart = 4;
    ListenableEditingState editable = sampleEditable(selStart, selStart);
    InputConnectionAdaptor adaptor = sampleInputConnectionAdaptor(editable);

    KeyEvent downKeyDown = new KeyEvent(KeyEvent.ACTION_DOWN, KeyEvent.KEYCODE_DPAD_DOWN);
    boolean didConsume = adaptor.handleKeyEvent(downKeyDown);

    assertTrue(didConsume);
    // Checks the caret moved right (to some following character). Selection.moveDown() behaves
    // different in tests than on a real device, we can't verify the exact position.
    assertTrue(Selection.getSelectionStart(editable) > selStart);
  }

  @Test
  public void testSendKeyEvent_MovementKeysAreNopWhenNoSelection() {
    // Regression test for https://github.com/flutter/flutter/issues/76283.
    ListenableEditingState editable = sampleEditable(-1, -1);
    InputConnectionAdaptor adaptor = sampleInputConnectionAdaptor(editable);

    KeyEvent keyEvent = new KeyEvent(KeyEvent.ACTION_DOWN, KeyEvent.KEYCODE_DPAD_DOWN);
    boolean didConsume = adaptor.handleKeyEvent(keyEvent);
    assertFalse(didConsume);
    assertEquals(Selection.getSelectionStart(editable), -1);
    assertEquals(Selection.getSelectionEnd(editable), -1);

    keyEvent = new KeyEvent(KeyEvent.ACTION_DOWN, KeyEvent.KEYCODE_DPAD_UP);
    didConsume = adaptor.handleKeyEvent(keyEvent);
    assertFalse(didConsume);
    assertEquals(Selection.getSelectionStart(editable), -1);
    assertEquals(Selection.getSelectionEnd(editable), -1);

    keyEvent = new KeyEvent(KeyEvent.ACTION_DOWN, KeyEvent.KEYCODE_DPAD_LEFT);
    didConsume = adaptor.handleKeyEvent(keyEvent);
    assertFalse(didConsume);
    assertEquals(Selection.getSelectionStart(editable), -1);
    assertEquals(Selection.getSelectionEnd(editable), -1);

    keyEvent = new KeyEvent(KeyEvent.ACTION_DOWN, KeyEvent.KEYCODE_DPAD_RIGHT);
    didConsume = adaptor.handleKeyEvent(keyEvent);
    assertFalse(didConsume);
    assertEquals(Selection.getSelectionStart(editable), -1);
    assertEquals(Selection.getSelectionEnd(editable), -1);
  }

  @Test
  public void testMethod_getExtractedText() {
    int selStart = 5;

    ListenableEditingState editable = sampleEditable(selStart, selStart);
    InputConnectionAdaptor adaptor = sampleInputConnectionAdaptor(editable);

    ExtractedText extractedText = adaptor.getExtractedText(null, 0);

    assertEquals(extractedText.text, SAMPLE_TEXT);
    assertEquals(extractedText.selectionStart, selStart);
    assertEquals(extractedText.selectionEnd, selStart);
  }

  @Test
  public void testExtractedText_monitoring() {
    ListenableEditingState editable = sampleEditable(5, 5);
    View testView = new View(ctx);
    InputConnectionAdaptor adaptor =
        new InputConnectionAdaptor(
            testView,
            1,
            mock(TextInputChannel.class),
            mock(ScribeChannel.class),
            mockKeyboardManager,
            editable,
            new EditorInfo());
    TestImm testImm = Shadow.extract(ctx.getSystemService(Context.INPUT_METHOD_SERVICE));

    testImm.resetStates();

    ExtractedTextRequest request = new ExtractedTextRequest();
    request.token = 123;

    ExtractedText extractedText = adaptor.getExtractedText(request, 0);
    assertEquals(5, extractedText.selectionStart);
    assertEquals(5, extractedText.selectionEnd);
    assertFalse(extractedText.text instanceof SpannableStringBuilder);

    // Move the cursor. Should not report extracted text.
    adaptor.setSelection(2, 3);
    assertNull(testImm.lastExtractedText);

    // Now request monitoring, and update the request text flag.
    request.flags = InputConnection.GET_TEXT_WITH_STYLES;
    extractedText = adaptor.getExtractedText(request, InputConnection.GET_EXTRACTED_TEXT_MONITOR);
    assertEquals(2, extractedText.selectionStart);
    assertEquals(3, extractedText.selectionEnd);
    assertTrue(extractedText.text instanceof SpannableStringBuilder);

    adaptor.setSelection(3, 5);
    assertEquals(3, testImm.lastExtractedText.selectionStart);
    assertEquals(5, testImm.lastExtractedText.selectionEnd);
    assertTrue(testImm.lastExtractedText.text instanceof SpannableStringBuilder);

    // Stop monitoring.
    testImm.resetStates();
    extractedText = adaptor.getExtractedText(request, 0);
    assertEquals(3, extractedText.selectionStart);
    assertEquals(5, extractedText.selectionEnd);
    assertTrue(extractedText.text instanceof SpannableStringBuilder);

    adaptor.setSelection(1, 3);
    assertNull(testImm.lastExtractedText);
  }

  @Test
  public void testCursorAnchorInfo() {
    ListenableEditingState editable = sampleEditable(5, 5);
    View testView = new View(ctx);
    InputConnectionAdaptor adaptor =
        new InputConnectionAdaptor(
            testView,
            1,
            mock(TextInputChannel.class),
            mock(ScribeChannel.class),
            mockKeyboardManager,
            editable,
            new EditorInfo());
    TestImm testImm = Shadow.extract(ctx.getSystemService(Context.INPUT_METHOD_SERVICE));

    testImm.resetStates();

    // Monitoring only. Does not send update immediately.
    adaptor.requestCursorUpdates(InputConnection.CURSOR_UPDATE_MONITOR);
    assertNull(testImm.lastCursorAnchorInfo);

    // Monitor selection changes.
    adaptor.setSelection(0, 1);
    CursorAnchorInfo cursorAnchorInfo = testImm.lastCursorAnchorInfo;
    assertEquals(0, cursorAnchorInfo.getSelectionStart());
    assertEquals(1, cursorAnchorInfo.getSelectionEnd());

    // Turn monitoring off.
    testImm.resetStates();
    assertNull(testImm.lastCursorAnchorInfo);
    adaptor.requestCursorUpdates(InputConnection.CURSOR_UPDATE_IMMEDIATE);
    cursorAnchorInfo = testImm.lastCursorAnchorInfo;
    assertEquals(0, cursorAnchorInfo.getSelectionStart());
    assertEquals(1, cursorAnchorInfo.getSelectionEnd());

    // No more updates.
    testImm.resetStates();
    adaptor.setSelection(1, 3);
    assertNull(testImm.lastCursorAnchorInfo);
  }

  @Test
  public void testSendKeyEvent_sendSoftKeyEvents() {
    ListenableEditingState editable = sampleEditable(5, 5);
    InputConnectionAdaptor adaptor = sampleInputConnectionAdaptor(editable, mockKeyboardManager);

    KeyEvent shiftKeyDown = new KeyEvent(KeyEvent.ACTION_DOWN, KeyEvent.KEYCODE_SHIFT_LEFT);

    boolean didConsume = adaptor.handleKeyEvent(shiftKeyDown);
    assertFalse(didConsume);
    verify(mockKeyboardManager, never()).handleEvent(shiftKeyDown);
  }

  @Test
  public void testSendKeyEvent_sendHardwareKeyEvents() {
    ListenableEditingState editable = sampleEditable(5, 5);
    when(mockKeyboardManager.handleEvent(any())).thenReturn(true);
    InputConnectionAdaptor adaptor = sampleInputConnectionAdaptor(editable, mockKeyboardManager);

    KeyEvent shiftKeyDown = new KeyEvent(KeyEvent.ACTION_DOWN, KeyEvent.KEYCODE_SHIFT_LEFT);

    // Call sendKeyEvent instead of handleKeyEvent.
    boolean didConsume = adaptor.sendKeyEvent(shiftKeyDown);
    assertTrue(didConsume);
    verify(mockKeyboardManager, times(1)).handleEvent(shiftKeyDown);
  }

  @Test
  public void testSendKeyEvent_delKeyNotConsumed() {
    ListenableEditingState editable = sampleEditable(5, 5);
    InputConnectionAdaptor adaptor = sampleInputConnectionAdaptor(editable);

    KeyEvent downKeyDown = new KeyEvent(KeyEvent.ACTION_DOWN, KeyEvent.KEYCODE_DEL);

    for (int i = 0; i < 4; i++) {
      boolean didConsume = adaptor.handleKeyEvent(downKeyDown);
      assertFalse(didConsume);
    }
    assertEquals(5, Selection.getSelectionStart(editable));
  }

  @Test
  public void testDoesNotConsumeBackButton() {
    ListenableEditingState editable = sampleEditable(0, 0);
    InputConnectionAdaptor adaptor = sampleInputConnectionAdaptor(editable);

    FakeKeyEvent keyEvent = new FakeKeyEvent(KeyEvent.ACTION_DOWN, KeyEvent.KEYCODE_BACK, '\b');
    boolean didConsume = adaptor.handleKeyEvent(keyEvent);

    assertFalse(didConsume);
  }

  @Test
  public void testCleanUpBatchEndsOnCloseConnection() {
    final ListenableEditingState editable = sampleEditable(0, 0);
    InputConnectionAdaptor adaptor = spy(sampleInputConnectionAdaptor(editable));
    for (int i = 0; i < 5; i++) {
      adaptor.beginBatchEdit();
    }
    adaptor.endBatchEdit();
    verify(adaptor, times(1)).endBatchEdit();
    adaptor.closeConnection();
    verify(adaptor, times(4)).endBatchEdit();
  }

  private static final String SAMPLE_TEXT =
      "Lorem ipsum dolor sit amet," + "\nconsectetur adipiscing elit.";

  private static final String SAMPLE_EMOJI_TEXT =
      "a" // First CodePoint
          + "😂" // Simple Emoji
          + "🇮🇷" // Regional Indicator Symbol even
          + "🇷" // Regional Indicator Symbol odd
          + "\r\n" // Carriage Return and Line Feed
          + "\r\n"
          + "✋🏿" // Emoji Modifier
          + "✋🏿"
          + "⚠️" // Variant Selector
          + "⚠️"
          + "🏴󠁧󠁢󠁥󠁮󠁧󠁿" // Emoji Tag Sequence
          + "🏴󠁧󠁢󠁥󠁮󠁧󠁿"
          + "a‍👨" // Zero Width Joiner
          + "👨‍👩‍👧‍👦"
          + "5️⃣" // Keycap
          + "5️⃣"
          + "عَ" // Non-Spacing Mark
          + "a"; // Normal Character

  private static final String SAMPLE_RTL_TEXT = "متن ساختگی" + "\nبرای تستfor test😊";

  private static ListenableEditingState sampleEditable(int selStart, int selEnd) {
    ListenableEditingState sample =
        new ListenableEditingState(null, new View(ApplicationProvider.getApplicationContext()));
    sample.replace(0, 0, SAMPLE_TEXT);
    Selection.setSelection(sample, selStart, selEnd);
    return sample;
  }

  private static ListenableEditingState sampleEditable(int selStart, int selEnd, String text) {
    ListenableEditingState sample =
        new ListenableEditingState(null, new View(ApplicationProvider.getApplicationContext()));
    sample.replace(0, 0, text);
    Selection.setSelection(sample, selStart, selEnd);
    return sample;
  }

  private static InputConnectionAdaptor sampleInputConnectionAdaptor(
      ListenableEditingState editable) {
    return sampleInputConnectionAdaptor(editable, mock(KeyboardManager.class));
  }

  private static InputConnectionAdaptor sampleInputConnectionAdaptor(
      ListenableEditingState editable, KeyboardManager mockKeyboardManager) {
    View testView = new View(ApplicationProvider.getApplicationContext());
    int client = 0;
    TextInputChannel textInputChannel = mock(TextInputChannel.class);
    ScribeChannel scribeChannel = mock(ScribeChannel.class);
    FlutterJNI mockFlutterJNI = mock(FlutterJNI.class);
    when(mockFlutterJNI.isCodePointEmoji(anyInt()))
        .thenAnswer((invocation) -> Emoji.isEmoji((int) invocation.getArguments()[0]));
    when(mockFlutterJNI.isCodePointEmojiModifier(anyInt()))
        .thenAnswer((invocation) -> Emoji.isEmojiModifier((int) invocation.getArguments()[0]));
    when(mockFlutterJNI.isCodePointEmojiModifierBase(anyInt()))
        .thenAnswer((invocation) -> Emoji.isEmojiModifierBase((int) invocation.getArguments()[0]));
    when(mockFlutterJNI.isCodePointVariantSelector(anyInt()))
        .thenAnswer((invocation) -> Emoji.isVariationSelector((int) invocation.getArguments()[0]));
    when(mockFlutterJNI.isCodePointRegionalIndicator(anyInt()))
        .thenAnswer(
            (invocation) -> Emoji.isRegionalIndicatorSymbol((int) invocation.getArguments()[0]));
    return new InputConnectionAdaptor(
        testView,
        client,
        textInputChannel,
        scribeChannel,
        mockKeyboardManager,
        editable,
        null,
        mockFlutterJNI);
  }

  private static class Emoji {
    public static boolean isEmoji(int codePoint) {
      return UCharacter.hasBinaryProperty(codePoint, UProperty.EMOJI);
    }

    public static boolean isEmojiModifier(int codePoint) {
      return UCharacter.hasBinaryProperty(codePoint, UProperty.EMOJI_MODIFIER);
    }

    public static boolean isEmojiModifierBase(int codePoint) {
      return UCharacter.hasBinaryProperty(codePoint, UProperty.EMOJI_MODIFIER_BASE);
    }

    public static boolean isRegionalIndicatorSymbol(int codePoint) {
      return UCharacter.hasBinaryProperty(codePoint, UProperty.REGIONAL_INDICATOR);
    }

    public static boolean isVariationSelector(int codePoint) {
      return UCharacter.hasBinaryProperty(codePoint, UProperty.VARIATION_SELECTOR);
    }
  }

  private class TestTextInputChannel extends TextInputChannel {
    public TestTextInputChannel(DartExecutor dartExecutor) {
      super(dartExecutor);
    }

    public int inputClientId;
    public String text;
    public int selectionStart;
    public int selectionEnd;
    public int composingStart;
    public int composingEnd;
    public int updateEditingStateInvocations = 0;

    @Override
    public void updateEditingState(
        int inputClientId,
        String text,
        int selectionStart,
        int selectionEnd,
        int composingStart,
        int composingEnd) {
      this.inputClientId = inputClientId;
      this.text = text;
      this.selectionStart = selectionStart;
      this.selectionEnd = selectionEnd;
      this.composingStart = composingStart;
      this.composingEnd = composingEnd;
      updateEditingStateInvocations++;
    }
  }

  @Implements(InputMethodManager.class)
  public static class TestImm extends ShadowInputMethodManager {
    public static int empty = -999;
    CursorAnchorInfo lastCursorAnchorInfo;
    int lastExtractedTextToken = empty;
    ExtractedText lastExtractedText;

    int lastSelectionStart = empty;
    int lastSelectionEnd = empty;
    int lastCandidatesStart = empty;
    int lastCandidatesEnd = empty;

    public TestImm() {}

    @Implementation
    public void updateCursorAnchorInfo(View view, CursorAnchorInfo cursorAnchorInfo) {
      lastCursorAnchorInfo = cursorAnchorInfo;
    }

    @Implementation
    public void updateExtractedText(View view, int token, ExtractedText text) {
      lastExtractedTextToken = token;
      lastExtractedText = text;
    }

    @Implementation
    public void updateSelection(
        View view, int selStart, int selEnd, int candidatesStart, int candidatesEnd) {
      lastSelectionStart = selStart;
      lastSelectionEnd = selEnd;
      lastCandidatesStart = candidatesStart;
      lastCandidatesEnd = candidatesEnd;
    }

    public void resetStates() {
      lastExtractedText = null;
      lastExtractedTextToken = empty;

      lastSelectionStart = empty;
      lastSelectionEnd = empty;
      lastCandidatesStart = empty;
      lastCandidatesEnd = empty;

      lastCursorAnchorInfo = null;
    }
  }
}
