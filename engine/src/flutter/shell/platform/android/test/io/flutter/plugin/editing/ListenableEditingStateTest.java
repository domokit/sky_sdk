package io.flutter.plugin.editing;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.mockito.Mockito.mock;

import android.content.Context;
import android.text.Editable;
import android.text.Selection;
import android.view.View;
import android.view.inputmethod.BaseInputConnection;
import android.view.inputmethod.EditorInfo;
import androidx.test.core.app.ApplicationProvider;
import androidx.test.ext.junit.runners.AndroidJUnit4;
import io.flutter.embedding.android.KeyboardManager;
import io.flutter.embedding.engine.systemchannels.ScribeChannel;
import io.flutter.embedding.engine.systemchannels.TextInputChannel;
import java.util.ArrayList;
import org.junit.Before;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.mockito.Mock;
import org.mockito.MockitoAnnotations;
import org.robolectric.annotation.Config;

@Config(manifest = Config.NONE)
@RunWith(AndroidJUnit4.class)
public class ListenableEditingStateTest {
  private final Context ctx = ApplicationProvider.getApplicationContext();
  @Mock KeyboardManager mockKeyboardManager;

  private BaseInputConnection getTestInputConnection(View view, Editable mEditable) {
    new View(ctx);
    return new BaseInputConnection(view, true) {
      @Override
      public Editable getEditable() {
        return mEditable;
      }
    };
  }

  @Before
  public void setUp() {
    MockitoAnnotations.openMocks(this);
  }

  @Test
  public void testConstructor() {
    // When provided valid composing range, should not fail
    new ListenableEditingState(
        new TextInputChannel.TextEditState("hello", 1, 4, 1, 4), new View(ctx));
  }

  // -------- Start: Test BatchEditing   -------
  @Test
  public void testBatchEditing() {
    final ListenableEditingState editingState = new ListenableEditingState(null, new View(ctx));
    final Listener listener = new Listener();
    final View testView = new View(ctx);
    final BaseInputConnection inputConnection = getTestInputConnection(testView, editingState);

    editingState.addEditingStateListener(listener);

    editingState.replace(0, editingState.length(), "update");
    assertTrue(listener.isCalled());
    assertTrue(listener.textChanged);
    assertFalse(listener.selectionChanged);
    assertFalse(listener.composingRegionChanged);

    assertEquals(-1, editingState.getSelectionStart());
    assertEquals(-1, editingState.getSelectionEnd());

    listener.reset();

    // Batch edit depth = 1.
    editingState.beginBatchEdit();
    editingState.replace(0, editingState.length(), "update1");
    assertFalse(listener.isCalled());
    // Batch edit depth = 2.
    editingState.beginBatchEdit();
    editingState.replace(0, editingState.length(), "update2");
    inputConnection.setComposingRegion(0, editingState.length());
    assertFalse(listener.isCalled());
    // Batch edit depth = 1.
    editingState.endBatchEdit();
    assertFalse(listener.isCalled());

    // Batch edit depth = 2.
    editingState.beginBatchEdit();
    assertFalse(listener.isCalled());
    inputConnection.setSelection(0, 0);
    assertFalse(listener.isCalled());
    // Batch edit depth = 1.
    editingState.endBatchEdit();
    assertFalse(listener.isCalled());

    // Remove composing region.
    inputConnection.finishComposingText();

    // Batch edit depth = 0. Last endBatchEdit.
    editingState.endBatchEdit();

    // Now notify the listener.
    assertTrue(listener.isCalled());
    assertTrue(listener.textChanged);
    assertFalse(listener.composingRegionChanged);
  }

  @Test
  public void testBatchingEditing_callEndBeforeBegin() {
    final ListenableEditingState editingState = new ListenableEditingState(null, new View(ctx));
    final Listener listener = new Listener();
    editingState.addEditingStateListener(listener);

    editingState.endBatchEdit();
    assertFalse(listener.isCalled());

    editingState.replace(0, editingState.length(), "text");
    assertTrue(listener.isCalled());
    assertTrue(listener.textChanged);

    listener.reset();
    // Does not disrupt the followup events.
    editingState.beginBatchEdit();
    editingState.replace(0, editingState.length(), "more text");
    assertFalse(listener.isCalled());
    editingState.endBatchEdit();
    assertTrue(listener.isCalled());
  }

  @Test
  public void testBatchingEditing_addListenerDuringBatchEdit() {
    final ListenableEditingState editingState = new ListenableEditingState(null, new View(ctx));
    final Listener listener = new Listener();

    editingState.beginBatchEdit();
    editingState.addEditingStateListener(listener);
    editingState.replace(0, editingState.length(), "update");
    editingState.endBatchEdit();

    assertTrue(listener.isCalled());
    assertTrue(listener.textChanged);
    assertTrue(listener.selectionChanged);
    assertTrue(listener.composingRegionChanged);

    listener.reset();

    // Verifies the listener is officially added.
    editingState.replace(0, editingState.length(), "more updates");
    assertTrue(listener.isCalled());
    assertTrue(listener.textChanged);
    editingState.removeEditingStateListener(listener);

    listener.reset();
    // Now remove before endBatchEdit();
    editingState.beginBatchEdit();
    editingState.addEditingStateListener(listener);
    editingState.replace(0, editingState.length(), "update");
    editingState.removeEditingStateListener(listener);
    editingState.endBatchEdit();

    assertFalse(listener.isCalled());
  }

  @Test
  public void testBatchingEditing_removeListenerDuringBatchEdit() {
    final ListenableEditingState editingState = new ListenableEditingState(null, new View(ctx));
    final Listener listener = new Listener();
    editingState.addEditingStateListener(listener);

    editingState.beginBatchEdit();
    editingState.replace(0, editingState.length(), "update");
    editingState.removeEditingStateListener(listener);
    editingState.endBatchEdit();

    assertFalse(listener.isCalled());
  }

  @Test
  public void testBatchingEditing_listenerCallsReplaceWhenBatchEditEnds() {
    final ListenableEditingState editingState = new ListenableEditingState(null, new View(ctx));

    final Listener listener =
        new Listener() {
          @Override
          public void didChangeEditingState(
              boolean textChanged, boolean selectionChanged, boolean composingRegionChanged) {
            super.didChangeEditingState(textChanged, selectionChanged, composingRegionChanged);
            editingState.replace(
                0, editingState.length(), "one does not simply replace the text in the listener");
          }
        };
    editingState.addEditingStateListener(listener);

    editingState.beginBatchEdit();
    editingState.replace(0, editingState.length(), "update");
    editingState.endBatchEdit();

    assertTrue(listener.isCalled());
    assertEquals(1, listener.timesCalled);
    assertEquals("one does not simply replace the text in the listener", editingState.toString());
  }
  // -------- End: Test BatchEditing   -------

  @Test
  public void testSetComposingRegion() {
    final ListenableEditingState editingState = new ListenableEditingState(null, new View(ctx));
    editingState.replace(0, editingState.length(), "text");

    // (-1, -1) clears the composing region.
    editingState.setComposingRange(-1, -1);
    assertEquals(-1, editingState.getComposingStart());
    assertEquals(-1, editingState.getComposingEnd());

    editingState.setComposingRange(-1, 5);
    assertEquals(-1, editingState.getComposingStart());
    assertEquals(-1, editingState.getComposingEnd());

    editingState.setComposingRange(2, 3);
    assertEquals(2, editingState.getComposingStart());
    assertEquals(3, editingState.getComposingEnd());

    // Empty range is invalid. Clears composing region.
    editingState.setComposingRange(1, 1);
    assertEquals(-1, editingState.getComposingStart());
    assertEquals(-1, editingState.getComposingEnd());

    // Covers everything.
    editingState.setComposingRange(0, editingState.length());
    assertEquals(0, editingState.getComposingStart());
    assertEquals(editingState.length(), editingState.getComposingEnd());
  }

  @Test
  public void testClearBatchDeltas() {
    final ListenableEditingState editingState = new ListenableEditingState(null, new View(ctx));
    editingState.replace(0, editingState.length(), "text");
    editingState.delete(0, 1);
    editingState.insert(0, "This is t");
    editingState.clearBatchDeltas();
    assertEquals(0, editingState.extractBatchTextEditingDeltas().size());
  }

  @Test
  public void testExtractBatchTextEditingDeltas() {
    final ListenableEditingState editingState = new ListenableEditingState(null, new View(ctx));

    // Creating some deltas.
    editingState.replace(0, editingState.length(), "test");
    editingState.delete(0, 1);
    editingState.insert(0, "This is a t");

    ArrayList<TextEditingDelta> batchDeltas = editingState.extractBatchTextEditingDeltas();
    assertEquals(3, batchDeltas.size());
  }

  // -------- Start: Test InputMethods actions   -------
  @Test
  public void inputMethod_batchEditingBeginAndEnd() {
    final ArrayList<String> batchMarkers = new ArrayList<>();
    final ListenableEditingState editingState =
        new ListenableEditingState(null, new View(ctx)) {
          @Override
          public final void beginBatchEdit() {
            super.beginBatchEdit();
            batchMarkers.add("begin");
          }

          @Override
          public void endBatchEdit() {
            super.endBatchEdit();
            batchMarkers.add("end");
          }
        };

    final Listener listener = new Listener();
    final View testView = new View(ctx);
    final InputConnectionAdaptor inputConnection =
        new InputConnectionAdaptor(
            testView,
            0,
            mock(TextInputChannel.class),
            mock(ScribeChannel.class),
            mockKeyboardManager,
            editingState,
            new EditorInfo());

    // Make sure begin/endBatchEdit is called on the Editable when the input method calls
    // InputConnection#begin/endBatchEdit.
    inputConnection.beginBatchEdit();
    assertEquals(1, batchMarkers.size());
    assertEquals("begin", batchMarkers.get(0));

    inputConnection.endBatchEdit();
    assertEquals(2, batchMarkers.size());
    assertEquals("end", batchMarkers.get(1));
  }

  @Test
  public void inputMethod_testSetSelection() {
    final ListenableEditingState editingState = new ListenableEditingState(null, new View(ctx));
    final Listener listener = new Listener();
    final View testView = new View(ctx);
    final InputConnectionAdaptor inputConnection =
        new InputConnectionAdaptor(
            testView,
            0,
            mock(TextInputChannel.class),
            mock(ScribeChannel.class),
            mockKeyboardManager,
            editingState,
            new EditorInfo());
    editingState.replace(0, editingState.length(), "initial text");

    editingState.addEditingStateListener(listener);

    inputConnection.setSelection(0, 0);

    assertTrue(listener.isCalled());
    assertFalse(listener.textChanged);
    assertTrue(listener.selectionChanged);
    assertFalse(listener.composingRegionChanged);

    listener.reset();

    inputConnection.setSelection(5, 5);

    assertTrue(listener.isCalled());
    assertFalse(listener.textChanged);
    assertTrue(listener.selectionChanged);
    assertFalse(listener.composingRegionChanged);
  }

  @Test
  public void inputMethod_testSetComposition() {
    final ListenableEditingState editingState = new ListenableEditingState(null, new View(ctx));
    final Listener listener = new Listener();
    final View testView = new View(ctx);
    final InputConnectionAdaptor inputConnection =
        new InputConnectionAdaptor(
            testView,
            0,
            mock(TextInputChannel.class),
            mock(ScribeChannel.class),
            mockKeyboardManager,
            editingState,
            new EditorInfo());
    editingState.replace(0, editingState.length(), "initial text");

    editingState.addEditingStateListener(listener);

    // setComposingRegion test.
    inputConnection.setComposingRegion(1, 3);
    assertTrue(listener.isCalled());
    assertFalse(listener.textChanged);
    assertFalse(listener.selectionChanged);
    assertTrue(listener.composingRegionChanged);

    Selection.setSelection(editingState, 0, 0);
    listener.reset();

    // setComposingText test: non-empty text, does not move cursor.
    inputConnection.setComposingText("composing", -1);
    assertTrue(listener.isCalled());
    assertTrue(listener.textChanged);
    assertFalse(listener.selectionChanged);
    assertTrue(listener.composingRegionChanged);

    listener.reset();
    // setComposingText test: non-empty text, moves cursor.
    inputConnection.setComposingText("composing2", 1);
    assertTrue(listener.isCalled());
    assertTrue(listener.textChanged);
    assertTrue(listener.selectionChanged);
    assertTrue(listener.composingRegionChanged);

    listener.reset();
    // setComposingText test: empty text.
    inputConnection.setComposingText("", 1);
    assertTrue(listener.isCalled());
    assertTrue(listener.textChanged);
    assertTrue(listener.selectionChanged);
    assertTrue(listener.composingRegionChanged);

    // finishComposingText test.
    inputConnection.setComposingText("composing text", 1);
    listener.reset();
    inputConnection.finishComposingText();
    assertTrue(listener.isCalled());
    assertFalse(listener.textChanged);
    assertFalse(listener.selectionChanged);
    assertTrue(listener.composingRegionChanged);
  }

  @Test
  public void inputMethod_testCommitText() {
    final ListenableEditingState editingState = new ListenableEditingState(null, new View(ctx));
    final Listener listener = new Listener();
    final View testView = new View(ctx);
    final InputConnectionAdaptor inputConnection =
        new InputConnectionAdaptor(
            testView,
            0,
            mock(TextInputChannel.class),
            mock(ScribeChannel.class),
            mockKeyboardManager,
            editingState,
            new EditorInfo());
    editingState.replace(0, editingState.length(), "initial text");

    editingState.addEditingStateListener(listener);
  }
  // -------- End: Test InputMethods actions   -------

  public static class Listener implements ListenableEditingState.EditingStateWatcher {
    public boolean isCalled() {
      return timesCalled > 0;
    }

    int timesCalled = 0;
    boolean textChanged = false;
    boolean selectionChanged = false;
    boolean composingRegionChanged = false;

    @Override
    public void didChangeEditingState(
        boolean textChanged, boolean selectionChanged, boolean composingRegionChanged) {
      timesCalled++;
      this.textChanged = textChanged;
      this.selectionChanged = selectionChanged;
      this.composingRegionChanged = composingRegionChanged;
    }

    public void reset() {
      timesCalled = 0;
      textChanged = false;
      selectionChanged = false;
      composingRegionChanged = false;
    }
  }
}
