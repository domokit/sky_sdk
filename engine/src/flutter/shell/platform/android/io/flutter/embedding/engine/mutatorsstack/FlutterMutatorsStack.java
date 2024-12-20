// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.engine.mutatorsstack;

import android.graphics.Matrix;
import android.graphics.Path;
import android.graphics.Rect;
import android.graphics.RectF;
import androidx.annotation.Keep;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import java.util.ArrayList;
import java.util.List;

/**
 * The mutator stack containing a list of mutators
 *
 * <p>The mutators can be applied to a {@link io.flutter.plugin.platform.PlatformView} to perform a
 * series mutations. See {@link FlutterMutatorsStack.FlutterMutator} for informations on Mutators.
 */
@Keep
public class FlutterMutatorsStack {
  /**
   * The type of a Mutator See {@link FlutterMutatorsStack.FlutterMutator} for informations on
   * Mutators.
   */
  public enum FlutterMutatorType {
    CLIP_RECT,
    CLIP_RRECT,
    CLIP_PATH,
    TRANSFORM,
    OPACITY
  }

  /**
   * A class represents a mutator
   *
   * <p>A mutator contains information of a single mutation operation that can be applied to a
   * {@link io.flutter.plugin.platform.PlatformView}. See {@link
   * FlutterMutatorsStack.FlutterMutator} for informations on Mutators.
   */
  public class FlutterMutator {

    @Nullable private Matrix matrix;
    @Nullable private Rect rect;
    @Nullable private Path path;
    @Nullable private float[] radiis;

    private FlutterMutatorType type;

    /**
     * Initialize a clip rect mutator.
     *
     * @param rect the rect to be clipped.
     */
    public FlutterMutator(Rect rect) {
      this.type = FlutterMutatorType.CLIP_RECT;
      this.rect = rect;
    }

    /**
     * Initialize a clip rrect mutator.
     *
     * @param rect the rect of the rrect
     * @param radiis the radiis of the rrect. Array of 8 values, 4 pairs of [X,Y]. This value cannot
     *     be null.
     */
    public FlutterMutator(Rect rect, float[] radiis) {
      this.type = FlutterMutatorType.CLIP_RRECT;
      this.rect = rect;
      this.radiis = radiis;
    }

    /**
     * Initialize a clip path mutator.
     *
     * @param path the path to be clipped.
     */
    public FlutterMutator(Path path) {
      this.type = FlutterMutatorType.CLIP_PATH;
      this.path = path;
    }

    /**
     * Initialize a transform mutator.
     *
     * @param matrix the transform matrix to apply.
     */
    public FlutterMutator(Matrix matrix) {
      this.type = FlutterMutatorType.TRANSFORM;
      this.matrix = matrix;
    }

    /**
     * Get the mutator type.
     *
     * @return The type of the mutator.
     */
    public FlutterMutatorType getType() {
      return type;
    }

    /**
     * Get the rect of the mutator if the {@link #getType()} returns FlutterMutatorType.CLIP_RECT.
     *
     * @return the clipping rect if the type is FlutterMutatorType.CLIP_RECT; otherwise null.
     */
    public Rect getRect() {
      return rect;
    }

    /**
     * Get the path of the mutator if the {@link #getType()} returns FlutterMutatorType.CLIP_PATH.
     *
     * @return the clipping path if the type is FlutterMutatorType.CLIP_PATH; otherwise null.
     */
    public Path getPath() {
      return path;
    }

    /**
     * Get the matrix of the mutator if the {@link #getType()} returns FlutterMutatorType.TRANSFORM.
     *
     * @return the matrix if the type is FlutterMutatorType.TRANSFORM; otherwise null.
     */
    public Matrix getMatrix() {
      return matrix;
    }
  }

  private @NonNull List<FlutterMutator> mutators;

  private List<Path> finalClippingPaths;
  private Matrix finalMatrix;

  /** Initialize the mutator stack. */
  public FlutterMutatorsStack() {
    this.mutators = new ArrayList<FlutterMutator>();
    finalMatrix = new Matrix();
    finalClippingPaths = new ArrayList<Path>();
  }

  /**
   * Push a transform {@link FlutterMutatorsStack.FlutterMutator} to the stack.
   *
   * @param values the transform matrix to be pushed to the stack. The array matches how a {@link
   *     android.graphics.Matrix} is constructed.
   */
  public void pushTransform(float[] values) {
    Matrix matrix = new Matrix();
    matrix.setValues(values);
    FlutterMutator mutator = new FlutterMutator(matrix);
    mutators.add(mutator);
    finalMatrix.preConcat(mutator.getMatrix());
  }

  /** Push a clipRect {@link FlutterMutatorsStack.FlutterMutator} to the stack. */
  public void pushClipRect(int left, int top, int right, int bottom) {
    Rect rect = new Rect(left, top, right, bottom);
    FlutterMutator mutator = new FlutterMutator(rect);
    mutators.add(mutator);
    Path path = new Path();
    path.addRect(new RectF(rect), Path.Direction.CCW);
    path.transform(finalMatrix);
    finalClippingPaths.add(path);
  }

  /**
   * Push a clipRRect {@link FlutterMutatorsStack.FlutterMutator} to the stack.
   *
   * @param left left offset of the rrect.
   * @param top top offset of the rrect.
   * @param right right position of the rrect.
   * @param bottom bottom position of the rrect.
   * @param radiis the radiis of the rrect. It must be size of 8, including an x and y for each
   *     corner.
   */
  public void pushClipRRect(int left, int top, int right, int bottom, float[] radiis) {
    Rect rect = new Rect(left, top, right, bottom);
    FlutterMutator mutator = new FlutterMutator(rect, radiis);
    mutators.add(mutator);
    Path path = new Path();
    path.addRoundRect(new RectF(rect), radiis, Path.Direction.CCW);
    path.transform(finalMatrix);
    finalClippingPaths.add(path);
  }

  /**
   * Get a list of all the raw mutators. The 0 index of the returned list is the top of the stack.
   */
  public List<FlutterMutator> getMutators() {
    return mutators;
  }

  /**
   * Get a list of all the clipping operations. All the clipping operations -- whether it is clip
   * rect, clip rrect, or clip path -- are converted into Paths. The paths are also transformed with
   * the matrix that up to their stack positions. For example: If the stack looks like (from top to
   * bottom): TransA -&gt; ClipA -&gt; TransB -&gt; ClipB, the final paths will look like
   * [TransA*ClipA, TransA*TransB*ClipB].
   *
   * <p>Clipping this list to the parent canvas of a view results the final clipping path.
   */
  public List<Path> getFinalClippingPaths() {
    return finalClippingPaths;
  }

  /**
   * Returns the final matrix. Apply this matrix to the canvas of a view results the final
   * transformation of the view.
   */
  public Matrix getFinalMatrix() {
    return finalMatrix;
  }
}
