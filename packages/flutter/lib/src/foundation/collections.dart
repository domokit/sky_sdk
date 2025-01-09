// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:collection/collection.dart';
library;

// TODO(ianh): These should be on the Set and List classes themselves.

/// Compares two sets for element-by-element equality.
///
/// Returns true if the sets are both null, or if they are both non-null, have
/// the same length, and contain the same members. Returns false otherwise.
/// Order is not compared.
///
/// If the elements are maps, lists, sets, or other collections/composite
/// objects, then the contents of those elements are not compared element by
/// element unless their equality operators ([Object.==]) do so. For checking
/// deep equality, consider using the [DeepCollectionEquality] class.
///
/// See also:
///
///  * [listEquals], which does something similar for lists.
///  * [mapEquals], which does something similar for maps.
bool setEquals<T>(Set<T>? a, Set<T>? b) {
  if (a == null) {
    return b == null;
  }
  if (b == null || a.length != b.length) {
    return false;
  }
  if (identical(a, b)) {
    return true;
  }
  for (final T value in a) {
    if (!b.contains(value)) {
      return false;
    }
  }
  return true;
}

/// Compares two lists for element-by-element equality.
///
/// Returns true if the lists are both null, or if they are both non-null, have
/// the same length, and contain the same members in the same order. Returns
/// false otherwise.
///
/// If the elements are maps, lists, sets, or other collections/composite
/// objects, then the contents of those elements are not compared element by
/// element unless their equality operators ([Object.==]) do so. For checking
/// deep equality, consider using the [DeepCollectionEquality] class.
///
/// See also:
///
///  * [setEquals], which does something similar for sets.
///  * [mapEquals], which does something similar for maps.
bool listEquals<T>(List<T>? a, List<T>? b) {
  if (a == null) {
    return b == null;
  }
  if (b == null || a.length != b.length) {
    return false;
  }
  if (identical(a, b)) {
    return true;
  }
  for (int index = 0; index < a.length; index += 1) {
    if (a[index] != b[index]) {
      return false;
    }
  }
  return true;
}

/// Compares two maps for element-by-element equality.
///
/// Returns true if the maps are both null, or if they are both non-null, have
/// the same length, and contain the same keys associated with the same values.
/// Returns false otherwise.
///
/// If the elements are maps, lists, sets, or other collections/composite
/// objects, then the contents of those elements are not compared element by
/// element unless their equality operators ([Object.==]) do so. For checking
/// deep equality, consider using the [DeepCollectionEquality] class.
///
/// See also:
///
///  * [setEquals], which does something similar for sets.
///  * [listEquals], which does something similar for lists.
bool mapEquals<T, U>(Map<T, U>? a, Map<T, U>? b) {
  if (a == null) {
    return b == null;
  }
  if (b == null || a.length != b.length) {
    return false;
  }
  if (identical(a, b)) {
    return true;
  }
  for (final T key in a.keys) {
    if (!b.containsKey(key) || b[key] != a[key]) {
      return false;
    }
  }
  return true;
}

/// Returns the position of `value` in the `sortedList`, if it exists.
///
/// Returns `-1` if the `value` is not in the list. Requires the list items
/// to implement [Comparable] and the `sortedList` to already be ordered.
int binarySearch<T extends Comparable<Object>>(List<T> sortedList, T value) {
  int min = 0;
  int max = sortedList.length;
  while (min < max) {
    final int mid = min + ((max - min) >> 1);
    final T element = sortedList[mid];
    final int comp = element.compareTo(value);
    if (comp == 0) {
      return mid;
    }
    if (comp < 0) {
      min = mid + 1;
    } else {
      max = mid;
    }
  }
  return -1;
}

/// Limit below which merge sort defaults to insertion sort.
const int _kMergeSortLimit = 32;

/// Sorts a list between `start` (inclusive) and `end` (exclusive) using the
/// merge sort algorithm.
///
/// If `compare` is omitted, this defaults to calling [Comparable.compareTo] on
/// the objects. If any object is not [Comparable], this throws a [TypeError]
/// (The stack trace may call it `_CastError` or `_TypeError`, but to catch it,
/// use [TypeError]).
///
/// Merge-sorting works by splitting the job into two parts, sorting each
/// recursively, and then merging the two sorted parts.
///
/// This takes on the order of `n * log(n)` comparisons and moves to sort `n`
/// elements, but requires extra space of about the same size as the list being
/// sorted.
///
/// This merge sort is stable: Equal elements end up in the same order as they
/// started in.
///
/// For small lists (less than 32 elements), [mergeSort] automatically uses an
/// insertion sort instead, as that is more efficient for small lists. The
/// insertion sort is also stable.
void mergeSort<T>(List<T> list, {int start = 0, int? end, int Function(T, T)? compare}) {
  end ??= list.length;
  compare ??= _defaultCompare<T>();

  final int length = end - start;
  if (length < 2) {
    return;
  }
  if (length < _kMergeSortLimit) {
    _insertionSort<T>(list, compare: compare, start: start, end: end);
    return;
  }
  // Special case the first split instead of directly calling _mergeSort,
  // because the _mergeSort requires its target to be different from its source,
  // and it requires extra space of the same size as the list to sort. This
  // split allows us to have only half as much extra space, and it ends up in
  // the original place.
  final int middle = start + ((end - start) >> 1);
  final int firstLength = middle - start;
  final int secondLength = end - middle;
  // secondLength is always the same as firstLength, or one greater.
  final List<T> scratchSpace = List<T>.filled(secondLength, list[start]);
  _mergeSort<T>(list, compare, middle, end, scratchSpace, 0);
  final int firstTarget = end - firstLength;
  _mergeSort<T>(list, compare, start, middle, list, firstTarget);
  _merge<T>(compare, list, firstTarget, end, scratchSpace, 0, secondLength, list, start);
}

/// Returns a [Comparator] that asserts that its first argument is comparable.
Comparator<T> _defaultCompare<T>() {
  // If we specify Comparable<T> here, it fails if the type is an int, because
  // int isn't a subtype of comparable. Leaving out the type implicitly converts
  // it to a num, which is a comparable.
  return (T value1, T value2) => (value1 as Comparable<dynamic>).compareTo(value2);
}

/// Sort a list between `start` (inclusive) and `end` (exclusive) using
/// insertion sort.
///
/// If `compare` is omitted, this defaults to calling [Comparable.compareTo] on
/// the objects. If any object is not [Comparable], this throws a [TypeError]
/// (The stack trace may call it `_CastError` or `_TypeError`, but to catch it,
/// use [TypeError]).
///
/// Insertion sort is a simple sorting algorithm. For `n` elements it does on
/// the order of `n * log(n)` comparisons but up to `n` squared moves. The
/// sorting is performed in-place, without using extra memory.
///
/// For short lists the many moves have less impact than the simple algorithm,
/// and it is often the favored sorting algorithm for short lists.
///
/// This insertion sort is stable: Equal elements end up in the same order as
/// they started in.
void _insertionSort<T>(List<T> list, {int Function(T, T)? compare, int start = 0, int? end}) {
  // If the same method could have both positional and named optional
  // parameters, this should be (list, [start, end], {compare}).
  compare ??= _defaultCompare<T>();
  end ??= list.length;

  for (int pos = start + 1; pos < end; pos++) {
    int min = start;
    int max = pos;
    final T element = list[pos];
    while (min < max) {
      final int mid = min + ((max - min) >> 1);
      final int comparison = compare(element, list[mid]);
      if (comparison < 0) {
        max = mid;
      } else {
        min = mid + 1;
      }
    }
    list.setRange(min + 1, pos + 1, list, min);
    list[min] = element;
  }
}

/// Performs an insertion sort into a potentially different list than the one
/// containing the original values.
///
/// It will work in-place as well.
void _movingInsertionSort<T>(
  List<T> list,
  int Function(T, T) compare,
  int start,
  int end,
  List<T> target,
  int targetOffset,
) {
  final int length = end - start;
  if (length == 0) {
    return;
  }
  target[targetOffset] = list[start];
  for (int i = 1; i < length; i++) {
    final T element = list[start + i];
    int min = targetOffset;
    int max = targetOffset + i;
    while (min < max) {
      final int mid = min + ((max - min) >> 1);
      if (compare(element, target[mid]) < 0) {
        max = mid;
      } else {
        min = mid + 1;
      }
    }
    target.setRange(min + 1, targetOffset + i + 1, target, min);
    target[min] = element;
  }
}

/// Sorts `list` from `start` to `end` into `target` at `targetOffset`.
///
/// The `target` list must be able to contain the range from `start` to `end`
/// after `targetOffset`.
///
/// Allows target to be the same list as `list`, as long as it's not overlapping
/// the `start..end` range.
void _mergeSort<T>(
  List<T> list,
  int Function(T, T) compare,
  int start,
  int end,
  List<T> target,
  int targetOffset,
) {
  final int length = end - start;
  if (length < _kMergeSortLimit) {
    _movingInsertionSort<T>(list, compare, start, end, target, targetOffset);
    return;
  }
  final int middle = start + (length >> 1);
  final int firstLength = middle - start;
  final int secondLength = end - middle;
  // Here secondLength >= firstLength (differs by at most one).
  final int targetMiddle = targetOffset + firstLength;
  // Sort the second half into the end of the target area.
  _mergeSort<T>(list, compare, middle, end, target, targetMiddle);
  // Sort the first half into the end of the source area.
  _mergeSort<T>(list, compare, start, middle, list, middle);
  // Merge the two parts into the target area.
  _merge<T>(
    compare,
    list,
    middle,
    middle + firstLength,
    target,
    targetMiddle,
    targetMiddle + secondLength,
    target,
    targetOffset,
  );
}

/// Merges two lists into a target list.
///
/// One of the input lists may be positioned at the end of the target list.
///
/// For equal object, elements from `firstList` are always preferred. This
/// allows the merge to be stable if the first list contains elements that
/// started out earlier than the ones in `secondList`.
void _merge<T>(
  int Function(T, T) compare,
  List<T> firstList,
  int firstStart,
  int firstEnd,
  List<T> secondList,
  int secondStart,
  int secondEnd,
  List<T> target,
  int targetOffset,
) {
  // No empty lists reaches here.
  assert(firstStart < firstEnd);
  assert(secondStart < secondEnd);
  int cursor1 = firstStart;
  int cursor2 = secondStart;
  T firstElement = firstList[cursor1++];
  T secondElement = secondList[cursor2++];
  while (true) {
    if (compare(firstElement, secondElement) <= 0) {
      target[targetOffset++] = firstElement;
      if (cursor1 == firstEnd) {
        // Flushing second list after loop.
        break;
      }
      firstElement = firstList[cursor1++];
    } else {
      target[targetOffset++] = secondElement;
      if (cursor2 != secondEnd) {
        secondElement = secondList[cursor2++];
        continue;
      }
      // Second list empties first. Flushing first list here.
      target[targetOffset++] = firstElement;
      target.setRange(targetOffset, targetOffset + (firstEnd - cursor1), firstList, cursor1);
      return;
    }
  }
  // First list empties first. Reached by break above.
  target[targetOffset++] = secondElement;
  target.setRange(targetOffset, targetOffset + (secondEnd - cursor2), secondList, cursor2);
}
