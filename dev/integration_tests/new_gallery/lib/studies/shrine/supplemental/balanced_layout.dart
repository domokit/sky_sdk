// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

import '../model/product.dart';
import 'desktop_product_columns.dart';
import 'layout_cache.dart';

/// A placeholder id for an empty element. See [_iterateUntilBalanced]
/// for more information.
const int _emptyElement = -1;

/// To avoid infinite loops, improvements to the layout are only performed
/// when a column's height changes by more than
/// [_deviationImprovementThreshold] pixels.
const int _deviationImprovementThreshold = 10;

/// Height of a product image, paired with the product's id.
class _TaggedHeightData {
  const _TaggedHeightData({required this.index, required this.height});

  /// The id of the corresponding product.
  final int index;

  /// The height of the product image.
  final double height;
}

/// Converts a set of [_TaggedHeightData] elements to a list,
/// and add an empty element.
/// Used for iteration.
List<_TaggedHeightData> _toListAndAddEmpty(Set<_TaggedHeightData> set) {
  final List<_TaggedHeightData> result = List<_TaggedHeightData>.from(set);
  result.add(const _TaggedHeightData(index: _emptyElement, height: 0));
  return result;
}

/// Encode parameters for caching.
String _encodeParameters({
  required int columnCount,
  required List<Product> products,
  required double largeImageWidth,
  required double smallImageWidth,
}) {
  final String productString = <String>[
    for (final Product product in products) product.id.toString(),
  ].join(',');
  return '$columnCount;$productString,$largeImageWidth,$smallImageWidth';
}

/// Given a layout, replace integers by their corresponding products.
List<List<Product>> _generateLayout({
  required List<Product> products,
  required List<List<int>> layout,
}) {
  return <List<Product>>[
    for (final List<int> column in layout)
      <Product>[for (final int index in column) products[index]],
  ];
}

/// Given [columnObjects], list of the set of objects in each column,
/// and [columnHeights], list of heights of each column,
/// [_iterateUntilBalanced] moves and swaps objects between columns
/// until their heights are sufficiently close to each other.
/// This prevents the layout having significant, avoidable gaps at the bottom.
void _iterateUntilBalanced(List<Set<_TaggedHeightData>> columnObjects, List<double> columnHeights) {
  int failedMoves = 0;
  final int columnCount = columnObjects.length;

  // No need to rearrange a 1-column layout.
  if (columnCount == 1) {
    return;
  }

  while (true) {
    // Loop through all possible 2-combinations of columns.
    for (int source = 0; source < columnCount; ++source) {
      for (int target = source + 1; target < columnCount; ++target) {
        // Tries to find an object A from source column
        // and an object B from target column, such that switching them
        // causes the height of the two columns to be closer.

        // A or B can be empty; in this case, moving an object from one
        // column to the other is the best choice.

        bool success = false;

        final double bestHeight = (columnHeights[source] + columnHeights[target]) / 2;
        final double scoreLimit = (columnHeights[source] - bestHeight).abs();

        final List<_TaggedHeightData> sourceObjects = _toListAndAddEmpty(columnObjects[source]);
        final List<_TaggedHeightData> targetObjects = _toListAndAddEmpty(columnObjects[target]);

        _TaggedHeightData? bestA, bestB;
        double? bestScore;

        for (final _TaggedHeightData a in sourceObjects) {
          for (final _TaggedHeightData b in targetObjects) {
            if (a.index == _emptyElement && b.index == _emptyElement) {
              continue;
            } else {
              final double score = (columnHeights[source] - a.height + b.height - bestHeight).abs();
              if (score < scoreLimit - _deviationImprovementThreshold) {
                success = true;
                if (bestScore == null || score < bestScore) {
                  bestScore = score;
                  bestA = a;
                  bestB = b;
                }
              }
            }
          }
        }

        if (!success) {
          ++failedMoves;
        } else {
          failedMoves = 0;

          // Switch A and B.
          if (bestA != null && bestA.index != _emptyElement) {
            columnObjects[source].remove(bestA);
            columnObjects[target].add(bestA);
          }
          if (bestB != null && bestB.index != _emptyElement) {
            columnObjects[target].remove(bestB);
            columnObjects[source].add(bestB);
          }
          columnHeights[source] += bestB!.height - bestA!.height;
          columnHeights[target] += bestA.height - bestB.height;
        }

        // If no two columns' heights can be made closer by switching
        // elements, the layout is sufficiently balanced.
        if (failedMoves >= columnCount * (columnCount - 1) ~/ 2) {
          return;
        }
      }
    }
  }
}

/// Given a list of numbers [data], representing the heights of each image,
/// and a list of numbers [biases], representing the heights of the space
/// above each column, [_balancedDistribution] returns a layout of [data]
/// so that the height of each column is sufficiently close to each other,
/// represented as a list of lists of integers, each integer being an ID
/// for a product.
List<List<int>> _balancedDistribution({
  required int columnCount,
  required List<double> data,
  required List<double> biases,
}) {
  assert(biases.length == columnCount);

  final List<Set<_TaggedHeightData>> columnObjects = List<Set<_TaggedHeightData>>.generate(
    columnCount,
    (int column) => <_TaggedHeightData>{},
  );

  final List<double> columnHeights = List<double>.from(biases);

  for (int i = 0; i < data.length; ++i) {
    final int column = i % columnCount;
    columnHeights[column] += data[i];
    columnObjects[column].add(_TaggedHeightData(index: i, height: data[i]));
  }

  _iterateUntilBalanced(columnObjects, columnHeights);

  return <List<int>>[
    for (final Set<_TaggedHeightData> column in columnObjects)
      <int>[for (final _TaggedHeightData object in column) object.index]..sort(),
  ];
}

/// Generates a balanced layout for [columnCount] columns,
/// with products specified by the list [products],
/// where the larger images have width [largeImageWidth]
/// and the smaller images have width [smallImageWidth].
/// The current [context] is also given to allow caching.
List<List<Product>> balancedLayout({
  required BuildContext context,
  required int columnCount,
  required List<Product> products,
  required double largeImageWidth,
  required double smallImageWidth,
}) {
  final String encodedParameters = _encodeParameters(
    columnCount: columnCount,
    products: products,
    largeImageWidth: largeImageWidth,
    smallImageWidth: smallImageWidth,
  );

  // Check if this layout is cached.
  if (LayoutCache.of(context).containsKey(encodedParameters)) {
    return _generateLayout(products: products, layout: LayoutCache.of(context)[encodedParameters]!);
  }

  final List<double> productHeights = <double>[
    for (final Product product in products)
      1 / product.assetAspectRatio * (largeImageWidth + smallImageWidth) / 2 +
          productCardAdditionalHeight,
  ];

  final List<List<int>> layout = _balancedDistribution(
    columnCount: columnCount,
    data: productHeights,
    biases: List<double>.generate(
      columnCount,
      (int column) => (column.isEven ? 0 : columnTopSpace),
    ),
  );

  // Add tailored layout to cache.

  LayoutCache.of(context)[encodedParameters] = layout;

  final List<List<Product>> result = _generateLayout(products: products, layout: layout);

  return result;
}
