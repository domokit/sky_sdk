// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math';

import 'package:flutter/material.dart';

import '../model/product.dart';
import 'product_card.dart';

/// Height of the text below each product card.
const double productCardAdditionalHeight = 84.0 * 2;

/// Height of the divider between product cards.
const double productCardDividerHeight = 84.0;

/// Height of the space at the top of every other column.
const double columnTopSpace = 84.0;

class DesktopProductCardColumn extends StatelessWidget {
  const DesktopProductCardColumn({
    super.key,
    required this.alignToEnd,
    required this.startLarge,
    required this.lowerStart,
    required this.products,
    required this.largeImageWidth,
    required this.smallImageWidth,
  });

  final List<Product> products;

  final bool alignToEnd;
  final bool startLarge;
  final bool lowerStart;

  final double largeImageWidth;
  final double smallImageWidth;

  @override
  Widget build(BuildContext context) {
    return LayoutBuilder(
      builder: (BuildContext context, BoxConstraints constraints) {
        final int currentColumnProductCount = products.length;
        final int currentColumnWidgetCount = max(2 * currentColumnProductCount - 1, 0);

        return SizedBox(
          width: largeImageWidth,
          child: Column(
            crossAxisAlignment: alignToEnd ? CrossAxisAlignment.end : CrossAxisAlignment.start,
            children: <Widget>[
              if (lowerStart) Container(height: columnTopSpace),
              ...List<Widget>.generate(currentColumnWidgetCount, (int index) {
                Widget card;
                if (index.isEven) {
                  // This is a product.
                  final int productCardIndex = index ~/ 2;
                  card = DesktopProductCard(
                    product: products[productCardIndex],
                    imageWidth:
                        startLarge == productCardIndex.isEven ? largeImageWidth : smallImageWidth,
                  );
                } else {
                  // This is just a divider.
                  card = Container(height: productCardDividerHeight);
                }
                return RepaintBoundary(child: card);
              }),
            ],
          ),
        );
      },
    );
  }
}
