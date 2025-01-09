// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

import '../model/product.dart';
import 'product_card.dart';

class TwoProductCardColumn extends StatelessWidget {
  const TwoProductCardColumn({
    super.key,
    required this.bottom,
    this.top,
    required this.imageAspectRatio,
  });

  static const double spacerHeight = 44;
  static const double horizontalPadding = 28;

  final Product bottom;
  final Product? top;
  final double imageAspectRatio;

  @override
  Widget build(BuildContext context) {
    return LayoutBuilder(
      builder: (BuildContext context, BoxConstraints constraints) {
        return ListView(
          physics: const ClampingScrollPhysics(),
          children: <Widget>[
            Padding(
              padding: const EdgeInsetsDirectional.only(start: horizontalPadding),
              child:
                  top != null
                      ? MobileProductCard(imageAspectRatio: imageAspectRatio, product: top!)
                      : const SizedBox(height: spacerHeight),
            ),
            const SizedBox(height: spacerHeight),
            Padding(
              padding: const EdgeInsetsDirectional.only(end: horizontalPadding),
              child: MobileProductCard(imageAspectRatio: imageAspectRatio, product: bottom),
            ),
          ],
        );
      },
    );
  }
}

class OneProductCardColumn extends StatelessWidget {
  const OneProductCardColumn({super.key, required this.product, required this.reverse});

  final Product product;

  // Whether the product column should align to the bottom.
  final bool reverse;

  @override
  Widget build(BuildContext context) {
    return ListView(
      physics: const ClampingScrollPhysics(),
      reverse: reverse,
      children: <Widget>[const SizedBox(height: 40), MobileProductCard(product: product)],
    );
  }
}
