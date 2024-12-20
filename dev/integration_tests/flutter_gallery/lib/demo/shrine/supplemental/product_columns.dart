// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

import '../model/product.dart';
import 'product_card.dart';

class TwoProductCardColumn extends StatelessWidget {
  const TwoProductCardColumn({super.key, required this.bottom, this.top});

  final Product? bottom, top;

  @override
  Widget build(BuildContext context) {
    return LayoutBuilder(
      builder: (BuildContext context, BoxConstraints constraints) {
        const double spacerHeight = 44.0;

        final double heightOfCards = (constraints.biggest.height - spacerHeight) / 2.0;
        final double availableHeightForImages = heightOfCards - ProductCard.kTextBoxHeight;
        // Ensure the cards take up the available space as long as the screen is
        // sufficiently tall, otherwise fallback on a constant aspect ratio.
        final double imageAspectRatio =
            availableHeightForImages >= 0.0
                ? constraints.biggest.width / availableHeightForImages
                : 49.0 / 33.0;

        return ListView(
          physics: const ClampingScrollPhysics(),
          children: <Widget>[
            Padding(
              padding: const EdgeInsetsDirectional.only(start: 28.0),
              child:
                  top != null
                      ? ProductCard(imageAspectRatio: imageAspectRatio, product: top)
                      : SizedBox(height: heightOfCards > 0 ? heightOfCards : spacerHeight),
            ),
            const SizedBox(height: spacerHeight),
            Padding(
              padding: const EdgeInsetsDirectional.only(end: 28.0),
              child: ProductCard(imageAspectRatio: imageAspectRatio, product: bottom),
            ),
          ],
        );
      },
    );
  }
}

class OneProductCardColumn extends StatelessWidget {
  const OneProductCardColumn({super.key, this.product});

  final Product? product;

  @override
  Widget build(BuildContext context) {
    return ListView(
      physics: const ClampingScrollPhysics(),
      reverse: true,
      children: <Widget>[const SizedBox(height: 40.0), ProductCard(product: product)],
    );
  }
}
