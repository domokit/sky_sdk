// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:intl/intl.dart';
import 'package:scoped_model/scoped_model.dart';

import '../model/app_state_model.dart';
import '../model/product.dart';

class ProductCard extends StatelessWidget {
  const ProductCard({super.key, this.imageAspectRatio = 33 / 49, this.product})
    : assert(imageAspectRatio > 0);

  final double imageAspectRatio;
  final Product? product;

  static const double kTextBoxHeight = 65.0;

  @override
  Widget build(BuildContext context) {
    final NumberFormat formatter = NumberFormat.simpleCurrency(
      decimalDigits: 0,
      locale: Localizations.localeOf(context).toString(),
    );

    final ThemeData theme = Theme.of(context);

    final Image imageWidget = Image.asset(
      product!.assetName,
      package: product!.assetPackage,
      fit: BoxFit.cover,
    );

    // The fontSize to use for computing the heuristic UI scaling factor.
    const double defaultFontSize = 14.0;
    final double containerScalingFactor =
        MediaQuery.textScalerOf(context).scale(defaultFontSize) / defaultFontSize;

    return ScopedModelDescendant<AppStateModel>(
      builder: (BuildContext context, Widget? child, AppStateModel model) {
        return GestureDetector(
          onTap: () {
            model.addProductToCart(product!.id);
          },
          child: child,
        );
      },
      child: Stack(
        children: <Widget>[
          Column(
            mainAxisAlignment: MainAxisAlignment.center,
            children: <Widget>[
              AspectRatio(aspectRatio: imageAspectRatio, child: imageWidget),
              SizedBox(
                height: kTextBoxHeight * containerScalingFactor,
                width: 121.0,
                child: Column(
                  mainAxisAlignment: MainAxisAlignment.end,
                  children: <Widget>[
                    Text(
                      product == null ? '' : product!.name,
                      style: theme.textTheme.labelLarge,
                      softWrap: false,
                      overflow: TextOverflow.ellipsis,
                      maxLines: 1,
                    ),
                    const SizedBox(height: 4.0),
                    Text(
                      product == null ? '' : formatter.format(product!.price),
                      style: theme.textTheme.bodySmall,
                    ),
                  ],
                ),
              ),
            ],
          ),
          const Padding(padding: EdgeInsets.all(16.0), child: Icon(Icons.add_shopping_cart)),
        ],
      ),
    );
  }
}
