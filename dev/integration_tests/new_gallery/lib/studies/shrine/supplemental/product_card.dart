// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:intl/intl.dart';
import 'package:scoped_model/scoped_model.dart';

import '../../../gallery_localizations.dart';
import '../../../layout/adaptive.dart';
import '../../../layout/image_placeholder.dart';
import '../model/app_state_model.dart';
import '../model/product.dart';

class MobileProductCard extends StatelessWidget {
  const MobileProductCard({super.key, this.imageAspectRatio = 33 / 49, required this.product})
    : assert(imageAspectRatio > 0);

  final double imageAspectRatio;
  final Product product;

  static const double defaultTextBoxHeight = 65;

  @override
  Widget build(BuildContext context) {
    return Semantics(
      container: true,
      button: true,
      enabled: true,
      child: _buildProductCard(
        context: context,
        product: product,
        imageAspectRatio: imageAspectRatio,
      ),
    );
  }
}

class DesktopProductCard extends StatelessWidget {
  const DesktopProductCard({super.key, required this.product, required this.imageWidth});

  final Product product;
  final double imageWidth;

  @override
  Widget build(BuildContext context) {
    return _buildProductCard(context: context, product: product, imageWidth: imageWidth);
  }
}

Widget _buildProductCard({
  required BuildContext context,
  required Product product,
  double? imageWidth,
  double? imageAspectRatio,
}) {
  final bool isDesktop = isDisplayDesktop(context);
  // In case of desktop , imageWidth is passed through [DesktopProductCard] in
  // case of mobile imageAspectRatio is passed through [MobileProductCard].
  // Below assert is so that correct combination should always be present.
  assert(isDesktop && imageWidth != null || !isDesktop && imageAspectRatio != null);

  final NumberFormat formatter = NumberFormat.simpleCurrency(
    decimalDigits: 0,
    locale: Localizations.localeOf(context).toString(),
  );
  final ThemeData theme = Theme.of(context);
  final FadeInImagePlaceholder imageWidget = FadeInImagePlaceholder(
    image: AssetImage(product.assetName, package: product.assetPackage),
    placeholder: Container(
      color: Colors.black.withOpacity(0.1),
      width: imageWidth,
      height: imageWidth == null ? null : imageWidth / product.assetAspectRatio,
    ),
    fit: BoxFit.cover,
    width: isDesktop ? imageWidth : null,
    height: isDesktop ? null : double.infinity,
    excludeFromSemantics: true,
  );

  return ScopedModelDescendant<AppStateModel>(
    builder: (BuildContext context, Widget? child, AppStateModel model) {
      return Semantics(
        hint: GalleryLocalizations.of(context)!.shrineScreenReaderProductAddToCart,
        child: MouseRegion(
          cursor: SystemMouseCursors.click,
          child: GestureDetector(
            onTap: () {
              model.addProductToCart(product.id);
            },
            child: child,
          ),
        ),
      );
    },
    child: Stack(
      children: <Widget>[
        Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: <Widget>[
            if (isDesktop)
              imageWidget
            else
              AspectRatio(aspectRatio: imageAspectRatio!, child: imageWidget),
            SizedBox(
              child: Column(
                children: <Widget>[
                  const SizedBox(height: 23),
                  SizedBox(
                    width: imageWidth,
                    child: Text(
                      product.name(context),
                      style: theme.textTheme.labelLarge,
                      softWrap: true,
                      textAlign: TextAlign.center,
                    ),
                  ),
                  const SizedBox(height: 4),
                  Text(formatter.format(product.price), style: theme.textTheme.bodySmall),
                ],
              ),
            ),
          ],
        ),
        const Padding(padding: EdgeInsets.all(16), child: Icon(Icons.add_shopping_cart)),
      ],
    ),
  );
}
