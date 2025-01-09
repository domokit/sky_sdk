// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:intl/intl.dart';
import 'package:scoped_model/scoped_model.dart';

import '../../gallery_localizations.dart';
import '../../layout/letter_spacing.dart';
import 'colors.dart';
import 'expanding_bottom_sheet.dart';
import 'model/app_state_model.dart';
import 'model/product.dart';
import 'theme.dart';

const double _startColumnWidth = 60.0;
const String _ordinalSortKeyName = 'shopping_cart';

class ShoppingCartPage extends StatefulWidget {
  const ShoppingCartPage({super.key});

  @override
  State<ShoppingCartPage> createState() => _ShoppingCartPageState();
}

class _ShoppingCartPageState extends State<ShoppingCartPage> {
  List<Widget> _createShoppingCartRows(AppStateModel model) {
    return model.productsInCart.keys
        .map(
          (int id) => ShoppingCartRow(
            product: model.getProductById(id),
            quantity: model.productsInCart[id],
            onPressed: () {
              model.removeItemFromCart(id);
            },
          ),
        )
        .toList();
  }

  @override
  Widget build(BuildContext context) {
    final ThemeData localTheme = Theme.of(context);
    return Scaffold(
      backgroundColor: shrinePink50,
      body: SafeArea(
        child: ScopedModelDescendant<AppStateModel>(
          builder: (BuildContext context, Widget? child, AppStateModel model) {
            final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;
            final ExpandingBottomSheetState? expandingBottomSheet = ExpandingBottomSheet.of(
              context,
            );
            return Stack(
              children: <Widget>[
                ListView(
                  children: <Widget>[
                    Semantics(
                      sortKey: const OrdinalSortKey(0, name: _ordinalSortKeyName),
                      child: Row(
                        children: <Widget>[
                          SizedBox(
                            width: _startColumnWidth,
                            child: IconButton(
                              icon: const Icon(Icons.keyboard_arrow_down),
                              onPressed: () => expandingBottomSheet!.close(),
                              tooltip: localizations.shrineTooltipCloseCart,
                            ),
                          ),
                          Text(
                            localizations.shrineCartPageCaption,
                            style: localTheme.textTheme.titleMedium!.copyWith(
                              fontWeight: FontWeight.w600,
                            ),
                          ),
                          const SizedBox(width: 16),
                          Text(localizations.shrineCartItemCount(model.totalCartQuantity)),
                        ],
                      ),
                    ),
                    const SizedBox(height: 16),
                    Semantics(
                      sortKey: const OrdinalSortKey(1, name: _ordinalSortKeyName),
                      child: Column(children: _createShoppingCartRows(model)),
                    ),
                    Semantics(
                      sortKey: const OrdinalSortKey(2, name: _ordinalSortKeyName),
                      child: ShoppingCartSummary(model: model),
                    ),
                    const SizedBox(height: 100),
                  ],
                ),
                PositionedDirectional(
                  bottom: 16,
                  start: 16,
                  end: 16,
                  child: Semantics(
                    sortKey: const OrdinalSortKey(3, name: _ordinalSortKeyName),
                    child: ElevatedButton(
                      style: ElevatedButton.styleFrom(
                        shape: const BeveledRectangleBorder(
                          borderRadius: BorderRadius.all(Radius.circular(7)),
                        ),
                        backgroundColor: shrinePink100,
                      ),
                      onPressed: () {
                        model.clearCart();
                        expandingBottomSheet!.close();
                      },
                      child: Padding(
                        padding: const EdgeInsets.symmetric(vertical: 12),
                        child: Text(
                          localizations.shrineCartClearButtonCaption,
                          style: TextStyle(letterSpacing: letterSpacingOrNone(largeLetterSpacing)),
                        ),
                      ),
                    ),
                  ),
                ),
              ],
            );
          },
        ),
      ),
    );
  }
}

class ShoppingCartSummary extends StatelessWidget {
  const ShoppingCartSummary({super.key, required this.model});

  final AppStateModel model;

  @override
  Widget build(BuildContext context) {
    final TextStyle smallAmountStyle = Theme.of(
      context,
    ).textTheme.bodyMedium!.copyWith(color: shrineBrown600);
    final TextStyle largeAmountStyle = Theme.of(
      context,
    ).textTheme.headlineMedium!.copyWith(letterSpacing: letterSpacingOrNone(mediumLetterSpacing));
    final NumberFormat formatter = NumberFormat.simpleCurrency(
      decimalDigits: 2,
      locale: Localizations.localeOf(context).toString(),
    );
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;

    return Row(
      children: <Widget>[
        const SizedBox(width: _startColumnWidth),
        Expanded(
          child: Padding(
            padding: const EdgeInsetsDirectional.only(end: 16),
            child: Column(
              children: <Widget>[
                MergeSemantics(
                  child: Row(
                    children: <Widget>[
                      SelectableText(localizations.shrineCartTotalCaption),
                      Expanded(
                        child: SelectableText(
                          formatter.format(model.totalCost),
                          style: largeAmountStyle,
                          textAlign: TextAlign.end,
                        ),
                      ),
                    ],
                  ),
                ),
                const SizedBox(height: 16),
                MergeSemantics(
                  child: Row(
                    children: <Widget>[
                      SelectableText(localizations.shrineCartSubtotalCaption),
                      Expanded(
                        child: SelectableText(
                          formatter.format(model.subtotalCost),
                          style: smallAmountStyle,
                          textAlign: TextAlign.end,
                        ),
                      ),
                    ],
                  ),
                ),
                const SizedBox(height: 4),
                MergeSemantics(
                  child: Row(
                    children: <Widget>[
                      SelectableText(localizations.shrineCartShippingCaption),
                      Expanded(
                        child: SelectableText(
                          formatter.format(model.shippingCost),
                          style: smallAmountStyle,
                          textAlign: TextAlign.end,
                        ),
                      ),
                    ],
                  ),
                ),
                const SizedBox(height: 4),
                MergeSemantics(
                  child: Row(
                    children: <Widget>[
                      SelectableText(localizations.shrineCartTaxCaption),
                      Expanded(
                        child: SelectableText(
                          formatter.format(model.tax),
                          style: smallAmountStyle,
                          textAlign: TextAlign.end,
                        ),
                      ),
                    ],
                  ),
                ),
              ],
            ),
          ),
        ),
      ],
    );
  }
}

class ShoppingCartRow extends StatelessWidget {
  const ShoppingCartRow({super.key, required this.product, required this.quantity, this.onPressed});

  final Product product;
  final int? quantity;
  final VoidCallback? onPressed;

  @override
  Widget build(BuildContext context) {
    final NumberFormat formatter = NumberFormat.simpleCurrency(
      decimalDigits: 0,
      locale: Localizations.localeOf(context).toString(),
    );
    final ThemeData localTheme = Theme.of(context);

    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;

    return Padding(
      padding: const EdgeInsets.only(bottom: 16),
      child: Row(
        key: ValueKey<int>(product.id),
        crossAxisAlignment: CrossAxisAlignment.start,
        children: <Widget>[
          Semantics(
            container: true,
            label: localizations.shrineScreenReaderRemoveProductButton(product.name(context)),
            button: true,
            enabled: true,
            child: ExcludeSemantics(
              child: SizedBox(
                width: _startColumnWidth,
                child: IconButton(
                  icon: const Icon(Icons.remove_circle_outline),
                  onPressed: onPressed,
                  tooltip: localizations.shrineTooltipRemoveItem,
                ),
              ),
            ),
          ),
          Expanded(
            child: Padding(
              padding: const EdgeInsetsDirectional.only(end: 16),
              child: Column(
                children: <Widget>[
                  Row(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: <Widget>[
                      Image.asset(
                        product.assetName,
                        package: product.assetPackage,
                        fit: BoxFit.cover,
                        width: 75,
                        height: 75,
                        excludeFromSemantics: true,
                      ),
                      const SizedBox(width: 16),
                      Expanded(
                        child: MergeSemantics(
                          child: Column(
                            crossAxisAlignment: CrossAxisAlignment.start,
                            children: <Widget>[
                              MergeSemantics(
                                child: Row(
                                  children: <Widget>[
                                    Expanded(
                                      child: SelectableText(
                                        localizations.shrineProductQuantity(quantity!),
                                      ),
                                    ),
                                    SelectableText(
                                      localizations.shrineProductPrice(
                                        formatter.format(product.price),
                                      ),
                                    ),
                                  ],
                                ),
                              ),
                              SelectableText(
                                product.name(context),
                                style: localTheme.textTheme.titleMedium!.copyWith(
                                  fontWeight: FontWeight.w600,
                                ),
                              ),
                            ],
                          ),
                        ),
                      ),
                    ],
                  ),
                  const SizedBox(height: 16),
                  const Divider(color: shrineBrown900, height: 10),
                ],
              ),
            ),
          ),
        ],
      ),
    );
  }
}
