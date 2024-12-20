// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:intl/intl.dart';
import 'package:scoped_model/scoped_model.dart';

import 'colors.dart';
import 'expanding_bottom_sheet.dart';
import 'model/app_state_model.dart';
import 'model/product.dart';

const double _leftColumnWidth = 60.0;

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
      backgroundColor: kShrinePink50,
      body: SafeArea(
        child: ScopedModelDescendant<AppStateModel>(
          builder: (BuildContext context, Widget? child, AppStateModel model) {
            return Stack(
              children: <Widget>[
                ListView(
                  children: <Widget>[
                    Row(
                      children: <Widget>[
                        SizedBox(
                          width: _leftColumnWidth,
                          child: IconButton(
                            icon: const Icon(Icons.keyboard_arrow_down),
                            onPressed: () => ExpandingBottomSheet.of(context)!.close(),
                          ),
                        ),
                        Text(
                          'CART',
                          style: localTheme.textTheme.titleMedium!.copyWith(
                            fontWeight: FontWeight.w600,
                          ),
                        ),
                        const SizedBox(width: 16.0),
                        Text('${model.totalCartQuantity} ITEMS'),
                      ],
                    ),
                    const SizedBox(height: 16.0),
                    Column(children: _createShoppingCartRows(model)),
                    ShoppingCartSummary(model: model),
                    const SizedBox(height: 100.0),
                  ],
                ),
                Positioned(
                  bottom: 16.0,
                  left: 16.0,
                  right: 16.0,
                  child: ElevatedButton(
                    style: ElevatedButton.styleFrom(
                      backgroundColor: kShrinePink100,
                      shape: const BeveledRectangleBorder(
                        borderRadius: BorderRadius.all(Radius.circular(7.0)),
                      ),
                    ),
                    child: const Padding(
                      padding: EdgeInsets.symmetric(vertical: 12.0),
                      child: Text('CLEAR CART'),
                    ),
                    onPressed: () {
                      model.clearCart();
                      ExpandingBottomSheet.of(context)!.close();
                    },
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
  const ShoppingCartSummary({super.key, this.model});

  final AppStateModel? model;

  @override
  Widget build(BuildContext context) {
    final TextStyle smallAmountStyle = Theme.of(
      context,
    ).textTheme.bodyMedium!.copyWith(color: kShrineBrown600);
    final TextStyle? largeAmountStyle = Theme.of(context).textTheme.headlineMedium;
    final NumberFormat formatter = NumberFormat.simpleCurrency(
      decimalDigits: 2,
      locale: Localizations.localeOf(context).toString(),
    );

    return Row(
      children: <Widget>[
        const SizedBox(width: _leftColumnWidth),
        Expanded(
          child: Padding(
            padding: const EdgeInsets.only(right: 16.0),
            child: Column(
              children: <Widget>[
                Row(
                  children: <Widget>[
                    const Expanded(child: Text('TOTAL')),
                    Text(formatter.format(model!.totalCost), style: largeAmountStyle),
                  ],
                ),
                const SizedBox(height: 16.0),
                Row(
                  children: <Widget>[
                    const Expanded(child: Text('Subtotal:')),
                    Text(formatter.format(model!.subtotalCost), style: smallAmountStyle),
                  ],
                ),
                const SizedBox(height: 4.0),
                Row(
                  children: <Widget>[
                    const Expanded(child: Text('Shipping:')),
                    Text(formatter.format(model!.shippingCost), style: smallAmountStyle),
                  ],
                ),
                const SizedBox(height: 4.0),
                Row(
                  children: <Widget>[
                    const Expanded(child: Text('Tax:')),
                    Text(formatter.format(model!.tax), style: smallAmountStyle),
                  ],
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

    return Padding(
      padding: const EdgeInsets.only(bottom: 16.0),
      child: Row(
        key: ValueKey<int>(product.id),
        crossAxisAlignment: CrossAxisAlignment.start,
        children: <Widget>[
          SizedBox(
            width: _leftColumnWidth,
            child: IconButton(icon: const Icon(Icons.remove_circle_outline), onPressed: onPressed),
          ),
          Expanded(
            child: Padding(
              padding: const EdgeInsets.only(right: 16.0),
              child: Column(
                children: <Widget>[
                  Row(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: <Widget>[
                      Image.asset(
                        product.assetName,
                        package: product.assetPackage,
                        fit: BoxFit.cover,
                        width: 75.0,
                        height: 75.0,
                      ),
                      const SizedBox(width: 16.0),
                      Expanded(
                        child: Column(
                          crossAxisAlignment: CrossAxisAlignment.start,
                          children: <Widget>[
                            Row(
                              children: <Widget>[
                                Expanded(child: Text('Quantity: $quantity')),
                                Text('x ${formatter.format(product.price)}'),
                              ],
                            ),
                            Text(
                              product.name,
                              style: localTheme.textTheme.titleMedium!.copyWith(
                                fontWeight: FontWeight.w600,
                              ),
                            ),
                          ],
                        ),
                      ),
                    ],
                  ),
                  const SizedBox(height: 16.0),
                  const Divider(color: kShrineBrown900, height: 10.0),
                ],
              ),
            ),
          ),
        ],
      ),
    );
  }
}
