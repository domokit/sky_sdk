// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

import '../model/product.dart';
import 'product_columns.dart';

class AsymmetricView extends StatelessWidget {
  const AsymmetricView({super.key, this.products});

  final List<Product>? products;

  List<SizedBox> _buildColumns(BuildContext context) {
    if (products == null || products!.isEmpty) {
      return const <SizedBox>[];
    }

    // This will return a list of columns. It will oscillate between the two
    // kinds of columns. Even cases of the index (0, 2, 4, etc) will be
    // TwoProductCardColumn and the odd cases will be OneProductCardColumn.
    //
    // Each pair of columns will advance us 3 products forward (2 + 1). That's
    // some kinda awkward math so we use _evenCasesIndex and _oddCasesIndex as
    // helpers for creating the index of the product list that will correspond
    // to the index of the list of columns.
    return List<SizedBox>.generate(_listItemCount(products!.length), (int index) {
      double width = .59 * MediaQuery.of(context).size.width;
      Widget column;
      if (index.isEven) {
        /// Even cases
        final int bottom = _evenCasesIndex(index);
        column = TwoProductCardColumn(
          bottom: products![bottom],
          top: products!.length - 1 >= bottom + 1 ? products![bottom + 1] : null,
        );
        width += 32.0;
      } else {
        /// Odd cases
        column = OneProductCardColumn(product: products![_oddCasesIndex(index)]);
      }
      return SizedBox(
        width: width,
        child: Padding(padding: const EdgeInsets.symmetric(horizontal: 16.0), child: column),
      );
    }).toList();
  }

  int _evenCasesIndex(int input) {
    // The operator ~/ is a cool one. It's the truncating division operator. It
    // divides the number and if there's a remainder / decimal, it cuts it off.
    // This is like dividing and then casting the result to int. Also, it's
    // functionally equivalent to floor() in this case.
    return input ~/ 2 * 3;
  }

  int _oddCasesIndex(int input) {
    assert(input > 0);
    return (input / 2).ceil() * 3 - 1;
  }

  int _listItemCount(int totalItems) {
    return (totalItems % 3 == 0) ? totalItems ~/ 3 * 2 : (totalItems / 3).ceil() * 2 - 1;
  }

  @override
  Widget build(BuildContext context) {
    return ListView(
      scrollDirection: Axis.horizontal,
      padding: const EdgeInsets.fromLTRB(0.0, 34.0, 16.0, 44.0),
      physics: const AlwaysScrollableScrollPhysics(),
      children: _buildColumns(context),
    );
  }
}
