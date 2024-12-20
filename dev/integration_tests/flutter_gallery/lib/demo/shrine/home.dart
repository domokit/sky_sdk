// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:scoped_model/scoped_model.dart';

import 'backdrop.dart';
import 'expanding_bottom_sheet.dart';
import 'model/app_state_model.dart';
import 'model/product.dart';
import 'supplemental/asymmetric_view.dart';

class ProductPage extends StatelessWidget {
  const ProductPage({super.key, this.category = Category.all});

  final Category category;

  @override
  Widget build(BuildContext context) {
    return ScopedModelDescendant<AppStateModel>(
      builder: (BuildContext context, Widget? child, AppStateModel model) {
        return AsymmetricView(products: model.getProducts());
      },
    );
  }
}

class HomePage extends StatelessWidget {
  const HomePage({this.expandingBottomSheet, this.backdrop, super.key});

  final ExpandingBottomSheet? expandingBottomSheet;
  final Backdrop? backdrop;

  @override
  Widget build(BuildContext context) {
    return Stack(
      children: <Widget>[
        if (backdrop != null) backdrop!,
        Align(alignment: Alignment.bottomRight, child: expandingBottomSheet),
      ],
    );
  }
}
