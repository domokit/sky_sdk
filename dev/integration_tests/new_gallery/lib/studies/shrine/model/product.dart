// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

import '../../../gallery_localizations.dart';

class Category {
  const Category({required this.name});

  // A function taking a BuildContext as input and
  // returns the internationalized name of the category.
  final String Function(BuildContext) name;
}

Category categoryAll = Category(
  name: (BuildContext context) => GalleryLocalizations.of(context)!.shrineCategoryNameAll,
);

Category categoryAccessories = Category(
  name: (BuildContext context) => GalleryLocalizations.of(context)!.shrineCategoryNameAccessories,
);

Category categoryClothing = Category(
  name: (BuildContext context) => GalleryLocalizations.of(context)!.shrineCategoryNameClothing,
);

Category categoryHome = Category(
  name: (BuildContext context) => GalleryLocalizations.of(context)!.shrineCategoryNameHome,
);

List<Category> categories = <Category>[
  categoryAll,
  categoryAccessories,
  categoryClothing,
  categoryHome,
];

class Product {
  const Product({
    required this.category,
    required this.id,
    required this.isFeatured,
    required this.name,
    required this.price,
    this.assetAspectRatio = 1,
  });

  final Category category;
  final int id;
  final bool isFeatured;
  final double assetAspectRatio;

  // A function taking a BuildContext as input and
  // returns the internationalized name of the product.
  final String Function(BuildContext) name;

  final int price;

  String get assetName => '$id-0.jpg';

  String get assetPackage => 'shrine_images';
}
