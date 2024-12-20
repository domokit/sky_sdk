// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui' as ui;

import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

import '../common.dart';

const List<String> assets = <String>[
  'packages/flutter_gallery_assets/people/ali_landscape.png',
  'packages/flutter_gallery_assets/monochrome/red-square-1024x1024.png',
  'packages/flutter_gallery_assets/logos/flutter_white/logo.png',
  'packages/flutter_gallery_assets/logos/fortnightly/fortnightly_logo.png',
  'packages/flutter_gallery_assets/videos/bee.mp4',
  'packages/flutter_gallery_assets/videos/butterfly.mp4',
  'packages/flutter_gallery_assets/animated_images/animated_flutter_lgtm.gif',
  'packages/flutter_gallery_assets/animated_images/animated_flutter_stickers.webp',
  'packages/flutter_gallery_assets/food/butternut_squash_soup.png',
  'packages/flutter_gallery_assets/food/cherry_pie.png',
  'packages/flutter_gallery_assets/food/chopped_beet_leaves.png',
  'packages/flutter_gallery_assets/food/fruits.png',
  'packages/flutter_gallery_assets/food/pesto_pasta.png',
  'packages/flutter_gallery_assets/food/roasted_chicken.png',
  'packages/flutter_gallery_assets/food/spanakopita.png',
  'packages/flutter_gallery_assets/food/spinach_onion_salad.png',
  'packages/flutter_gallery_assets/food/icons/fish.png',
  'packages/flutter_gallery_assets/food/icons/healthy.png',
  'packages/flutter_gallery_assets/food/icons/main.png',
  'packages/flutter_gallery_assets/food/icons/meat.png',
  'packages/flutter_gallery_assets/food/icons/quick.png',
  'packages/flutter_gallery_assets/food/icons/spicy.png',
  'packages/flutter_gallery_assets/food/icons/veggie.png',
  'packages/flutter_gallery_assets/logos/pesto/logo_small.png',
  'packages/flutter_gallery_assets/places/india_chennai_flower_market.png',
  'packages/flutter_gallery_assets/places/india_thanjavur_market.png',
  'packages/flutter_gallery_assets/places/india_tanjore_bronze_works.png',
  'packages/flutter_gallery_assets/places/india_tanjore_market_merchant.png',
  'packages/flutter_gallery_assets/places/india_tanjore_thanjavur_temple.png',
  'packages/flutter_gallery_assets/places/india_pondicherry_salt_farm.png',
  'packages/flutter_gallery_assets/places/india_chennai_highway.png',
  'packages/flutter_gallery_assets/places/india_chettinad_silk_maker.png',
  'packages/flutter_gallery_assets/places/india_tanjore_thanjavur_temple_carvings.png',
  'packages/flutter_gallery_assets/places/india_chettinad_produce.png',
  'packages/flutter_gallery_assets/places/india_tanjore_market_technology.png',
  'packages/flutter_gallery_assets/places/india_pondicherry_beach.png',
  'packages/flutter_gallery_assets/places/india_pondicherry_fisherman.png',
  'packages/flutter_gallery_assets/products/backpack.png',
  'packages/flutter_gallery_assets/products/belt.png',
  'packages/flutter_gallery_assets/products/cup.png',
  'packages/flutter_gallery_assets/products/deskset.png',
  'packages/flutter_gallery_assets/products/dress.png',
  'packages/flutter_gallery_assets/products/earrings.png',
  'packages/flutter_gallery_assets/products/flatwear.png',
  'packages/flutter_gallery_assets/products/hat.png',
  'packages/flutter_gallery_assets/products/jacket.png',
  'packages/flutter_gallery_assets/products/jumper.png',
  'packages/flutter_gallery_assets/products/kitchen_quattro.png',
  'packages/flutter_gallery_assets/products/napkins.png',
  'packages/flutter_gallery_assets/products/planters.png',
  'packages/flutter_gallery_assets/products/platter.png',
  'packages/flutter_gallery_assets/products/scarf.png',
  'packages/flutter_gallery_assets/products/shirt.png',
  'packages/flutter_gallery_assets/products/sunnies.png',
  'packages/flutter_gallery_assets/products/sweater.png',
  'packages/flutter_gallery_assets/products/sweats.png',
  'packages/flutter_gallery_assets/products/table.png',
  'packages/flutter_gallery_assets/products/teaset.png',
  'packages/flutter_gallery_assets/products/top.png',
  'packages/flutter_gallery_assets/people/square/ali.png',
  'packages/flutter_gallery_assets/people/square/peter.png',
  'packages/flutter_gallery_assets/people/square/sandra.png',
  'packages/flutter_gallery_assets/people/square/stella.png',
  'packages/flutter_gallery_assets/people/square/trevor.png',
];

// Measures the time it takes to load a fixed number of assets into an
// immutable buffer to later be decoded by skia.
Future<void> execute() async {
  assert(false, "Don't run benchmarks in debug mode! Use 'flutter run --release'.");

  final Stopwatch watch = Stopwatch();
  await benchmarkWidgets((WidgetTester tester) async {
    watch.start();
    for (int i = 0; i < 10; i += 1) {
      await Future.wait(<Future<ui.ImmutableBuffer>>[
        for (final String asset in assets) rootBundle.loadBuffer(asset),
      ]);
    }
    watch.stop();
  });

  final BenchmarkResultPrinter printer = BenchmarkResultPrinter();
  printer.addResult(
    description: 'Image loading',
    value: watch.elapsedMilliseconds.toDouble(),
    unit: 'ms',
    name: 'image_load_ms',
  );
  printer.printToStdout();
}
