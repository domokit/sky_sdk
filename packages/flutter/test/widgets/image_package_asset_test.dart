// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('AssetImage from package', () {
    const AssetImage image = AssetImage('assets/image.png', package: 'test_package');
    expect(image.keyName, 'packages/test_package/assets/image.png');
  });

  test('ExactAssetImage from package', () {
    const ExactAssetImage image = ExactAssetImage(
      'assets/image.png',
      scale: 1.5,
      package: 'test_package',
    );
    expect(image.keyName, 'packages/test_package/assets/image.png');
  });

  test('Image.asset from package', () {
    final Image imageWidget = Image.asset('assets/image.png', package: 'test_package');
    assert(imageWidget.image is AssetImage);
    final AssetImage assetImage = imageWidget.image as AssetImage;
    expect(assetImage.keyName, 'packages/test_package/assets/image.png');
  });

  test('Image.asset from package', () {
    final Image imageWidget = Image.asset('assets/image.png', scale: 1.5, package: 'test_package');
    assert(imageWidget.image is ExactAssetImage);
    final ExactAssetImage assetImage = imageWidget.image as ExactAssetImage;
    expect(assetImage.keyName, 'packages/test_package/assets/image.png');
  });
}
