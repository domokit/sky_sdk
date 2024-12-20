// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';

import 'package:flutter/material.dart';
import 'package:google_mobile_ads/google_mobile_ads.dart';

void main() {
  runApp(const PlatformViewApp());
}

class PlatformViewApp extends StatefulWidget {
  const PlatformViewApp({super.key});

  @override
  PlatformViewAppState createState() => PlatformViewAppState();
}

class PlatformViewAppState extends State<PlatformViewApp> {
  AdWidget _getBannerWidget() {
    // Test IDs from Admob:
    // https://developers.google.com/admob/ios/test-ads
    // https://developers.google.com/admob/android/test-ads
    final String bannerId =
        Platform.isAndroid
            ? 'ca-app-pub-3940256099942544/6300978111'
            : 'ca-app-pub-3940256099942544/2934735716';
    final BannerAd bannerAd = BannerAd(
      adUnitId: bannerId,
      request: const AdRequest(),
      size: AdSize.banner,
      listener: const BannerAdListener(),
    );
    bannerAd.load();
    return AdWidget(ad: bannerAd);
  }

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      theme: ThemeData.light(),
      title: 'Advanced Layout',
      home: Scaffold(
        appBar: AppBar(title: const Text('Platform View Ad Banners')),
        body: ListView.builder(
          key: const Key('platform-views-scroll'), // This key is used by the driver test.
          itemCount: 250,
          itemBuilder: (BuildContext context, int index) {
            return index.isEven
                // Use 320x50 Admob standard banner size.
                ? SizedBox(width: 320, height: 50, child: _getBannerWidget())
                // Adjust the height to control number of platform views on screen.
                // TODO(hellohuanlin): Having more than 5 banners on screen causes an unknown crash.
                // See: https://github.com/flutter/flutter/issues/144339
                : const SizedBox(height: 150, child: ColoredBox(color: Colors.yellow));
          },
        ),
      ),
    );
  }
}
