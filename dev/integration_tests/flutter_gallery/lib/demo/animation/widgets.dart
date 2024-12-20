// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

import 'sections.dart';

const double kSectionIndicatorWidth = 32.0;

// The card for a single section. Displays the section's gradient and background image.
class SectionCard extends StatelessWidget {
  const SectionCard({super.key, required this.section});

  final Section section;

  @override
  Widget build(BuildContext context) {
    return Semantics(
      label: section.title,
      button: true,
      child: DecoratedBox(
        decoration: BoxDecoration(
          gradient: LinearGradient(colors: <Color>[section.leftColor!, section.rightColor!]),
        ),
        child: Image.asset(
          section.backgroundAsset!,
          package: section.backgroundAssetPackage,
          color: const Color.fromRGBO(255, 255, 255, 0.075),
          colorBlendMode: BlendMode.modulate,
          fit: BoxFit.cover,
        ),
      ),
    );
  }
}

// The title is rendered with two overlapping text widgets that are vertically
// offset a little. It's supposed to look sort-of 3D.
class SectionTitle extends StatelessWidget {
  const SectionTitle({super.key, required this.section, required this.scale, required this.opacity})
    : assert(opacity >= 0.0 && opacity <= 1.0);

  final Section section;
  final double scale;
  final double opacity;

  static const TextStyle sectionTitleStyle = TextStyle(
    fontFamily: 'Raleway',
    inherit: false,
    fontSize: 24.0,
    fontWeight: FontWeight.w500,
    color: Colors.white,
    textBaseline: TextBaseline.alphabetic,
  );

  static final TextStyle sectionTitleShadowStyle = sectionTitleStyle.copyWith(
    color: const Color(0x19000000),
  );

  @override
  Widget build(BuildContext context) {
    return IgnorePointer(
      child: Opacity(
        opacity: opacity,
        child: Transform(
          transform: Matrix4.identity()..scale(scale),
          alignment: Alignment.center,
          child: Stack(
            children: <Widget>[
              Positioned(top: 4.0, child: Text(section.title!, style: sectionTitleShadowStyle)),
              Text(section.title!, style: sectionTitleStyle),
            ],
          ),
        ),
      ),
    );
  }
}

// Small horizontal bar that indicates the selected section.
class SectionIndicator extends StatelessWidget {
  const SectionIndicator({super.key, this.opacity = 1.0});

  final double opacity;

  @override
  Widget build(BuildContext context) {
    return IgnorePointer(
      child: Container(
        width: kSectionIndicatorWidth,
        height: 3.0,
        color: Colors.white.withOpacity(opacity),
      ),
    );
  }
}

// Display a single SectionDetail.
class SectionDetailView extends StatelessWidget {
  SectionDetailView({super.key, required this.detail})
    : assert(detail.imageAsset != null),
      assert((detail.imageAsset ?? detail.title) != null);

  final SectionDetail detail;

  @override
  Widget build(BuildContext context) {
    final Widget image = DecoratedBox(
      decoration: BoxDecoration(
        borderRadius: BorderRadius.circular(6.0),
        image: DecorationImage(
          image: AssetImage(detail.imageAsset!, package: detail.imageAssetPackage),
          fit: BoxFit.cover,
        ),
      ),
    );

    Widget item;
    if (detail.title == null && detail.subtitle == null) {
      item = Container(
        height: 240.0,
        padding: const EdgeInsets.all(16.0),
        child: SafeArea(top: false, bottom: false, child: image),
      );
    } else {
      item = ListTile(
        title: Text(detail.title!),
        subtitle: Text(detail.subtitle!),
        leading: SizedBox(width: 32.0, height: 32.0, child: image),
      );
    }

    return DecoratedBox(decoration: BoxDecoration(color: Colors.grey.shade200), child: item);
  }
}
