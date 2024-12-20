// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import '../../gallery_localizations.dart';
import 'material_demo_types.dart';

// BEGIN gridListsDemo

class GridListDemo extends StatelessWidget {
  const GridListDemo({super.key, required this.type});

  final GridListDemoType type;

  List<_Photo> _photos(BuildContext context) {
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;
    return <_Photo>[
      _Photo(
        assetName: 'places/india_chennai_flower_market.png',
        title: localizations.placeChennai,
        subtitle: localizations.placeFlowerMarket,
      ),
      _Photo(
        assetName: 'places/india_tanjore_bronze_works.png',
        title: localizations.placeTanjore,
        subtitle: localizations.placeBronzeWorks,
      ),
      _Photo(
        assetName: 'places/india_tanjore_market_merchant.png',
        title: localizations.placeTanjore,
        subtitle: localizations.placeMarket,
      ),
      _Photo(
        assetName: 'places/india_tanjore_thanjavur_temple.png',
        title: localizations.placeTanjore,
        subtitle: localizations.placeThanjavurTemple,
      ),
      _Photo(
        assetName: 'places/india_tanjore_thanjavur_temple_carvings.png',
        title: localizations.placeTanjore,
        subtitle: localizations.placeThanjavurTemple,
      ),
      _Photo(
        assetName: 'places/india_pondicherry_salt_farm.png',
        title: localizations.placePondicherry,
        subtitle: localizations.placeSaltFarm,
      ),
      _Photo(
        assetName: 'places/india_chennai_highway.png',
        title: localizations.placeChennai,
        subtitle: localizations.placeScooters,
      ),
      _Photo(
        assetName: 'places/india_chettinad_silk_maker.png',
        title: localizations.placeChettinad,
        subtitle: localizations.placeSilkMaker,
      ),
      _Photo(
        assetName: 'places/india_chettinad_produce.png',
        title: localizations.placeChettinad,
        subtitle: localizations.placeLunchPrep,
      ),
      _Photo(
        assetName: 'places/india_tanjore_market_technology.png',
        title: localizations.placeTanjore,
        subtitle: localizations.placeMarket,
      ),
      _Photo(
        assetName: 'places/india_pondicherry_beach.png',
        title: localizations.placePondicherry,
        subtitle: localizations.placeBeach,
      ),
      _Photo(
        assetName: 'places/india_pondicherry_fisherman.png',
        title: localizations.placePondicherry,
        subtitle: localizations.placeFisherman,
      ),
    ];
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        automaticallyImplyLeading: false,
        title: Text(GalleryLocalizations.of(context)!.demoGridListsTitle),
      ),
      body: GridView.count(
        restorationId: 'grid_view_demo_grid_offset',
        crossAxisCount: 2,
        mainAxisSpacing: 8,
        crossAxisSpacing: 8,
        padding: const EdgeInsets.all(8),
        children:
            _photos(context).map<Widget>((_Photo photo) {
              return _GridDemoPhotoItem(photo: photo, tileStyle: type);
            }).toList(),
      ),
    );
  }
}

class _Photo {
  _Photo({required this.assetName, required this.title, required this.subtitle});

  final String assetName;
  final String title;
  final String subtitle;
}

/// Allow the text size to shrink to fit in the space
class _GridTitleText extends StatelessWidget {
  const _GridTitleText(this.text);

  final String text;

  @override
  Widget build(BuildContext context) {
    return FittedBox(
      fit: BoxFit.scaleDown,
      alignment: AlignmentDirectional.centerStart,
      child: Text(text),
    );
  }
}

class _GridDemoPhotoItem extends StatelessWidget {
  const _GridDemoPhotoItem({required this.photo, required this.tileStyle});

  final _Photo photo;
  final GridListDemoType tileStyle;

  @override
  Widget build(BuildContext context) {
    final Widget image = Semantics(
      label: '${photo.title} ${photo.subtitle}',
      child: Material(
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(4)),
        clipBehavior: Clip.antiAlias,
        child: Image.asset(photo.assetName, package: 'flutter_gallery_assets', fit: BoxFit.cover),
      ),
    );

    switch (tileStyle) {
      case GridListDemoType.imageOnly:
        return image;
      case GridListDemoType.header:
        return GridTile(
          header: Material(
            color: Colors.transparent,
            shape: const RoundedRectangleBorder(
              borderRadius: BorderRadius.vertical(top: Radius.circular(4)),
            ),
            clipBehavior: Clip.antiAlias,
            child: GridTileBar(title: _GridTitleText(photo.title), backgroundColor: Colors.black45),
          ),
          child: image,
        );
      case GridListDemoType.footer:
        return GridTile(
          footer: Material(
            color: Colors.transparent,
            shape: const RoundedRectangleBorder(
              borderRadius: BorderRadius.vertical(bottom: Radius.circular(4)),
            ),
            clipBehavior: Clip.antiAlias,
            child: GridTileBar(
              backgroundColor: Colors.black45,
              title: _GridTitleText(photo.title),
              subtitle: _GridTitleText(photo.subtitle),
            ),
          ),
          child: image,
        );
    }
  }
}

// END
