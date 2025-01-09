// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

import '../../gallery/demo.dart';

enum GridDemoTileStyle { imageOnly, oneLine, twoLine }

typedef BannerTapCallback = void Function(Photo photo);

const double _kMinFlingVelocity = 800.0;
const String _kGalleryAssetsPackage = 'flutter_gallery_assets';

class Photo {
  Photo({this.assetName, this.assetPackage, this.title, this.caption, this.isFavorite = false});

  final String? assetName;
  final String? assetPackage;
  final String? title;
  final String? caption;

  bool isFavorite;
  String? get tag => assetName; // Assuming that all asset names are unique.

  bool get isValid => assetName != null && title != null && caption != null;
}

class GridPhotoViewer extends StatefulWidget {
  const GridPhotoViewer({super.key, this.photo});

  final Photo? photo;

  @override
  State<GridPhotoViewer> createState() => _GridPhotoViewerState();
}

class _GridTitleText extends StatelessWidget {
  const _GridTitleText(this.text);

  final String? text;

  @override
  Widget build(BuildContext context) {
    return FittedBox(fit: BoxFit.scaleDown, alignment: Alignment.centerLeft, child: Text(text!));
  }
}

class _GridPhotoViewerState extends State<GridPhotoViewer> with SingleTickerProviderStateMixin {
  late AnimationController _controller;
  late Animation<Offset> _flingAnimation;
  Offset _offset = Offset.zero;
  double _scale = 1.0;
  late Offset _normalizedOffset;
  late double _previousScale;

  @override
  void initState() {
    super.initState();
    _controller = AnimationController(vsync: this)..addListener(_handleFlingAnimation);
  }

  @override
  void dispose() {
    _controller.dispose();
    super.dispose();
  }

  // The maximum offset value is 0,0. If the size of this renderer's box is w,h
  // then the minimum offset value is w - _scale * w, h - _scale * h.
  Offset _clampOffset(Offset offset) {
    final Size size = context.size!;
    final Offset minOffset = Offset(size.width, size.height) * (1.0 - _scale);
    return Offset(offset.dx.clamp(minOffset.dx, 0.0), offset.dy.clamp(minOffset.dy, 0.0));
  }

  void _handleFlingAnimation() {
    setState(() {
      _offset = _flingAnimation.value;
    });
  }

  void _handleOnScaleStart(ScaleStartDetails details) {
    setState(() {
      _previousScale = _scale;
      _normalizedOffset = (details.focalPoint - _offset) / _scale;
      // The fling animation stops if an input gesture starts.
      _controller.stop();
    });
  }

  void _handleOnScaleUpdate(ScaleUpdateDetails details) {
    setState(() {
      _scale = (_previousScale * details.scale).clamp(1.0, 4.0);
      // Ensure that image location under the focal point stays in the same place despite scaling.
      _offset = _clampOffset(details.focalPoint - _normalizedOffset * _scale);
    });
  }

  void _handleOnScaleEnd(ScaleEndDetails details) {
    final double magnitude = details.velocity.pixelsPerSecond.distance;
    if (magnitude < _kMinFlingVelocity) {
      return;
    }
    final Offset direction = details.velocity.pixelsPerSecond / magnitude;
    final double distance = (Offset.zero & context.size!).shortestSide;
    _flingAnimation = _controller.drive(
      Tween<Offset>(begin: _offset, end: _clampOffset(_offset + direction * distance)),
    );
    _controller
      ..value = 0.0
      ..fling(velocity: magnitude / 1000.0);
  }

  @override
  Widget build(BuildContext context) {
    return GestureDetector(
      onScaleStart: _handleOnScaleStart,
      onScaleUpdate: _handleOnScaleUpdate,
      onScaleEnd: _handleOnScaleEnd,
      child: ClipRect(
        child: Transform(
          transform:
              Matrix4.identity()
                ..translate(_offset.dx, _offset.dy)
                ..scale(_scale),
          child: Image.asset(
            widget.photo!.assetName!,
            package: widget.photo!.assetPackage,
            fit: BoxFit.cover,
          ),
        ),
      ),
    );
  }
}

class GridDemoPhotoItem extends StatelessWidget {
  GridDemoPhotoItem({
    super.key,
    required this.photo,
    required this.tileStyle,
    required this.onBannerTap,
  }) : assert(photo.isValid);

  final Photo photo;
  final GridDemoTileStyle tileStyle;
  final BannerTapCallback onBannerTap; // User taps on the photo's header or footer.

  void showPhoto(BuildContext context) {
    Navigator.push(
      context,
      MaterialPageRoute<void>(
        builder: (BuildContext context) {
          return Scaffold(
            appBar: AppBar(title: Text(photo.title!)),
            body: SizedBox.expand(
              child: Hero(tag: photo.tag!, child: GridPhotoViewer(photo: photo)),
            ),
          );
        },
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    final Widget image = Semantics(
      label: '${photo.title} - ${photo.caption}',
      child: GestureDetector(
        onTap: () {
          showPhoto(context);
        },
        child: Hero(
          key: Key(photo.assetName!),
          tag: photo.tag!,
          child: Image.asset(photo.assetName!, package: photo.assetPackage, fit: BoxFit.cover),
        ),
      ),
    );

    final IconData icon = photo.isFavorite ? Icons.star : Icons.star_border;

    switch (tileStyle) {
      case GridDemoTileStyle.imageOnly:
        return image;

      case GridDemoTileStyle.oneLine:
        return GridTile(
          header: GestureDetector(
            onTap: () {
              onBannerTap(photo);
            },
            child: GridTileBar(
              title: _GridTitleText(photo.title),
              backgroundColor: Colors.black45,
              leading: Icon(icon, color: Colors.white),
            ),
          ),
          child: image,
        );

      case GridDemoTileStyle.twoLine:
        return GridTile(
          footer: GestureDetector(
            onTap: () {
              onBannerTap(photo);
            },
            child: GridTileBar(
              backgroundColor: Colors.black45,
              title: _GridTitleText(photo.title),
              subtitle: _GridTitleText(photo.caption),
              trailing: Icon(icon, color: Colors.white),
            ),
          ),
          child: image,
        );
    }
  }
}

class GridListDemo extends StatefulWidget {
  const GridListDemo({super.key});

  static const String routeName = '/material/grid-list';

  @override
  GridListDemoState createState() => GridListDemoState();
}

class GridListDemoState extends State<GridListDemo> {
  GridDemoTileStyle _tileStyle = GridDemoTileStyle.twoLine;

  List<Photo> photos = <Photo>[
    Photo(
      assetName: 'places/india_chennai_flower_market.png',
      assetPackage: _kGalleryAssetsPackage,
      title: 'Chennai',
      caption: 'Flower Market',
    ),
    Photo(
      assetName: 'places/india_tanjore_bronze_works.png',
      assetPackage: _kGalleryAssetsPackage,
      title: 'Tanjore',
      caption: 'Bronze Works',
    ),
    Photo(
      assetName: 'places/india_tanjore_market_merchant.png',
      assetPackage: _kGalleryAssetsPackage,
      title: 'Tanjore',
      caption: 'Market',
    ),
    Photo(
      assetName: 'places/india_tanjore_thanjavur_temple.png',
      assetPackage: _kGalleryAssetsPackage,
      title: 'Tanjore',
      caption: 'Thanjavur Temple',
    ),
    Photo(
      assetName: 'places/india_tanjore_thanjavur_temple_carvings.png',
      assetPackage: _kGalleryAssetsPackage,
      title: 'Tanjore',
      caption: 'Thanjavur Temple',
    ),
    Photo(
      assetName: 'places/india_pondicherry_salt_farm.png',
      assetPackage: _kGalleryAssetsPackage,
      title: 'Pondicherry',
      caption: 'Salt Farm',
    ),
    Photo(
      assetName: 'places/india_chennai_highway.png',
      assetPackage: _kGalleryAssetsPackage,
      title: 'Chennai',
      caption: 'Scooters',
    ),
    Photo(
      assetName: 'places/india_chettinad_silk_maker.png',
      assetPackage: _kGalleryAssetsPackage,
      title: 'Chettinad',
      caption: 'Silk Maker',
    ),
    Photo(
      assetName: 'places/india_chettinad_produce.png',
      assetPackage: _kGalleryAssetsPackage,
      title: 'Chettinad',
      caption: 'Lunch Prep',
    ),
    Photo(
      assetName: 'places/india_tanjore_market_technology.png',
      assetPackage: _kGalleryAssetsPackage,
      title: 'Tanjore',
      caption: 'Market',
    ),
    Photo(
      assetName: 'places/india_pondicherry_beach.png',
      assetPackage: _kGalleryAssetsPackage,
      title: 'Pondicherry',
      caption: 'Beach',
    ),
    Photo(
      assetName: 'places/india_pondicherry_fisherman.png',
      assetPackage: _kGalleryAssetsPackage,
      title: 'Pondicherry',
      caption: 'Fisherman',
    ),
  ];

  void changeTileStyle(GridDemoTileStyle value) {
    setState(() {
      _tileStyle = value;
    });
  }

  @override
  Widget build(BuildContext context) {
    final Orientation orientation = MediaQuery.of(context).orientation;
    return Scaffold(
      appBar: AppBar(
        title: const Text('Grid list'),
        actions: <Widget>[
          MaterialDemoDocumentationButton(GridListDemo.routeName),
          PopupMenuButton<GridDemoTileStyle>(
            onSelected: changeTileStyle,
            itemBuilder:
                (BuildContext context) => <PopupMenuItem<GridDemoTileStyle>>[
                  const PopupMenuItem<GridDemoTileStyle>(
                    value: GridDemoTileStyle.imageOnly,
                    child: Text('Image only'),
                  ),
                  const PopupMenuItem<GridDemoTileStyle>(
                    value: GridDemoTileStyle.oneLine,
                    child: Text('One line'),
                  ),
                  const PopupMenuItem<GridDemoTileStyle>(
                    value: GridDemoTileStyle.twoLine,
                    child: Text('Two line'),
                  ),
                ],
          ),
        ],
      ),
      body: Column(
        children: <Widget>[
          Expanded(
            child: SafeArea(
              top: false,
              bottom: false,
              child: GridView.count(
                crossAxisCount: (orientation == Orientation.portrait) ? 2 : 3,
                mainAxisSpacing: 4.0,
                crossAxisSpacing: 4.0,
                padding: const EdgeInsets.all(4.0),
                childAspectRatio: (orientation == Orientation.portrait) ? 1.0 : 1.3,
                children:
                    photos.map<Widget>((Photo photo) {
                      return GridDemoPhotoItem(
                        photo: photo,
                        tileStyle: _tileStyle,
                        onBannerTap: (Photo photo) {
                          setState(() {
                            photo.isFavorite = !photo.isFavorite;
                          });
                        },
                      );
                    }).toList(),
              ),
            ),
          ),
        ],
      ),
    );
  }
}
