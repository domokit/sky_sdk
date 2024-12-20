// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import '../../gallery_localizations.dart';

const String _kGalleryAssetsPackage = 'flutter_gallery_assets';

// BEGIN cardsDemo

enum CardType { standard, tappable, selectable }

class TravelDestination {
  const TravelDestination({
    required this.assetName,
    required this.assetPackage,
    required this.title,
    required this.description,
    required this.city,
    required this.location,
    this.cardType = CardType.standard,
  });

  final String assetName;
  final String assetPackage;
  final String title;
  final String description;
  final String city;
  final String location;
  final CardType cardType;
}

List<TravelDestination> destinations(BuildContext context) {
  final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;

  return <TravelDestination>[
    TravelDestination(
      assetName: 'places/india_thanjavur_market.png',
      assetPackage: _kGalleryAssetsPackage,
      title: localizations.cardsDemoTravelDestinationTitle1,
      description: localizations.cardsDemoTravelDestinationDescription1,
      city: localizations.cardsDemoTravelDestinationCity1,
      location: localizations.cardsDemoTravelDestinationLocation1,
    ),
    TravelDestination(
      assetName: 'places/india_chettinad_silk_maker.png',
      assetPackage: _kGalleryAssetsPackage,
      title: localizations.cardsDemoTravelDestinationTitle2,
      description: localizations.cardsDemoTravelDestinationDescription2,
      city: localizations.cardsDemoTravelDestinationCity2,
      location: localizations.cardsDemoTravelDestinationLocation2,
      cardType: CardType.tappable,
    ),
    TravelDestination(
      assetName: 'places/india_tanjore_thanjavur_temple.png',
      assetPackage: _kGalleryAssetsPackage,
      title: localizations.cardsDemoTravelDestinationTitle3,
      description: localizations.cardsDemoTravelDestinationDescription3,
      city: localizations.cardsDemoTravelDestinationCity1,
      location: localizations.cardsDemoTravelDestinationLocation1,
      cardType: CardType.selectable,
    ),
  ];
}

class TravelDestinationItem extends StatelessWidget {
  const TravelDestinationItem({super.key, required this.destination, this.shape});

  // This height will allow for all the Card's content to fit comfortably within the card.
  static const double height = 360.0;
  final TravelDestination destination;
  final ShapeBorder? shape;

  @override
  Widget build(BuildContext context) {
    return SafeArea(
      top: false,
      bottom: false,
      child: Padding(
        padding: const EdgeInsets.all(8),
        child: Column(
          children: <Widget>[
            SectionTitle(title: GalleryLocalizations.of(context)!.settingsTextScalingNormal),
            SizedBox(
              height: height,
              child: Card(
                // This ensures that the Card's children are clipped correctly.
                clipBehavior: Clip.antiAlias,
                shape: shape,
                child: Semantics(
                  label: destination.title,
                  child: TravelDestinationContent(destination: destination),
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }
}

class TappableTravelDestinationItem extends StatelessWidget {
  const TappableTravelDestinationItem({super.key, required this.destination, this.shape});

  // This height will allow for all the Card's content to fit comfortably within the card.
  static const double height = 298.0;
  final TravelDestination destination;
  final ShapeBorder? shape;

  @override
  Widget build(BuildContext context) {
    return SafeArea(
      top: false,
      bottom: false,
      child: Padding(
        padding: const EdgeInsets.all(8),
        child: Column(
          children: <Widget>[
            SectionTitle(title: GalleryLocalizations.of(context)!.cardsDemoTappable),
            SizedBox(
              height: height,
              child: Card(
                // This ensures that the Card's children (including the ink splash) are clipped correctly.
                clipBehavior: Clip.antiAlias,
                shape: shape,
                child: InkWell(
                  onTap: () {},
                  // Generally, material cards use onSurface with 12% opacity for the pressed state.
                  splashColor: Theme.of(context).colorScheme.onSurface.withOpacity(0.12),
                  // Generally, material cards do not have a highlight overlay.
                  highlightColor: Colors.transparent,
                  child: Semantics(
                    label: destination.title,
                    child: TravelDestinationContent(destination: destination),
                  ),
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }
}

class SelectableTravelDestinationItem extends StatelessWidget {
  const SelectableTravelDestinationItem({
    super.key,
    required this.destination,
    required this.isSelected,
    required this.onSelected,
    this.shape,
  });

  final TravelDestination destination;
  final ShapeBorder? shape;
  final bool isSelected;
  final VoidCallback onSelected;

  // This height will allow for all the Card's content to fit comfortably within the card.
  static const double height = 298.0;

  @override
  Widget build(BuildContext context) {
    final ColorScheme colorScheme = Theme.of(context).colorScheme;
    final String selectedStatus =
        isSelected
            ? GalleryLocalizations.of(context)!.selected
            : GalleryLocalizations.of(context)!.notSelected;

    return SafeArea(
      top: false,
      bottom: false,
      child: Padding(
        padding: const EdgeInsets.all(8),
        child: Column(
          children: <Widget>[
            SectionTitle(title: GalleryLocalizations.of(context)!.selectable),
            SizedBox(
              height: height,
              child: Card(
                // This ensures that the Card's children (including the ink splash) are clipped correctly.
                clipBehavior: Clip.antiAlias,
                shape: shape,
                child: InkWell(
                  onLongPress: () {
                    onSelected();
                  },
                  // Generally, material cards use onSurface with 12% opacity for the pressed state.
                  splashColor: colorScheme.onSurface.withOpacity(0.12),
                  // Generally, material cards do not have a highlight overlay.
                  highlightColor: Colors.transparent,
                  child: Stack(
                    children: <Widget>[
                      Container(
                        color:
                            isSelected
                                // Generally, material cards use primary with 8% opacity for the selected state.
                                // See: https://material.io/design/interaction/states.html#anatomy
                                ? colorScheme.primary.withOpacity(0.08)
                                : Colors.transparent,
                      ),
                      Semantics(
                        label: '${destination.title}, $selectedStatus',
                        onLongPressHint:
                            isSelected
                                ? GalleryLocalizations.of(context)!.deselect
                                : GalleryLocalizations.of(context)!.select,
                        child: TravelDestinationContent(destination: destination),
                      ),
                      Align(
                        alignment: Alignment.topRight,
                        child: Padding(
                          padding: const EdgeInsets.all(8),
                          child: Icon(
                            Icons.check_circle,
                            color: isSelected ? colorScheme.primary : Colors.transparent,
                          ),
                        ),
                      ),
                    ],
                  ),
                  //),
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }
}

class SectionTitle extends StatelessWidget {
  const SectionTitle({super.key, required this.title});

  final String title;

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.fromLTRB(4, 4, 4, 12),
      child: Align(
        alignment: Alignment.centerLeft,
        child: Text(title, style: Theme.of(context).textTheme.titleMedium),
      ),
    );
  }
}

class TravelDestinationContent extends StatelessWidget {
  const TravelDestinationContent({super.key, required this.destination});

  final TravelDestination destination;

  @override
  Widget build(BuildContext context) {
    final ThemeData theme = Theme.of(context);
    final TextStyle titleStyle = theme.textTheme.headlineSmall!.copyWith(color: Colors.white);
    final TextStyle descriptionStyle = theme.textTheme.titleMedium!;
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;

    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: <Widget>[
        SizedBox(
          height: 184,
          child: Stack(
            children: <Widget>[
              Positioned.fill(
                // In order to have the ink splash appear above the image, you
                // must use Ink.image. This allows the image to be painted as
                // part of the Material and display ink effects above it. Using
                // a standard Image will obscure the ink splash.
                child: Ink.image(
                  image: AssetImage(destination.assetName, package: destination.assetPackage),
                  fit: BoxFit.cover,
                  child: Container(),
                ),
              ),
              Positioned(
                bottom: 16,
                left: 16,
                right: 16,
                child: FittedBox(
                  fit: BoxFit.scaleDown,
                  alignment: Alignment.centerLeft,
                  child: Semantics(
                    container: true,
                    header: true,
                    child: Text(destination.title, style: titleStyle),
                  ),
                ),
              ),
            ],
          ),
        ),
        // Description and share/explore buttons.
        Semantics(
          container: true,
          child: Padding(
            padding: const EdgeInsets.fromLTRB(16, 16, 16, 0),
            child: DefaultTextStyle(
              softWrap: false,
              overflow: TextOverflow.ellipsis,
              style: descriptionStyle,
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: <Widget>[
                  // This array contains the three line description on each card
                  // demo.
                  Padding(
                    padding: const EdgeInsets.only(bottom: 8),
                    child: Text(
                      destination.description,
                      style: descriptionStyle.copyWith(color: Colors.black54),
                    ),
                  ),
                  Text(destination.city),
                  Text(destination.location),
                ],
              ),
            ),
          ),
        ),
        if (destination.cardType == CardType.standard)
          // share, explore buttons
          Padding(
            padding: const EdgeInsets.all(8),
            child: OverflowBar(
              alignment: MainAxisAlignment.start,
              spacing: 8,
              children: <Widget>[
                TextButton(
                  onPressed: () {},
                  child: Text(
                    localizations.demoMenuShare,
                    semanticsLabel: localizations.cardsDemoShareSemantics(destination.title),
                  ),
                ),
                TextButton(
                  onPressed: () {},
                  child: Text(
                    localizations.cardsDemoExplore,
                    semanticsLabel: localizations.cardsDemoExploreSemantics(destination.title),
                  ),
                ),
              ],
            ),
          ),
      ],
    );
  }
}

class CardsDemo extends StatefulWidget {
  const CardsDemo({super.key});

  @override
  State<CardsDemo> createState() => _CardsDemoState();
}

class _CardsDemoState extends State<CardsDemo> with RestorationMixin {
  final RestorableBool _isSelected = RestorableBool(false);

  @override
  String get restorationId => 'cards_demo';

  @override
  void restoreState(RestorationBucket? oldBucket, bool initialRestore) {
    registerForRestoration(_isSelected, 'is_selected');
  }

  @override
  void dispose() {
    _isSelected.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        automaticallyImplyLeading: false,
        title: Text(GalleryLocalizations.of(context)!.demoCardTitle),
      ),
      body: Scrollbar(
        child: ListView(
          restorationId: 'cards_demo_list_view',
          padding: const EdgeInsets.only(top: 8, left: 8, right: 8),
          children: <Widget>[
            for (final TravelDestination destination in destinations(context))
              Container(
                margin: const EdgeInsets.only(bottom: 8),
                child: switch (destination.cardType) {
                  CardType.standard => TravelDestinationItem(destination: destination),
                  CardType.tappable => TappableTravelDestinationItem(destination: destination),
                  CardType.selectable => SelectableTravelDestinationItem(
                    destination: destination,
                    isSelected: _isSelected.value,
                    onSelected:
                        () => setState(() {
                          _isSelected.value = !_isSelected.value;
                        }),
                  ),
                },
              ),
          ],
        ),
      ),
    );
  }
}

// END
