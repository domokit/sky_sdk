// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import '../../gallery_localizations.dart';
import 'material_demo_types.dart';

class ChipDemo extends StatelessWidget {
  const ChipDemo({super.key, required this.type});

  final ChipDemoType type;

  String _title(BuildContext context) {
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;
    return switch (type) {
      ChipDemoType.action => localizations.demoActionChipTitle,
      ChipDemoType.choice => localizations.demoChoiceChipTitle,
      ChipDemoType.filter => localizations.demoFilterChipTitle,
      ChipDemoType.input => localizations.demoInputChipTitle,
    };
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(automaticallyImplyLeading: false, title: Text(_title(context))),
      body: switch (type) {
        ChipDemoType.action => _ActionChipDemo(),
        ChipDemoType.choice => _ChoiceChipDemo(),
        ChipDemoType.filter => _FilterChipDemo(),
        ChipDemoType.input => _InputChipDemo(),
      },
    );
  }
}

// BEGIN chipDemoAction

class _ActionChipDemo extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return Center(
      child: ActionChip(
        onPressed: () {},
        avatar: const Icon(Icons.brightness_5, color: Colors.black54),
        label: Text(GalleryLocalizations.of(context)!.chipTurnOnLights),
      ),
    );
  }
}

// END

// BEGIN chipDemoChoice

class _ChoiceChipDemo extends StatefulWidget {
  @override
  _ChoiceChipDemoState createState() => _ChoiceChipDemoState();
}

class _ChoiceChipDemoState extends State<_ChoiceChipDemo> with RestorationMixin {
  final RestorableIntN _indexSelected = RestorableIntN(null);

  @override
  String get restorationId => 'choice_chip_demo';

  @override
  void restoreState(RestorationBucket? oldBucket, bool initialRestore) {
    registerForRestoration(_indexSelected, 'choice_chip');
  }

  @override
  void dispose() {
    _indexSelected.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;
    return Center(
      child: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: <Widget>[
          Wrap(
            children: <Widget>[
              ChoiceChip(
                label: Text(localizations.chipSmall),
                selected: _indexSelected.value == 0,
                onSelected: (bool value) {
                  setState(() {
                    _indexSelected.value = value ? 0 : -1;
                  });
                },
              ),
              const SizedBox(width: 8),
              ChoiceChip(
                label: Text(localizations.chipMedium),
                selected: _indexSelected.value == 1,
                onSelected: (bool value) {
                  setState(() {
                    _indexSelected.value = value ? 1 : -1;
                  });
                },
              ),
              const SizedBox(width: 8),
              ChoiceChip(
                label: Text(localizations.chipLarge),
                selected: _indexSelected.value == 2,
                onSelected: (bool value) {
                  setState(() {
                    _indexSelected.value = value ? 2 : -1;
                  });
                },
              ),
            ],
          ),
          const SizedBox(height: 12),
          // Disabled chips
          Wrap(
            children: <Widget>[
              ChoiceChip(label: Text(localizations.chipSmall), selected: _indexSelected.value == 0),
              const SizedBox(width: 8),
              ChoiceChip(
                label: Text(localizations.chipMedium),
                selected: _indexSelected.value == 1,
              ),
              const SizedBox(width: 8),
              ChoiceChip(label: Text(localizations.chipLarge), selected: _indexSelected.value == 2),
            ],
          ),
        ],
      ),
    );
  }
}

// END

// BEGIN chipDemoFilter

class _FilterChipDemo extends StatefulWidget {
  @override
  _FilterChipDemoState createState() => _FilterChipDemoState();
}

class _FilterChipDemoState extends State<_FilterChipDemo> with RestorationMixin {
  final RestorableBool isSelectedElevator = RestorableBool(false);
  final RestorableBool isSelectedWasher = RestorableBool(false);
  final RestorableBool isSelectedFireplace = RestorableBool(false);

  @override
  String get restorationId => 'filter_chip_demo';

  @override
  void restoreState(RestorationBucket? oldBucket, bool initialRestore) {
    registerForRestoration(isSelectedElevator, 'selected_elevator');
    registerForRestoration(isSelectedWasher, 'selected_washer');
    registerForRestoration(isSelectedFireplace, 'selected_fireplace');
  }

  @override
  void dispose() {
    isSelectedElevator.dispose();
    isSelectedWasher.dispose();
    isSelectedFireplace.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;

    return Center(
      child: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: <Widget>[
          Wrap(
            spacing: 8.0,
            children: <Widget>[
              FilterChip(
                label: Text(localizations.chipElevator),
                selected: isSelectedElevator.value,
                onSelected: (bool value) {
                  setState(() {
                    isSelectedElevator.value = !isSelectedElevator.value;
                  });
                },
              ),
              FilterChip(
                label: Text(localizations.chipWasher),
                selected: isSelectedWasher.value,
                onSelected: (bool value) {
                  setState(() {
                    isSelectedWasher.value = !isSelectedWasher.value;
                  });
                },
              ),
              FilterChip(
                label: Text(localizations.chipFireplace),
                selected: isSelectedFireplace.value,
                onSelected: (bool value) {
                  setState(() {
                    isSelectedFireplace.value = !isSelectedFireplace.value;
                  });
                },
              ),
            ],
          ),
          const SizedBox(height: 12),
          // Disabled chips
          Wrap(
            spacing: 8.0,
            children: <Widget>[
              FilterChip(
                label: Text(localizations.chipElevator),
                selected: isSelectedElevator.value,
                onSelected: null,
              ),
              FilterChip(
                label: Text(localizations.chipWasher),
                selected: isSelectedWasher.value,
                onSelected: null,
              ),
              FilterChip(
                label: Text(localizations.chipFireplace),
                selected: isSelectedFireplace.value,
                onSelected: null,
              ),
            ],
          ),
        ],
      ),
    );
  }
}

// END

// BEGIN chipDemoInput

class _InputChipDemo extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return Center(
      child: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: <Widget>[
          InputChip(
            onPressed: () {},
            onDeleted: () {},
            avatar: const Icon(Icons.directions_bike, size: 20, color: Colors.black54),
            deleteIconColor: Colors.black54,
            label: Text(GalleryLocalizations.of(context)!.chipBiking),
          ),
          const SizedBox(height: 12),
          // Disabled chip
          InputChip(
            avatar: const Icon(Icons.directions_bike, size: 20, color: Colors.black54),
            deleteIconColor: Colors.black54,
            label: Text(GalleryLocalizations.of(context)!.chipBiking),
          ),
        ],
      ),
    );
  }
}

// END
