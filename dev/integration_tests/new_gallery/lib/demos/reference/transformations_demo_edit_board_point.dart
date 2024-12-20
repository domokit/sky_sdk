// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import '../../themes/gallery_theme_data.dart';
import 'transformations_demo_board.dart';
import 'transformations_demo_color_picker.dart';

const Color backgroundColor = Color(0xFF272727);

// The panel for editing a board point.
@immutable
class EditBoardPoint extends StatelessWidget {
  const EditBoardPoint({super.key, required this.boardPoint, this.onColorSelection});

  final BoardPoint boardPoint;
  final ValueChanged<Color>? onColorSelection;

  @override
  Widget build(BuildContext context) {
    final Set<Color> boardPointColors = <Color>{
      Colors.white,
      GalleryThemeData.darkColorScheme.primary,
      GalleryThemeData.darkColorScheme.primaryContainer,
      GalleryThemeData.darkColorScheme.secondary,
      backgroundColor,
    };

    return Column(
      crossAxisAlignment: CrossAxisAlignment.stretch,
      children: <Widget>[
        Text(
          '${boardPoint.q}, ${boardPoint.r}',
          textAlign: TextAlign.right,
          style: const TextStyle(fontWeight: FontWeight.bold),
        ),
        ColorPicker(
          colors: boardPointColors,
          selectedColor: boardPoint.color,
          onColorSelection: onColorSelection,
        ),
      ],
    );
  }
}
