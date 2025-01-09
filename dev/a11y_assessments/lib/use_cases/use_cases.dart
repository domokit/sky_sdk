// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';

import '../common/dynamic_title.dart';
import 'action_chip.dart';
import 'app_bar.dart';
import 'auto_complete.dart';
import 'badge.dart';
import 'card.dart';
import 'check_box_list_tile.dart';
import 'date_picker.dart';
import 'dialog.dart';
import 'drawer.dart';
import 'expansion_tile.dart';
import 'material_banner.dart';
import 'navigation_bar.dart';
import 'navigation_drawer.dart';
import 'navigation_rail.dart';
import 'radio_list_tile.dart';
import 'slider.dart';
import 'snack_bar.dart';
import 'switch_list_tile.dart';
import 'tab_bar_view.dart';
import 'text_button.dart';
import 'text_field.dart';
import 'text_field_password.dart';

abstract class UseCase {
  String get name;
  String get route;

  Widget buildWithTitle(BuildContext context) {
    return DynamicTitle(title: name, child: build(context));
  }

  Widget build(BuildContext context);
}

final List<UseCase> useCases = <UseCase>[
  CheckBoxListTile(),
  DialogUseCase(),
  SliderUseCase(),
  TextFieldUseCase(),
  TextFieldPasswordUseCase(),
  DatePickerUseCase(),
  AutoCompleteUseCase(),
  BadgeUseCase(),
  MaterialBannerUseCase(),
  NavigationBarUseCase(),
  TextButtonUseCase(),
  RadioListTileUseCase(),
  ActionChipUseCase(),
  SnackBarUseCase(),
  SwitchListTileUseCase(),
  ExpansionTileUseCase(),
  CardUseCase(),
  DrawerUseCase(),
  NavigationDrawerUseCase(),
  NavigationRailUseCase(),
  AppBarUseCase(),
  TabBarViewUseCase(),
];
