// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// ## Usage
//
// Run this program from the root of the git repository.
//
// ```
// dart dev/tools/gen_defaults/bin/gen_defaults.dart [-v]
// ```

import 'dart:convert';
import 'dart:io';

import 'package:args/args.dart';
import 'package:gen_defaults/action_chip_template.dart';
import 'package:gen_defaults/app_bar_template.dart';
import 'package:gen_defaults/badge_template.dart';
import 'package:gen_defaults/banner_template.dart';
import 'package:gen_defaults/bottom_app_bar_template.dart';
import 'package:gen_defaults/bottom_sheet_template.dart';
import 'package:gen_defaults/button_template.dart';
import 'package:gen_defaults/card_template.dart';
import 'package:gen_defaults/checkbox_template.dart';
import 'package:gen_defaults/chip_template.dart';
import 'package:gen_defaults/color_scheme_template.dart';
import 'package:gen_defaults/date_picker_template.dart';
import 'package:gen_defaults/dialog_template.dart';
import 'package:gen_defaults/divider_template.dart';
import 'package:gen_defaults/drawer_template.dart';
import 'package:gen_defaults/expansion_tile_template.dart';
import 'package:gen_defaults/fab_template.dart';
import 'package:gen_defaults/filter_chip_template.dart';
import 'package:gen_defaults/icon_button_template.dart';
import 'package:gen_defaults/input_chip_template.dart';
import 'package:gen_defaults/input_decorator_template.dart';
import 'package:gen_defaults/list_tile_template.dart';
import 'package:gen_defaults/menu_template.dart';
import 'package:gen_defaults/motion_template.dart';
import 'package:gen_defaults/navigation_bar_template.dart';
import 'package:gen_defaults/navigation_drawer_template.dart';
import 'package:gen_defaults/navigation_rail_template.dart';
import 'package:gen_defaults/popup_menu_template.dart';
import 'package:gen_defaults/progress_indicator_template.dart';
import 'package:gen_defaults/radio_template.dart';
import 'package:gen_defaults/range_slider_template.dart';
import 'package:gen_defaults/search_bar_template.dart';
import 'package:gen_defaults/search_view_template.dart';
import 'package:gen_defaults/segmented_button_template.dart';
import 'package:gen_defaults/slider_template.dart';
import 'package:gen_defaults/snackbar_template.dart';
import 'package:gen_defaults/surface_tint.dart';
import 'package:gen_defaults/switch_template.dart';
import 'package:gen_defaults/tabs_template.dart';
import 'package:gen_defaults/text_field_template.dart';
import 'package:gen_defaults/time_picker_template.dart';
import 'package:gen_defaults/token_logger.dart';
import 'package:gen_defaults/typography_template.dart';

Map<String, dynamic> _readTokenFile(File file) {
  return jsonDecode(file.readAsStringSync()) as Map<String, dynamic>;
}

const String materialLib = 'packages/flutter/lib/src/material';
const String dataDir = 'dev/tools/gen_defaults/data';

Future<void> main(List<String> args) async {
  // Parse arguments
  final ArgParser parser = ArgParser();
  parser.addFlag('verbose', abbr: 'v', help: 'Enable verbose output', negatable: false);
  final ArgResults argResults = parser.parse(args);
  final bool verbose = argResults['verbose'] as bool;

  // Map of version number to list of data files that use that version.
  final Map<String, List<String>> versionMap = <String, List<String>>{};
  // Map of all tokens to their values.
  final Map<String, dynamic> tokens = <String, dynamic>{};

  // Initialize.
  for (final FileSystemEntity tokenFile in Directory(dataDir).listSync()) {
    final Map<String, dynamic> tokenFileTokens = _readTokenFile(tokenFile as File);
    final String version = tokenFileTokens['version'] as String;
    tokenFileTokens.remove('version');
    versionMap[version] ??= <String>[];
    versionMap[version]!.add(tokenFile.uri.pathSegments.last);

    tokens.addAll(tokenFileTokens);
  }
  tokenLogger.init(allTokens: tokens, versionMap: versionMap);
  // Handle light/dark color tokens separately because they share identical token names.
  final Map<String, dynamic> colorLightTokens = _readTokenFile(File('$dataDir/color_light.json'));
  final Map<String, dynamic> colorDarkTokens = _readTokenFile(File('$dataDir/color_dark.json'));

  // The verifyTokenTemplatesUpdateCorrectFiles check in dev/bots/analyze.dart depends on the exact formatting of the next few lines.
  // dart format off
  // Generate tokens files.
  ChipTemplate('Chip', '$materialLib/chip.dart', tokens).updateFile();
  ActionChipTemplate('ActionChip', '$materialLib/action_chip.dart', tokens).updateFile();
  AppBarTemplate('AppBar', '$materialLib/app_bar.dart', tokens).updateFile();
  BottomAppBarTemplate('BottomAppBar', '$materialLib/bottom_app_bar.dart', tokens).updateFile();
  BadgeTemplate('Badge', '$materialLib/badge.dart', tokens).updateFile();
  BannerTemplate('Banner', '$materialLib/banner.dart', tokens).updateFile();
  BottomAppBarTemplate('BottomAppBar', '$materialLib/bottom_app_bar.dart', tokens).updateFile();
  BottomSheetTemplate('BottomSheet', '$materialLib/bottom_sheet.dart', tokens).updateFile();
  ButtonTemplate('md.comp.elevated-button', 'ElevatedButton', '$materialLib/elevated_button.dart', tokens).updateFile();
  ButtonTemplate('md.comp.filled-button', 'FilledButton', '$materialLib/filled_button.dart', tokens).updateFile();
  ButtonTemplate('md.comp.filled-tonal-button', 'FilledTonalButton', '$materialLib/filled_button.dart', tokens).updateFile();
  ButtonTemplate('md.comp.outlined-button', 'OutlinedButton', '$materialLib/outlined_button.dart', tokens).updateFile();
  ButtonTemplate('md.comp.text-button', 'TextButton', '$materialLib/text_button.dart', tokens).updateFile();
  CardTemplate('md.comp.elevated-card', 'Card', '$materialLib/card.dart', tokens).updateFile();
  CardTemplate('md.comp.filled-card', 'FilledCard', '$materialLib/card.dart', tokens).updateFile();
  CardTemplate('md.comp.outlined-card', 'OutlinedCard', '$materialLib/card.dart', tokens).updateFile();
  CheckboxTemplate('Checkbox', '$materialLib/checkbox.dart', tokens).updateFile();
  ColorSchemeTemplate(colorLightTokens, colorDarkTokens, 'ColorScheme', '$materialLib/theme_data.dart', tokens).updateFile();
  DatePickerTemplate('DatePicker', '$materialLib/date_picker_theme.dart', tokens).updateFile();
  DialogFullscreenTemplate('DialogFullscreen', '$materialLib/dialog.dart', tokens).updateFile();
  DialogTemplate('Dialog', '$materialLib/dialog.dart', tokens).updateFile();
  DividerTemplate('Divider', '$materialLib/divider.dart', tokens).updateFile();
  DrawerTemplate('Drawer', '$materialLib/drawer.dart', tokens).updateFile();
  ExpansionTileTemplate('ExpansionTile', '$materialLib/expansion_tile.dart', tokens).updateFile();
  FABTemplate('FAB', '$materialLib/floating_action_button.dart', tokens).updateFile();
  FilterChipTemplate('ChoiceChip', '$materialLib/choice_chip.dart', tokens).updateFile();
  FilterChipTemplate('FilterChip', '$materialLib/filter_chip.dart', tokens).updateFile();
  IconButtonTemplate('md.comp.icon-button', 'IconButton', '$materialLib/icon_button.dart', tokens).updateFile();
  IconButtonTemplate('md.comp.filled-icon-button', 'FilledIconButton', '$materialLib/icon_button.dart', tokens).updateFile();
  IconButtonTemplate('md.comp.filled-tonal-icon-button', 'FilledTonalIconButton', '$materialLib/icon_button.dart', tokens).updateFile();
  IconButtonTemplate('md.comp.outlined-icon-button', 'OutlinedIconButton', '$materialLib/icon_button.dart', tokens).updateFile();
  InputChipTemplate('InputChip', '$materialLib/input_chip.dart', tokens).updateFile();
  ListTileTemplate('LisTile', '$materialLib/list_tile.dart', tokens).updateFile();
  InputDecoratorTemplate('InputDecorator', '$materialLib/input_decorator.dart', tokens).updateFile();
  MenuTemplate('Menu', '$materialLib/menu_anchor.dart', tokens).updateFile();
  MotionTemplate('Motion', '$materialLib/motion.dart', tokens, tokenLogger).updateFile();
  NavigationBarTemplate('NavigationBar', '$materialLib/navigation_bar.dart', tokens).updateFile();
  NavigationDrawerTemplate('NavigationDrawer', '$materialLib/navigation_drawer.dart', tokens).updateFile();
  NavigationRailTemplate('NavigationRail', '$materialLib/navigation_rail.dart', tokens).updateFile();
  PopupMenuTemplate('PopupMenu', '$materialLib/popup_menu.dart', tokens).updateFile();
  ProgressIndicatorTemplate('ProgressIndicator', '$materialLib/progress_indicator.dart', tokens).updateFile();
  RadioTemplate('Radio<T>', '$materialLib/radio.dart', tokens).updateFile();
  RangeSliderTemplate('md.comp.slider', 'RangeSlider', '$materialLib/range_slider.dart', tokens).updateFile();
  SearchBarTemplate('SearchBar', '$materialLib/search_anchor.dart', tokens).updateFile();
  SearchViewTemplate('SearchView', '$materialLib/search_anchor.dart', tokens).updateFile();
  SegmentedButtonTemplate('md.comp.outlined-segmented-button', 'SegmentedButton', '$materialLib/segmented_button.dart', tokens).updateFile();
  SnackbarTemplate('md.comp.snackbar', 'Snackbar', '$materialLib/snack_bar.dart', tokens).updateFile();
  SliderTemplate('md.comp.slider', 'Slider', '$materialLib/slider.dart', tokens).updateFile();
  SurfaceTintTemplate('SurfaceTint', '$materialLib/elevation_overlay.dart', tokens).updateFile();
  SwitchTemplate('Switch', '$materialLib/switch.dart', tokens).updateFile();
  TimePickerTemplate('TimePicker', '$materialLib/time_picker.dart', tokens).updateFile();
  TextFieldTemplate('TextField', '$materialLib/text_field.dart', tokens).updateFile();
  TabsTemplate('Tabs', '$materialLib/tabs.dart', tokens).updateFile();
  TypographyTemplate('Typography', '$materialLib/typography.dart', tokens).updateFile();
  // dart format on

  tokenLogger.printVersionUsage(verbose: verbose);
  tokenLogger.printTokensUsage(verbose: verbose);
  if (!verbose) {
    print('\nTo see detailed version and token usage, run with --verbose (-v).');
  }

  tokenLogger.dumpToFile('dev/tools/gen_defaults/generated/used_tokens.csv');
}
