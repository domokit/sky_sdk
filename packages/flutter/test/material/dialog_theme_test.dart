// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file is run as part of a reduced test set in CI on Mac and Windows
// machines.
@Tags(<String>['reduced-test-set'])
library;

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

MaterialApp _appWithDialog(
  WidgetTester tester,
  Widget dialog, {
  ThemeData? theme,
  DialogThemeData? dialogTheme,
}) {
  Widget dialogBuilder = Builder(
    builder: (BuildContext context) {
      return Center(
        child: ElevatedButton(
          child: const Text('X'),
          onPressed: () {
            showDialog<void>(
              context: context,
              builder: (BuildContext context) {
                return RepaintBoundary(key: _painterKey, child: dialog);
              },
            );
          },
        ),
      );
    },
  );

  if (dialogTheme != null) {
    dialogBuilder = DialogTheme(data: dialogTheme, child: dialogBuilder);
  }

  return MaterialApp(theme: theme, home: Material(child: dialogBuilder));
}

final Key _painterKey = UniqueKey();

Material _getMaterialAlertDialog(WidgetTester tester) {
  return tester.widget<Material>(
    find.descendant(of: find.byType(AlertDialog), matching: find.byType(Material)),
  );
}

Material _getMaterialDialog(WidgetTester tester) {
  return tester.widget<Material>(
    find.descendant(of: find.byType(Dialog), matching: find.byType(Material)),
  );
}

RenderParagraph _getTextRenderObject(WidgetTester tester, String text) {
  return tester.element<StatelessElement>(find.text(text)).renderObject! as RenderParagraph;
}

RenderParagraph _getIconRenderObject(WidgetTester tester, IconData icon) {
  return tester.renderObject<RenderParagraph>(
    find.descendant(of: find.byIcon(icon), matching: find.byType(RichText)),
  );
}

void main() {
  test('DialogThemeData copyWith, ==, hashCode basics', () {
    expect(const DialogThemeData(), const DialogThemeData().copyWith());
    expect(const DialogThemeData().hashCode, const DialogThemeData().copyWith().hashCode);
  });

  test('DialogThemeData lerp special cases', () {
    expect(DialogThemeData.lerp(null, null, 0), const DialogThemeData());
    const DialogThemeData theme = DialogThemeData();
    expect(identical(DialogThemeData.lerp(theme, theme, 0.5), theme), true);
  });

  test('DialogThemeData defaults', () {
    const DialogThemeData dialogThemeData = DialogThemeData();

    expect(dialogThemeData.backgroundColor, null);
    expect(dialogThemeData.elevation, null);
    expect(dialogThemeData.shadowColor, null);
    expect(dialogThemeData.surfaceTintColor, null);
    expect(dialogThemeData.shape, null);
    expect(dialogThemeData.alignment, null);
    expect(dialogThemeData.iconColor, null);
    expect(dialogThemeData.titleTextStyle, null);
    expect(dialogThemeData.contentTextStyle, null);
    expect(dialogThemeData.actionsPadding, null);
    expect(dialogThemeData.barrierColor, null);
    expect(dialogThemeData.insetPadding, null);
    expect(dialogThemeData.clipBehavior, null);

    const DialogTheme dialogTheme = DialogTheme(data: DialogThemeData(), child: SizedBox());
    expect(dialogTheme.backgroundColor, null);
    expect(dialogTheme.elevation, null);
    expect(dialogTheme.shadowColor, null);
    expect(dialogTheme.surfaceTintColor, null);
    expect(dialogTheme.shape, null);
    expect(dialogTheme.alignment, null);
    expect(dialogTheme.iconColor, null);
    expect(dialogTheme.titleTextStyle, null);
    expect(dialogTheme.contentTextStyle, null);
    expect(dialogTheme.actionsPadding, null);
    expect(dialogTheme.barrierColor, null);
    expect(dialogTheme.insetPadding, null);
    expect(dialogTheme.clipBehavior, null);
  });

  testWidgets('Default DialogThemeData debugFillProperties', (WidgetTester tester) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
    const DialogThemeData().debugFillProperties(builder);

    final List<String> description =
        builder.properties
            .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode node) => node.toString())
            .toList();

    expect(description, <String>[]);
  });

  testWidgets('DialogThemeData implements debugFillProperties', (WidgetTester tester) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
    const DialogThemeData(
      backgroundColor: Color(0xff123456),
      elevation: 8.0,
      shadowColor: Color(0xff000001),
      surfaceTintColor: Color(0xff000002),
      shape: BeveledRectangleBorder(borderRadius: BorderRadius.all(Radius.circular(20.5))),
      alignment: Alignment.bottomLeft,
      iconColor: Color(0xff654321),
      titleTextStyle: TextStyle(color: Color(0xffffffff)),
      contentTextStyle: TextStyle(color: Color(0xff000000)),
      actionsPadding: EdgeInsets.all(8.0),
      barrierColor: Color(0xff000005),
      insetPadding: EdgeInsets.all(20.0),
      clipBehavior: Clip.antiAlias,
    ).debugFillProperties(builder);
    final List<String> description =
        builder.properties
            .where((DiagnosticsNode n) => !n.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode n) => n.toString())
            .toList();
    expect(description, <String>[
      'backgroundColor: ${const Color(0xff123456)}',
      'elevation: 8.0',
      'shadowColor: ${const Color(0xff000001)}',
      'surfaceTintColor: ${const Color(0xff000002)}',
      'shape: BeveledRectangleBorder(BorderSide(width: 0.0, style: none), BorderRadius.circular(20.5))',
      'alignment: Alignment.bottomLeft',
      'iconColor: ${const Color(0xff654321)}',
      'titleTextStyle: TextStyle(inherit: true, color: ${const Color(0xffffffff)})',
      'contentTextStyle: TextStyle(inherit: true, color: ${const Color(0xff000000)})',
      'actionsPadding: EdgeInsets.all(8.0)',
      'barrierColor: ${const Color(0xff000005)}',
      'insetPadding: EdgeInsets.all(20.0)',
      'clipBehavior: Clip.antiAlias',
    ]);
  });

  testWidgets('Local DialogThemeData overrides dialog defaults', (WidgetTester tester) async {
    const Color themeBackgroundColor = Color(0xff123456);
    const double themeElevation = 8.0;
    const Color themeShadowColor = Color(0xff000001);
    const Color themeSurfaceTintColor = Color(0xff000002);
    const BeveledRectangleBorder themeShape = BeveledRectangleBorder(
      borderRadius: BorderRadius.all(Radius.circular(20.5)),
    );
    const AlignmentGeometry themeAlignment = Alignment.bottomLeft;
    const Color themeIconColor = Color(0xff654321);
    const TextStyle themeTitleTextStyle = TextStyle(color: Color(0xffffffff));
    const TextStyle themeContentTextStyle = TextStyle(color: Color(0xff000000));
    const EdgeInsetsGeometry themeActionsPadding = EdgeInsets.all(8.0);
    const Color themeBarrierColor = Color(0xff000005);
    const EdgeInsets themeInsetPadding = EdgeInsets.all(30.0);
    const Clip themeClipBehavior = Clip.antiAlias;
    const AlertDialog dialog = AlertDialog(
      title: Text('Title'),
      content: Text('Content'),
      icon: Icon(Icons.search),
      actions: <Widget>[Icon(Icons.cancel)],
    );

    const DialogThemeData dialogTheme = DialogThemeData(
      backgroundColor: themeBackgroundColor,
      elevation: themeElevation,
      shadowColor: themeShadowColor,
      surfaceTintColor: themeSurfaceTintColor,
      shape: themeShape,
      alignment: themeAlignment,
      iconColor: themeIconColor,
      titleTextStyle: themeTitleTextStyle,
      contentTextStyle: themeContentTextStyle,
      actionsPadding: themeActionsPadding,
      barrierColor: themeBarrierColor,
      insetPadding: themeInsetPadding,
      clipBehavior: themeClipBehavior,
    );

    await tester.pumpWidget(_appWithDialog(tester, dialog, dialogTheme: dialogTheme));
    await tester.tap(find.text('X'));
    await tester.pumpAndSettle();

    final Material materialWidget = _getMaterialAlertDialog(tester);
    expect(materialWidget.color, themeBackgroundColor);
    expect(materialWidget.elevation, themeElevation);
    expect(materialWidget.shadowColor, themeShadowColor);
    expect(materialWidget.surfaceTintColor, themeSurfaceTintColor);
    expect(materialWidget.shape, themeShape);
    expect(materialWidget.clipBehavior, Clip.antiAlias);
    final Offset bottomLeft = tester.getBottomLeft(
      find.descendant(of: find.byType(Dialog), matching: find.byType(Material)),
    );
    expect(bottomLeft.dx, 30.0); // 30 is the padding value.
    expect(bottomLeft.dy, 570.0); // 600 - 30
    expect(_getIconRenderObject(tester, Icons.search).text.style?.color, themeIconColor);
    expect(_getTextRenderObject(tester, 'Title').text.style?.color, themeTitleTextStyle.color);
    expect(_getTextRenderObject(tester, 'Content').text.style?.color, themeContentTextStyle.color);
    final ModalBarrier modalBarrier = tester.widget(find.byType(ModalBarrier).last);
    expect(modalBarrier.color, themeBarrierColor);

    final Finder findPadding =
        find.ancestor(of: find.byIcon(Icons.cancel), matching: find.byType(Padding)).first;
    final Padding padding = tester.widget<Padding>(findPadding);
    expect(padding.padding, themeActionsPadding);
  });

  testWidgets('Local DialogThemeData overrides global dialogTheme', (WidgetTester tester) async {
    const Color themeBackgroundColor = Color(0xff123456);
    const double themeElevation = 8.0;
    const Color themeShadowColor = Color(0xff000001);
    const Color themeSurfaceTintColor = Color(0xff000002);
    const BeveledRectangleBorder themeShape = BeveledRectangleBorder(
      borderRadius: BorderRadius.all(Radius.circular(20.5)),
    );
    const AlignmentGeometry themeAlignment = Alignment.bottomLeft;
    const Color themeIconColor = Color(0xff654321);
    const TextStyle themeTitleTextStyle = TextStyle(color: Color(0xffffffff));
    const TextStyle themeContentTextStyle = TextStyle(color: Color(0xff000000));
    const EdgeInsetsGeometry themeActionsPadding = EdgeInsets.all(8.0);
    const Color themeBarrierColor = Color(0xff000005);
    const EdgeInsets themeInsetPadding = EdgeInsets.all(30.0);
    const Clip themeClipBehavior = Clip.antiAlias;

    const Color globalBackgroundColor = Color(0xff654321);
    const double globalElevation = 7.0;
    const Color globalShadowColor = Color(0xff200001);
    const Color globalSurfaceTintColor = Color(0xff222002);
    const BeveledRectangleBorder globalShape = BeveledRectangleBorder(
      borderRadius: BorderRadius.all(Radius.circular(25.5)),
    );
    const AlignmentGeometry globalAlignment = Alignment.centerRight;
    const Color globalIconColor = Color(0xff666666);
    const TextStyle globalTitleTextStyle = TextStyle(color: Color(0xff000000));
    const TextStyle globalContentTextStyle = TextStyle(color: Color(0xffdddddd));
    const EdgeInsetsGeometry globalActionsPadding = EdgeInsets.all(18.0);
    const Color globalBarrierColor = Color(0xff111115);
    const EdgeInsets globalInsetPadding = EdgeInsets.all(35.0);
    const Clip globalClipBehavior = Clip.hardEdge;
    const AlertDialog dialog = AlertDialog(
      title: Text('Title'),
      content: Text('Content'),
      icon: Icon(Icons.search),
      actions: <Widget>[Icon(Icons.cancel)],
    );

    const DialogThemeData dialogTheme = DialogThemeData(
      backgroundColor: themeBackgroundColor,
      elevation: themeElevation,
      shadowColor: themeShadowColor,
      surfaceTintColor: themeSurfaceTintColor,
      shape: themeShape,
      alignment: themeAlignment,
      iconColor: themeIconColor,
      titleTextStyle: themeTitleTextStyle,
      contentTextStyle: themeContentTextStyle,
      actionsPadding: themeActionsPadding,
      barrierColor: themeBarrierColor,
      insetPadding: themeInsetPadding,
      clipBehavior: themeClipBehavior,
    );

    const DialogThemeData globalDialogTheme = DialogThemeData(
      backgroundColor: globalBackgroundColor,
      elevation: globalElevation,
      shadowColor: globalShadowColor,
      surfaceTintColor: globalSurfaceTintColor,
      shape: globalShape,
      alignment: globalAlignment,
      iconColor: globalIconColor,
      titleTextStyle: globalTitleTextStyle,
      contentTextStyle: globalContentTextStyle,
      actionsPadding: globalActionsPadding,
      barrierColor: globalBarrierColor,
      insetPadding: globalInsetPadding,
      clipBehavior: globalClipBehavior,
    );

    await tester.pumpWidget(
      _appWithDialog(
        tester,
        dialog,
        dialogTheme: dialogTheme,
        theme: ThemeData(dialogTheme: globalDialogTheme),
      ),
    );
    await tester.tap(find.text('X'));
    await tester.pumpAndSettle();

    final Material materialWidget = _getMaterialAlertDialog(tester);
    expect(materialWidget.color, themeBackgroundColor);
    expect(materialWidget.elevation, themeElevation);
    expect(materialWidget.shadowColor, themeShadowColor);
    expect(materialWidget.surfaceTintColor, themeSurfaceTintColor);
    expect(materialWidget.shape, themeShape);
    expect(materialWidget.clipBehavior, Clip.antiAlias);
    final Offset bottomLeft = tester.getBottomLeft(
      find.descendant(of: find.byType(Dialog), matching: find.byType(Material)),
    );
    expect(bottomLeft.dx, 30.0); // 30 is the padding value.
    expect(bottomLeft.dy, 570.0); // 600 - 30
    expect(_getIconRenderObject(tester, Icons.search).text.style?.color, themeIconColor);
    expect(_getTextRenderObject(tester, 'Title').text.style?.color, themeTitleTextStyle.color);
    expect(_getTextRenderObject(tester, 'Content').text.style?.color, themeContentTextStyle.color);
    final ModalBarrier modalBarrier = tester.widget(find.byType(ModalBarrier).last);
    expect(modalBarrier.color, themeBarrierColor);

    final Finder findPadding =
        find.ancestor(of: find.byIcon(Icons.cancel), matching: find.byType(Padding)).first;
    final Padding padding = tester.widget<Padding>(findPadding);
    expect(padding.padding, themeActionsPadding);
  });

  testWidgets('Dialog background color', (WidgetTester tester) async {
    const Color customColor = Colors.pink;
    const AlertDialog dialog = AlertDialog(title: Text('Title'), actions: <Widget>[]);
    final ThemeData theme = ThemeData(
      dialogTheme: const DialogThemeData(backgroundColor: customColor),
    );

    await tester.pumpWidget(_appWithDialog(tester, dialog, theme: theme));
    await tester.tap(find.text('X'));
    await tester.pumpAndSettle();

    final Material materialWidget = _getMaterialAlertDialog(tester);
    expect(materialWidget.color, customColor);
  });

  testWidgets('Custom dialog elevation', (WidgetTester tester) async {
    const double customElevation = 12.0;
    const Color shadowColor = Color(0xFF000001);
    const Color surfaceTintColor = Color(0xFF000002);
    const AlertDialog dialog = AlertDialog(title: Text('Title'), actions: <Widget>[]);
    final ThemeData theme = ThemeData(
      dialogTheme: const DialogThemeData(
        elevation: customElevation,
        shadowColor: shadowColor,
        surfaceTintColor: surfaceTintColor,
      ),
    );

    await tester.pumpWidget(_appWithDialog(tester, dialog, theme: theme));
    await tester.tap(find.text('X'));
    await tester.pumpAndSettle();

    final Material materialWidget = _getMaterialAlertDialog(tester);
    expect(materialWidget.elevation, customElevation);
    expect(materialWidget.shadowColor, shadowColor);
    expect(materialWidget.surfaceTintColor, surfaceTintColor);
  });

  testWidgets('Custom dialog shape', (WidgetTester tester) async {
    const RoundedRectangleBorder customBorder = RoundedRectangleBorder(
      borderRadius: BorderRadius.all(Radius.circular(16.0)),
    );
    const AlertDialog dialog = AlertDialog(title: Text('Title'), actions: <Widget>[]);
    final ThemeData theme = ThemeData(dialogTheme: const DialogThemeData(shape: customBorder));

    await tester.pumpWidget(_appWithDialog(tester, dialog, theme: theme));
    await tester.tap(find.text('X'));
    await tester.pumpAndSettle();

    final Material materialWidget = _getMaterialAlertDialog(tester);
    expect(materialWidget.shape, customBorder);
  });

  testWidgets('Custom dialog alignment', (WidgetTester tester) async {
    const AlertDialog dialog = AlertDialog(title: Text('Title'), actions: <Widget>[]);
    final ThemeData theme = ThemeData(
      dialogTheme: const DialogThemeData(alignment: Alignment.bottomLeft),
    );

    await tester.pumpWidget(_appWithDialog(tester, dialog, theme: theme));
    await tester.tap(find.text('X'));
    await tester.pumpAndSettle();

    final Offset bottomLeft = tester.getBottomLeft(
      find.descendant(of: find.byType(Dialog), matching: find.byType(Material)),
    );
    expect(bottomLeft.dx, 40.0);
    expect(bottomLeft.dy, 576.0);
  });

  testWidgets('Material3 - Dialog alignment takes priority over theme', (
    WidgetTester tester,
  ) async {
    const AlertDialog dialog = AlertDialog(
      title: Text('Title'),
      actions: <Widget>[],
      alignment: Alignment.topRight,
    );
    final ThemeData theme = ThemeData(
      dialogTheme: const DialogThemeData(alignment: Alignment.bottomLeft),
    );

    await tester.pumpWidget(_appWithDialog(tester, dialog, theme: theme));
    await tester.tap(find.text('X'));
    await tester.pumpAndSettle();

    final Offset bottomLeft = tester.getBottomLeft(
      find.descendant(of: find.byType(Dialog), matching: find.byType(Material)),
    );
    expect(bottomLeft.dx, 480.0);
    expect(bottomLeft.dy, 124.0);
  });

  testWidgets('Material2 - Dialog alignment takes priority over theme', (
    WidgetTester tester,
  ) async {
    const AlertDialog dialog = AlertDialog(
      title: Text('Title'),
      actions: <Widget>[],
      alignment: Alignment.topRight,
    );
    final ThemeData theme = ThemeData(
      useMaterial3: false,
      dialogTheme: const DialogThemeData(alignment: Alignment.bottomLeft),
    );

    await tester.pumpWidget(_appWithDialog(tester, dialog, theme: theme));
    await tester.tap(find.text('X'));
    await tester.pumpAndSettle();

    final Offset bottomLeft = tester.getBottomLeft(
      find.descendant(of: find.byType(Dialog), matching: find.byType(Material)),
    );
    expect(bottomLeft.dx, 480.0);
    expect(bottomLeft.dy, 104.0);
  });

  testWidgets('Material3 - Custom dialog shape matches golden', (WidgetTester tester) async {
    const RoundedRectangleBorder customBorder = RoundedRectangleBorder(
      borderRadius: BorderRadius.all(Radius.circular(16.0)),
    );
    const AlertDialog dialog = AlertDialog(title: Text('Title'), actions: <Widget>[]);
    final ThemeData theme = ThemeData(dialogTheme: const DialogThemeData(shape: customBorder));

    await tester.pumpWidget(_appWithDialog(tester, dialog, theme: theme));
    await tester.tap(find.text('X'));
    await tester.pumpAndSettle();

    await expectLater(
      find.byKey(_painterKey),
      matchesGoldenFile('m3_dialog_theme.dialog_with_custom_border.png'),
    );
  });

  testWidgets('Material2 - Custom dialog shape matches golden', (WidgetTester tester) async {
    const RoundedRectangleBorder customBorder = RoundedRectangleBorder(
      borderRadius: BorderRadius.all(Radius.circular(16.0)),
    );
    const AlertDialog dialog = AlertDialog(title: Text('Title'), actions: <Widget>[]);
    final ThemeData theme = ThemeData(
      useMaterial3: false,
      dialogTheme: const DialogThemeData(shape: customBorder),
    );

    await tester.pumpWidget(_appWithDialog(tester, dialog, theme: theme));
    await tester.tap(find.text('X'));
    await tester.pumpAndSettle();

    await expectLater(
      find.byKey(_painterKey),
      matchesGoldenFile('m2_dialog_theme.dialog_with_custom_border.png'),
    );
  });

  testWidgets('Custom Icon Color - Constructor Param - highest preference', (
    WidgetTester tester,
  ) async {
    const Color iconColor = Colors.pink,
        dialogThemeColor = Colors.green,
        iconThemeColor = Colors.yellow;
    final ThemeData theme = ThemeData(
      iconTheme: const IconThemeData(color: iconThemeColor),
      dialogTheme: const DialogThemeData(iconColor: dialogThemeColor),
    );
    const AlertDialog dialog = AlertDialog(
      icon: Icon(Icons.ac_unit),
      iconColor: iconColor,
      actions: <Widget>[],
    );

    await tester.pumpWidget(_appWithDialog(tester, dialog, theme: theme));
    await tester.tap(find.text('X'));
    await tester.pumpAndSettle();

    // first is Text('X')
    final RichText text = tester.widget(find.byType(RichText).last);
    expect(text.text.style!.color, iconColor);
  });

  testWidgets('Custom Icon Color - Dialog Theme - preference over Theme', (
    WidgetTester tester,
  ) async {
    const Color dialogThemeColor = Colors.green, iconThemeColor = Colors.yellow;
    final ThemeData theme = ThemeData(
      iconTheme: const IconThemeData(color: iconThemeColor),
      dialogTheme: const DialogThemeData(iconColor: dialogThemeColor),
    );
    const AlertDialog dialog = AlertDialog(icon: Icon(Icons.ac_unit), actions: <Widget>[]);

    await tester.pumpWidget(_appWithDialog(tester, dialog, theme: theme));
    await tester.tap(find.text('X'));
    await tester.pumpAndSettle();

    // first is Text('X')
    final RichText text = tester.widget(find.byType(RichText).last);
    expect(text.text.style!.color, dialogThemeColor);
  });

  testWidgets('Material3 - Custom Icon Color - Theme - lowest preference', (
    WidgetTester tester,
  ) async {
    final ThemeData theme = ThemeData();
    const AlertDialog dialog = AlertDialog(icon: Icon(Icons.ac_unit), actions: <Widget>[]);

    await tester.pumpWidget(_appWithDialog(tester, dialog, theme: theme));
    await tester.tap(find.text('X'));
    await tester.pumpAndSettle();

    // first is Text('X')
    final RichText text = tester.widget(find.byType(RichText).last);
    expect(text.text.style!.color, theme.colorScheme.secondary);
  });

  testWidgets('Material2 - Custom Icon Color - Theme - lowest preference', (
    WidgetTester tester,
  ) async {
    const Color iconThemeColor = Colors.yellow;
    final ThemeData theme = ThemeData(
      useMaterial3: false,
      iconTheme: const IconThemeData(color: iconThemeColor),
    );
    const AlertDialog dialog = AlertDialog(icon: Icon(Icons.ac_unit), actions: <Widget>[]);

    await tester.pumpWidget(_appWithDialog(tester, dialog, theme: theme));
    await tester.tap(find.text('X'));
    await tester.pumpAndSettle();

    // first is Text('X')
    final RichText text = tester.widget(find.byType(RichText).last);
    expect(text.text.style!.color, iconThemeColor);
  });

  testWidgets('Custom Title Text Style - Constructor Param', (WidgetTester tester) async {
    const String titleText = 'Title';
    const TextStyle titleTextStyle = TextStyle(color: Colors.pink);
    const AlertDialog dialog = AlertDialog(
      title: Text(titleText),
      titleTextStyle: titleTextStyle,
      actions: <Widget>[],
    );

    await tester.pumpWidget(_appWithDialog(tester, dialog));
    await tester.tap(find.text('X'));
    await tester.pumpAndSettle();

    final RenderParagraph title = _getTextRenderObject(tester, titleText);
    expect(title.text.style, titleTextStyle);
  });

  testWidgets('Custom Title Text Style - Dialog Theme', (WidgetTester tester) async {
    const String titleText = 'Title';
    const TextStyle titleTextStyle = TextStyle(color: Colors.pink);
    const AlertDialog dialog = AlertDialog(title: Text(titleText), actions: <Widget>[]);
    final ThemeData theme = ThemeData(
      dialogTheme: const DialogThemeData(titleTextStyle: titleTextStyle),
    );

    await tester.pumpWidget(_appWithDialog(tester, dialog, theme: theme));
    await tester.tap(find.text('X'));
    await tester.pumpAndSettle();

    final RenderParagraph title = _getTextRenderObject(tester, titleText);
    expect(title.text.style, titleTextStyle);
  });

  testWidgets('Material3 - Custom Title Text Style - Theme', (WidgetTester tester) async {
    const String titleText = 'Title';
    const TextStyle titleTextStyle = TextStyle(color: Colors.pink);
    const AlertDialog dialog = AlertDialog(title: Text(titleText));
    final ThemeData theme = ThemeData(textTheme: const TextTheme(headlineSmall: titleTextStyle));

    await tester.pumpWidget(_appWithDialog(tester, dialog, theme: theme));
    await tester.tap(find.text('X'));
    await tester.pumpAndSettle();

    final RenderParagraph title = _getTextRenderObject(tester, titleText);
    expect(title.text.style!.color, titleTextStyle.color);
  });

  testWidgets('Material2 - Custom Title Text Style - Theme', (WidgetTester tester) async {
    const String titleText = 'Title';
    const TextStyle titleTextStyle = TextStyle(color: Colors.pink);
    const AlertDialog dialog = AlertDialog(title: Text(titleText));
    final ThemeData theme = ThemeData(
      useMaterial3: false,
      textTheme: const TextTheme(titleLarge: titleTextStyle),
    );

    await tester.pumpWidget(_appWithDialog(tester, dialog, theme: theme));
    await tester.tap(find.text('X'));
    await tester.pumpAndSettle();

    final RenderParagraph title = _getTextRenderObject(tester, titleText);
    expect(title.text.style!.color, titleTextStyle.color);
  });

  testWidgets('Simple Dialog - Custom Title Text Style - Constructor Param', (
    WidgetTester tester,
  ) async {
    const String titleText = 'Title';
    const TextStyle titleTextStyle = TextStyle(color: Colors.pink);
    const SimpleDialog dialog = SimpleDialog(
      title: Text(titleText),
      titleTextStyle: titleTextStyle,
    );

    await tester.pumpWidget(_appWithDialog(tester, dialog));
    await tester.tap(find.text('X'));
    await tester.pumpAndSettle();

    final RenderParagraph title = _getTextRenderObject(tester, titleText);
    expect(title.text.style, titleTextStyle);
  });

  testWidgets('Simple Dialog - Custom Title Text Style - Dialog Theme', (
    WidgetTester tester,
  ) async {
    const String titleText = 'Title';
    const TextStyle titleTextStyle = TextStyle(color: Colors.pink);
    const SimpleDialog dialog = SimpleDialog(title: Text(titleText));
    final ThemeData theme = ThemeData(
      dialogTheme: const DialogThemeData(titleTextStyle: titleTextStyle),
    );

    await tester.pumpWidget(_appWithDialog(tester, dialog, theme: theme));
    await tester.tap(find.text('X'));
    await tester.pumpAndSettle();

    final RenderParagraph title = _getTextRenderObject(tester, titleText);
    expect(title.text.style, titleTextStyle);
  });

  testWidgets('Simple Dialog - Custom Title Text Style - Theme', (WidgetTester tester) async {
    const String titleText = 'Title';
    const TextStyle titleTextStyle = TextStyle(color: Colors.pink);
    const SimpleDialog dialog = SimpleDialog(title: Text(titleText));
    final ThemeData theme = ThemeData(textTheme: const TextTheme(titleLarge: titleTextStyle));

    await tester.pumpWidget(_appWithDialog(tester, dialog, theme: theme));
    await tester.tap(find.text('X'));
    await tester.pumpAndSettle();

    final RenderParagraph title = _getTextRenderObject(tester, titleText);
    expect(title.text.style!.color, titleTextStyle.color);
  });

  testWidgets('Custom Content Text Style - Constructor Param', (WidgetTester tester) async {
    const String contentText = 'Content';
    const TextStyle contentTextStyle = TextStyle(color: Colors.pink);
    const AlertDialog dialog = AlertDialog(
      content: Text(contentText),
      contentTextStyle: contentTextStyle,
      actions: <Widget>[],
    );

    await tester.pumpWidget(_appWithDialog(tester, dialog));
    await tester.tap(find.text('X'));
    await tester.pumpAndSettle();

    final RenderParagraph content = _getTextRenderObject(tester, contentText);
    expect(content.text.style, contentTextStyle);
  });

  testWidgets('Custom Content Text Style - Dialog Theme', (WidgetTester tester) async {
    const String contentText = 'Content';
    const TextStyle contentTextStyle = TextStyle(color: Colors.pink);
    const AlertDialog dialog = AlertDialog(content: Text(contentText), actions: <Widget>[]);
    final ThemeData theme = ThemeData(
      dialogTheme: const DialogThemeData(contentTextStyle: contentTextStyle),
    );

    await tester.pumpWidget(_appWithDialog(tester, dialog, theme: theme));
    await tester.tap(find.text('X'));
    await tester.pumpAndSettle();

    final RenderParagraph content = _getTextRenderObject(tester, contentText);
    expect(content.text.style, contentTextStyle);
  });

  testWidgets('Material3 - Custom Content Text Style - Theme', (WidgetTester tester) async {
    const String contentText = 'Content';
    const TextStyle contentTextStyle = TextStyle(color: Colors.pink);
    const AlertDialog dialog = AlertDialog(content: Text(contentText));
    final ThemeData theme = ThemeData(textTheme: const TextTheme(bodyMedium: contentTextStyle));

    await tester.pumpWidget(_appWithDialog(tester, dialog, theme: theme));
    await tester.tap(find.text('X'));
    await tester.pumpAndSettle();

    final RenderParagraph content = _getTextRenderObject(tester, contentText);
    expect(content.text.style!.color, contentTextStyle.color);
  });

  testWidgets('Material2 - Custom Content Text Style - Theme', (WidgetTester tester) async {
    const String contentText = 'Content';
    const TextStyle contentTextStyle = TextStyle(color: Colors.pink);
    const AlertDialog dialog = AlertDialog(content: Text(contentText));
    final ThemeData theme = ThemeData(
      useMaterial3: false,
      textTheme: const TextTheme(titleMedium: contentTextStyle),
    );

    await tester.pumpWidget(_appWithDialog(tester, dialog, theme: theme));
    await tester.tap(find.text('X'));
    await tester.pumpAndSettle();

    final RenderParagraph content = _getTextRenderObject(tester, contentText);
    expect(content.text.style!.color, contentTextStyle.color);
  });

  testWidgets('Custom barrierColor - Theme', (WidgetTester tester) async {
    const Color barrierColor = Colors.blue;
    const SimpleDialog dialog = SimpleDialog();
    final ThemeData theme = ThemeData(
      dialogTheme: const DialogThemeData(barrierColor: barrierColor),
    );

    await tester.pumpWidget(_appWithDialog(tester, dialog, theme: theme));
    await tester.tap(find.text('X'));
    await tester.pumpAndSettle();

    final ModalBarrier modalBarrier = tester.widget(find.byType(ModalBarrier).last);
    expect(modalBarrier.color, barrierColor);
  });

  testWidgets('DialogTheme.insetPadding updates Dialog insetPadding', (WidgetTester tester) async {
    // The default testing screen (800, 600)
    const Rect screenRect = Rect.fromLTRB(0.0, 0.0, 800.0, 600.0);
    const DialogThemeData dialogTheme = DialogThemeData(
      insetPadding: EdgeInsets.fromLTRB(10, 15, 20, 25),
    );
    const Dialog dialog = Dialog(child: Placeholder());

    await tester.pumpWidget(
      _appWithDialog(tester, dialog, theme: ThemeData(dialogTheme: dialogTheme)),
    );
    await tester.tap(find.text('X'));
    await tester.pump();

    expect(
      tester.getRect(find.byType(Placeholder)),
      Rect.fromLTRB(
        screenRect.left + dialogTheme.insetPadding!.left,
        screenRect.top + dialogTheme.insetPadding!.top,
        screenRect.right - dialogTheme.insetPadding!.right,
        screenRect.bottom - dialogTheme.insetPadding!.bottom,
      ),
    );
  });

  testWidgets('DialogTheme.clipBehavior updates the dialogs clip behavior', (
    WidgetTester tester,
  ) async {
    const DialogThemeData dialogTheme = DialogThemeData(clipBehavior: Clip.hardEdge);
    const Dialog dialog = Dialog(child: Placeholder());

    await tester.pumpWidget(
      _appWithDialog(tester, dialog, theme: ThemeData(dialogTheme: dialogTheme)),
    );
    await tester.tap(find.text('X'));
    await tester.pumpAndSettle();

    final Material materialWidget = _getMaterialDialog(tester);
    expect(materialWidget.clipBehavior, dialogTheme.clipBehavior);
  });

  testWidgets('Dialog.clipBehavior takes priority over theme', (WidgetTester tester) async {
    const Dialog dialog = Dialog(clipBehavior: Clip.antiAlias, child: Placeholder());
    final ThemeData theme = ThemeData(
      dialogTheme: const DialogThemeData(clipBehavior: Clip.hardEdge),
    );

    await tester.pumpWidget(_appWithDialog(tester, dialog, theme: theme));
    await tester.tap(find.text('X'));
    await tester.pumpAndSettle();

    final Material materialWidget = _getMaterialDialog(tester);
    expect(materialWidget.clipBehavior, Clip.antiAlias);
  });

  testWidgets('Material2 - Dialog.clipBehavior takes priority over theme', (
    WidgetTester tester,
  ) async {
    const Dialog dialog = Dialog(clipBehavior: Clip.antiAlias, child: Placeholder());
    final ThemeData theme = ThemeData(
      useMaterial3: false,
      dialogTheme: const DialogThemeData(clipBehavior: Clip.hardEdge),
    );

    await tester.pumpWidget(_appWithDialog(tester, dialog, theme: theme));
    await tester.tap(find.text('X'));
    await tester.pumpAndSettle();

    final Material materialWidget = _getMaterialDialog(tester);
    expect(materialWidget.clipBehavior, Clip.antiAlias);
  });
}
