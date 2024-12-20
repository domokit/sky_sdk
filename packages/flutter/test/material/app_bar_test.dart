// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

import '../widgets/semantics_tester.dart';
import 'app_bar_utils.dart';

TextStyle? _iconStyle(WidgetTester tester, IconData icon) {
  final RichText iconRichText = tester.widget<RichText>(
    find.descendant(of: find.byIcon(icon).first, matching: find.byType(RichText)),
  );
  return iconRichText.text.style;
}

void main() {
  setUp(() {
    debugResetSemanticsIdCounter();
  });

  testWidgets('AppBar centers title on iOS', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(platform: TargetPlatform.android),
        home: Scaffold(appBar: AppBar(title: const Text('X'))),
      ),
    );

    final Finder title = find.text('X');
    Offset center = tester.getCenter(title);
    Size size = tester.getSize(title);
    expect(center.dx, lessThan(400 - size.width / 2.0));

    for (final TargetPlatform platform in <TargetPlatform>[
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    ]) {
      // Clear the widget tree to avoid animating between platforms.
      await tester.pumpWidget(Container(key: UniqueKey()));

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(platform: platform),
          home: Scaffold(appBar: AppBar(title: const Text('X'))),
        ),
      );

      center = tester.getCenter(title);
      size = tester.getSize(title);
      expect(center.dx, greaterThan(400 - size.width / 2.0), reason: 'on ${platform.name}');
      expect(center.dx, lessThan(400 + size.width / 2.0), reason: 'on ${platform.name}');

      // One action is still centered.

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(platform: platform),
          home: Scaffold(
            appBar: AppBar(title: const Text('X'), actions: const <Widget>[Icon(Icons.thumb_up)]),
          ),
        ),
      );

      center = tester.getCenter(title);
      size = tester.getSize(title);
      expect(center.dx, greaterThan(400 - size.width / 2.0), reason: 'on ${platform.name}');
      expect(center.dx, lessThan(400 + size.width / 2.0), reason: 'on ${platform.name}');

      // Two actions is left aligned again.

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(platform: platform),
          home: Scaffold(
            appBar: AppBar(
              title: const Text('X'),
              actions: const <Widget>[Icon(Icons.thumb_up), Icon(Icons.thumb_up)],
            ),
          ),
        ),
      );

      center = tester.getCenter(title);
      size = tester.getSize(title);
      expect(center.dx, lessThan(400 - size.width / 2.0), reason: 'on ${platform.name}');
    }
  });

  testWidgets('AppBar centerTitle:true centers on Android', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(platform: TargetPlatform.android),
        home: Scaffold(appBar: AppBar(centerTitle: true, title: const Text('X'))),
      ),
    );

    final Finder title = find.text('X');
    final Offset center = tester.getCenter(title);
    final Size size = tester.getSize(title);
    expect(center.dx, greaterThan(400 - size.width / 2.0));
    expect(center.dx, lessThan(400 + size.width / 2.0));
  });

  testWidgets('AppBar centerTitle:false title start edge is 16.0 (LTR)', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(appBar: AppBar(centerTitle: false, title: const Placeholder(key: Key('X')))),
      ),
    );

    final Finder titleWidget = find.byKey(const Key('X'));
    expect(tester.getTopLeft(titleWidget).dx, 16.0);
    expect(tester.getTopRight(titleWidget).dx, 800 - 16.0);
  });

  testWidgets('AppBar centerTitle:false title start edge is 16.0 (RTL)', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Directionality(
          textDirection: TextDirection.rtl,
          child: Scaffold(
            appBar: AppBar(centerTitle: false, title: const Placeholder(key: Key('X'))),
          ),
        ),
      ),
    );

    final Finder titleWidget = find.byKey(const Key('X'));
    expect(tester.getTopRight(titleWidget).dx, 800.0 - 16.0);
    expect(tester.getTopLeft(titleWidget).dx, 16.0);
  });

  testWidgets('AppBar titleSpacing:32 title start edge is 32.0 (LTR)', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          appBar: AppBar(
            centerTitle: false,
            titleSpacing: 32.0,
            title: const Placeholder(key: Key('X')),
          ),
        ),
      ),
    );

    final Finder titleWidget = find.byKey(const Key('X'));
    expect(tester.getTopLeft(titleWidget).dx, 32.0);
    expect(tester.getTopRight(titleWidget).dx, 800 - 32.0);
  });

  testWidgets('AppBar titleSpacing:32 title start edge is 32.0 (RTL)', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Directionality(
          textDirection: TextDirection.rtl,
          child: Scaffold(
            appBar: AppBar(
              centerTitle: false,
              titleSpacing: 32.0,
              title: const Placeholder(key: Key('X')),
            ),
          ),
        ),
      ),
    );

    final Finder titleWidget = find.byKey(const Key('X'));
    expect(tester.getTopRight(titleWidget).dx, 800.0 - 32.0);
    expect(tester.getTopLeft(titleWidget).dx, 32.0);
  });

  testWidgets('AppBar centerTitle:false leading button title left edge is 72.0 (LTR)', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          appBar: AppBar(centerTitle: false, title: const Text('X')),
          // A drawer causes a leading hamburger.
          drawer: const Drawer(),
        ),
      ),
    );

    expect(tester.getTopLeft(find.text('X')).dx, 72.0);
  });

  testWidgets('AppBar centerTitle:false leading button title left edge is 72.0 (RTL)', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Directionality(
          textDirection: TextDirection.rtl,
          child: Scaffold(
            appBar: AppBar(centerTitle: false, title: const Text('X')),
            // A drawer causes a leading hamburger.
            drawer: const Drawer(),
          ),
        ),
      ),
    );

    expect(tester.getTopRight(find.text('X')).dx, 800.0 - 72.0);
  });

  testWidgets('AppBar centerTitle:false title overflow OK', (WidgetTester tester) async {
    // The app bar's title should be constrained to fit within the available space
    // between the leading and actions widgets.

    final Key titleKey = UniqueKey();
    Widget leading = Container();
    List<Widget> actions = <Widget>[];

    Widget buildApp() {
      return MaterialApp(
        home: Scaffold(
          appBar: AppBar(
            leading: leading,
            centerTitle: false,
            title: Container(
              key: titleKey,
              constraints: BoxConstraints.loose(const Size(1000.0, 1000.0)),
            ),
            actions: actions,
          ),
        ),
      );
    }

    await tester.pumpWidget(buildApp());

    final Finder title = find.byKey(titleKey);
    expect(tester.getTopLeft(title).dx, 72.0);
    expect(
      tester.getSize(title).width,
      equals(
        800.0 // Screen width.
            -
            56.0 // Leading button width.
            -
            16.0 // Leading button to title padding.
            -
            16.0, // Title right side padding.
      ),
    );

    actions = <Widget>[const SizedBox(width: 100.0), const SizedBox(width: 100.0)];
    await tester.pumpWidget(buildApp());

    expect(tester.getTopLeft(title).dx, 72.0);
    // The title shrinks by 200.0 to allow for the actions widgets.
    expect(
      tester.getSize(title).width,
      equals(
        800.0 // Screen width.
            -
            56.0 // Leading button width.
            -
            16.0 // Leading button to title padding.
            -
            16.0 // Title to actions padding
            -
            200.0,
      ),
    ); // Actions' width.

    leading = Container(); // AppBar will constrain the width to 24.0
    await tester.pumpWidget(buildApp());
    expect(tester.getTopLeft(title).dx, 72.0);
    // Adding a leading widget shouldn't effect the title's size
    expect(tester.getSize(title).width, equals(800.0 - 56.0 - 16.0 - 16.0 - 200.0));
  });

  testWidgets('AppBar centerTitle:true title overflow OK (LTR)', (WidgetTester tester) async {
    // The app bar's title should be constrained to fit within the available space
    // between the leading and actions widgets. When it's also centered it may
    // also be start or end justified if it doesn't fit in the overall center.

    final Key titleKey = UniqueKey();
    double titleWidth = 700.0;
    Widget? leading = Container();
    List<Widget> actions = <Widget>[];

    Widget buildApp() {
      return MaterialApp(
        home: Scaffold(
          appBar: AppBar(
            leading: leading,
            centerTitle: true,
            title: Container(
              key: titleKey,
              constraints: BoxConstraints.loose(Size(titleWidth, 1000.0)),
            ),
            actions: actions,
          ),
        ),
      );
    }

    // Centering a title with width 700 within the 800 pixel wide test widget
    // would mean that its start edge would have to be 50. The material spec says
    // that the start edge of the title must be at least 72.
    await tester.pumpWidget(buildApp());

    final Finder title = find.byKey(titleKey);
    expect(tester.getTopLeft(title).dx, 72.0);
    expect(tester.getSize(title).width, equals(700.0));

    // Centering a title with width 620 within the 800 pixel wide test widget
    // would mean that its start edge would have to be 90. We reserve 72
    // on the start and the padded actions occupy 96 on the end. That
    // leaves 632, so the title is end justified but its width isn't changed.

    await tester.pumpWidget(buildApp());
    leading = null;
    titleWidth = 620.0;
    actions = <Widget>[const SizedBox(width: 48.0), const SizedBox(width: 48.0)];
    await tester.pumpWidget(buildApp());
    expect(tester.getTopLeft(title).dx, 800 - 620 - 48 - 48 - 16);
    expect(tester.getSize(title).width, equals(620.0));
  });

  testWidgets('AppBar centerTitle:true title overflow OK (RTL)', (WidgetTester tester) async {
    // The app bar's title should be constrained to fit within the available space
    // between the leading and actions widgets. When it's also centered it may
    // also be start or end justified if it doesn't fit in the overall center.

    final Key titleKey = UniqueKey();
    double titleWidth = 700.0;
    Widget? leading = Container();
    List<Widget> actions = <Widget>[];

    Widget buildApp() {
      return MaterialApp(
        home: Directionality(
          textDirection: TextDirection.rtl,
          child: Scaffold(
            appBar: AppBar(
              leading: leading,
              centerTitle: true,
              title: Container(
                key: titleKey,
                constraints: BoxConstraints.loose(Size(titleWidth, 1000.0)),
              ),
              actions: actions,
            ),
          ),
        ),
      );
    }

    // Centering a title with width 700 within the 800 pixel wide test widget
    // would mean that its start edge would have to be 50. The material spec says
    // that the start edge of the title must be at least 72.
    await tester.pumpWidget(buildApp());

    final Finder title = find.byKey(titleKey);
    expect(tester.getTopRight(title).dx, 800.0 - 72.0);
    expect(tester.getSize(title).width, equals(700.0));

    // Centering a title with width 620 within the 800 pixel wide test widget
    // would mean that its start edge would have to be 90. We reserve 72
    // on the start and the padded actions occupy 96 on the end. That
    // leaves 632, so the title is end justified but its width isn't changed.

    await tester.pumpWidget(buildApp());
    leading = null;
    titleWidth = 620.0;
    actions = <Widget>[const SizedBox(width: 48.0), const SizedBox(width: 48.0)];
    await tester.pumpWidget(buildApp());
    expect(tester.getTopRight(title).dx, 620 + 48 + 48 + 16);
    expect(tester.getSize(title).width, equals(620.0));
  });

  testWidgets('AppBar with no Scaffold', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: SizedBox(
          height: kToolbarHeight,
          child: AppBar(
            leading: const Text('L'),
            title: const Text('No Scaffold'),
            actions: const <Widget>[Text('A1'), Text('A2')],
          ),
        ),
      ),
    );

    expect(find.text('L'), findsOneWidget);
    expect(find.text('No Scaffold'), findsOneWidget);
    expect(find.text('A1'), findsOneWidget);
    expect(find.text('A2'), findsOneWidget);
  });

  testWidgets('AppBar render at zero size', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Center(
          child: SizedBox.shrink(child: Scaffold(appBar: AppBar(title: const Text('X')))),
        ),
      ),
    );

    final Finder title = find.text('X');
    expect(tester.getSize(title).isEmpty, isTrue);
  });

  testWidgets('AppBar actions are vertically centered', (WidgetTester tester) async {
    final UniqueKey appBarKey = UniqueKey();
    final UniqueKey leadingKey = UniqueKey();
    final UniqueKey titleKey = UniqueKey();
    final UniqueKey action0Key = UniqueKey();
    final UniqueKey action1Key = UniqueKey();

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          appBar: AppBar(
            key: appBarKey,
            leading: SizedBox(key: leadingKey, height: 50.0),
            title: SizedBox(key: titleKey, height: 40.0),
            actions: <Widget>[
              SizedBox(key: action0Key, height: 20.0),
              SizedBox(key: action1Key, height: 30.0),
            ],
          ),
        ),
      ),
    );

    // The vertical center of the widget with key, in global coordinates.
    double yCenter(Key key) => tester.getCenter(find.byKey(key)).dy;

    expect(yCenter(appBarKey), equals(yCenter(leadingKey)));
    expect(yCenter(appBarKey), equals(yCenter(titleKey)));
    expect(yCenter(appBarKey), equals(yCenter(action0Key)));
    expect(yCenter(appBarKey), equals(yCenter(action1Key)));
  });

  testWidgets('AppBar drawer icon has default size', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(appBar: AppBar(title: const Text('Howdy!')), drawer: const Drawer()),
      ),
    );
    final double iconSize = const IconThemeData.fallback().size!;
    expect(tester.getSize(find.byIcon(Icons.menu)), equals(Size(iconSize, iconSize)));
  });

  testWidgets('Material3 - AppBar drawer icon has default color', (WidgetTester tester) async {
    final ThemeData themeData = ThemeData.from(
      colorScheme: const ColorScheme.light(),
      useMaterial3: true,
    );
    await tester.pumpWidget(
      MaterialApp(
        theme: themeData,
        home: Scaffold(appBar: AppBar(title: const Text('Howdy!')), drawer: const Drawer()),
      ),
    );

    expect(_iconStyle(tester, Icons.menu)?.color, themeData.colorScheme.onSurfaceVariant);
  });

  testWidgets('AppBar drawer icon is sized by iconTheme', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          appBar: AppBar(title: const Text('Howdy!'), iconTheme: const IconThemeData(size: 30)),
          drawer: const Drawer(),
        ),
      ),
    );
    expect(tester.getSize(find.byIcon(Icons.menu)), equals(const Size(30, 30)));
  });

  testWidgets('AppBar drawer icon is colored by iconTheme', (WidgetTester tester) async {
    final ThemeData themeData = ThemeData.from(colorScheme: const ColorScheme.light());
    const Color color = Color(0xFF2196F3);

    await tester.pumpWidget(
      MaterialApp(
        theme: themeData,
        home: Scaffold(
          appBar: AppBar(title: const Text('Howdy!'), iconTheme: const IconThemeData(color: color)),
          drawer: const Drawer(),
        ),
      ),
    );

    expect(_iconStyle(tester, Icons.menu)?.color, color);
  });

  testWidgets('AppBar endDrawer icon has default size', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(appBar: AppBar(title: const Text('Howdy!')), endDrawer: const Drawer()),
      ),
    );

    final double iconSize = const IconThemeData.fallback().size!;
    expect(tester.getSize(find.byIcon(Icons.menu)), equals(Size(iconSize, iconSize)));
  });

  testWidgets('Material3 - AppBar endDrawer icon has default color', (WidgetTester tester) async {
    final ThemeData themeData = ThemeData.from(
      colorScheme: const ColorScheme.light(),
      useMaterial3: true,
    );
    await tester.pumpWidget(
      MaterialApp(
        theme: themeData,
        home: Scaffold(appBar: AppBar(title: const Text('Howdy!')), endDrawer: const Drawer()),
      ),
    );

    expect(_iconStyle(tester, Icons.menu)?.color, themeData.colorScheme.onSurfaceVariant);
  });

  testWidgets('AppBar endDrawer icon is sized by iconTheme', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          appBar: AppBar(title: const Text('Howdy!'), iconTheme: const IconThemeData(size: 30)),
          endDrawer: const Drawer(),
        ),
      ),
    );
    expect(tester.getSize(find.byIcon(Icons.menu)), equals(const Size(30, 30)));
  });

  testWidgets('AppBar endDrawer icon is colored by iconTheme', (WidgetTester tester) async {
    final ThemeData themeData = ThemeData.from(colorScheme: const ColorScheme.light());
    const Color color = Color(0xFF2196F3);

    await tester.pumpWidget(
      MaterialApp(
        theme: themeData,
        home: Scaffold(
          appBar: AppBar(title: const Text('Howdy!'), iconTheme: const IconThemeData(color: color)),
          endDrawer: const Drawer(),
        ),
      ),
    );

    expect(_iconStyle(tester, Icons.menu)?.color, color);
  });

  testWidgets('Material3 - leading widget extends to edge and is square', (
    WidgetTester tester,
  ) async {
    final ThemeData themeData = ThemeData(platform: TargetPlatform.android, useMaterial3: true);
    await tester.pumpWidget(
      MaterialApp(
        theme: themeData,
        home: Scaffold(
          appBar: AppBar(
            leading: IconButton(icon: const Icon(Icons.menu), onPressed: () {}),
            title: const Text('X'),
          ),
          drawer: const Column(), // Doesn't really matter. Triggers a hamburger regardless.
        ),
      ),
    );

    // Default IconButton has a size of (48x48).
    final Finder hamburger = find.byType(IconButton);
    expect(tester.getTopLeft(hamburger), const Offset(4.0, 4.0));
    expect(tester.getSize(hamburger), const Size(48.0, 48.0));

    await tester.pumpWidget(
      MaterialApp(
        theme: themeData,
        home: Scaffold(appBar: AppBar(leading: Container(), title: const Text('X'))),
      ),
    );

    // Default leading widget has a size of (56x56).
    final Finder leadingBox = find.byType(Container);
    expect(tester.getTopLeft(leadingBox), Offset.zero);
    expect(tester.getSize(leadingBox), const Size(56.0, 56.0));

    // The custom leading widget should still be 56x56 even if its size is smaller.
    await tester.pumpWidget(
      MaterialApp(
        theme: themeData,
        home: Scaffold(
          appBar: AppBar(
            leading: const SizedBox(height: 36, width: 36),
            title: const Text('X'),
          ), // Doesn't really matter. Triggers a hamburger regardless.
        ),
      ),
    );

    final Finder leading = find.byType(SizedBox);
    expect(tester.getTopLeft(leading), Offset.zero);
    expect(tester.getSize(leading), const Size(56.0, 56.0));
  });

  testWidgets('Material3 - Action is 4dp from edge and 48dp min', (WidgetTester tester) async {
    final ThemeData theme = ThemeData(platform: TargetPlatform.android, useMaterial3: true);
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Scaffold(
          appBar: AppBar(
            title: const Text('X'),
            actions: const <Widget>[
              IconButton(
                icon: Icon(Icons.share),
                onPressed: null,
                tooltip: 'Share',
                iconSize: 20.0,
              ),
              IconButton(icon: Icon(Icons.add), onPressed: null, tooltip: 'Add', iconSize: 60.0),
            ],
          ),
        ),
      ),
    );

    final Finder addButton = find.widgetWithIcon(IconButton, Icons.add);
    expect(tester.getTopRight(addButton), const Offset(800.0, 0.0));
    // It's still the size it was plus the 2 * 8dp padding from IconButton.
    expect(tester.getSize(addButton), const Size(60.0 + 2 * 8.0, 56.0));

    final Finder shareButton = find.widgetWithIcon(IconButton, Icons.share);
    // The 20dp icon is expanded to fill the IconButton's touch target to 48dp.
    expect(tester.getSize(shareButton), const Size(48.0, 48.0));
  });

  testWidgets('Material3 - AppBar uses the specified elevation or defaults to 0', (
    WidgetTester tester,
  ) async {
    Widget buildAppBar([double? elevation]) {
      return MaterialApp(
        theme: ThemeData(useMaterial3: true),
        home: Scaffold(appBar: AppBar(title: const Text('Title'), elevation: elevation)),
      );
    }

    Material getMaterial() => tester.widget<Material>(
      find.descendant(of: find.byType(AppBar), matching: find.byType(Material)),
    );

    // Default elevation should be used for the material.
    await tester.pumpWidget(buildAppBar());
    expect(getMaterial().elevation, 0);

    // AppBar should use the specified elevation.
    await tester.pumpWidget(buildAppBar(8.0));
    expect(getMaterial().elevation, 8.0);
  });

  testWidgets('scrolledUnderElevation', (WidgetTester tester) async {
    Widget buildAppBar({double? elevation, double? scrolledUnderElevation}) {
      return MaterialApp(
        home: Scaffold(
          appBar: AppBar(
            title: const Text('Title'),
            elevation: elevation,
            scrolledUnderElevation: scrolledUnderElevation,
          ),
          body: ListView.builder(
            itemCount: 100,
            itemBuilder: (BuildContext context, int index) => ListTile(title: Text('Item $index')),
          ),
        ),
      );
    }

    Material getMaterial() => tester.widget<Material>(
      find.descendant(of: find.byType(AppBar), matching: find.byType(Material)),
    );

    await tester.pumpWidget(buildAppBar(elevation: 2, scrolledUnderElevation: 10));
    // Starts with the base elevation.
    expect(getMaterial().elevation, 2);

    await tester.fling(find.text('Item 2'), const Offset(0.0, -600.0), 2000.0);
    await tester.pumpAndSettle();

    // After scrolling it should be the scrolledUnderElevation.
    expect(getMaterial().elevation, 10);
  });

  testWidgets('Material3 - scrolledUnderElevation with nested scroll view', (
    WidgetTester tester,
  ) async {
    Widget buildAppBar({double? scrolledUnderElevation}) {
      return MaterialApp(
        theme: ThemeData(useMaterial3: true),
        home: Scaffold(
          appBar: AppBar(
            title: const Text('Title'),
            scrolledUnderElevation: scrolledUnderElevation,
            notificationPredicate: (ScrollNotification notification) {
              return notification.depth == 1;
            },
          ),
          body: ListView.builder(
            scrollDirection: Axis.horizontal,
            itemCount: 4,
            itemBuilder: (BuildContext context, int index) {
              return SizedBox(
                height: 600.0,
                width: 800.0,
                child: ListView.builder(
                  itemCount: 100,
                  itemBuilder:
                      (BuildContext context, int index) => ListTile(title: Text('Item $index')),
                ),
              );
            },
          ),
        ),
      );
    }

    Material getMaterial() => tester.widget<Material>(
      find.descendant(of: find.byType(AppBar), matching: find.byType(Material)),
    );

    await tester.pumpWidget(buildAppBar(scrolledUnderElevation: 10));
    // Starts with the base elevation.
    expect(getMaterial().elevation, 0.0);

    await tester.fling(find.text('Item 2'), const Offset(0.0, -600.0), 2000.0);
    await tester.pumpAndSettle();

    // After scrolling it should be the scrolledUnderElevation.
    expect(getMaterial().elevation, 10);
  });

  testWidgets('AppBar dimensions, with and without bottom, primary', (WidgetTester tester) async {
    const MediaQueryData topPadding100 = MediaQueryData(padding: EdgeInsets.only(top: 100.0));

    await tester.pumpWidget(
      Localizations(
        locale: const Locale('en', 'US'),
        delegates: const <LocalizationsDelegate<dynamic>>[
          DefaultMaterialLocalizations.delegate,
          DefaultWidgetsLocalizations.delegate,
        ],
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: MediaQuery(data: topPadding100, child: Scaffold(primary: false, appBar: AppBar())),
        ),
      ),
    );
    expect(appBarTop(tester), 0.0);
    expect(appBarHeight(tester), kToolbarHeight);

    await tester.pumpWidget(
      Localizations(
        locale: const Locale('en', 'US'),
        delegates: const <LocalizationsDelegate<dynamic>>[
          DefaultMaterialLocalizations.delegate,
          DefaultWidgetsLocalizations.delegate,
        ],
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: MediaQuery(
            data: topPadding100,
            child: Scaffold(appBar: AppBar(title: const Text('title'))),
          ),
        ),
      ),
    );
    expect(appBarTop(tester), 0.0);
    expect(tester.getTopLeft(find.text('title')).dy, greaterThan(100.0));
    expect(appBarHeight(tester), kToolbarHeight + 100.0);

    await tester.pumpWidget(
      Localizations(
        locale: const Locale('en', 'US'),
        delegates: const <LocalizationsDelegate<dynamic>>[
          DefaultMaterialLocalizations.delegate,
          DefaultWidgetsLocalizations.delegate,
        ],
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: MediaQuery(
            data: topPadding100,
            child: Scaffold(
              primary: false,
              appBar: AppBar(
                bottom: PreferredSize(
                  preferredSize: const Size.fromHeight(200.0),
                  child: Container(),
                ),
              ),
            ),
          ),
        ),
      ),
    );
    expect(appBarTop(tester), 0.0);
    expect(appBarHeight(tester), kToolbarHeight + 200.0);

    await tester.pumpWidget(
      Localizations(
        locale: const Locale('en', 'US'),
        delegates: const <LocalizationsDelegate<dynamic>>[
          DefaultMaterialLocalizations.delegate,
          DefaultWidgetsLocalizations.delegate,
        ],
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: MediaQuery(
            data: topPadding100,
            child: Scaffold(
              appBar: AppBar(
                bottom: PreferredSize(
                  preferredSize: const Size.fromHeight(200.0),
                  child: Container(),
                ),
              ),
            ),
          ),
        ),
      ),
    );
    expect(appBarTop(tester), 0.0);
    expect(appBarHeight(tester), kToolbarHeight + 100.0 + 200.0);

    await tester.pumpWidget(
      Localizations(
        locale: const Locale('en', 'US'),
        delegates: const <LocalizationsDelegate<dynamic>>[
          DefaultMaterialLocalizations.delegate,
          DefaultWidgetsLocalizations.delegate,
        ],
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: MediaQuery(
            data: topPadding100,
            child: AppBar(primary: false, title: const Text('title')),
          ),
        ),
      ),
    );
    expect(appBarTop(tester), 0.0);
    expect(tester.getTopLeft(find.text('title')).dy, lessThan(100.0));
  });

  testWidgets('AppBar in body excludes bottom SafeArea padding', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/26163
    await tester.pumpWidget(
      Localizations(
        locale: const Locale('en', 'US'),
        delegates: const <LocalizationsDelegate<dynamic>>[
          DefaultMaterialLocalizations.delegate,
          DefaultWidgetsLocalizations.delegate,
        ],
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: MediaQuery(
            data: const MediaQueryData(padding: EdgeInsets.symmetric(vertical: 100.0)),
            child: Scaffold(body: Column(children: <Widget>[AppBar(title: const Text('title'))])),
          ),
        ),
      ),
    );
    expect(appBarTop(tester), 0.0);
    expect(appBarHeight(tester), kToolbarHeight + 100.0);
  });

  testWidgets('AppBar.title sees the correct padding from MediaQuery', (WidgetTester tester) async {
    bool titleBuilt = false;
    await tester.pumpWidget(
      Localizations(
        locale: const Locale('en', 'US'),
        delegates: const <LocalizationsDelegate<dynamic>>[
          DefaultMaterialLocalizations.delegate,
          DefaultWidgetsLocalizations.delegate,
        ],
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: MediaQuery(
            data: const MediaQueryData(padding: EdgeInsets.fromLTRB(12, 34, 56, 78)),
            child: Scaffold(
              appBar: AppBar(
                title: Builder(
                  builder: (BuildContext context) {
                    titleBuilt = true;
                    final EdgeInsets padding = MediaQuery.paddingOf(context);
                    expect(padding, EdgeInsets.zero);
                    return const Text('heh');
                  },
                ),
              ),
            ),
          ),
        ),
      ),
    );
    expect(titleBuilt, isTrue);
  });

  testWidgets('AppBar updates when you add a drawer', (WidgetTester tester) async {
    await tester.pumpWidget(MaterialApp(home: Scaffold(appBar: AppBar())));
    expect(find.byIcon(Icons.menu), findsNothing);
    await tester.pumpWidget(MaterialApp(home: Scaffold(drawer: const Drawer(), appBar: AppBar())));
    expect(find.byIcon(Icons.menu), findsOneWidget);
  });

  testWidgets('AppBar does not draw menu for drawer if automaticallyImplyLeading is false', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(drawer: const Drawer(), appBar: AppBar(automaticallyImplyLeading: false)),
      ),
    );
    expect(find.byIcon(Icons.menu), findsNothing);
  });

  testWidgets('AppBar does not update the leading if a route is popped case 1', (
    WidgetTester tester,
  ) async {
    final Page<void> page1 = MaterialPage<void>(
      key: const ValueKey<String>('1'),
      child: Scaffold(key: const ValueKey<String>('1'), appBar: AppBar()),
    );
    final Page<void> page2 = MaterialPage<void>(
      key: const ValueKey<String>('2'),
      child: Scaffold(key: const ValueKey<String>('2'), appBar: AppBar()),
    );
    List<Page<void>> pages = <Page<void>>[page1];
    await tester.pumpWidget(
      MaterialApp(
        home: Navigator(pages: pages, onPopPage: (Route<dynamic> route, dynamic result) => false),
      ),
    );
    expect(find.byType(BackButton), findsNothing);
    // Update pages
    pages = <Page<void>>[page2];
    await tester.pumpWidget(
      MaterialApp(
        home: Navigator(pages: pages, onPopPage: (Route<dynamic> route, dynamic result) => false),
      ),
    );
    expect(find.byType(BackButton), findsNothing);
  });

  testWidgets('AppBar does not update the leading if a route is popped case 2', (
    WidgetTester tester,
  ) async {
    final Page<void> page1 = MaterialPage<void>(
      key: const ValueKey<String>('1'),
      child: Scaffold(key: const ValueKey<String>('1'), appBar: AppBar()),
    );
    final Page<void> page2 = MaterialPage<void>(
      key: const ValueKey<String>('2'),
      child: Scaffold(key: const ValueKey<String>('2'), appBar: AppBar()),
    );
    List<Page<void>> pages = <Page<void>>[page1, page2];
    await tester.pumpWidget(
      MaterialApp(
        home: Navigator(pages: pages, onPopPage: (Route<dynamic> route, dynamic result) => false),
      ),
    );
    // The page2 should have a back button
    expect(
      find.descendant(
        of: find.byKey(const ValueKey<String>('2')),
        matching: find.byType(BackButton),
      ),
      findsOneWidget,
    );
    // Update pages
    pages = <Page<void>>[page1];
    await tester.pumpWidget(
      MaterialApp(
        home: Navigator(pages: pages, onPopPage: (Route<dynamic> route, dynamic result) => false),
      ),
    );
    await tester.pump(const Duration(milliseconds: 10));
    // The back button should persist during the pop animation.
    expect(
      find.descendant(
        of: find.byKey(const ValueKey<String>('2')),
        matching: find.byType(BackButton),
      ),
      findsOneWidget,
    );
  });

  testWidgets('Material3 - AppBar ink splash draw on the correct canvas', (
    WidgetTester tester,
  ) async {
    // This is a regression test for https://github.com/flutter/flutter/issues/58665
    final Key key = UniqueKey();
    await tester.pumpWidget(
      MaterialApp(
        // Test was designed against InkSplash so need to make sure that is used.
        theme: ThemeData(useMaterial3: true, splashFactory: InkSplash.splashFactory),
        home: Center(
          child: AppBar(
            title: const Text('Abc'),
            actions: <Widget>[
              IconButton(
                key: key,
                icon: const Icon(Icons.add_circle),
                tooltip: 'First button',
                onPressed: () {},
              ),
            ],
            flexibleSpace: DecoratedBox(
              decoration: BoxDecoration(
                gradient: LinearGradient(
                  begin: Alignment.topCenter,
                  end: const Alignment(-0.04, 1.0),
                  colors: <Color>[Colors.blue.shade500, Colors.blue.shade800],
                ),
              ),
            ),
          ),
        ),
      ),
    );
    final RenderObject painter = tester.renderObject(
      find.descendant(
        of: find.descendant(of: find.byType(AppBar), matching: find.byType(Stack)),
        matching: find.byType(Material).last,
      ),
    );
    await tester.tap(find.byKey(key));
    expect(
      painter,
      paints
        ..save()
        ..translate()
        ..save()
        ..translate()
        ..circle(x: 20.0, y: 20.0),
    );
  });

  testWidgets('AppBar handles loose children 0', (WidgetTester tester) async {
    final GlobalKey key = GlobalKey();
    await tester.pumpWidget(
      MaterialApp(
        home: Center(
          child: AppBar(
            leading: Placeholder(key: key),
            title: const Text('Abc'),
            actions: const <Widget>[
              Placeholder(fallbackWidth: 10.0),
              Placeholder(fallbackWidth: 10.0),
              Placeholder(fallbackWidth: 10.0),
            ],
          ),
        ),
      ),
    );
    expect(tester.renderObject<RenderBox>(find.byKey(key)).localToGlobal(Offset.zero), Offset.zero);
    expect(tester.renderObject<RenderBox>(find.byKey(key)).size, const Size(56.0, 56.0));
  });

  testWidgets('AppBar handles loose children 1', (WidgetTester tester) async {
    final GlobalKey key = GlobalKey();
    await tester.pumpWidget(
      MaterialApp(
        home: Center(
          child: AppBar(
            leading: Placeholder(key: key),
            title: const Text('Abc'),
            actions: const <Widget>[
              Placeholder(fallbackWidth: 10.0),
              Placeholder(fallbackWidth: 10.0),
              Placeholder(fallbackWidth: 10.0),
            ],
            flexibleSpace: DecoratedBox(
              decoration: BoxDecoration(
                gradient: LinearGradient(
                  begin: Alignment.topCenter,
                  end: const Alignment(-0.04, 1.0),
                  colors: <Color>[Colors.blue.shade500, Colors.blue.shade800],
                ),
              ),
            ),
          ),
        ),
      ),
    );
    expect(tester.renderObject<RenderBox>(find.byKey(key)).localToGlobal(Offset.zero), Offset.zero);
    expect(tester.renderObject<RenderBox>(find.byKey(key)).size, const Size(56.0, 56.0));
  });

  testWidgets('AppBar handles loose children 2', (WidgetTester tester) async {
    final GlobalKey key = GlobalKey();
    await tester.pumpWidget(
      MaterialApp(
        home: Center(
          child: AppBar(
            leading: Placeholder(key: key),
            title: const Text('Abc'),
            actions: const <Widget>[
              Placeholder(fallbackWidth: 10.0),
              Placeholder(fallbackWidth: 10.0),
              Placeholder(fallbackWidth: 10.0),
            ],
            flexibleSpace: DecoratedBox(
              decoration: BoxDecoration(
                gradient: LinearGradient(
                  begin: Alignment.topCenter,
                  end: const Alignment(-0.04, 1.0),
                  colors: <Color>[Colors.blue.shade500, Colors.blue.shade800],
                ),
              ),
            ),
            bottom: PreferredSize(
              preferredSize: const Size(0.0, kToolbarHeight),
              child: Container(
                height: 50.0,
                padding: const EdgeInsets.all(4.0),
                child: const Placeholder(color: Color(0xFFFFFFFF)),
              ),
            ),
          ),
        ),
      ),
    );
    expect(tester.renderObject<RenderBox>(find.byKey(key)).localToGlobal(Offset.zero), Offset.zero);
    expect(tester.renderObject<RenderBox>(find.byKey(key)).size, const Size(56.0, 56.0));
  });

  testWidgets('AppBar handles loose children 3', (WidgetTester tester) async {
    final GlobalKey key = GlobalKey();
    await tester.pumpWidget(
      MaterialApp(
        home: Center(
          child: AppBar(
            leading: Placeholder(key: key),
            title: const Text('Abc'),
            actions: const <Widget>[
              Placeholder(fallbackWidth: 10.0),
              Placeholder(fallbackWidth: 10.0),
              Placeholder(fallbackWidth: 10.0),
            ],
            bottom: PreferredSize(
              preferredSize: const Size(0.0, kToolbarHeight),
              child: Container(
                height: 50.0,
                padding: const EdgeInsets.all(4.0),
                child: const Placeholder(color: Color(0xFFFFFFFF)),
              ),
            ),
          ),
        ),
      ),
    );
    expect(tester.renderObject<RenderBox>(find.byKey(key)).localToGlobal(Offset.zero), Offset.zero);
    expect(tester.renderObject<RenderBox>(find.byKey(key)).size, const Size(56.0, 56.0));
  });

  testWidgets('AppBar positioning of leading and trailing widgets with top padding', (
    WidgetTester tester,
  ) async {
    const MediaQueryData topPadding100 = MediaQueryData(padding: EdgeInsets.only(top: 100));
    final Key leadingKey = UniqueKey();
    final Key titleKey = UniqueKey();
    final Key trailingKey = UniqueKey();

    await tester.pumpWidget(
      Localizations(
        locale: const Locale('en', 'US'),
        delegates: const <LocalizationsDelegate<dynamic>>[
          DefaultMaterialLocalizations.delegate,
          DefaultWidgetsLocalizations.delegate,
        ],
        child: Directionality(
          textDirection: TextDirection.rtl,
          child: MediaQuery(
            data: topPadding100,
            child: Scaffold(
              primary: false,
              appBar: AppBar(
                leading: Placeholder(
                  key: leadingKey,
                ), // Forced to 56x56, see _kLeadingWidth in app_bar.dart.
                title: Placeholder(key: titleKey, fallbackHeight: kToolbarHeight),
                actions: <Widget>[Placeholder(key: trailingKey, fallbackWidth: 10)],
              ),
            ),
          ),
        ),
      ),
    );
    expect(tester.getTopLeft(find.byType(AppBar)), Offset.zero);
    expect(tester.getTopLeft(find.byKey(leadingKey)), const Offset(800.0 - 56.0, 100));
    expect(tester.getTopLeft(find.byKey(trailingKey)), const Offset(0.0, 100));

    // Because the topPadding eliminates the vertical space for the
    // NavigationToolbar within the AppBar, the toolbar is constrained
    // with minHeight=maxHeight=0. The _AppBarTitle widget vertically centers
    // the title, so its Y coordinate relative to the toolbar is -kToolbarHeight / 2
    // (-28). The top of the toolbar is at (screen coordinates) y=100, so the
    // top of the title is 100 + -28 = 72. The toolbar clips its contents
    // so the title isn't actually visible.
    expect(
      tester.getTopLeft(find.byKey(titleKey)),
      const Offset(10 + NavigationToolbar.kMiddleSpacing, 72),
    );
  });

  testWidgets('AppBar excludes header semantics correctly', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    await tester.pumpWidget(
      MaterialApp(
        home: Center(
          child: AppBar(
            leading: const Text('Leading'),
            title: const ExcludeSemantics(child: Text('Title')),
            excludeHeaderSemantics: true,
            actions: const <Widget>[Text('Action 1')],
          ),
        ),
      ),
    );

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics(
              children: <TestSemantics>[
                TestSemantics(
                  children: <TestSemantics>[
                    TestSemantics(
                      flags: <SemanticsFlag>[SemanticsFlag.scopesRoute],
                      children: <TestSemantics>[
                        TestSemantics(
                          children: <TestSemantics>[
                            TestSemantics(label: 'Leading', textDirection: TextDirection.ltr),
                            TestSemantics(label: 'Action 1', textDirection: TextDirection.ltr),
                          ],
                        ),
                      ],
                    ),
                  ],
                ),
              ],
            ),
          ],
        ),
        ignoreRect: true,
        ignoreTransform: true,
        ignoreId: true,
      ),
    );

    semantics.dispose();
  });

  testWidgets('Material3 - AppBar draws a light system bar for a dark background', (
    WidgetTester tester,
  ) async {
    final ThemeData darkTheme = ThemeData.dark(useMaterial3: true);
    await tester.pumpWidget(
      MaterialApp(theme: darkTheme, home: Scaffold(appBar: AppBar(title: const Text('test')))),
    );

    expect(darkTheme.colorScheme.brightness, Brightness.dark);
    expect(
      SystemChrome.latestStyle,
      const SystemUiOverlayStyle(
        statusBarColor: Colors.transparent,
        statusBarBrightness: Brightness.dark,
        statusBarIconBrightness: Brightness.light,
      ),
    );
  });

  testWidgets('Material3 - AppBar draws a dark system bar for a light background', (
    WidgetTester tester,
  ) async {
    final ThemeData lightTheme = ThemeData(useMaterial3: true);
    await tester.pumpWidget(
      MaterialApp(theme: lightTheme, home: Scaffold(appBar: AppBar(title: const Text('test')))),
    );

    expect(lightTheme.colorScheme.brightness, Brightness.light);
    expect(
      SystemChrome.latestStyle,
      const SystemUiOverlayStyle(
        statusBarColor: Colors.transparent,
        statusBarBrightness: Brightness.light,
        statusBarIconBrightness: Brightness.dark,
      ),
    );
  });

  testWidgets(
    'Material3 - Default system bar brightness based on AppBar background color brightness.',
    (WidgetTester tester) async {
      Widget buildAppBar(ThemeData theme) {
        return MaterialApp(
          theme: theme,
          home: Scaffold(appBar: AppBar(title: const Text('Title'))),
        );
      }

      // Using a light theme.
      {
        await tester.pumpWidget(buildAppBar(ThemeData(useMaterial3: true)));
        final Material appBarMaterial = tester.widget<Material>(
          find.descendant(of: find.byType(AppBar), matching: find.byType(Material)),
        );
        final Brightness appBarBrightness = ThemeData.estimateBrightnessForColor(
          appBarMaterial.color!,
        );
        final Brightness onAppBarBrightness =
            appBarBrightness == Brightness.light ? Brightness.dark : Brightness.light;

        expect(
          SystemChrome.latestStyle,
          SystemUiOverlayStyle(
            statusBarColor: Colors.transparent,
            statusBarBrightness: appBarBrightness,
            statusBarIconBrightness: onAppBarBrightness,
          ),
        );
      }

      // Using a dark theme.
      {
        await tester.pumpWidget(buildAppBar(ThemeData.dark(useMaterial3: true)));
        final Material appBarMaterial = tester.widget<Material>(
          find.descendant(of: find.byType(AppBar), matching: find.byType(Material)),
        );
        final Brightness appBarBrightness = ThemeData.estimateBrightnessForColor(
          appBarMaterial.color!,
        );
        final Brightness onAppBarBrightness =
            appBarBrightness == Brightness.light ? Brightness.dark : Brightness.light;

        expect(
          SystemChrome.latestStyle,
          SystemUiOverlayStyle(
            statusBarColor: Colors.transparent,
            statusBarBrightness: appBarBrightness,
            statusBarIconBrightness: onAppBarBrightness,
          ),
        );
      }
    },
  );

  testWidgets('Material3 - Default status bar color', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        key: GlobalKey(),
        theme: ThemeData.light().copyWith(useMaterial3: true, appBarTheme: const AppBarTheme()),
        home: Scaffold(appBar: AppBar(title: const Text('title'))),
      ),
    );

    expect(SystemChrome.latestStyle!.statusBarColor, Colors.transparent);
  });

  testWidgets('AppBar systemOverlayStyle is use to style status bar and navigation bar', (
    WidgetTester tester,
  ) async {
    final SystemUiOverlayStyle systemOverlayStyle = SystemUiOverlayStyle.light.copyWith(
      statusBarColor: Colors.red,
      systemNavigationBarColor: Colors.green,
    );
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          appBar: AppBar(title: const Text('test'), systemOverlayStyle: systemOverlayStyle),
        ),
      ),
    );

    expect(SystemChrome.latestStyle!.statusBarColor, Colors.red);
    expect(SystemChrome.latestStyle!.systemNavigationBarColor, Colors.green);
  });

  testWidgets('AppBar shape default', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: AppBar(
          leading: const Text('L'),
          title: const Text('No Scaffold'),
          actions: const <Widget>[Text('A1'), Text('A2')],
        ),
      ),
    );

    final Finder appBarFinder = find.byType(AppBar);
    AppBar getAppBarWidget(Finder finder) => tester.widget<AppBar>(finder);
    expect(getAppBarWidget(appBarFinder).shape, null);

    final Finder materialFinder = find.byType(Material);
    Material getMaterialWidget(Finder finder) => tester.widget<Material>(finder);
    expect(getMaterialWidget(materialFinder).shape, null);
  });

  testWidgets('AppBar with shape', (WidgetTester tester) async {
    const RoundedRectangleBorder roundedRectangleBorder = RoundedRectangleBorder(
      borderRadius: BorderRadius.all(Radius.circular(15.0)),
    );
    await tester.pumpWidget(
      MaterialApp(
        home: AppBar(
          leading: const Text('L'),
          title: const Text('No Scaffold'),
          actions: const <Widget>[Text('A1'), Text('A2')],
          shape: roundedRectangleBorder,
        ),
      ),
    );

    final Finder appBarFinder = find.byType(AppBar);
    AppBar getAppBarWidget(Finder finder) => tester.widget<AppBar>(finder);
    expect(getAppBarWidget(appBarFinder).shape, roundedRectangleBorder);

    final Finder materialFinder = find.byType(Material);
    Material getMaterialWidget(Finder finder) => tester.widget<Material>(finder);
    expect(getMaterialWidget(materialFinder).shape, roundedRectangleBorder);
  });

  testWidgets('AppBars title has upper limit on text scaling, textScaleFactor = 1, 1.34, 2', (
    WidgetTester tester,
  ) async {
    late double textScaleFactor;

    Widget buildFrame() {
      return MaterialApp(
        // Test designed against 2014 font sizes.
        theme: ThemeData(textTheme: Typography.englishLike2014),
        home: Builder(
          builder: (BuildContext context) {
            return MediaQuery.withClampedTextScaling(
              minScaleFactor: textScaleFactor,
              maxScaleFactor: textScaleFactor,
              child: Scaffold(
                appBar: AppBar(
                  centerTitle: false,
                  title: const Text('Jumbo', style: TextStyle(fontSize: 18)),
                ),
              ),
            );
          },
        ),
      );
    }

    final Finder appBarTitle = find.text('Jumbo');

    textScaleFactor = 1;
    await tester.pumpWidget(buildFrame());
    expect(tester.getRect(appBarTitle).height, 18);

    textScaleFactor = 1.34;
    await tester.pumpWidget(buildFrame());
    expect(tester.getRect(appBarTitle).height, 24);

    textScaleFactor = 2;
    await tester.pumpWidget(buildFrame());
    expect(tester.getRect(appBarTitle).height, 24);
  });

  testWidgets('AppBars with jumbo titles, textScaleFactor = 3, 3.5, 4', (
    WidgetTester tester,
  ) async {
    double textScaleFactor = 1.0;
    TextDirection textDirection = TextDirection.ltr;
    bool centerTitle = false;

    Widget buildFrame() {
      return MaterialApp(
        // Test designed against 2014 font sizes.
        theme: ThemeData(textTheme: Typography.englishLike2014),
        home: Builder(
          builder: (BuildContext context) {
            return Directionality(
              textDirection: textDirection,
              child: Builder(
                builder: (BuildContext context) {
                  return Scaffold(
                    appBar: AppBar(
                      centerTitle: centerTitle,
                      title: MediaQuery.withClampedTextScaling(
                        minScaleFactor: textScaleFactor,
                        maxScaleFactor: textScaleFactor,
                        child: const Text('Jumbo'),
                      ),
                    ),
                  );
                },
              ),
            );
          },
        ),
      );
    }

    final Finder appBarTitle = find.text('Jumbo');
    final Finder toolbar = find.byType(NavigationToolbar);

    // Overall screen size is 800x600
    // Left or right justified title is padded by 16 on the "start" side.
    // Toolbar height is 56.
    // "Jumbo" title is 100x20.

    await tester.pumpWidget(buildFrame());
    expect(tester.getRect(appBarTitle), const Rect.fromLTRB(16, 18, 116, 38));
    expect(tester.getCenter(appBarTitle).dy, tester.getCenter(toolbar).dy);

    textScaleFactor = 3; // "Jumbo" title is 300x60.
    await tester.pumpWidget(buildFrame());
    expect(tester.getRect(appBarTitle), const Rect.fromLTRB(16, -2, 316, 58));
    expect(tester.getCenter(appBarTitle).dy, tester.getCenter(toolbar).dy);

    textScaleFactor = 3.5; // "Jumbo" title is 350x70.
    await tester.pumpWidget(buildFrame());
    expect(tester.getRect(appBarTitle), const Rect.fromLTRB(16, -7, 366, 63));
    expect(tester.getCenter(appBarTitle).dy, tester.getCenter(toolbar).dy);

    textScaleFactor = 4; // "Jumbo" title is 400x80.
    await tester.pumpWidget(buildFrame());
    expect(tester.getRect(appBarTitle), const Rect.fromLTRB(16, -12, 416, 68));
    expect(tester.getCenter(appBarTitle).dy, tester.getCenter(toolbar).dy);

    textDirection = TextDirection.rtl; // Changed to rtl. "Jumbo" title is still 400x80.
    await tester.pumpWidget(buildFrame());
    expect(
      tester.getRect(appBarTitle),
      const Rect.fromLTRB(800.0 - 400.0 - 16.0, -12, 800.0 - 16.0, 68),
    );
    expect(tester.getCenter(appBarTitle).dy, tester.getCenter(toolbar).dy);

    centerTitle = true; // Changed to true. "Jumbo" title is still 400x80.
    await tester.pumpWidget(buildFrame());
    expect(tester.getRect(appBarTitle), const Rect.fromLTRB(200, -12, 800.0 - 200.0, 68));
    expect(tester.getCenter(appBarTitle).dy, tester.getCenter(toolbar).dy);
  });

  testWidgets('AppBar respects toolbarHeight', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          appBar: AppBar(title: const Text('Title'), toolbarHeight: 48),
          body: Container(),
        ),
      ),
    );

    expect(appBarHeight(tester), 48);
  });

  testWidgets('AppBar respects leadingWidth', (WidgetTester tester) async {
    const Key key = Key('leading');
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          appBar: AppBar(
            leading: const Placeholder(key: key),
            leadingWidth: 100,
            title: const Text('Title'),
          ),
        ),
      ),
    );

    // By default toolbarHeight is 56.0.
    expect(tester.getRect(find.byKey(key)), const Rect.fromLTRB(0, 0, 100, 56));
  });

  testWidgets("AppBar with EndDrawer doesn't have leading", (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(home: Scaffold(appBar: AppBar(), endDrawer: const Drawer())),
    );

    final Finder endDrawerFinder = find.byTooltip('Open navigation menu');
    await tester.tap(endDrawerFinder);
    await tester.pump();

    final Finder appBarFinder = find.byType(NavigationToolbar);
    NavigationToolbar getAppBarWidget(Finder finder) => tester.widget<NavigationToolbar>(finder);
    expect(getAppBarWidget(appBarFinder).leading, null);
  });

  testWidgets('AppBar.titleSpacing defaults to NavigationToolbar.kMiddleSpacing', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(home: Scaffold(appBar: AppBar(title: const Text('Title')))),
    );

    final NavigationToolbar navToolBar = tester.widget(find.byType(NavigationToolbar));
    expect(navToolBar.middleSpacing, NavigationToolbar.kMiddleSpacing);
  });

  testWidgets('AppBar foregroundColor and backgroundColor', (WidgetTester tester) async {
    const Color foregroundColor = Color(0xff00ff00);
    const Color backgroundColor = Color(0xff00ffff);
    final Key leadingIconKey = UniqueKey();
    final Key actionIconKey = UniqueKey();

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          appBar: AppBar(
            foregroundColor: foregroundColor,
            backgroundColor: backgroundColor,
            leading: Icon(Icons.add_circle, key: leadingIconKey),
            title: const Text('title'),
            actions: <Widget>[Icon(Icons.ac_unit, key: actionIconKey), const Text('action')],
          ),
        ),
      ),
    );

    final Material appBarMaterial = tester.widget<Material>(
      find.descendant(of: find.byType(AppBar), matching: find.byType(Material)),
    );
    expect(appBarMaterial.color, backgroundColor);

    final TextStyle titleTextStyle =
        tester
            .widget<DefaultTextStyle>(
              find.ancestor(of: find.text('title'), matching: find.byType(DefaultTextStyle)).first,
            )
            .style;
    expect(titleTextStyle.color, foregroundColor);

    final IconThemeData leadingIconTheme =
        tester
            .widget<IconTheme>(
              find.ancestor(of: find.byKey(leadingIconKey), matching: find.byType(IconTheme)).first,
            )
            .data;
    expect(leadingIconTheme.color, foregroundColor);

    final IconThemeData actionIconTheme =
        tester
            .widget<IconTheme>(
              find.ancestor(of: find.byKey(actionIconKey), matching: find.byType(IconTheme)).first,
            )
            .data;
    expect(actionIconTheme.color, foregroundColor);

    // Test icon color
    Color? leadingIconColor() => _iconStyle(tester, Icons.add_circle)?.color;
    Color? actionIconColor() => _iconStyle(tester, Icons.ac_unit)?.color;

    expect(leadingIconColor(), foregroundColor);
    expect(actionIconColor(), foregroundColor);
  });

  testWidgets('Leading, title, and actions show correct default colors', (
    WidgetTester tester,
  ) async {
    final ThemeData themeData = ThemeData.from(
      colorScheme: const ColorScheme.light(
        onPrimary: Colors.blue,
        onSurface: Colors.red,
        onSurfaceVariant: Colors.yellow,
      ),
    );
    final bool material3 = themeData.useMaterial3;
    await tester.pumpWidget(
      MaterialApp(
        theme: themeData,
        home: Scaffold(
          appBar: AppBar(
            leading: const Icon(Icons.add_circle),
            title: const Text('title'),
            actions: const <Widget>[Icon(Icons.ac_unit)],
          ),
        ),
      ),
    );

    Color textColor() {
      return tester.renderObject<RenderParagraph>(find.text('title')).text.style!.color!;
    }

    Color? leadingIconColor() => _iconStyle(tester, Icons.add_circle)?.color;
    Color? actionIconColor() => _iconStyle(tester, Icons.ac_unit)?.color;

    // M2 default color are onPrimary, and M3 has onSurface for leading and title,
    // onSurfaceVariant for actions.
    expect(textColor(), material3 ? Colors.red : Colors.blue);
    expect(leadingIconColor(), material3 ? Colors.red : Colors.blue);
    expect(actionIconColor(), material3 ? Colors.yellow : Colors.blue);
  });

  // Regression test for https://github.com/flutter/flutter/issues/107305
  group('Material3 - Icons are colored correctly by IconTheme and ActionIconTheme', () {
    testWidgets('Material3 - Icons and IconButtons are colored by IconTheme', (
      WidgetTester tester,
    ) async {
      const Color iconColor = Color(0xff00ff00);
      final Key leadingIconKey = UniqueKey();
      final Key actionIconKey = UniqueKey();

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData.from(colorScheme: const ColorScheme.light(), useMaterial3: true),
          home: Scaffold(
            appBar: AppBar(
              iconTheme: const IconThemeData(color: iconColor),
              leading: Icon(Icons.add_circle, key: leadingIconKey),
              title: const Text('title'),
              actions: <Widget>[
                Icon(Icons.ac_unit, key: actionIconKey),
                IconButton(icon: const Icon(Icons.add), onPressed: () {}),
              ],
            ),
          ),
        ),
      );

      Color? leadingIconColor() => _iconStyle(tester, Icons.add_circle)?.color;
      Color? actionIconColor() => _iconStyle(tester, Icons.ac_unit)?.color;
      Color? actionIconButtonColor() => _iconStyle(tester, Icons.add)?.color;

      expect(leadingIconColor(), iconColor);
      expect(actionIconColor(), iconColor);
      expect(actionIconButtonColor(), iconColor);
    });

    testWidgets('Material3 - Action icons and IconButtons are colored by ActionIconTheme', (
      WidgetTester tester,
    ) async {
      final ThemeData themeData = ThemeData.from(
        colorScheme: const ColorScheme.light(),
        useMaterial3: true,
      );

      const Color actionsIconColor = Color(0xff0000ff);
      final Key leadingIconKey = UniqueKey();
      final Key actionIconKey = UniqueKey();

      await tester.pumpWidget(
        MaterialApp(
          theme: themeData,
          home: Scaffold(
            appBar: AppBar(
              actionsIconTheme: const IconThemeData(color: actionsIconColor),
              leading: Icon(Icons.add_circle, key: leadingIconKey),
              title: const Text('title'),
              actions: <Widget>[
                Icon(Icons.ac_unit, key: actionIconKey),
                IconButton(icon: const Icon(Icons.add), onPressed: () {}),
              ],
            ),
          ),
        ),
      );

      Color? leadingIconColor() => _iconStyle(tester, Icons.add_circle)?.color;
      Color? actionIconColor() => _iconStyle(tester, Icons.ac_unit)?.color;
      Color? actionIconButtonColor() => _iconStyle(tester, Icons.add)?.color;

      expect(leadingIconColor(), themeData.colorScheme.onSurface);
      expect(actionIconColor(), actionsIconColor);
      expect(actionIconButtonColor(), actionsIconColor);
    });

    testWidgets('Material3 - The actionIconTheme property overrides iconTheme', (
      WidgetTester tester,
    ) async {
      final ThemeData themeData = ThemeData.from(
        colorScheme: const ColorScheme.light(),
        useMaterial3: true,
      );

      const Color overallIconColor = Color(0xff00ff00);
      const Color actionsIconColor = Color(0xff0000ff);
      final Key leadingIconKey = UniqueKey();
      final Key actionIconKey = UniqueKey();

      await tester.pumpWidget(
        MaterialApp(
          theme: themeData,
          home: Scaffold(
            appBar: AppBar(
              iconTheme: const IconThemeData(color: overallIconColor),
              actionsIconTheme: const IconThemeData(color: actionsIconColor),
              leading: Icon(Icons.add_circle, key: leadingIconKey),
              title: const Text('title'),
              actions: <Widget>[
                Icon(Icons.ac_unit, key: actionIconKey),
                IconButton(icon: const Icon(Icons.add), onPressed: () {}),
              ],
            ),
          ),
        ),
      );

      Color? leadingIconColor() => _iconStyle(tester, Icons.add_circle)?.color;
      Color? actionIconColor() => _iconStyle(tester, Icons.ac_unit)?.color;
      Color? actionIconButtonColor() => _iconStyle(tester, Icons.add)?.color;

      expect(leadingIconColor(), overallIconColor);
      expect(actionIconColor(), actionsIconColor);
      expect(actionIconButtonColor(), actionsIconColor);
    });

    testWidgets(
      'Material3 - AppBar.iconTheme should override any IconButtonTheme present in the theme',
      (WidgetTester tester) async {
        final ThemeData themeData = ThemeData(
          iconButtonTheme: IconButtonThemeData(
            style: IconButton.styleFrom(foregroundColor: Colors.red, iconSize: 32.0),
          ),
          useMaterial3: true,
        );

        const IconThemeData overallIconTheme = IconThemeData(color: Colors.yellow, size: 30.0);
        await tester.pumpWidget(
          MaterialApp(
            theme: themeData,
            home: Scaffold(
              appBar: AppBar(
                iconTheme: overallIconTheme,
                leading: IconButton(icon: const Icon(Icons.menu), onPressed: () {}),
                title: const Text('title'),
                actions: <Widget>[IconButton(icon: const Icon(Icons.add), onPressed: () {})],
              ),
            ),
          ),
        );

        Color? leadingIconButtonColor() => _iconStyle(tester, Icons.menu)?.color;
        double? leadingIconButtonSize() => _iconStyle(tester, Icons.menu)?.fontSize;
        Color? actionIconButtonColor() => _iconStyle(tester, Icons.add)?.color;
        double? actionIconButtonSize() => _iconStyle(tester, Icons.menu)?.fontSize;

        expect(leadingIconButtonColor(), Colors.yellow);
        expect(leadingIconButtonSize(), 30.0);
        expect(actionIconButtonColor(), Colors.yellow);
        expect(actionIconButtonSize(), 30.0);
      },
    );

    testWidgets(
      'Material3 - AppBar.iconTheme should override any IconButtonTheme present in the theme for widgets containing an iconButton',
      (WidgetTester tester) async {
        final ThemeData themeData = ThemeData(
          iconButtonTheme: IconButtonThemeData(
            style: IconButton.styleFrom(foregroundColor: Colors.red, iconSize: 32.0),
          ),
          useMaterial3: true,
        );

        const IconThemeData overallIconTheme = IconThemeData(color: Colors.yellow, size: 30.0);
        await tester.pumpWidget(
          MaterialApp(
            theme: themeData,
            home: Scaffold(
              appBar: AppBar(
                iconTheme: overallIconTheme,
                leading: BackButton(onPressed: () {}),
                title: const Text('title'),
              ),
            ),
          ),
        );

        Color? leadingIconButtonColor() => _iconStyle(tester, Icons.arrow_back)?.color;
        double? leadingIconButtonSize() => _iconStyle(tester, Icons.arrow_back)?.fontSize;

        expect(leadingIconButtonColor(), Colors.yellow);
        expect(leadingIconButtonSize(), 30.0);
      },
    );

    testWidgets(
      'Material3 - AppBar.actionsIconTheme should override any IconButtonTheme present in the theme',
      (WidgetTester tester) async {
        final ThemeData themeData = ThemeData(
          iconButtonTheme: IconButtonThemeData(
            style: IconButton.styleFrom(foregroundColor: Colors.red, iconSize: 32.0),
          ),
          useMaterial3: true,
        );

        const IconThemeData actionsIconTheme = IconThemeData(color: Colors.yellow, size: 30.0);
        await tester.pumpWidget(
          MaterialApp(
            theme: themeData,
            home: Scaffold(
              appBar: AppBar(
                actionsIconTheme: actionsIconTheme,
                title: const Text('title'),
                leading: IconButton(icon: const Icon(Icons.menu), onPressed: () {}),
                actions: <Widget>[IconButton(icon: const Icon(Icons.add), onPressed: () {})],
              ),
            ),
          ),
        );

        Color? leadingIconButtonColor() => _iconStyle(tester, Icons.menu)?.color;
        double? leadingIconButtonSize() => _iconStyle(tester, Icons.menu)?.fontSize;
        Color? actionIconButtonColor() => _iconStyle(tester, Icons.add)?.color;
        double? actionIconButtonSize() => _iconStyle(tester, Icons.add)?.fontSize;

        // The leading icon button uses the style in the IconButtonTheme because only actionsIconTheme is provided.
        expect(leadingIconButtonColor(), Colors.red);
        expect(leadingIconButtonSize(), 32.0);
        expect(actionIconButtonColor(), Colors.yellow);
        expect(actionIconButtonSize(), 30.0);
      },
    );

    testWidgets(
      'Material3 - AppBar.actionsIconTheme should override any IconButtonTheme present in the theme for widgets containing an iconButton',
      (WidgetTester tester) async {
        final ThemeData themeData = ThemeData(
          iconButtonTheme: IconButtonThemeData(
            style: IconButton.styleFrom(foregroundColor: Colors.red, iconSize: 32.0),
          ),
          useMaterial3: true,
        );

        const IconThemeData actionsIconTheme = IconThemeData(color: Colors.yellow, size: 30.0);
        await tester.pumpWidget(
          MaterialApp(
            theme: themeData,
            home: Scaffold(
              appBar: AppBar(
                actionsIconTheme: actionsIconTheme,
                title: const Text('title'),
                actions: <Widget>[BackButton(onPressed: () {})],
              ),
            ),
          ),
        );

        Color? actionIconButtonColor() => _iconStyle(tester, Icons.arrow_back)?.color;
        double? actionIconButtonSize() => _iconStyle(tester, Icons.arrow_back)?.fontSize;

        expect(actionIconButtonColor(), Colors.yellow);
        expect(actionIconButtonSize(), 30.0);
      },
    );

    testWidgets(
      'Material3 - The foregroundColor property of the AppBar overrides any IconButtonTheme present in the theme',
      (WidgetTester tester) async {
        final ThemeData themeData = ThemeData(
          iconButtonTheme: IconButtonThemeData(
            style: IconButton.styleFrom(foregroundColor: Colors.red),
          ),
          useMaterial3: true,
        );

        await tester.pumpWidget(
          MaterialApp(
            theme: themeData,
            home: Scaffold(
              appBar: AppBar(
                foregroundColor: Colors.purple,
                title: const Text('title'),
                leading: IconButton(icon: const Icon(Icons.menu), onPressed: () {}),
                actions: <Widget>[IconButton(icon: const Icon(Icons.add), onPressed: () {})],
              ),
            ),
          ),
        );

        Color? leadingIconButtonColor() => _iconStyle(tester, Icons.menu)?.color;
        Color? actionIconButtonColor() => _iconStyle(tester, Icons.add)?.color;

        expect(leadingIconButtonColor(), Colors.purple);
        expect(actionIconButtonColor(), Colors.purple);
      },
    );

    // This is a regression test for https://github.com/flutter/flutter/issues/130485.
    testWidgets('Material3 - AppBar.iconTheme is correctly applied in dark mode', (
      WidgetTester tester,
    ) async {
      final ThemeData themeData = ThemeData(
        colorScheme: const ColorScheme.dark().copyWith(onSurfaceVariant: Colors.red),
        useMaterial3: true,
      );
      await tester.pumpWidget(
        MaterialApp(
          theme: themeData,
          home: Scaffold(
            appBar: AppBar(
              iconTheme: const IconThemeData(color: Colors.white),
              leading: IconButton(icon: const Icon(Icons.menu), onPressed: () {}),
              actions: <Widget>[IconButton(icon: const Icon(Icons.add), onPressed: () {})],
            ),
          ),
        ),
      );

      Color? leadingIconButtonColor() => _iconStyle(tester, Icons.menu)?.color;
      Color? actionIconButtonColor() => _iconStyle(tester, Icons.add)?.color;

      expect(leadingIconButtonColor(), Colors.white);
      expect(actionIconButtonColor(), Colors.white);
    });

    // This is a regression test for https://github.com/flutter/flutter/issues/130485.
    testWidgets('Material3 - AppBar.foregroundColor is correctly applied in dark mode', (
      WidgetTester tester,
    ) async {
      final ThemeData themeData = ThemeData(
        colorScheme: const ColorScheme.dark().copyWith(onSurfaceVariant: Colors.red),
        useMaterial3: true,
      );
      await tester.pumpWidget(
        MaterialApp(
          theme: themeData,
          home: Scaffold(
            appBar: AppBar(
              foregroundColor: Colors.white,
              leading: IconButton(icon: const Icon(Icons.menu), onPressed: () {}),
              actions: <Widget>[IconButton(icon: const Icon(Icons.add), onPressed: () {})],
            ),
          ),
        ),
      );

      Color? leadingIconButtonColor() => _iconStyle(tester, Icons.menu)?.color;
      Color? actionIconButtonColor() => _iconStyle(tester, Icons.add)?.color;

      expect(leadingIconButtonColor(), Colors.white);
      expect(actionIconButtonColor(), Colors.white);
    });

    // This is a regression test for https://github.com/flutter/flutter/issues/130485.
    testWidgets('Material3 - AppBar.iconTheme is correctly applied in light mode', (
      WidgetTester tester,
    ) async {
      final ThemeData themeData = ThemeData(
        colorScheme: const ColorScheme.light().copyWith(onSurfaceVariant: Colors.red),
        useMaterial3: true,
      );
      await tester.pumpWidget(
        MaterialApp(
          theme: themeData,
          home: Scaffold(
            appBar: AppBar(
              iconTheme: const IconThemeData(color: Colors.black87),
              leading: IconButton(icon: const Icon(Icons.menu), onPressed: () {}),
              actions: <Widget>[IconButton(icon: const Icon(Icons.add), onPressed: () {})],
            ),
          ),
        ),
      );

      Color? leadingIconButtonColor() => _iconStyle(tester, Icons.menu)?.color;
      Color? actionIconButtonColor() => _iconStyle(tester, Icons.add)?.color;

      expect(leadingIconButtonColor(), Colors.black87);
      expect(actionIconButtonColor(), Colors.black87);
    });

    // This is a regression test for https://github.com/flutter/flutter/issues/130485.
    testWidgets('Material3 - AppBar.foregroundColor is correctly applied in light mode', (
      WidgetTester tester,
    ) async {
      final ThemeData themeData = ThemeData(
        colorScheme: const ColorScheme.light().copyWith(onSurfaceVariant: Colors.red),
        useMaterial3: true,
      );
      await tester.pumpWidget(
        MaterialApp(
          theme: themeData,
          home: Scaffold(
            appBar: AppBar(
              foregroundColor: Colors.black87,
              leading: IconButton(icon: const Icon(Icons.menu), onPressed: () {}),
              actions: <Widget>[IconButton(icon: const Icon(Icons.add), onPressed: () {})],
            ),
          ),
        ),
      );

      Color? leadingIconButtonColor() => _iconStyle(tester, Icons.menu)?.color;
      Color? actionIconButtonColor() => _iconStyle(tester, Icons.add)?.color;

      expect(leadingIconButtonColor(), Colors.black87);
      expect(actionIconButtonColor(), Colors.black87);
    });
  });

  group('MaterialStateColor scrolledUnder', () {
    const Color scrolledColor = Color(0xff00ff00);
    const Color defaultColor = Color(0xff0000ff);

    Widget buildAppBar({
      required double contentHeight,
      bool reverse = false,
      bool includeFlexibleSpace = false,
    }) {
      return MaterialApp(
        home: Scaffold(
          appBar: AppBar(
            elevation: 0,
            backgroundColor: MaterialStateColor.resolveWith((Set<MaterialState> states) {
              return states.contains(MaterialState.scrolledUnder) ? scrolledColor : defaultColor;
            }),
            title: const Text('AppBar'),
            flexibleSpace:
                includeFlexibleSpace ? const FlexibleSpaceBar(title: Text('FlexibleSpace')) : null,
          ),
          body: ListView(
            reverse: reverse,
            children: <Widget>[Container(height: contentHeight, color: Colors.teal)],
          ),
        ),
      );
    }

    testWidgets('backgroundColor for horizontal scrolling', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            appBar: AppBar(
              elevation: 0,
              backgroundColor: MaterialStateColor.resolveWith((Set<MaterialState> states) {
                return states.contains(MaterialState.scrolledUnder) ? scrolledColor : defaultColor;
              }),
              title: const Text('AppBar'),
              notificationPredicate: (ScrollNotification notification) {
                // Represents both scroll views below being treated as a
                // single viewport.
                return notification.depth <= 1;
              },
            ),
            body: SingleChildScrollView(
              child: SingleChildScrollView(
                scrollDirection: Axis.horizontal,
                child: Container(height: 1200, width: 1200, color: Colors.teal),
              ),
            ),
          ),
        ),
      );

      expect(getAppBarBackgroundColor(tester), defaultColor);
      expect(tester.getSize(findAppBarMaterial()).height, kToolbarHeight);

      TestGesture gesture = await tester.startGesture(const Offset(50.0, 400.0));
      await gesture.moveBy(const Offset(0.0, -kToolbarHeight));
      await tester.pump();
      await gesture.moveBy(const Offset(0.0, -kToolbarHeight));
      await gesture.up();
      await tester.pumpAndSettle();

      expect(getAppBarBackgroundColor(tester), scrolledColor);
      expect(tester.getSize(findAppBarMaterial()).height, kToolbarHeight);

      gesture = await tester.startGesture(const Offset(50.0, 300.0));
      // Scroll horizontally
      await gesture.moveBy(const Offset(-kToolbarHeight, 0.0));
      await tester.pump();
      await gesture.moveBy(const Offset(-kToolbarHeight, 0.0));
      await gesture.up();
      await tester.pumpAndSettle();
      // The app bar is still scrolled under vertically, so it should not have
      // changed back in response to horizontal scrolling.
      expect(getAppBarBackgroundColor(tester), scrolledColor);
      expect(tester.getSize(findAppBarMaterial()).height, kToolbarHeight);
    });

    testWidgets('backgroundColor', (WidgetTester tester) async {
      await tester.pumpWidget(buildAppBar(contentHeight: 1200.0));

      expect(getAppBarBackgroundColor(tester), defaultColor);
      expect(tester.getSize(findAppBarMaterial()).height, kToolbarHeight);

      TestGesture gesture = await tester.startGesture(const Offset(50.0, 400.0));
      await gesture.moveBy(const Offset(0.0, -kToolbarHeight));
      await gesture.up();
      await tester.pumpAndSettle();

      expect(getAppBarBackgroundColor(tester), scrolledColor);
      expect(tester.getSize(findAppBarMaterial()).height, kToolbarHeight);

      gesture = await tester.startGesture(const Offset(50.0, 300.0));
      await gesture.moveBy(const Offset(0.0, kToolbarHeight));
      await gesture.up();
      await tester.pumpAndSettle();

      expect(getAppBarBackgroundColor(tester), defaultColor);
      expect(tester.getSize(findAppBarMaterial()).height, kToolbarHeight);
    });

    testWidgets('backgroundColor with FlexibleSpace', (WidgetTester tester) async {
      await tester.pumpWidget(buildAppBar(contentHeight: 1200.0, includeFlexibleSpace: true));

      expect(getAppBarBackgroundColor(tester), defaultColor);
      expect(tester.getSize(findAppBarMaterial()).height, kToolbarHeight);

      TestGesture gesture = await tester.startGesture(const Offset(50.0, 400.0));
      await gesture.moveBy(const Offset(0.0, -kToolbarHeight));
      await gesture.up();
      await tester.pumpAndSettle();

      expect(getAppBarBackgroundColor(tester), scrolledColor);
      expect(tester.getSize(findAppBarMaterial()).height, kToolbarHeight);

      gesture = await tester.startGesture(const Offset(50.0, 300.0));
      await gesture.moveBy(const Offset(0.0, kToolbarHeight));
      await gesture.up();
      await tester.pumpAndSettle();

      expect(getAppBarBackgroundColor(tester), defaultColor);
      expect(tester.getSize(findAppBarMaterial()).height, kToolbarHeight);
    });

    testWidgets('backgroundColor - reverse', (WidgetTester tester) async {
      await tester.pumpWidget(buildAppBar(contentHeight: 1200.0, reverse: true));
      await tester.pump();

      // In this test case, the content always extends under the AppBar, so it
      // should always be the scrolledColor.
      expect(getAppBarBackgroundColor(tester), scrolledColor);
      expect(tester.getSize(findAppBarMaterial()).height, kToolbarHeight);

      TestGesture gesture = await tester.startGesture(const Offset(50.0, 400.0));
      await gesture.moveBy(const Offset(0.0, kToolbarHeight));
      await gesture.up();
      await tester.pumpAndSettle();

      expect(getAppBarBackgroundColor(tester), scrolledColor);
      expect(tester.getSize(findAppBarMaterial()).height, kToolbarHeight);

      gesture = await tester.startGesture(const Offset(50.0, 300.0));
      await gesture.moveBy(const Offset(0.0, -kToolbarHeight));
      await gesture.up();
      await tester.pumpAndSettle();

      expect(getAppBarBackgroundColor(tester), scrolledColor);
      expect(tester.getSize(findAppBarMaterial()).height, kToolbarHeight);
    });

    testWidgets('backgroundColor with FlexibleSpace - reverse', (WidgetTester tester) async {
      await tester.pumpWidget(
        buildAppBar(contentHeight: 1200.0, reverse: true, includeFlexibleSpace: true),
      );
      await tester.pump();

      // In this test case, the content always extends under the AppBar, so it
      // should always be the scrolledColor.
      expect(getAppBarBackgroundColor(tester), scrolledColor);
      expect(tester.getSize(findAppBarMaterial()).height, kToolbarHeight);

      TestGesture gesture = await tester.startGesture(const Offset(50.0, 400.0));
      await gesture.moveBy(const Offset(0.0, kToolbarHeight));
      await gesture.up();
      await tester.pumpAndSettle();

      expect(getAppBarBackgroundColor(tester), scrolledColor);
      expect(tester.getSize(findAppBarMaterial()).height, kToolbarHeight);

      gesture = await tester.startGesture(const Offset(50.0, 300.0));
      await gesture.moveBy(const Offset(0.0, -kToolbarHeight));
      await gesture.up();
      await tester.pumpAndSettle();

      expect(getAppBarBackgroundColor(tester), scrolledColor);
      expect(tester.getSize(findAppBarMaterial()).height, kToolbarHeight);
    });

    testWidgets('_handleScrollNotification safely calls setState()', (WidgetTester tester) async {
      // Regression test for failures found in Google internal issue b/185192049.
      final ScrollController controller = ScrollController(initialScrollOffset: 400);
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            appBar: AppBar(title: const Text('AppBar')),
            body: Scrollbar(
              thumbVisibility: true,
              controller: controller,
              child: ListView(
                controller: controller,
                children: <Widget>[Container(height: 1200.0, color: Colors.teal)],
              ),
            ),
          ),
        ),
      );

      expect(tester.takeException(), isNull);

      controller.dispose();
    });

    testWidgets('does not trigger on horizontal scroll', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            appBar: AppBar(
              elevation: 0,
              backgroundColor: MaterialStateColor.resolveWith((Set<MaterialState> states) {
                return states.contains(MaterialState.scrolledUnder) ? scrolledColor : defaultColor;
              }),
              title: const Text('AppBar'),
            ),
            body: ListView(
              scrollDirection: Axis.horizontal,
              children: <Widget>[Container(height: 600.0, width: 1200.0, color: Colors.teal)],
            ),
          ),
        ),
      );

      expect(getAppBarBackgroundColor(tester), defaultColor);

      TestGesture gesture = await tester.startGesture(const Offset(50.0, 400.0));
      await gesture.moveBy(const Offset(-100.0, 0.0));
      await gesture.up();
      await tester.pumpAndSettle();

      expect(getAppBarBackgroundColor(tester), defaultColor);

      gesture = await tester.startGesture(const Offset(50.0, 400.0));
      await gesture.moveBy(const Offset(100.0, 0.0));
      await gesture.up();
      await tester.pumpAndSettle();

      expect(getAppBarBackgroundColor(tester), defaultColor);
    });

    testWidgets('backgroundColor - not triggered in reverse for short content', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(buildAppBar(contentHeight: 200.0, reverse: true));
      await tester.pump();

      // In reverse, the content here is not long enough to scroll under the app
      // bar.
      expect(getAppBarBackgroundColor(tester), defaultColor);
      expect(tester.getSize(findAppBarMaterial()).height, kToolbarHeight);

      final TestGesture gesture = await tester.startGesture(const Offset(50.0, 400.0));
      await gesture.moveBy(const Offset(0.0, kToolbarHeight));
      await gesture.up();
      await tester.pumpAndSettle();

      expect(getAppBarBackgroundColor(tester), defaultColor);
      expect(tester.getSize(findAppBarMaterial()).height, kToolbarHeight);
    });

    testWidgets('backgroundColor with FlexibleSpace - not triggered in reverse for short content', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        buildAppBar(contentHeight: 200.0, reverse: true, includeFlexibleSpace: true),
      );
      await tester.pump();

      // In reverse, the content here is not long enough to scroll under the app
      // bar.
      expect(getAppBarBackgroundColor(tester), defaultColor);
      expect(tester.getSize(findAppBarMaterial()).height, kToolbarHeight);

      final TestGesture gesture = await tester.startGesture(const Offset(50.0, 400.0));
      await gesture.moveBy(const Offset(0.0, kToolbarHeight));
      await gesture.up();
      await tester.pumpAndSettle();

      expect(getAppBarBackgroundColor(tester), defaultColor);
      expect(tester.getSize(findAppBarMaterial()).height, kToolbarHeight);
    });

    testWidgets('scrolledUnderElevation should be maintained when drawer is opened', (
      WidgetTester tester,
    ) async {
      final GlobalKey drawerListKey = GlobalKey();
      final GlobalKey bodyListKey = GlobalKey();
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            appBar: AppBar(
              elevation: 0,
              backgroundColor: MaterialStateColor.resolveWith((Set<MaterialState> states) {
                return states.contains(MaterialState.scrolledUnder) ? scrolledColor : defaultColor;
              }),
              title: const Text('AppBar'),
            ),
            drawer: Drawer(
              child: ListView(
                key: drawerListKey,
                children: <Widget>[Container(height: 1200, color: Colors.red)],
              ),
            ),
            body: ListView(
              key: bodyListKey,
              children: <Widget>[Container(height: 1200, color: Colors.teal)],
            ),
          ),
        ),
      );

      // Initial state: AppBar should have the default color.
      expect(getAppBarBackgroundColor(tester), defaultColor);

      // Scroll the list view.
      await tester.drag(find.byKey(bodyListKey), const Offset(0, -300));
      await tester.pumpAndSettle();

      // The AppBar should now have the scrolled color.
      expect(getAppBarBackgroundColor(tester), scrolledColor);

      // Open the drawer.
      await tester.tap(find.byIcon(Icons.menu));
      await tester.pumpAndSettle();

      // The AppBar should still have the scrolled color.
      expect(getAppBarBackgroundColor(tester), scrolledColor);

      // Scroll the list inside the drawer.
      await tester.drag(find.byKey(drawerListKey), const Offset(0, -300));
      await tester.pumpAndSettle();

      // The AppBar should still have the scrolled color.
      expect(getAppBarBackgroundColor(tester), scrolledColor);

      // Scroll list inside the drawer back to the top.
      await tester.drag(find.byKey(drawerListKey), const Offset(0, 300));
      await tester.pumpAndSettle();

      // The AppBar should still have the scrolled color.
      expect(getAppBarBackgroundColor(tester), scrolledColor);

      // Close the drawer using the Scaffold's method.
      tester.state<ScaffoldState>(find.byType(Scaffold)).closeDrawer();
      await tester.pumpAndSettle();

      // The AppBar should still have the scrolled color.
      expect(getAppBarBackgroundColor(tester), scrolledColor);

      // Scroll the list view back to the top.
      await tester.drag(find.byKey(bodyListKey), const Offset(0, 300));
      await tester.pumpAndSettle();

      // The AppBar should be back to the default color.
      expect(getAppBarBackgroundColor(tester), defaultColor);
    });

    testWidgets('scrolledUnderElevation should be maintained when endDrawer is opened', (
      WidgetTester tester,
    ) async {
      final GlobalKey drawerListKey = GlobalKey();
      final GlobalKey bodyListKey = GlobalKey();
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            appBar: AppBar(
              elevation: 0,
              backgroundColor: MaterialStateColor.resolveWith((Set<MaterialState> states) {
                return states.contains(MaterialState.scrolledUnder) ? scrolledColor : defaultColor;
              }),
              title: const Text('AppBar'),
            ),
            endDrawer: Drawer(
              child: ListView(
                key: drawerListKey,
                children: <Widget>[Container(height: 1200, color: Colors.red)],
              ),
            ),
            body: ListView(
              key: bodyListKey,
              children: <Widget>[Container(height: 1200, color: Colors.teal)],
            ),
          ),
        ),
      );

      // Initial state: AppBar should have the default color.
      expect(getAppBarBackgroundColor(tester), defaultColor);

      // Scroll the list view.
      await tester.drag(find.byKey(bodyListKey), const Offset(0, -300));
      await tester.pumpAndSettle();

      // The AppBar should now have the scrolled color.
      expect(getAppBarBackgroundColor(tester), scrolledColor);

      // Open the drawer.
      await tester.tap(find.byIcon(Icons.menu));
      await tester.pumpAndSettle();

      // The AppBar should still have the scrolled color.
      expect(getAppBarBackgroundColor(tester), scrolledColor);

      // Scroll the list inside the drawer.
      await tester.drag(find.byKey(drawerListKey), const Offset(0, -300));
      await tester.pumpAndSettle();

      // The AppBar should still have the scrolled color.
      expect(getAppBarBackgroundColor(tester), scrolledColor);

      // Scroll list inside the drawer back to the top.
      await tester.drag(find.byKey(drawerListKey), const Offset(0, 300));
      await tester.pumpAndSettle();

      // The AppBar should still have the scrolled color.
      expect(getAppBarBackgroundColor(tester), scrolledColor);

      // Close the drawer using the Scaffold's method.
      tester.state<ScaffoldState>(find.byType(Scaffold)).closeEndDrawer();
      await tester.pumpAndSettle();

      // The AppBar should still have the scrolled color.
      expect(getAppBarBackgroundColor(tester), scrolledColor);

      // Scroll the list view back to the top.
      await tester.drag(find.byKey(bodyListKey), const Offset(0, 300));
      await tester.pumpAndSettle();

      // The AppBar should be back to the default color.
      expect(getAppBarBackgroundColor(tester), defaultColor);
    });
  });

  // Regression test for https://github.com/flutter/flutter/issues/80256
  testWidgets('The second page should have a back button even it has an end drawer', (
    WidgetTester tester,
  ) async {
    final Page<void> page1 = MaterialPage<void>(
      key: const ValueKey<String>('1'),
      child: Scaffold(
        key: const ValueKey<String>('1'),
        appBar: AppBar(),
        endDrawer: const Drawer(),
      ),
    );
    final Page<void> page2 = MaterialPage<void>(
      key: const ValueKey<String>('2'),
      child: Scaffold(
        key: const ValueKey<String>('2'),
        appBar: AppBar(),
        endDrawer: const Drawer(),
      ),
    );
    final List<Page<void>> pages = <Page<void>>[page1, page2];
    await tester.pumpWidget(
      MaterialApp(
        home: Navigator(pages: pages, onPopPage: (Route<Object?> route, Object? result) => false),
      ),
    );

    // The page2 should have a back button.
    expect(
      find.descendant(
        of: find.byKey(const ValueKey<String>('2')),
        matching: find.byType(BackButton),
      ),
      findsOneWidget,
    );
  });

  testWidgets('Only local entries that imply app bar dismissal will introduce an back button', (
    WidgetTester tester,
  ) async {
    final GlobalKey key = GlobalKey();
    await tester.pumpWidget(MaterialApp(home: Scaffold(key: key, appBar: AppBar())));
    expect(find.byType(BackButton), findsNothing);

    // Push one entry that doesn't imply app bar dismissal.
    ModalRoute.of(
      key.currentContext!,
    )!.addLocalHistoryEntry(LocalHistoryEntry(onRemove: () {}, impliesAppBarDismissal: false));
    await tester.pump();
    expect(find.byType(BackButton), findsNothing);

    // Push one entry that implies app bar dismissal.
    ModalRoute.of(key.currentContext!)!.addLocalHistoryEntry(LocalHistoryEntry(onRemove: () {}));
    await tester.pump();
    expect(find.byType(BackButton), findsOneWidget);
  });

  testWidgets('AppBar.preferredHeightFor', (WidgetTester tester) async {
    late double preferredHeight;
    late Size preferredSize;

    Widget buildFrame({double? themeToolbarHeight, double? appBarToolbarHeight}) {
      final AppBar appBar = AppBar(toolbarHeight: appBarToolbarHeight);
      return MaterialApp(
        theme: ThemeData.light().copyWith(
          appBarTheme: AppBarTheme(toolbarHeight: themeToolbarHeight),
        ),
        home: Builder(
          builder: (BuildContext context) {
            preferredHeight = AppBar.preferredHeightFor(context, appBar.preferredSize);
            preferredSize = appBar.preferredSize;
            return Scaffold(appBar: appBar, body: const Placeholder());
          },
        ),
      );
    }

    await tester.pumpWidget(buildFrame());
    expect(tester.getSize(find.byType(AppBar)).height, kToolbarHeight);
    expect(preferredHeight, kToolbarHeight);
    expect(preferredSize.height, kToolbarHeight);

    await tester.pumpWidget(buildFrame(themeToolbarHeight: 96));
    await tester.pumpAndSettle(); // Animate MaterialApp theme change.
    expect(tester.getSize(find.byType(AppBar)).height, 96);
    expect(preferredHeight, 96);
    // Special case: AppBarTheme.toolbarHeight specified,
    // AppBar.theme.toolbarHeight is null.
    expect(preferredSize.height, kToolbarHeight);

    await tester.pumpWidget(buildFrame(appBarToolbarHeight: 64));
    await tester.pumpAndSettle(); // Animate MaterialApp theme change.
    expect(tester.getSize(find.byType(AppBar)).height, 64);
    expect(preferredHeight, 64);
    expect(preferredSize.height, 64);

    await tester.pumpWidget(buildFrame(appBarToolbarHeight: 64, themeToolbarHeight: 96));
    await tester.pumpAndSettle(); // Animate MaterialApp theme change.
    expect(tester.getSize(find.byType(AppBar)).height, 64);
    expect(preferredHeight, 64);
    expect(preferredSize.height, 64);
  });

  testWidgets('AppBar title with actions should have the same position regardless of centerTitle', (
    WidgetTester tester,
  ) async {
    final Key titleKey = UniqueKey();
    bool centerTitle = false;

    Widget buildApp() {
      return MaterialApp(
        home: Scaffold(
          appBar: AppBar(
            centerTitle: centerTitle,
            title: Container(
              key: titleKey,
              constraints: BoxConstraints.loose(const Size(1000.0, 1000.0)),
            ),
            actions: const <Widget>[SizedBox(width: 48.0)],
          ),
        ),
      );
    }

    await tester.pumpWidget(buildApp());
    final Finder title = find.byKey(titleKey);
    expect(tester.getTopLeft(title).dx, 16.0);

    centerTitle = true;
    await tester.pumpWidget(buildApp());
    expect(tester.getTopLeft(title).dx, 16.0);
  });

  testWidgets('AppBar leading widget can take up arbitrary space', (WidgetTester tester) async {
    final Key leadingKey = UniqueKey();
    final Key titleKey = UniqueKey();
    late double leadingWidth;

    Widget buildApp() {
      return MaterialApp(
        home: LayoutBuilder(
          builder: (BuildContext context, BoxConstraints constraints) {
            leadingWidth = constraints.maxWidth / 2;
            return Scaffold(
              appBar: AppBar(
                leading: Container(key: leadingKey, width: leadingWidth),
                leadingWidth: leadingWidth,
                title: Text('Title', key: titleKey),
              ),
            );
          },
        ),
      );
    }

    await tester.pumpWidget(buildApp());
    expect(tester.getTopLeft(find.byKey(titleKey)).dx, leadingWidth + 16.0);
    expect(tester.getSize(find.byKey(leadingKey)).width, leadingWidth);
  });

  group('AppBar.forceMaterialTransparency', () {
    Material getAppBarMaterial(WidgetTester tester) {
      return tester.widget<Material>(
        find.descendant(of: find.byType(AppBar), matching: find.byType(Material)).first,
      );
    }

    // Generates a MaterialApp with an AppBar with a TextButton beneath it
    // (via extendBodyBehindAppBar = true).
    Widget buildWidget({required bool forceMaterialTransparency, required VoidCallback onPressed}) {
      return MaterialApp(
        home: Scaffold(
          extendBodyBehindAppBar: true,
          appBar: AppBar(
            forceMaterialTransparency: forceMaterialTransparency,
            elevation: 3,
            backgroundColor: Colors.red,
            title: const Text('AppBar'),
          ),
          body: Align(
            alignment: Alignment.topCenter,
            child: TextButton(onPressed: onPressed, child: const Text('press me')),
          ),
        ),
      );
    }

    testWidgets('forceMaterialTransparency == true allows gestures beneath the app bar', (
      WidgetTester tester,
    ) async {
      bool buttonWasPressed = false;
      final Widget widget = buildWidget(
        forceMaterialTransparency: true,
        onPressed: () {
          buttonWasPressed = true;
        },
      );
      await tester.pumpWidget(widget);

      final Material material = getAppBarMaterial(tester);
      expect(material.type, MaterialType.transparency);

      final Finder buttonFinder = find.byType(TextButton);
      await tester.tap(buttonFinder);
      await tester.pump();
      expect(buttonWasPressed, isTrue);
    });

    testWidgets('forceMaterialTransparency == false does not allow gestures beneath the app bar', (
      WidgetTester tester,
    ) async {
      // Set this, and tester.tap(warnIfMissed:false), to suppress
      // errors/warning that the button is not hittable (which is expected).
      WidgetController.hitTestWarningShouldBeFatal = false;

      bool buttonWasPressed = false;
      final Widget widget = buildWidget(
        forceMaterialTransparency: false,
        onPressed: () {
          buttonWasPressed = true;
        },
      );
      await tester.pumpWidget(widget);

      final Material material = getAppBarMaterial(tester);
      expect(material.type, MaterialType.canvas);

      final Finder buttonFinder = find.byType(TextButton);
      await tester.tap(buttonFinder, warnIfMissed: false);
      await tester.pump();
      expect(buttonWasPressed, isFalse);
    });
  });

  testWidgets('AppBar.leading size with custom IconButton', (WidgetTester tester) async {
    final Key leadingKey = UniqueKey();
    final Key titleKey = UniqueKey();
    const double titleSpacing = 16.0;
    final ThemeData theme = ThemeData();

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          appBar: AppBar(
            leading: IconButton(key: leadingKey, onPressed: () {}, icon: const Icon(Icons.menu)),
            centerTitle: false,
            title: Text('Title', key: titleKey),
          ),
        ),
      ),
    );

    final Finder buttonFinder = find.byType(IconButton);
    expect(tester.getSize(buttonFinder), const Size(48.0, 48.0));

    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(tester.getCenter(buttonFinder));
    await tester.pumpAndSettle();
    expect(
      buttonFinder,
      paints..rect(
        rect: const Rect.fromLTRB(0.0, 0.0, 40.0, 40.0),
        color: theme.colorScheme.onSurface.withOpacity(0.08),
      ),
    );

    // Get the offset of the Center widget that wraps the IconButton.
    final Offset backButtonOffset = tester.getTopRight(
      find.ancestor(of: buttonFinder, matching: find.byType(Center)),
    );
    final Offset titleOffset = tester.getTopLeft(find.byKey(titleKey));
    expect(titleOffset.dx, backButtonOffset.dx + titleSpacing);
  });

  testWidgets('AppBar.leading size with custom BackButton', (WidgetTester tester) async {
    final Key leadingKey = UniqueKey();
    final Key titleKey = UniqueKey();
    const double titleSpacing = 16.0;
    final ThemeData theme = ThemeData();

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          appBar: AppBar(
            leading: BackButton(key: leadingKey, onPressed: () {}),
            centerTitle: false,
            title: Text('Title', key: titleKey),
          ),
        ),
      ),
    );

    final Finder buttonFinder = find.byType(BackButton);
    expect(tester.getSize(buttonFinder), const Size(48.0, 48.0));

    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(tester.getCenter(buttonFinder));
    await tester.pumpAndSettle();
    expect(
      buttonFinder,
      paints..rect(
        rect: const Rect.fromLTRB(0.0, 0.0, 40.0, 40.0),
        color: theme.colorScheme.onSurface.withOpacity(0.08),
      ),
    );

    // Get the offset of the Center widget that wraps the IconButton.
    final Offset backButtonOffset = tester.getTopRight(
      find.ancestor(of: buttonFinder, matching: find.byType(Center)),
    );
    final Offset titleOffset = tester.getTopLeft(find.byKey(titleKey));
    expect(titleOffset.dx, backButtonOffset.dx + titleSpacing);
  });

  testWidgets('AppBar.leading size with custom CloseButton', (WidgetTester tester) async {
    final Key leadingKey = UniqueKey();
    final Key titleKey = UniqueKey();
    const double titleSpacing = 16.0;
    final ThemeData theme = ThemeData();

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          appBar: AppBar(
            leading: CloseButton(key: leadingKey, onPressed: () {}),
            centerTitle: false,
            title: Text('Title', key: titleKey),
          ),
        ),
      ),
    );

    final Finder buttonFinder = find.byType(CloseButton);
    expect(tester.getSize(buttonFinder), const Size(48.0, 48.0));

    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(tester.getCenter(buttonFinder));
    await tester.pumpAndSettle();
    expect(
      buttonFinder,
      paints..rect(
        rect: const Rect.fromLTRB(0.0, 0.0, 40.0, 40.0),
        color: theme.colorScheme.onSurface.withOpacity(0.08),
      ),
    );

    // Get the offset of the Center widget that wraps the IconButton.
    final Offset backButtonOffset = tester.getTopRight(
      find.ancestor(of: buttonFinder, matching: find.byType(Center)),
    );
    final Offset titleOffset = tester.getTopLeft(find.byKey(titleKey));
    expect(titleOffset.dx, backButtonOffset.dx + titleSpacing);
  });

  testWidgets('AppBar.leading size with custom DrawerButton', (WidgetTester tester) async {
    final Key leadingKey = UniqueKey();
    final Key titleKey = UniqueKey();
    const double titleSpacing = 16.0;
    final ThemeData theme = ThemeData();

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          appBar: AppBar(
            leading: DrawerButton(key: leadingKey, onPressed: () {}),
            centerTitle: false,
            title: Text('Title', key: titleKey),
          ),
        ),
      ),
    );

    final Finder buttonFinder = find.byType(DrawerButton);
    expect(tester.getSize(buttonFinder), const Size(48.0, 48.0));

    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(tester.getCenter(buttonFinder));
    await tester.pumpAndSettle();
    expect(
      buttonFinder,
      paints..rect(
        rect: const Rect.fromLTRB(0.0, 0.0, 40.0, 40.0),
        color: theme.colorScheme.onSurface.withOpacity(0.08),
      ),
    );

    // Get the offset of the Center widget that wraps the IconButton.
    final Offset backButtonOffset = tester.getTopRight(
      find.ancestor(of: buttonFinder, matching: find.byType(Center)),
    );
    final Offset titleOffset = tester.getTopLeft(find.byKey(titleKey));
    expect(titleOffset.dx, backButtonOffset.dx + titleSpacing);
  });

  // Regression test for https://github.com/flutter/flutter/issues/152315
  testWidgets('AppBar back button navigates to previous page on tap with TooltipTriggerMode.tap', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(tooltipTheme: const TooltipThemeData(triggerMode: TooltipTriggerMode.tap)),
        home: Scaffold(
          body: Center(
            child: Builder(
              builder: (BuildContext context) {
                return ElevatedButton(
                  onPressed: () {
                    Navigator.push(
                      context,
                      MaterialPageRoute<void>(
                        builder:
                            (_) => Scaffold(appBar: AppBar(title: const Text('Second Screen'))),
                      ),
                    );
                  },
                  child: const Text('Go to second screen'),
                );
              },
            ),
          ),
        ),
      ),
    );

    expect(find.text('Second Screen'), findsNothing);

    await tester.tap(find.text('Go to second screen'));
    await tester.pumpAndSettle();

    expect(find.text('Second Screen'), findsOneWidget);

    await tester.tap(find.byType(BackButton));
    await tester.pumpAndSettle();

    expect(find.text('Second Screen'), findsNothing);
  });

  // Regression test for https://github.com/flutter/flutter/issues/152315
  testWidgets(
    'Material2 - AppBar back button navigates to previous page on tap with TooltipTriggerMode.tap',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(
            useMaterial3: false,
            tooltipTheme: const TooltipThemeData(triggerMode: TooltipTriggerMode.tap),
          ),
          home: Scaffold(
            body: Center(
              child: Builder(
                builder: (BuildContext context) {
                  return ElevatedButton(
                    onPressed: () {
                      Navigator.push(
                        context,
                        MaterialPageRoute<void>(
                          builder:
                              (_) => Scaffold(appBar: AppBar(title: const Text('Second Screen'))),
                        ),
                      );
                    },
                    child: const Text('Go to second screen'),
                  );
                },
              ),
            ),
          ),
        ),
      );

      expect(find.text('Second Screen'), findsNothing);

      await tester.tap(find.text('Go to second screen'));
      await tester.pumpAndSettle();

      expect(find.text('Second Screen'), findsOneWidget);

      await tester.tap(find.byType(BackButton));
      await tester.pumpAndSettle();

      expect(find.text('Second Screen'), findsNothing);
    },
  );

  testWidgets('AppBar actions padding can be adjusted', (WidgetTester tester) async {
    final Key appBarKey = UniqueKey();
    final Key actionKey = UniqueKey();

    Widget buildAppBar({EdgeInsetsGeometry? actionsPadding}) {
      return MaterialApp(
        home: Scaffold(
          appBar: AppBar(
            key: appBarKey,
            actions: <Widget>[SizedBox.square(key: actionKey, dimension: 40.0)],
            actionsPadding: actionsPadding,
          ),
        ),
      );
    }

    await tester.pumpWidget(buildAppBar());

    // Actions padding default to zero padding.
    Offset actionsOffset = tester.getTopRight(find.byKey(actionKey));
    final Offset appBarOffset = tester.getTopRight(find.byKey(appBarKey));
    expect(appBarOffset.dx - actionsOffset.dx, 0);

    const EdgeInsets actionsPadding = EdgeInsets.only(right: 8.0);
    await tester.pumpWidget(buildAppBar(actionsPadding: actionsPadding));
    actionsOffset = tester.getTopRight(find.byKey(actionKey));
    expect(actionsOffset.dx, equals(appBarOffset.dx - actionsPadding.right));
  });

  group('Material 2', () {
    testWidgets('Material2 - AppBar draws a light system bar for a dark background', (
      WidgetTester tester,
    ) async {
      final ThemeData darkTheme = ThemeData.dark(useMaterial3: false);
      await tester.pumpWidget(
        MaterialApp(theme: darkTheme, home: Scaffold(appBar: AppBar(title: const Text('test')))),
      );

      expect(darkTheme.colorScheme.brightness, Brightness.dark);
      expect(
        SystemChrome.latestStyle,
        const SystemUiOverlayStyle(
          statusBarBrightness: Brightness.dark,
          statusBarIconBrightness: Brightness.light,
        ),
      );
    });

    testWidgets('Material2 - AppBar drawer icon has default color', (WidgetTester tester) async {
      final ThemeData themeData = ThemeData.from(
        colorScheme: const ColorScheme.light(),
        useMaterial3: false,
      );
      await tester.pumpWidget(
        MaterialApp(
          theme: themeData,
          home: Scaffold(appBar: AppBar(title: const Text('Howdy!')), drawer: const Drawer()),
        ),
      );

      expect(_iconStyle(tester, Icons.menu)?.color, themeData.colorScheme.onPrimary);
    });

    testWidgets('Material2 - AppBar endDrawer icon has default color', (WidgetTester tester) async {
      final ThemeData themeData = ThemeData.from(
        colorScheme: const ColorScheme.light(),
        useMaterial3: false,
      );
      await tester.pumpWidget(
        MaterialApp(
          theme: themeData,
          home: Scaffold(appBar: AppBar(title: const Text('Howdy!')), endDrawer: const Drawer()),
        ),
      );

      expect(_iconStyle(tester, Icons.menu)?.color, themeData.colorScheme.onPrimary);
    });

    testWidgets('Material2 - leading widget extends to edge and is square', (
      WidgetTester tester,
    ) async {
      final ThemeData themeData = ThemeData(platform: TargetPlatform.android, useMaterial3: false);
      await tester.pumpWidget(
        MaterialApp(
          theme: themeData,
          home: Scaffold(
            appBar: AppBar(
              leading: IconButton(icon: const Icon(Icons.menu), onPressed: () {}),
              title: const Text('X'),
            ),
            drawer: const Column(), // Doesn't really matter. Triggers a hamburger regardless.
          ),
        ),
      );

      // Default IconButton has a size of (56x56).
      final Finder hamburger = find.byType(IconButton);
      expect(tester.getTopLeft(hamburger), Offset.zero);
      expect(tester.getSize(hamburger), const Size(56.0, 56.0));

      await tester.pumpWidget(
        MaterialApp(
          theme: themeData,
          home: Scaffold(appBar: AppBar(leading: Container(), title: const Text('X'))),
        ),
      );

      // Default leading widget has a size of (56x56).
      final Finder leadingBox = find.byType(Container);
      expect(tester.getTopLeft(leadingBox), Offset.zero);
      expect(tester.getSize(leadingBox), const Size(56.0, 56.0));

      // The custom leading widget should still be 56x56 even if its size is smaller.
      await tester.pumpWidget(
        MaterialApp(
          theme: themeData,
          home: Scaffold(
            appBar: AppBar(
              leading: const SizedBox(height: 36, width: 36),
              title: const Text('X'),
            ), // Doesn't really matter. Triggers a hamburger regardless.
          ),
        ),
      );

      final Finder leading = find.byType(SizedBox);
      expect(tester.getTopLeft(leading), Offset.zero);
      expect(tester.getSize(leading), const Size(56.0, 56.0));
    });

    testWidgets('Material2 - Action is 4dp from edge and 48dp min', (WidgetTester tester) async {
      final ThemeData theme = ThemeData(platform: TargetPlatform.android, useMaterial3: false);
      await tester.pumpWidget(
        MaterialApp(
          theme: theme,
          home: Scaffold(
            appBar: AppBar(
              title: const Text('X'),
              actions: const <Widget>[
                IconButton(
                  icon: Icon(Icons.share),
                  onPressed: null,
                  tooltip: 'Share',
                  iconSize: 20.0,
                ),
                IconButton(icon: Icon(Icons.add), onPressed: null, tooltip: 'Add', iconSize: 60.0),
              ],
            ),
          ),
        ),
      );

      final Finder addButton = find.widgetWithIcon(IconButton, Icons.add);
      expect(tester.getTopRight(addButton), const Offset(800.0, 0.0));
      // It's still the size it was plus the 2 * 8dp padding from IconButton.
      expect(tester.getSize(addButton), const Size(60.0 + 2 * 8.0, 56.0));

      final Finder shareButton = find.widgetWithIcon(IconButton, Icons.share);
      // The 20dp icon is expanded to fill the IconButton's touch target to 48dp.
      expect(tester.getSize(shareButton), const Size(48.0, 56.0));
    });

    testWidgets('Material2 - AppBar uses the specified elevation or defaults to 4.0', (
      WidgetTester tester,
    ) async {
      Widget buildAppBar([double? elevation]) {
        return MaterialApp(
          theme: ThemeData(useMaterial3: false),
          home: Scaffold(appBar: AppBar(title: const Text('Title'), elevation: elevation)),
        );
      }

      Material getMaterial() => tester.widget<Material>(
        find.descendant(of: find.byType(AppBar), matching: find.byType(Material)),
      );

      // Default elevation should be used for the material.
      await tester.pumpWidget(buildAppBar());
      expect(getMaterial().elevation, 4);

      // AppBar should use the specified elevation.
      await tester.pumpWidget(buildAppBar(8.0));
      expect(getMaterial().elevation, 8.0);
    });

    testWidgets('Material2 - AppBar ink splash draw on the correct canvas', (
      WidgetTester tester,
    ) async {
      // This is a regression test for https://github.com/flutter/flutter/issues/58665
      final Key key = UniqueKey();
      await tester.pumpWidget(
        MaterialApp(
          // Test was designed against InkSplash so need to make sure that is used.
          theme: ThemeData(useMaterial3: false, splashFactory: InkSplash.splashFactory),
          home: Center(
            child: AppBar(
              title: const Text('Abc'),
              actions: <Widget>[
                IconButton(
                  key: key,
                  icon: const Icon(Icons.add_circle),
                  tooltip: 'First button',
                  onPressed: () {},
                ),
              ],
              flexibleSpace: DecoratedBox(
                decoration: BoxDecoration(
                  gradient: LinearGradient(
                    begin: Alignment.topCenter,
                    end: const Alignment(-0.04, 1.0),
                    colors: <Color>[Colors.blue.shade500, Colors.blue.shade800],
                  ),
                ),
              ),
            ),
          ),
        ),
      );
      final RenderObject painter = tester.renderObject(
        find.descendant(
          of: find.descendant(of: find.byType(AppBar), matching: find.byType(Stack)),
          matching: find.byType(Material),
        ),
      );
      await tester.tap(find.byKey(key));
      expect(
        painter,
        paints
          ..save()
          ..translate()
          ..save()
          ..translate()
          ..circle(x: 24.0, y: 28.0),
      );
    });

    testWidgets('Material2 - Default status bar color', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          key: GlobalKey(),
          theme: ThemeData.light().copyWith(useMaterial3: false, appBarTheme: const AppBarTheme()),
          home: Scaffold(appBar: AppBar(title: const Text('title'))),
        ),
      );

      expect(SystemChrome.latestStyle!.statusBarColor, null);
    });

    testWidgets('Material2 - AppBar draws a dark system bar for a light background', (
      WidgetTester tester,
    ) async {
      final ThemeData lightTheme = ThemeData(primarySwatch: Colors.lightBlue, useMaterial3: false);
      await tester.pumpWidget(
        MaterialApp(theme: lightTheme, home: Scaffold(appBar: AppBar(title: const Text('test')))),
      );

      expect(lightTheme.colorScheme.brightness, Brightness.light);
      expect(
        SystemChrome.latestStyle,
        const SystemUiOverlayStyle(
          statusBarBrightness: Brightness.light,
          statusBarIconBrightness: Brightness.dark,
        ),
      );
    });

    testWidgets(
      'Material2 - Default system bar brightness based on AppBar background color brightness.',
      (WidgetTester tester) async {
        Widget buildAppBar(ThemeData theme) {
          return MaterialApp(
            theme: theme,
            home: Scaffold(appBar: AppBar(title: const Text('Title'))),
          );
        }

        // Using a light theme.
        {
          await tester.pumpWidget(buildAppBar(ThemeData(useMaterial3: false)));
          final Material appBarMaterial = tester.widget<Material>(
            find.descendant(of: find.byType(AppBar), matching: find.byType(Material)),
          );
          final Brightness appBarBrightness = ThemeData.estimateBrightnessForColor(
            appBarMaterial.color!,
          );
          final Brightness onAppBarBrightness =
              appBarBrightness == Brightness.light ? Brightness.dark : Brightness.light;

          expect(
            SystemChrome.latestStyle,
            SystemUiOverlayStyle(
              statusBarBrightness: appBarBrightness,
              statusBarIconBrightness: onAppBarBrightness,
            ),
          );
        }

        // Using a dark theme.
        {
          await tester.pumpWidget(buildAppBar(ThemeData.dark(useMaterial3: false)));
          final Material appBarMaterial = tester.widget<Material>(
            find.descendant(of: find.byType(AppBar), matching: find.byType(Material)),
          );
          final Brightness appBarBrightness = ThemeData.estimateBrightnessForColor(
            appBarMaterial.color!,
          );
          final Brightness onAppBarBrightness =
              appBarBrightness == Brightness.light ? Brightness.dark : Brightness.light;

          expect(
            SystemChrome.latestStyle,
            SystemUiOverlayStyle(
              statusBarBrightness: appBarBrightness,
              statusBarIconBrightness: onAppBarBrightness,
            ),
          );
        }
      },
    );
  });
}
