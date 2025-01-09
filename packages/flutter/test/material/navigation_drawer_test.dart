// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Navigation drawer updates destinations when tapped', (WidgetTester tester) async {
    int mutatedIndex = -1;
    final GlobalKey<ScaffoldState> scaffoldKey = GlobalKey<ScaffoldState>();
    final ThemeData theme = ThemeData();
    widgetSetup(tester, 3000, viewHeight: 3000);
    final Widget widget = _buildWidget(
      scaffoldKey,
      NavigationDrawer(
        children: <Widget>[
          Text('Headline', style: theme.textTheme.bodyLarge),
          NavigationDrawerDestination(
            icon: Icon(Icons.ac_unit, color: theme.iconTheme.color),
            label: Text('AC', style: theme.textTheme.bodySmall),
          ),
          NavigationDrawerDestination(
            icon: Icon(Icons.access_alarm, color: theme.iconTheme.color),
            label: Text('Alarm', style: theme.textTheme.bodySmall),
          ),
        ],
        onDestinationSelected: (int i) {
          mutatedIndex = i;
        },
      ),
    );

    await tester.pumpWidget(widget);
    scaffoldKey.currentState!.openDrawer();
    await tester.pump();

    expect(find.text('Headline'), findsOneWidget);
    expect(find.text('AC'), findsOneWidget);
    expect(find.text('Alarm'), findsOneWidget);

    await tester.pump(const Duration(seconds: 1)); // animation done

    await tester.tap(find.text('Alarm'));
    expect(mutatedIndex, 1);

    await tester.tap(find.text('AC'));
    expect(mutatedIndex, 0);
  });

  testWidgets('NavigationDrawer can update background color', (WidgetTester tester) async {
    const Color color = Colors.yellow;
    final GlobalKey<ScaffoldState> scaffoldKey = GlobalKey<ScaffoldState>();
    final ThemeData theme = ThemeData();

    await tester.pumpWidget(
      _buildWidget(
        scaffoldKey,
        NavigationDrawer(
          backgroundColor: color,
          children: <Widget>[
            Text('Headline', style: theme.textTheme.bodyLarge),
            NavigationDrawerDestination(
              icon: Icon(Icons.ac_unit, color: theme.iconTheme.color),
              label: Text('AC', style: theme.textTheme.bodySmall),
            ),
            NavigationDrawerDestination(
              icon: Icon(Icons.access_alarm, color: theme.iconTheme.color),
              label: Text('Alarm', style: theme.textTheme.bodySmall),
            ),
          ],
          onDestinationSelected: (int i) {},
        ),
      ),
    );

    scaffoldKey.currentState!.openDrawer();
    await tester.pump(const Duration(seconds: 1)); // animation done

    expect(_getMaterial(tester).color, equals(color));
  });

  testWidgets('NavigationDestinationDrawer background color is customizable', (
    WidgetTester tester,
  ) async {
    const Color color = Colors.yellow;
    final GlobalKey<ScaffoldState> scaffoldKey = GlobalKey<ScaffoldState>();
    final ThemeData theme = ThemeData();

    await tester.pumpWidget(
      _buildWidget(
        scaffoldKey,
        NavigationDrawer(
          children: <Widget>[
            Text('Headline', style: theme.textTheme.bodyLarge),
            NavigationDrawerDestination(
              icon: Icon(Icons.ac_unit, color: theme.iconTheme.color),
              label: Text('AC', style: theme.textTheme.bodySmall),
            ),
            NavigationDrawerDestination(
              icon: Icon(Icons.access_alarm, color: theme.iconTheme.color),
              label: Text('Alarm', style: theme.textTheme.bodySmall),
              backgroundColor: color,
            ),
          ],
          onDestinationSelected: (int i) {},
        ),
      ),
    );

    Finder findDestinationInk(String label) {
      return find.descendant(
        of: find.ancestor(of: find.text(label), matching: find.byType(NavigationDrawerDestination)),
        matching: find.byType(Ink),
      );
    }

    scaffoldKey.currentState!.openDrawer();
    await tester.pump();
    await tester.pump(const Duration(seconds: 1)); // Animation done.

    // Destination with no custom background color.
    await tester.tap(find.text('AC'));
    await tester.pump();

    expect(findDestinationInk('AC'), findsNothing);

    // Destination with a custom background color.
    await tester.tap(find.byIcon(Icons.access_alarm));
    await tester.pump();

    // A Material is added with the custom color.
    expect(findDestinationInk('Alarm'), findsOne);
    final BoxDecoration destinationDecoration =
        tester.firstWidget<Ink>(findDestinationInk('Alarm')).decoration! as BoxDecoration;
    expect(destinationDecoration.color, color);
  });

  testWidgets('NavigationDrawer can update elevation', (WidgetTester tester) async {
    const double elevation = 42.0;
    final GlobalKey<ScaffoldState> scaffoldKey = GlobalKey<ScaffoldState>();
    final ThemeData theme = ThemeData();
    final NavigationDrawer drawer = NavigationDrawer(
      elevation: elevation,
      children: <Widget>[
        Text('Headline', style: theme.textTheme.bodyLarge),
        NavigationDrawerDestination(
          icon: Icon(Icons.ac_unit, color: theme.iconTheme.color),
          label: Text('AC', style: theme.textTheme.bodySmall),
        ),
        NavigationDrawerDestination(
          icon: Icon(Icons.access_alarm, color: theme.iconTheme.color),
          label: Text('Alarm', style: theme.textTheme.bodySmall),
        ),
      ],
    );

    await tester.pumpWidget(_buildWidget(scaffoldKey, drawer));
    scaffoldKey.currentState!.openDrawer();
    await tester.pump(const Duration(seconds: 1));

    expect(_getMaterial(tester).elevation, equals(elevation));
  });

  testWidgets('NavigationDrawer uses proper defaults when no parameters are given', (
    WidgetTester tester,
  ) async {
    final GlobalKey<ScaffoldState> scaffoldKey = GlobalKey<ScaffoldState>();
    final ThemeData theme = ThemeData(useMaterial3: true);
    await tester.pumpWidget(
      _buildWidget(
        scaffoldKey,
        NavigationDrawer(
          children: <Widget>[
            Text('Headline', style: theme.textTheme.bodyLarge),
            const NavigationDrawerDestination(icon: Icon(Icons.ac_unit), label: Text('AC')),
            const NavigationDrawerDestination(icon: Icon(Icons.access_alarm), label: Text('Alarm')),
          ],
          onDestinationSelected: (int i) {},
        ),
        useMaterial3: theme.useMaterial3,
      ),
    );
    scaffoldKey.currentState!.openDrawer();
    await tester.pump(const Duration(seconds: 1));

    // Test drawer Material.
    expect(_getMaterial(tester).color, theme.colorScheme.surfaceContainerLow);
    expect(_getMaterial(tester).surfaceTintColor, Colors.transparent);
    expect(_getMaterial(tester).shadowColor, Colors.transparent);
    expect(_getMaterial(tester).elevation, 1);
    // Test indicator decoration.
    expect(_getIndicatorDecoration(tester)?.color, theme.colorScheme.secondaryContainer);
    expect(_getIndicatorDecoration(tester)?.shape, const StadiumBorder());
    // Test selected and unselected icon colors.
    expect(_iconStyle(tester, Icons.ac_unit)?.color, theme.colorScheme.onSecondaryContainer);
    expect(_iconStyle(tester, Icons.access_alarm)?.color, theme.colorScheme.onSurfaceVariant);
    // Test selected and unselected label colors.
    expect(_labelStyle(tester, 'AC')?.color, theme.colorScheme.onSecondaryContainer);
    expect(_labelStyle(tester, 'Alarm')?.color, theme.colorScheme.onSurfaceVariant);
    // Test that the icon and label are the correct size.
    RenderBox iconBox = tester.renderObject(find.byIcon(Icons.ac_unit));
    expect(iconBox.size, const Size(24.0, 24.0));
    iconBox = tester.renderObject(find.byIcon(Icons.access_alarm));
    expect(iconBox.size, const Size(24.0, 24.0));
  });

  testWidgets('Navigation drawer is scrollable', (WidgetTester tester) async {
    final GlobalKey<ScaffoldState> scaffoldKey = GlobalKey<ScaffoldState>();
    widgetSetup(tester, 500, viewHeight: 300);
    await tester.pumpWidget(
      _buildWidget(
        scaffoldKey,
        NavigationDrawer(
          children: <Widget>[
            for (int i = 0; i < 100; i++)
              NavigationDrawerDestination(icon: const Icon(Icons.ac_unit), label: Text('Label$i')),
          ],
          onDestinationSelected: (int i) {},
        ),
      ),
    );
    scaffoldKey.currentState!.openDrawer();
    await tester.pump(const Duration(seconds: 1));

    expect(find.text('Label0'), findsOneWidget);
    expect(find.text('Label1'), findsOneWidget);
    expect(find.text('Label2'), findsOneWidget);
    expect(find.text('Label3'), findsOneWidget);
    expect(find.text('Label4'), findsOneWidget);
    expect(find.text('Label5'), findsOneWidget);
    expect(find.text('Label6'), findsNothing);
    expect(find.text('Label7'), findsNothing);
    expect(find.text('Label8'), findsNothing);

    await tester.dragFrom(const Offset(0, 200), const Offset(0.0, -200));
    await tester.pump();

    expect(find.text('Label0'), findsNothing);
    expect(find.text('Label1'), findsNothing);
    expect(find.text('Label2'), findsNothing);
    expect(find.text('Label3'), findsOneWidget);
    expect(find.text('Label4'), findsOneWidget);
    expect(find.text('Label5'), findsOneWidget);
    expect(find.text('Label6'), findsOneWidget);
    expect(find.text('Label7'), findsOneWidget);
    expect(find.text('Label8'), findsOneWidget);
    expect(find.text('Label9'), findsNothing);
    expect(find.text('Label10'), findsNothing);
  });

  testWidgets('Safe Area test', (WidgetTester tester) async {
    final GlobalKey<ScaffoldState> scaffoldKey = GlobalKey<ScaffoldState>();
    const double viewHeight = 300;
    widgetSetup(tester, 500, viewHeight: viewHeight);
    await tester.pumpWidget(
      MediaQuery(
        data: const MediaQueryData(padding: EdgeInsets.all(20.0)),
        child: MaterialApp(
          useInheritedMediaQuery: true,
          theme: ThemeData.light(),
          home: Scaffold(
            key: scaffoldKey,
            drawer: NavigationDrawer(
              children: <Widget>[
                for (int i = 0; i < 10; i++)
                  NavigationDrawerDestination(
                    icon: const Icon(Icons.ac_unit),
                    label: Text('Label$i'),
                  ),
              ],
              onDestinationSelected: (int i) {},
            ),
            body: Container(),
          ),
        ),
      ),
    );
    scaffoldKey.currentState!.openDrawer();
    await tester.pump();
    await tester.pump(const Duration(seconds: 1));

    // Safe area padding on the top and sides.
    expect(
      tester.getTopLeft(find.widgetWithText(NavigationDrawerDestination, 'Label0')),
      const Offset(20.0, 20.0),
    );

    // No Safe area padding at the bottom.
    expect(
      tester.getBottomRight(find.widgetWithText(NavigationDrawerDestination, 'Label4')).dy,
      viewHeight,
    );
  });

  testWidgets('Navigation drawer semantics', (WidgetTester tester) async {
    final GlobalKey<ScaffoldState> scaffoldKey = GlobalKey<ScaffoldState>();
    final ThemeData theme = ThemeData();
    Widget widget({int selectedIndex = 0}) {
      return _buildWidget(
        scaffoldKey,
        NavigationDrawer(
          selectedIndex: selectedIndex,
          children: <Widget>[
            Text('Headline', style: theme.textTheme.bodyLarge),
            NavigationDrawerDestination(
              icon: Icon(Icons.ac_unit, color: theme.iconTheme.color),
              label: Text('AC', style: theme.textTheme.bodySmall),
            ),
            NavigationDrawerDestination(
              icon: Icon(Icons.access_alarm, color: theme.iconTheme.color),
              label: Text('Alarm', style: theme.textTheme.bodySmall),
            ),
          ],
        ),
      );
    }

    await tester.pumpWidget(widget());
    scaffoldKey.currentState!.openDrawer();
    await tester.pump(const Duration(seconds: 1));

    expect(
      tester.getSemantics(find.text('AC')),
      matchesSemantics(
        label: 'AC\nTab 1 of 2',
        textDirection: TextDirection.ltr,
        isFocusable: true,
        isSelected: true,
        hasTapAction: true,
        hasFocusAction: true,
      ),
    );
    expect(
      tester.getSemantics(find.text('Alarm')),
      matchesSemantics(
        label: 'Alarm\nTab 2 of 2',
        textDirection: TextDirection.ltr,
        isFocusable: true,
        hasTapAction: true,
        hasFocusAction: true,
      ),
    );

    await tester.pumpWidget(widget(selectedIndex: 1));

    expect(
      tester.getSemantics(find.text('AC')),
      matchesSemantics(
        label: 'AC\nTab 1 of 2',
        textDirection: TextDirection.ltr,
        isFocusable: true,
        hasTapAction: true,
        hasFocusAction: true,
      ),
    );
    expect(
      tester.getSemantics(find.text('Alarm')),
      matchesSemantics(
        label: 'Alarm\nTab 2 of 2',
        textDirection: TextDirection.ltr,
        isFocusable: true,
        isSelected: true,
        hasTapAction: true,
        hasFocusAction: true,
      ),
    );
  });

  testWidgets('Navigation destination updates indicator color and shape', (
    WidgetTester tester,
  ) async {
    final GlobalKey<ScaffoldState> scaffoldKey = GlobalKey<ScaffoldState>();
    final ThemeData theme = ThemeData(useMaterial3: true);
    const Color color = Color(0xff0000ff);
    const ShapeBorder shape = RoundedRectangleBorder();

    Widget buildNavigationDrawer({Color? indicatorColor, ShapeBorder? indicatorShape}) {
      return MaterialApp(
        theme: theme,
        home: Scaffold(
          key: scaffoldKey,
          drawer: NavigationDrawer(
            indicatorColor: indicatorColor,
            indicatorShape: indicatorShape,
            children: <Widget>[
              Text('Headline', style: theme.textTheme.bodyLarge),
              const NavigationDrawerDestination(icon: Icon(Icons.ac_unit), label: Text('AC')),
              const NavigationDrawerDestination(
                icon: Icon(Icons.access_alarm),
                label: Text('Alarm'),
              ),
            ],
            onDestinationSelected: (int i) {},
          ),
          body: Container(),
        ),
      );
    }

    await tester.pumpWidget(buildNavigationDrawer());
    scaffoldKey.currentState!.openDrawer();
    await tester.pumpAndSettle();

    // Test default indicator color and shape.
    expect(_getIndicatorDecoration(tester)?.color, theme.colorScheme.secondaryContainer);
    expect(_getIndicatorDecoration(tester)?.shape, const StadiumBorder());
    // Test that InkWell for hover, focus and pressed use default shape.
    expect(_getInkWell(tester)?.customBorder, const StadiumBorder());

    await tester.pumpWidget(buildNavigationDrawer(indicatorColor: color, indicatorShape: shape));

    // Test custom indicator color and shape.
    expect(_getIndicatorDecoration(tester)?.color, color);
    expect(_getIndicatorDecoration(tester)?.shape, shape);
    // Test that InkWell for hover, focus and pressed use custom shape.
    expect(_getInkWell(tester)?.customBorder, shape);
  });

  testWidgets('NavigationDrawer.tilePadding defaults to EdgeInsets.symmetric(horizontal: 12.0)', (
    WidgetTester tester,
  ) async {
    final GlobalKey<ScaffoldState> scaffoldKey = GlobalKey<ScaffoldState>();
    widgetSetup(tester, 3000, viewHeight: 3000);
    final Widget widget = _buildWidget(
      scaffoldKey,
      NavigationDrawer(
        children: const <Widget>[
          NavigationDrawerDestination(icon: Icon(Icons.ac_unit), label: Text('AC')),
        ],
        onDestinationSelected: (int i) {},
      ),
    );

    await tester.pumpWidget(widget);
    scaffoldKey.currentState?.openDrawer();
    await tester.pump();
    final NavigationDrawer drawer = tester.widget(find.byType(NavigationDrawer));
    expect(drawer.tilePadding, const EdgeInsets.symmetric(horizontal: 12.0));
  });

  testWidgets('Destinations respect their disabled state', (WidgetTester tester) async {
    final GlobalKey<ScaffoldState> scaffoldKey = GlobalKey<ScaffoldState>();
    int selectedIndex = 0;

    widgetSetup(tester, 800);

    final Widget widget = _buildWidget(
      scaffoldKey,
      NavigationDrawer(
        children: const <Widget>[
          NavigationDrawerDestination(icon: Icon(Icons.ac_unit), label: Text('AC')),
          NavigationDrawerDestination(icon: Icon(Icons.access_alarm), label: Text('Alarm')),
          NavigationDrawerDestination(
            icon: Icon(Icons.accessible),
            label: Text('Accessible'),
            enabled: false,
          ),
        ],
        onDestinationSelected: (int i) {
          selectedIndex = i;
        },
      ),
    );

    await tester.pumpWidget(widget);
    scaffoldKey.currentState!.openDrawer();
    await tester.pump();

    expect(find.text('AC'), findsOneWidget);
    expect(find.text('Alarm'), findsOneWidget);
    expect(find.text('Accessible'), findsOneWidget);

    await tester.pump(const Duration(seconds: 1));

    expect(selectedIndex, 0);

    await tester.tap(find.text('Alarm'));
    expect(selectedIndex, 1);

    await tester.tap(find.text('Accessible'));
    expect(selectedIndex, 1);

    await tester.pumpAndSettle();
  });
}

Widget _buildWidget(GlobalKey<ScaffoldState> scaffoldKey, Widget child, {bool? useMaterial3}) {
  return MaterialApp(
    theme: ThemeData(useMaterial3: useMaterial3),
    home: Scaffold(key: scaffoldKey, drawer: child, body: Container()),
  );
}

Material _getMaterial(WidgetTester tester) {
  return tester.firstWidget<Material>(
    find.descendant(of: find.byType(NavigationDrawer), matching: find.byType(Material)),
  );
}

InkWell? _getInkWell(WidgetTester tester) {
  return tester.firstWidget<InkWell>(
    find.descendant(of: find.byType(NavigationDrawer), matching: find.byType(InkWell)),
  );
}

ShapeDecoration? _getIndicatorDecoration(WidgetTester tester) {
  return tester
          .firstWidget<Container>(
            find.descendant(of: find.byType(FadeTransition), matching: find.byType(Container)),
          )
          .decoration
      as ShapeDecoration?;
}

TextStyle? _iconStyle(WidgetTester tester, IconData icon) {
  final RichText iconRichText = tester.widget<RichText>(
    find.descendant(of: find.byIcon(icon), matching: find.byType(RichText)),
  );
  return iconRichText.text.style;
}

TextStyle? _labelStyle(WidgetTester tester, String label) {
  final RichText labelRichText = tester.widget<RichText>(
    find.descendant(of: find.text(label), matching: find.byType(RichText)),
  );
  return labelRichText.text.style;
}

void widgetSetup(WidgetTester tester, double viewWidth, {double viewHeight = 1000}) {
  tester.view.devicePixelRatio = 2;
  final double dpi = tester.view.devicePixelRatio;
  tester.view.physicalSize = Size(viewWidth * dpi, viewHeight * dpi);
}
