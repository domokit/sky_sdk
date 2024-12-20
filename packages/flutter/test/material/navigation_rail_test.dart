// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

import '../widgets/semantics_tester.dart';

void main() {
  testWidgets('Custom selected and unselected textStyles are honored', (WidgetTester tester) async {
    const TextStyle selectedTextStyle = TextStyle(fontWeight: FontWeight.w300, fontSize: 17.0);
    const TextStyle unselectedTextStyle = TextStyle(fontWeight: FontWeight.w800, fontSize: 11.0);

    await _pumpNavigationRail(
      tester,
      navigationRail: NavigationRail(
        selectedIndex: 0,
        destinations: _destinations(),
        labelType: NavigationRailLabelType.all,
        selectedLabelTextStyle: selectedTextStyle,
        unselectedLabelTextStyle: unselectedTextStyle,
      ),
    );

    final TextStyle actualSelectedTextStyle =
        tester.renderObject<RenderParagraph>(find.text('Abc')).text.style!;
    final TextStyle actualUnselectedTextStyle =
        tester.renderObject<RenderParagraph>(find.text('Def')).text.style!;
    expect(actualSelectedTextStyle.fontSize, equals(selectedTextStyle.fontSize));
    expect(actualSelectedTextStyle.fontWeight, equals(selectedTextStyle.fontWeight));
    expect(actualUnselectedTextStyle.fontSize, equals(actualUnselectedTextStyle.fontSize));
    expect(actualUnselectedTextStyle.fontWeight, equals(actualUnselectedTextStyle.fontWeight));
  });

  testWidgets('Custom selected and unselected iconThemes are honored', (WidgetTester tester) async {
    const IconThemeData selectedIconTheme = IconThemeData(size: 36, color: Color(0x00000001));
    const IconThemeData unselectedIconTheme = IconThemeData(size: 18, color: Color(0x00000002));

    await _pumpNavigationRail(
      tester,
      navigationRail: NavigationRail(
        selectedIndex: 0,
        destinations: _destinations(),
        labelType: NavigationRailLabelType.all,
        selectedIconTheme: selectedIconTheme,
        unselectedIconTheme: unselectedIconTheme,
      ),
    );

    final TextStyle actualSelectedIconTheme = _iconStyle(tester, Icons.favorite);
    final TextStyle actualUnselectedIconTheme = _iconStyle(tester, Icons.bookmark_border);
    expect(actualSelectedIconTheme.color, equals(selectedIconTheme.color));
    expect(actualSelectedIconTheme.fontSize, equals(selectedIconTheme.size));
    expect(actualUnselectedIconTheme.color, equals(unselectedIconTheme.color));
    expect(actualUnselectedIconTheme.fontSize, equals(unselectedIconTheme.size));
  });

  testWidgets('No selected destination when selectedIndex is null', (WidgetTester tester) async {
    await _pumpNavigationRail(
      tester,
      navigationRail: NavigationRail(selectedIndex: null, destinations: _destinations()),
    );

    final Iterable<Semantics> semantics = tester.widgetList<Semantics>(find.byType(Semantics));
    expect(semantics.where((Semantics s) => s.properties.selected ?? false), isEmpty);
  });

  testWidgets('backgroundColor can be changed', (WidgetTester tester) async {
    await _pumpNavigationRail(
      tester,
      navigationRail: NavigationRail(
        selectedIndex: 0,
        destinations: _destinations(),
        labelType: NavigationRailLabelType.all,
      ),
    );

    expect(
      _railMaterial(tester).color,
      equals(const Color(0xfffef7ff)),
    ); // default surface color in M3 colorScheme

    await _pumpNavigationRail(
      tester,
      navigationRail: NavigationRail(
        selectedIndex: 0,
        destinations: _destinations(),
        labelType: NavigationRailLabelType.all,
        backgroundColor: Colors.green,
      ),
    );

    expect(_railMaterial(tester).color, equals(Colors.green));
  });

  testWidgets('elevation can be changed', (WidgetTester tester) async {
    await _pumpNavigationRail(
      tester,
      navigationRail: NavigationRail(
        selectedIndex: 0,
        destinations: _destinations(),
        labelType: NavigationRailLabelType.all,
      ),
    );

    expect(_railMaterial(tester).elevation, equals(0));

    await _pumpNavigationRail(
      tester,
      navigationRail: NavigationRail(
        selectedIndex: 0,
        destinations: _destinations(),
        labelType: NavigationRailLabelType.all,
        elevation: 7,
      ),
    );

    expect(_railMaterial(tester).elevation, equals(7));
  });

  testWidgets('Renders at the correct default width - [labelType]=none (default)', (
    WidgetTester tester,
  ) async {
    await _pumpNavigationRail(
      tester,
      navigationRail: NavigationRail(selectedIndex: 0, destinations: _destinations()),
    );

    final RenderBox renderBox = tester.renderObject(find.byType(NavigationRail));
    expect(renderBox.size.width, 80.0);
  });

  testWidgets('Renders at the correct default width - [labelType]=selected', (
    WidgetTester tester,
  ) async {
    await _pumpNavigationRail(
      tester,
      navigationRail: NavigationRail(
        selectedIndex: 0,
        labelType: NavigationRailLabelType.selected,
        destinations: _destinations(),
      ),
    );

    final RenderBox renderBox = tester.renderObject(find.byType(NavigationRail));
    expect(renderBox.size.width, 80.0);
  });

  testWidgets('Renders at the correct default width - [labelType]=all', (
    WidgetTester tester,
  ) async {
    await _pumpNavigationRail(
      tester,
      navigationRail: NavigationRail(
        selectedIndex: 0,
        labelType: NavigationRailLabelType.all,
        destinations: _destinations(),
      ),
    );

    final RenderBox renderBox = tester.renderObject(find.byType(NavigationRail));
    expect(renderBox.size.width, 80.0);
  });

  testWidgets('Leading and trailing spacing is correct with 0~2 destinations', (
    WidgetTester tester,
  ) async {
    // Padding at the top of the rail.
    const double topPadding = 8.0;
    // Padding at after the leading widget.
    const double spacerPadding = 8.0;
    // Width of a destination.
    const double destinationWidth = 80.0;
    // Height of a destination indicator with icon.
    const double destinationHeight = 32.0;
    // Space between destinations.
    const double destinationSpacing = 12.0;
    // Height of the leading and trailing widgets.
    const double fabHeight = 56.0;

    late StateSetter stateSetter;
    List<NavigationRailDestination> destinations = const <NavigationRailDestination>[];
    Widget? leadingWidget;
    Widget? trailingWidget;

    const Key leadingWidgetKey = Key('leadingWidget');
    const Key trailingWidgetKey = Key('trailingWidget');

    void matchExpect(RenderBox renderBox, double nextDestinationY) {
      expect(
        renderBox.localToGlobal(Offset.zero),
        Offset(
          (destinationWidth - renderBox.size.width) / 2.0,
          nextDestinationY + (destinationHeight - renderBox.size.height) / 2.0,
        ),
      );
    }

    await tester.pumpWidget(
      MaterialApp(
        home: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            stateSetter = setState;
            return Scaffold(
              body: Row(
                children: <Widget>[
                  NavigationRail(
                    destinations: destinations,
                    selectedIndex: null,
                    leading: leadingWidget,
                    trailing: trailingWidget,
                  ),
                  const Expanded(child: Text('body')),
                ],
              ),
            );
          },
        ),
      ),
    );

    // empty destinations and leading widget
    stateSetter(() {
      destinations = const <NavigationRailDestination>[];
      leadingWidget = FloatingActionButton(key: leadingWidgetKey, onPressed: () {});
      trailingWidget = null;
    });
    await tester.pumpAndSettle();
    RenderBox leadingWidgetRenderBox = tester.renderObject<RenderBox>(find.byKey(leadingWidgetKey));
    expect(leadingWidgetRenderBox.localToGlobal(Offset.zero), const Offset(0.0, topPadding));

    // one destination and leading widget
    stateSetter(() {
      destinations = const <NavigationRailDestination>[
        NavigationRailDestination(
          icon: Icon(Icons.favorite_border),
          selectedIcon: Icon(Icons.favorite),
          label: Text('Abc'),
        ),
      ];
    });
    await tester.pumpAndSettle();
    double nextDestinationY = topPadding;
    leadingWidgetRenderBox = tester.renderObject<RenderBox>(find.byKey(leadingWidgetKey));
    expect(
      leadingWidgetRenderBox.localToGlobal(Offset.zero),
      Offset((destinationWidth - leadingWidgetRenderBox.size.width) / 2.0, nextDestinationY),
    );

    nextDestinationY += fabHeight + spacerPadding + destinationSpacing / 2;
    RenderBox firstIconRenderBox = _iconRenderBox(tester, Icons.favorite_border);
    matchExpect(firstIconRenderBox, nextDestinationY);

    // two destinations and leading widget
    stateSetter(() {
      destinations = const <NavigationRailDestination>[
        NavigationRailDestination(
          icon: Icon(Icons.favorite_border),
          selectedIcon: Icon(Icons.favorite),
          label: Text('Abc'),
        ),
        NavigationRailDestination(
          icon: Icon(Icons.bookmark_border),
          selectedIcon: Icon(Icons.bookmark),
          label: Text('Longer Label'),
        ),
      ];
    });
    await tester.pumpAndSettle();
    nextDestinationY = topPadding;
    leadingWidgetRenderBox = tester.renderObject<RenderBox>(find.byKey(leadingWidgetKey));
    expect(
      leadingWidgetRenderBox.localToGlobal(Offset.zero),
      Offset((destinationWidth - leadingWidgetRenderBox.size.width) / 2.0, nextDestinationY),
    );

    nextDestinationY += fabHeight + spacerPadding + destinationSpacing / 2;
    firstIconRenderBox = _iconRenderBox(tester, Icons.favorite_border);
    matchExpect(firstIconRenderBox, nextDestinationY);

    nextDestinationY += destinationHeight + destinationSpacing;
    RenderBox secondIconRenderBox = _iconRenderBox(tester, Icons.bookmark_border);
    matchExpect(secondIconRenderBox, nextDestinationY);

    // empty destinations and trailing widget
    stateSetter(() {
      destinations = const <NavigationRailDestination>[];
      leadingWidget = null;
      trailingWidget = FloatingActionButton(key: trailingWidgetKey, onPressed: () {});
    });
    await tester.pumpAndSettle();
    RenderBox trailingWidgetRenderBox = tester.renderObject<RenderBox>(
      find.byKey(trailingWidgetKey),
    );
    expect(trailingWidgetRenderBox.localToGlobal(Offset.zero), const Offset(0.0, topPadding));

    // one destination and trailing widget
    stateSetter(() {
      destinations = const <NavigationRailDestination>[
        NavigationRailDestination(
          icon: Icon(Icons.favorite_border),
          selectedIcon: Icon(Icons.favorite),
          label: Text('Abc'),
        ),
      ];
    });
    await tester.pumpAndSettle();
    nextDestinationY = topPadding + destinationSpacing / 2;
    firstIconRenderBox = _iconRenderBox(tester, Icons.favorite_border);
    matchExpect(firstIconRenderBox, nextDestinationY);

    nextDestinationY += destinationHeight + destinationSpacing / 2;
    trailingWidgetRenderBox = tester.renderObject<RenderBox>(find.byKey(trailingWidgetKey));
    expect(
      trailingWidgetRenderBox.localToGlobal(Offset.zero),
      Offset((destinationWidth - trailingWidgetRenderBox.size.width) / 2.0, nextDestinationY),
    );

    // two destinations and trailing widget
    stateSetter(() {
      destinations = const <NavigationRailDestination>[
        NavigationRailDestination(
          icon: Icon(Icons.favorite_border),
          selectedIcon: Icon(Icons.favorite),
          label: Text('Abc'),
        ),
        NavigationRailDestination(
          icon: Icon(Icons.bookmark_border),
          selectedIcon: Icon(Icons.bookmark),
          label: Text('Longer Label'),
        ),
      ];
    });
    await tester.pumpAndSettle();
    nextDestinationY = topPadding + destinationSpacing / 2;
    firstIconRenderBox = _iconRenderBox(tester, Icons.favorite_border);
    matchExpect(firstIconRenderBox, nextDestinationY);

    nextDestinationY += destinationHeight + destinationSpacing;
    secondIconRenderBox = _iconRenderBox(tester, Icons.bookmark_border);
    matchExpect(secondIconRenderBox, nextDestinationY);

    nextDestinationY += destinationHeight + destinationSpacing / 2.0;
    trailingWidgetRenderBox = tester.renderObject<RenderBox>(find.byKey(trailingWidgetKey));
    expect(
      trailingWidgetRenderBox.localToGlobal(Offset.zero),
      Offset((destinationWidth - trailingWidgetRenderBox.size.width) / 2.0, nextDestinationY),
    );
  });

  testWidgets('Change destinations and selectedIndex', (WidgetTester tester) async {
    late StateSetter stateSetter;
    int? selectedIndex;
    List<NavigationRailDestination> destinations = const <NavigationRailDestination>[];

    await tester.pumpWidget(
      MaterialApp(
        home: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            stateSetter = setState;
            return Scaffold(
              body: Row(
                children: <Widget>[
                  NavigationRail(selectedIndex: selectedIndex, destinations: destinations),
                  const Expanded(child: Text('body')),
                ],
              ),
            );
          },
        ),
      ),
    );

    expect(tester.widget<NavigationRail>(find.byType(NavigationRail)).destinations.length, 0);
    expect(tester.widget<NavigationRail>(find.byType(NavigationRail)).selectedIndex, isNull);

    stateSetter(() {
      destinations = const <NavigationRailDestination>[
        NavigationRailDestination(
          icon: Icon(Icons.favorite_border),
          selectedIcon: Icon(Icons.favorite),
          label: Text('Abc'),
        ),
        NavigationRailDestination(
          icon: Icon(Icons.bookmark_border),
          selectedIcon: Icon(Icons.bookmark),
          label: Text('Longer Label'),
        ),
      ];
    });

    await tester.pumpAndSettle();

    expect(tester.widget<NavigationRail>(find.byType(NavigationRail)).destinations.length, 2);
    expect(tester.widget<NavigationRail>(find.byType(NavigationRail)).selectedIndex, isNull);

    stateSetter(() {
      selectedIndex = 0;
    });

    await tester.pumpAndSettle();

    expect(tester.widget<NavigationRail>(find.byType(NavigationRail)).destinations.length, 2);
    expect(tester.widget<NavigationRail>(find.byType(NavigationRail)).selectedIndex, 0);

    stateSetter(() {
      destinations = const <NavigationRailDestination>[
        NavigationRailDestination(
          icon: Icon(Icons.favorite_border),
          selectedIcon: Icon(Icons.favorite),
          label: Text('Abc'),
        ),
      ];
    });

    await tester.pumpAndSettle();

    expect(tester.widget<NavigationRail>(find.byType(NavigationRail)).destinations.length, 1);
    expect(tester.widget<NavigationRail>(find.byType(NavigationRail)).selectedIndex, 0);
  });

  testWidgets('Renders wider for a destination with a long label - [labelType]=all', (
    WidgetTester tester,
  ) async {
    await _pumpNavigationRail(
      tester,
      navigationRail: NavigationRail(
        selectedIndex: 0,
        labelType: NavigationRailLabelType.all,
        destinations: const <NavigationRailDestination>[
          NavigationRailDestination(
            icon: Icon(Icons.favorite_border),
            selectedIcon: Icon(Icons.favorite),
            label: Text('Abc'),
          ),
          NavigationRailDestination(
            icon: Icon(Icons.bookmark_border),
            selectedIcon: Icon(Icons.bookmark),
            label: Text('Longer Label'),
          ),
        ],
      ),
    );

    final RenderBox renderBox = tester.renderObject(find.byType(NavigationRail));
    // Total padding is 16 (8 on each side).
    expect(renderBox.size.width, _labelRenderBox(tester, 'Longer Label').size.width + 16.0);
  });

  testWidgets('Renders only icons - [labelType]=none (default)', (WidgetTester tester) async {
    await _pumpNavigationRail(
      tester,
      navigationRail: NavigationRail(selectedIndex: 0, destinations: _destinations()),
    );

    expect(find.byIcon(Icons.favorite), findsOneWidget);
    expect(find.byIcon(Icons.bookmark_border), findsOneWidget);
    expect(find.byIcon(Icons.star_border), findsOneWidget);
    expect(find.byIcon(Icons.hotel), findsOneWidget);

    // When there are no labels, a 0 opacity label is still shown for semantics.
    expect(_labelOpacity(tester, 'Abc'), 0);
    expect(_labelOpacity(tester, 'Def'), 0);
    expect(_labelOpacity(tester, 'Ghi'), 0);
    expect(_labelOpacity(tester, 'Jkl'), 0);
  });

  testWidgets('Renders icons and labels - [labelType]=all', (WidgetTester tester) async {
    await _pumpNavigationRail(
      tester,
      navigationRail: NavigationRail(
        selectedIndex: 0,
        destinations: _destinations(),
        labelType: NavigationRailLabelType.all,
      ),
    );

    expect(find.byIcon(Icons.favorite), findsOneWidget);
    expect(find.byIcon(Icons.bookmark_border), findsOneWidget);
    expect(find.byIcon(Icons.star_border), findsOneWidget);
    expect(find.byIcon(Icons.hotel), findsOneWidget);

    expect(find.text('Abc'), findsOneWidget);
    expect(find.text('Def'), findsOneWidget);
    expect(find.text('Ghi'), findsOneWidget);
    expect(find.text('Jkl'), findsOneWidget);

    // When displaying all labels, there is no opacity.
    expect(_opacityAboveLabel('Abc'), findsNothing);
    expect(_opacityAboveLabel('Def'), findsNothing);
    expect(_opacityAboveLabel('Ghi'), findsNothing);
    expect(_opacityAboveLabel('Jkl'), findsNothing);
  });

  testWidgets('Renders icons and selected label - [labelType]=selected', (
    WidgetTester tester,
  ) async {
    await _pumpNavigationRail(
      tester,
      navigationRail: NavigationRail(
        selectedIndex: 0,
        destinations: _destinations(),
        labelType: NavigationRailLabelType.selected,
      ),
    );

    expect(find.byIcon(Icons.favorite), findsOneWidget);
    expect(find.byIcon(Icons.bookmark_border), findsOneWidget);
    expect(find.byIcon(Icons.star_border), findsOneWidget);
    expect(find.byIcon(Icons.hotel), findsOneWidget);

    // Only the selected label is visible.
    expect(_labelOpacity(tester, 'Abc'), 1);
    expect(_labelOpacity(tester, 'Def'), 0);
    expect(_labelOpacity(tester, 'Ghi'), 0);
    expect(_labelOpacity(tester, 'Jkl'), 0);
  });

  testWidgets(
    'Destination spacing is correct - [labelType]=none (default), [textScaleFactor]=1.0 (default)',
    (WidgetTester tester) async {
      // Padding at the top of the rail.
      const double topPadding = 8.0;
      // Width of a destination.
      const double destinationWidth = 80.0;
      // Height of a destination indicator with icon.
      const double destinationHeight = 32.0;
      // Space between destinations.
      const double destinationPadding = 12.0;

      await _pumpNavigationRail(
        tester,
        navigationRail: NavigationRail(selectedIndex: 0, destinations: _destinations()),
      );

      final RenderBox renderBox = tester.renderObject(find.byType(NavigationRail));
      expect(renderBox.size.width, destinationWidth);

      // The first destination below the rail top by some padding.
      double nextDestinationY = topPadding + destinationPadding / 2;
      final RenderBox firstIconRenderBox = _iconRenderBox(tester, Icons.favorite);
      expect(
        firstIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (destinationWidth - firstIconRenderBox.size.width) / 2.0,
            nextDestinationY + (destinationHeight - firstIconRenderBox.size.height) / 2.0,
          ),
        ),
      );

      // The second destination is one height below the first destination.
      nextDestinationY += destinationHeight + destinationPadding;
      final RenderBox secondIconRenderBox = _iconRenderBox(tester, Icons.bookmark_border);
      expect(
        secondIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (destinationWidth - secondIconRenderBox.size.width) / 2.0,
            nextDestinationY + (destinationHeight - secondIconRenderBox.size.height) / 2.0,
          ),
        ),
      );

      // The third destination is one height below the second destination.
      nextDestinationY += destinationHeight + destinationPadding;
      final RenderBox thirdIconRenderBox = _iconRenderBox(tester, Icons.star_border);
      expect(
        thirdIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (destinationWidth - thirdIconRenderBox.size.width) / 2.0,
            nextDestinationY + (destinationHeight - thirdIconRenderBox.size.height) / 2.0,
          ),
        ),
      );

      // The fourth destination is one height below the third destination.
      nextDestinationY += destinationHeight + destinationPadding;
      final RenderBox fourthIconRenderBox = _iconRenderBox(tester, Icons.hotel);
      expect(
        fourthIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (destinationWidth - fourthIconRenderBox.size.width) / 2.0,
            nextDestinationY + (destinationHeight - fourthIconRenderBox.size.height) / 2.0,
          ),
        ),
      );
    },
  );

  testWidgets(
    'Destination spacing is correct - [labelType]=none (default), [textScaleFactor]=3.0',
    (WidgetTester tester) async {
      // Since the rail is icon only, its destinations should not be affected by
      // textScaleFactor.

      // Padding at the top of the rail.
      const double topPadding = 8.0;
      // Width of a destination.
      const double destinationWidth = 80.0;
      // Height of a destination indicator with icon.
      const double destinationHeight = 32.0;
      // Space between destinations.
      const double destinationPadding = 12.0;

      await _pumpNavigationRail(
        tester,
        textScaleFactor: 3.0,
        navigationRail: NavigationRail(selectedIndex: 0, destinations: _destinations()),
      );

      final RenderBox renderBox = tester.renderObject(find.byType(NavigationRail));
      expect(renderBox.size.width, destinationWidth);

      // The first destination below the rail top by some padding.
      double nextDestinationY = topPadding + destinationPadding / 2;
      final RenderBox firstIconRenderBox = _iconRenderBox(tester, Icons.favorite);
      expect(
        firstIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (destinationWidth - firstIconRenderBox.size.width) / 2.0,
            nextDestinationY + (destinationHeight - firstIconRenderBox.size.height) / 2.0,
          ),
        ),
      );

      // The second destination is one height below the first destination.
      nextDestinationY += destinationHeight + destinationPadding;
      final RenderBox secondIconRenderBox = _iconRenderBox(tester, Icons.bookmark_border);
      expect(
        secondIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (destinationWidth - secondIconRenderBox.size.width) / 2.0,
            nextDestinationY + (destinationHeight - secondIconRenderBox.size.height) / 2.0,
          ),
        ),
      );

      // The third destination is one height below the second destination.
      nextDestinationY += destinationHeight + destinationPadding;
      final RenderBox thirdIconRenderBox = _iconRenderBox(tester, Icons.star_border);
      expect(
        thirdIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (destinationWidth - thirdIconRenderBox.size.width) / 2.0,
            nextDestinationY + (destinationHeight - thirdIconRenderBox.size.height) / 2.0,
          ),
        ),
      );

      // The fourth destination is one height below the third destination.
      nextDestinationY += destinationHeight + destinationPadding;
      final RenderBox fourthIconRenderBox = _iconRenderBox(tester, Icons.hotel);
      expect(
        fourthIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (destinationWidth - fourthIconRenderBox.size.width) / 2.0,
            nextDestinationY + (destinationHeight - fourthIconRenderBox.size.height) / 2.0,
          ),
        ),
      );
    },
  );

  testWidgets(
    'Destination spacing is correct - [labelType]=none (default), [textScaleFactor]=0.75',
    (WidgetTester tester) async {
      // Since the rail is icon only, its destinations should not be affected by
      // textScaleFactor.

      // Padding at the top of the rail.
      const double topPadding = 8.0;
      // Width of a destination.
      const double destinationWidth = 80.0;
      // Height of a destination indicator with icon.
      const double destinationHeight = 32.0;
      // Space between destinations.
      const double destinationPadding = 12.0;

      await _pumpNavigationRail(
        tester,
        textScaleFactor: 0.75,
        navigationRail: NavigationRail(selectedIndex: 0, destinations: _destinations()),
      );

      final RenderBox renderBox = tester.renderObject(find.byType(NavigationRail));
      expect(renderBox.size.width, destinationWidth);

      // The first destination below the rail top by some padding.
      double nextDestinationY = topPadding + destinationPadding / 2;
      final RenderBox firstIconRenderBox = _iconRenderBox(tester, Icons.favorite);
      expect(
        firstIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (destinationWidth - firstIconRenderBox.size.width) / 2.0,
            nextDestinationY + (destinationHeight - firstIconRenderBox.size.height) / 2.0,
          ),
        ),
      );

      // The second destination is one height below the first destination.
      nextDestinationY += destinationHeight + destinationPadding;
      final RenderBox secondIconRenderBox = _iconRenderBox(tester, Icons.bookmark_border);
      expect(
        secondIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (destinationWidth - secondIconRenderBox.size.width) / 2.0,
            nextDestinationY + (destinationHeight - secondIconRenderBox.size.height) / 2.0,
          ),
        ),
      );

      // The third destination is one height below the second destination.
      nextDestinationY += destinationHeight + destinationPadding;
      final RenderBox thirdIconRenderBox = _iconRenderBox(tester, Icons.star_border);
      expect(
        thirdIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (destinationWidth - thirdIconRenderBox.size.width) / 2.0,
            nextDestinationY + (destinationHeight - thirdIconRenderBox.size.height) / 2.0,
          ),
        ),
      );

      // The fourth destination is one height below the third destination.
      nextDestinationY += destinationHeight + destinationPadding;
      final RenderBox fourthIconRenderBox = _iconRenderBox(tester, Icons.hotel);
      expect(
        fourthIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (destinationWidth - fourthIconRenderBox.size.width) / 2.0,
            nextDestinationY + (destinationHeight - fourthIconRenderBox.size.height) / 2.0,
          ),
        ),
      );
    },
  );

  testWidgets(
    'Destination spacing is correct - [labelType]=selected, [textScaleFactor]=1.0 (default)',
    (WidgetTester tester) async {
      // Padding at the top of the rail.
      const double topPadding = 8.0;
      // Width of a destination.
      const double destinationWidth = 80.0;
      // Height of a destination indicator with icon.
      const double destinationHeight = 32.0;
      // Space between the indicator and label.
      const double destinationLabelSpacing = 4.0;
      // Height of the label.
      const double labelHeight = 16.0;
      // Height of a destination with both icon and label.
      const double destinationHeightWithLabel =
          destinationHeight + destinationLabelSpacing + labelHeight;
      // Space between destinations.
      const double destinationSpacing = 12.0;

      await _pumpNavigationRail(
        tester,
        navigationRail: NavigationRail(
          selectedIndex: 0,
          destinations: _destinations(),
          labelType: NavigationRailLabelType.selected,
        ),
      );

      final RenderBox renderBox = tester.renderObject(find.byType(NavigationRail));
      expect(renderBox.size.width, destinationWidth);

      // The first destination is topPadding below the rail top.
      double nextDestinationY = topPadding;
      final RenderBox firstIconRenderBox = _iconRenderBox(tester, Icons.favorite);
      final RenderBox firstLabelRenderBox = _labelRenderBox(tester, 'Abc');
      expect(
        firstIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (destinationWidth - firstIconRenderBox.size.width) / 2.0,
            nextDestinationY + (destinationHeight - firstIconRenderBox.size.height) / 2.0,
          ),
        ),
      );
      expect(
        firstLabelRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (destinationWidth - firstLabelRenderBox.size.width) / 2.0,
            nextDestinationY + destinationHeight + destinationLabelSpacing,
          ),
        ),
      );

      // The second destination is below the first with some spacing.
      nextDestinationY += destinationHeightWithLabel + destinationSpacing;
      final RenderBox secondIconRenderBox = _iconRenderBox(tester, Icons.bookmark_border);
      if (!kIsWeb || isSkiaWeb) {
        // https://github.com/flutter/flutter/issues/99933
        expect(
          secondIconRenderBox.localToGlobal(Offset.zero),
          equals(
            Offset(
              (destinationWidth - secondIconRenderBox.size.width) / 2.0,
              nextDestinationY + (destinationHeight - secondIconRenderBox.size.height) / 2.0,
            ),
          ),
        );
      }

      // The third destination is below the second with some spacing.
      nextDestinationY += destinationHeight + destinationSpacing;
      final RenderBox thirdIconRenderBox = _iconRenderBox(tester, Icons.star_border);
      if (!kIsWeb || isSkiaWeb) {
        // https://github.com/flutter/flutter/issues/99933
        expect(
          thirdIconRenderBox.localToGlobal(Offset.zero),
          equals(
            Offset(
              (destinationWidth - thirdIconRenderBox.size.width) / 2.0,
              nextDestinationY + (destinationHeight - thirdIconRenderBox.size.height) / 2.0,
            ),
          ),
        );
      }

      // The fourth destination is below the third with some spacing.
      nextDestinationY += destinationHeight + destinationSpacing;
      final RenderBox fourthIconRenderBox = _iconRenderBox(tester, Icons.hotel);
      if (!kIsWeb || isSkiaWeb) {
        // https://github.com/flutter/flutter/issues/99933
        expect(
          fourthIconRenderBox.localToGlobal(Offset.zero),
          equals(
            Offset(
              (destinationWidth - fourthIconRenderBox.size.width) / 2.0,
              nextDestinationY + (destinationHeight - fourthIconRenderBox.size.height) / 2.0,
            ),
          ),
        );
      }
    },
  );

  testWidgets('Destination spacing is correct - [labelType]=selected, [textScaleFactor]=3.0', (
    WidgetTester tester,
  ) async {
    // Padding at the top of the rail.
    const double topPadding = 8.0;
    // Width of a destination.
    const double destinationWidth = 125.5;
    // Height of a destination indicator with icon.
    const double destinationHeight = 32.0;
    // Space between the indicator and label.
    const double destinationLabelSpacing = 4.0;
    // Height of the label.
    const double labelHeight = 16.0 * 3.0;
    // Height of a destination with both icon and label.
    const double destinationHeightWithLabel =
        destinationHeight + destinationLabelSpacing + labelHeight;
    // Space between destinations.
    const double destinationSpacing = 12.0;

    await _pumpNavigationRail(
      tester,
      textScaleFactor: 3.0,
      navigationRail: NavigationRail(
        selectedIndex: 0,
        destinations: _destinations(),
        labelType: NavigationRailLabelType.selected,
      ),
    );

    final RenderBox renderBox = tester.renderObject(find.byType(NavigationRail));
    expect(renderBox.size.width, destinationWidth);

    // The first destination topPadding below the rail top.
    double nextDestinationY = topPadding;
    final RenderBox firstIconRenderBox = _iconRenderBox(tester, Icons.favorite);
    final RenderBox firstLabelRenderBox = _labelRenderBox(tester, 'Abc');
    expect(
      firstIconRenderBox.localToGlobal(Offset.zero),
      equals(
        Offset(
          (destinationWidth - firstIconRenderBox.size.width) / 2.0,
          nextDestinationY + (destinationHeight - firstIconRenderBox.size.height) / 2.0,
        ),
      ),
    );
    expect(
      firstLabelRenderBox.localToGlobal(Offset.zero),
      equals(
        Offset(
          (destinationWidth - firstLabelRenderBox.size.width) / 2.0,
          nextDestinationY + destinationHeight + destinationLabelSpacing,
        ),
      ),
    );

    // The second destination is below the first with some spacing.
    nextDestinationY += destinationHeightWithLabel + destinationSpacing;
    final RenderBox secondIconRenderBox = _iconRenderBox(tester, Icons.bookmark_border);
    if (!kIsWeb || isSkiaWeb) {
      // https://github.com/flutter/flutter/issues/99933
      expect(
        secondIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (destinationWidth - secondIconRenderBox.size.width) / 2.0,
            nextDestinationY + (destinationHeight - secondIconRenderBox.size.height) / 2.0,
          ),
        ),
      );
    }

    // The third destination is below the second with some spacing.
    nextDestinationY += destinationHeight + destinationSpacing;
    final RenderBox thirdIconRenderBox = _iconRenderBox(tester, Icons.star_border);
    if (!kIsWeb || isSkiaWeb) {
      // https://github.com/flutter/flutter/issues/99933
      expect(
        thirdIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (destinationWidth - thirdIconRenderBox.size.width) / 2.0,
            nextDestinationY + (destinationHeight - thirdIconRenderBox.size.height) / 2.0,
          ),
        ),
      );
    }

    // The fourth destination is below the third with some spacing.
    nextDestinationY += destinationHeight + destinationSpacing;
    final RenderBox fourthIconRenderBox = _iconRenderBox(tester, Icons.hotel);
    if (!kIsWeb || isSkiaWeb) {
      // https://github.com/flutter/flutter/issues/99933
      expect(
        fourthIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (destinationWidth - fourthIconRenderBox.size.width) / 2.0,
            nextDestinationY + (destinationHeight - fourthIconRenderBox.size.height) / 2.0,
          ),
        ),
      );
    }
  });

  testWidgets('Destination spacing is correct - [labelType]=selected, [textScaleFactor]=0.75', (
    WidgetTester tester,
  ) async {
    // Padding at the top of the rail.
    const double topPadding = 8.0;
    // Width of a destination.
    const double destinationWidth = 80.0;
    // Height of a destination indicator with icon.
    const double destinationHeight = 32.0;
    // Space between the indicator and label.
    const double destinationLabelSpacing = 4.0;
    // Height of the label.
    const double labelHeight = 16.0 * 0.75;
    // Height of a destination with both icon and label.
    const double destinationHeightWithLabel =
        destinationHeight + destinationLabelSpacing + labelHeight;
    // Space between destinations.
    const double destinationSpacing = 12.0;

    await _pumpNavigationRail(
      tester,
      textScaleFactor: 0.75,
      navigationRail: NavigationRail(
        selectedIndex: 0,
        destinations: _destinations(),
        labelType: NavigationRailLabelType.selected,
      ),
    );

    final RenderBox renderBox = tester.renderObject(find.byType(NavigationRail));
    expect(renderBox.size.width, destinationWidth);

    // The first destination topPadding below the rail top.
    double nextDestinationY = topPadding;
    final RenderBox firstIconRenderBox = _iconRenderBox(tester, Icons.favorite);
    final RenderBox firstLabelRenderBox = _labelRenderBox(tester, 'Abc');
    expect(
      firstIconRenderBox.localToGlobal(Offset.zero),
      equals(
        Offset(
          (destinationWidth - firstIconRenderBox.size.width) / 2.0,
          nextDestinationY + (destinationHeight - firstIconRenderBox.size.height) / 2.0,
        ),
      ),
    );
    expect(
      firstLabelRenderBox.localToGlobal(Offset.zero),
      equals(
        Offset(
          (destinationWidth - firstLabelRenderBox.size.width) / 2.0,
          nextDestinationY + destinationHeight + destinationLabelSpacing,
        ),
      ),
    );

    // The second destination is below the first with some spacing.
    nextDestinationY += destinationHeightWithLabel + destinationSpacing;
    final RenderBox secondIconRenderBox = _iconRenderBox(tester, Icons.bookmark_border);
    if (!kIsWeb || isSkiaWeb) {
      // https://github.com/flutter/flutter/issues/99933
      expect(
        secondIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (destinationWidth - secondIconRenderBox.size.width) / 2.0,
            nextDestinationY + (destinationHeight - secondIconRenderBox.size.height) / 2.0,
          ),
        ),
      );
    }

    // The third destination is below the second with some spacing.
    nextDestinationY += destinationHeight + destinationSpacing;
    final RenderBox thirdIconRenderBox = _iconRenderBox(tester, Icons.star_border);
    if (!kIsWeb || isSkiaWeb) {
      // https://github.com/flutter/flutter/issues/99933
      expect(
        thirdIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (destinationWidth - thirdIconRenderBox.size.width) / 2.0,
            nextDestinationY + (destinationHeight - thirdIconRenderBox.size.height) / 2.0,
          ),
        ),
      );
    }

    // The fourth destination is below the third with some spacing.
    nextDestinationY += destinationHeight + destinationSpacing;
    final RenderBox fourthIconRenderBox = _iconRenderBox(tester, Icons.hotel);
    if (!kIsWeb || isSkiaWeb) {
      // https://github.com/flutter/flutter/issues/99933
      expect(
        fourthIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (destinationWidth - fourthIconRenderBox.size.width) / 2.0,
            nextDestinationY + (destinationHeight - fourthIconRenderBox.size.height) / 2.0,
          ),
        ),
      );
    }
  });

  testWidgets('Destination spacing is correct - [labelType]=all, [textScaleFactor]=1.0 (default)', (
    WidgetTester tester,
  ) async {
    // Padding at the top of the rail.
    const double topPadding = 8.0;
    // Width of a destination.
    const double destinationWidth = 80.0;
    // Height of a destination indicator with icon.
    const double destinationHeight = 32.0;
    // Space between the indicator and label.
    const double destinationLabelSpacing = 4.0;
    // Height of the label.
    const double labelHeight = 16.0;
    // Height of a destination with both icon and label.
    const double destinationHeightWithLabel =
        destinationHeight + destinationLabelSpacing + labelHeight;
    // Space between destinations.
    const double destinationSpacing = 12.0;

    await _pumpNavigationRail(
      tester,
      navigationRail: NavigationRail(
        selectedIndex: 0,
        destinations: _destinations(),
        labelType: NavigationRailLabelType.all,
      ),
    );

    final RenderBox renderBox = tester.renderObject(find.byType(NavigationRail));
    expect(renderBox.size.width, destinationWidth);

    // The first destination topPadding below the rail top.
    double nextDestinationY = topPadding;
    final RenderBox firstIconRenderBox = _iconRenderBox(tester, Icons.favorite);
    final RenderBox firstLabelRenderBox = _labelRenderBox(tester, 'Abc');
    expect(
      firstIconRenderBox.localToGlobal(Offset.zero),
      equals(
        Offset(
          (destinationWidth - firstIconRenderBox.size.width) / 2.0,
          nextDestinationY + (destinationHeight - firstIconRenderBox.size.height) / 2.0,
        ),
      ),
    );
    expect(
      firstLabelRenderBox.localToGlobal(Offset.zero),
      equals(
        Offset(
          (destinationWidth - firstLabelRenderBox.size.width) / 2.0,
          nextDestinationY + destinationHeight + destinationLabelSpacing,
        ),
      ),
    );

    // The second destination is below the first with some spacing.
    nextDestinationY += destinationHeightWithLabel + destinationSpacing;
    final RenderBox secondIconRenderBox = _iconRenderBox(tester, Icons.bookmark_border);
    if (!kIsWeb || isSkiaWeb) {
      // https://github.com/flutter/flutter/issues/99933
      expect(
        secondIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (destinationWidth - secondIconRenderBox.size.width) / 2.0,
            nextDestinationY + (destinationHeight - secondIconRenderBox.size.height) / 2.0,
          ),
        ),
      );
    }

    // The third destination is below the second with some spacing.
    nextDestinationY += destinationHeightWithLabel + destinationSpacing;
    final RenderBox thirdIconRenderBox = _iconRenderBox(tester, Icons.star_border);
    if (!kIsWeb || isSkiaWeb) {
      // https://github.com/flutter/flutter/issues/99933
      expect(
        thirdIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (destinationWidth - thirdIconRenderBox.size.width) / 2.0,
            nextDestinationY + (destinationHeight - thirdIconRenderBox.size.height) / 2.0,
          ),
        ),
      );
    }

    // The fourth destination is below the third with some spacing.
    nextDestinationY += destinationHeightWithLabel + destinationSpacing;
    final RenderBox fourthIconRenderBox = _iconRenderBox(tester, Icons.hotel);
    if (!kIsWeb || isSkiaWeb) {
      // https://github.com/flutter/flutter/issues/99933
      expect(
        fourthIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (destinationWidth - fourthIconRenderBox.size.width) / 2.0,
            nextDestinationY + (destinationHeight - fourthIconRenderBox.size.height) / 2.0,
          ),
        ),
      );
    }
  });

  testWidgets('Destination spacing is correct - [labelType]=all, [textScaleFactor]=3.0', (
    WidgetTester tester,
  ) async {
    // Padding at the top of the rail.
    const double topPadding = 8.0;
    // Width of a destination.
    const double destinationWidth = 125.5;
    // Height of a destination indicator with icon.
    const double destinationHeight = 32.0;
    // Space between the indicator and label.
    const double destinationLabelSpacing = 4.0;
    // Height of the label.
    const double labelHeight = 16.0 * 3.0;
    // Height of a destination with both icon and label.
    const double destinationHeightWithLabel =
        destinationHeight + destinationLabelSpacing + labelHeight;
    // Space between destinations.
    const double destinationSpacing = 12.0;

    await _pumpNavigationRail(
      tester,
      textScaleFactor: 3.0,
      navigationRail: NavigationRail(
        selectedIndex: 0,
        destinations: _destinations(),
        labelType: NavigationRailLabelType.all,
      ),
    );

    final RenderBox renderBox = tester.renderObject(find.byType(NavigationRail));
    expect(renderBox.size.width, destinationWidth);

    // The first destination topPadding below the rail top.
    double nextDestinationY = topPadding;
    final RenderBox firstIconRenderBox = _iconRenderBox(tester, Icons.favorite);
    final RenderBox firstLabelRenderBox = _labelRenderBox(tester, 'Abc');
    expect(
      firstIconRenderBox.localToGlobal(Offset.zero),
      equals(
        Offset(
          (destinationWidth - firstIconRenderBox.size.width) / 2.0,
          nextDestinationY + (destinationHeight - firstIconRenderBox.size.height) / 2.0,
        ),
      ),
    );
    expect(
      firstLabelRenderBox.localToGlobal(Offset.zero),
      equals(
        Offset(
          (destinationWidth - firstLabelRenderBox.size.width) / 2.0,
          nextDestinationY + destinationHeight + destinationLabelSpacing,
        ),
      ),
    );

    // The second destination is below the first with some spacing.
    nextDestinationY += destinationHeightWithLabel + destinationSpacing;
    final RenderBox secondIconRenderBox = _iconRenderBox(tester, Icons.bookmark_border);
    if (!kIsWeb || isSkiaWeb) {
      // https://github.com/flutter/flutter/issues/99933
      expect(
        secondIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (destinationWidth - secondIconRenderBox.size.width) / 2.0,
            nextDestinationY + (destinationHeight - secondIconRenderBox.size.height) / 2.0,
          ),
        ),
      );
    }

    // The third destination is below the second with some spacing.
    nextDestinationY += destinationHeightWithLabel + destinationSpacing;
    final RenderBox thirdIconRenderBox = _iconRenderBox(tester, Icons.star_border);
    if (!kIsWeb || isSkiaWeb) {
      // https://github.com/flutter/flutter/issues/99933
      expect(
        thirdIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (destinationWidth - thirdIconRenderBox.size.width) / 2.0,
            nextDestinationY + (destinationHeight - thirdIconRenderBox.size.height) / 2.0,
          ),
        ),
      );
    }

    // The fourth destination is below the third with some spacing.
    nextDestinationY += destinationHeightWithLabel + destinationSpacing;
    final RenderBox fourthIconRenderBox = _iconRenderBox(tester, Icons.hotel);
    if (!kIsWeb || isSkiaWeb) {
      // https://github.com/flutter/flutter/issues/99933
      expect(
        fourthIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (destinationWidth - fourthIconRenderBox.size.width) / 2.0,
            nextDestinationY + (destinationHeight - fourthIconRenderBox.size.height) / 2.0,
          ),
        ),
      );
    }
  });

  testWidgets('Destination spacing is correct - [labelType]=all, [textScaleFactor]=0.75', (
    WidgetTester tester,
  ) async {
    // Padding at the top of the rail.
    const double topPadding = 8.0;
    // Width of a destination.
    const double destinationWidth = 80.0;
    // Height of a destination indicator with icon.
    const double destinationHeight = 32.0;
    // Space between the indicator and label.
    const double destinationLabelSpacing = 4.0;
    // Height of the label.
    const double labelHeight = 16.0 * 0.75;
    // Height of a destination with both icon and label.
    const double destinationHeightWithLabel =
        destinationHeight + destinationLabelSpacing + labelHeight;
    // Space between destinations.
    const double destinationSpacing = 12.0;

    await _pumpNavigationRail(
      tester,
      textScaleFactor: 0.75,
      navigationRail: NavigationRail(
        selectedIndex: 0,
        destinations: _destinations(),
        labelType: NavigationRailLabelType.all,
      ),
    );

    final RenderBox renderBox = tester.renderObject(find.byType(NavigationRail));
    expect(renderBox.size.width, destinationWidth);

    // The first destination topPadding below the rail top.
    double nextDestinationY = topPadding;
    final RenderBox firstIconRenderBox = _iconRenderBox(tester, Icons.favorite);
    final RenderBox firstLabelRenderBox = _labelRenderBox(tester, 'Abc');
    expect(
      firstIconRenderBox.localToGlobal(Offset.zero),
      equals(
        Offset(
          (destinationWidth - firstIconRenderBox.size.width) / 2.0,
          nextDestinationY + (destinationHeight - firstIconRenderBox.size.height) / 2.0,
        ),
      ),
    );
    expect(
      firstLabelRenderBox.localToGlobal(Offset.zero),
      equals(
        Offset(
          (destinationWidth - firstLabelRenderBox.size.width) / 2.0,
          nextDestinationY + destinationHeight + destinationLabelSpacing,
        ),
      ),
    );

    // The second destination is below the first with some spacing.
    nextDestinationY += destinationHeightWithLabel + destinationSpacing;
    final RenderBox secondIconRenderBox = _iconRenderBox(tester, Icons.bookmark_border);
    if (!kIsWeb || isSkiaWeb) {
      // https://github.com/flutter/flutter/issues/99933
      expect(
        secondIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (destinationWidth - secondIconRenderBox.size.width) / 2.0,
            nextDestinationY + (destinationHeight - secondIconRenderBox.size.height) / 2.0,
          ),
        ),
      );
    }

    // The third destination is below the second with some spacing.
    nextDestinationY += destinationHeightWithLabel + destinationSpacing;
    final RenderBox thirdIconRenderBox = _iconRenderBox(tester, Icons.star_border);
    if (!kIsWeb || isSkiaWeb) {
      // https://github.com/flutter/flutter/issues/99933
      expect(
        thirdIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (destinationWidth - thirdIconRenderBox.size.width) / 2.0,
            nextDestinationY + (destinationHeight - thirdIconRenderBox.size.height) / 2.0,
          ),
        ),
      );
    }

    // The fourth destination is below the third with some spacing.
    nextDestinationY += destinationHeightWithLabel + destinationSpacing;
    final RenderBox fourthIconRenderBox = _iconRenderBox(tester, Icons.hotel);
    if (!kIsWeb || isSkiaWeb) {
      // https://github.com/flutter/flutter/issues/99933
      expect(
        fourthIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (destinationWidth - fourthIconRenderBox.size.width) / 2.0,
            nextDestinationY + (destinationHeight - fourthIconRenderBox.size.height) / 2.0,
          ),
        ),
      );
    }
  });

  testWidgets(
    'Destination spacing is correct for a compact rail - [preferredWidth]=56, [textScaleFactor]=1.0 (default)',
    (WidgetTester tester) async {
      // Padding at the top of the rail.
      const double topPadding = 8.0;
      // Width of a destination.
      const double compactWidth = 56.0;
      // Height of a destination indicator with icon.
      const double destinationHeight = 32.0;
      // Space between destinations.
      const double destinationSpacing = 12.0;

      await _pumpNavigationRail(
        tester,
        navigationRail: NavigationRail(
          selectedIndex: 0,
          minWidth: 56.0,
          destinations: _destinations(),
        ),
      );

      final RenderBox renderBox = tester.renderObject(find.byType(NavigationRail));
      expect(renderBox.size.width, 56.0);

      // The first destination below the rail top by some padding.
      double nextDestinationY = topPadding + destinationSpacing / 2;
      final RenderBox firstIconRenderBox = _iconRenderBox(tester, Icons.favorite);
      expect(
        firstIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (compactWidth - firstIconRenderBox.size.width) / 2.0,
            nextDestinationY + (destinationHeight - firstIconRenderBox.size.height) / 2.0,
          ),
        ),
      );

      // The second destination is row below the first destination.
      nextDestinationY += destinationHeight + destinationSpacing;
      final RenderBox secondIconRenderBox = _iconRenderBox(tester, Icons.bookmark_border);
      expect(
        secondIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (compactWidth - secondIconRenderBox.size.width) / 2.0,
            nextDestinationY + (destinationHeight - secondIconRenderBox.size.height) / 2.0,
          ),
        ),
      );

      // The third destination is a row below the second destination.
      nextDestinationY += destinationHeight + destinationSpacing;
      final RenderBox thirdIconRenderBox = _iconRenderBox(tester, Icons.star_border);
      expect(
        thirdIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (compactWidth - thirdIconRenderBox.size.width) / 2.0,
            nextDestinationY + (destinationHeight - thirdIconRenderBox.size.height) / 2.0,
          ),
        ),
      );

      // The fourth destination is a row below the third destination.
      nextDestinationY += destinationHeight + destinationSpacing;
      final RenderBox fourthIconRenderBox = _iconRenderBox(tester, Icons.hotel);
      expect(
        fourthIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (compactWidth - fourthIconRenderBox.size.width) / 2.0,
            nextDestinationY + (destinationHeight - fourthIconRenderBox.size.height) / 2.0,
          ),
        ),
      );
    },
  );

  testWidgets(
    'Destination spacing is correct for a compact rail - [preferredWidth]=56, [textScaleFactor]=3.0',
    (WidgetTester tester) async {
      // Padding at the top of the rail.
      const double topPadding = 8.0;
      // Width of a destination.
      const double compactWidth = 56.0;
      // Height of a destination indicator with icon.
      const double destinationHeight = 32.0;
      // Space between destinations.
      const double destinationSpacing = 12.0;

      await _pumpNavigationRail(
        tester,
        textScaleFactor: 3.0,
        navigationRail: NavigationRail(
          selectedIndex: 0,
          minWidth: 56.0,
          destinations: _destinations(),
        ),
      );

      // Since the rail is icon only, its preferred width should not be affected
      // by textScaleFactor.
      final RenderBox renderBox = tester.renderObject(find.byType(NavigationRail));
      expect(renderBox.size.width, compactWidth);

      // The first destination below the rail top by some padding.
      double nextDestinationY = topPadding + destinationSpacing / 2;
      final RenderBox firstIconRenderBox = _iconRenderBox(tester, Icons.favorite);
      expect(
        firstIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (compactWidth - firstIconRenderBox.size.width) / 2.0,
            nextDestinationY + (destinationHeight - firstIconRenderBox.size.height) / 2.0,
          ),
        ),
      );

      // The second destination is row below the first destination.
      nextDestinationY += destinationHeight + destinationSpacing;
      final RenderBox secondIconRenderBox = _iconRenderBox(tester, Icons.bookmark_border);
      expect(
        secondIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (compactWidth - secondIconRenderBox.size.width) / 2.0,
            nextDestinationY + (destinationHeight - secondIconRenderBox.size.height) / 2.0,
          ),
        ),
      );

      // The third destination is a row below the second destination.
      nextDestinationY += destinationHeight + destinationSpacing;
      final RenderBox thirdIconRenderBox = _iconRenderBox(tester, Icons.star_border);
      expect(
        thirdIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (compactWidth - thirdIconRenderBox.size.width) / 2.0,
            nextDestinationY + (destinationHeight - thirdIconRenderBox.size.height) / 2.0,
          ),
        ),
      );

      // The fourth destination is a row below the third destination.
      nextDestinationY += destinationHeight + destinationSpacing;
      final RenderBox fourthIconRenderBox = _iconRenderBox(tester, Icons.hotel);
      expect(
        fourthIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (compactWidth - fourthIconRenderBox.size.width) / 2.0,
            nextDestinationY + (destinationHeight - fourthIconRenderBox.size.height) / 2.0,
          ),
        ),
      );
    },
  );

  testWidgets(
    'Destination spacing is correct for a compact rail - [preferredWidth]=56, [textScaleFactor]=0.75',
    (WidgetTester tester) async {
      // Padding at the top of the rail.
      const double topPadding = 8.0;
      // Width of a destination.
      const double compactWidth = 56.0;
      // Height of a destination indicator with icon.
      const double destinationHeight = 32.0;
      // Space between destinations.
      const double destinationSpacing = 12.0;

      await _pumpNavigationRail(
        tester,
        textScaleFactor: 0.75,
        navigationRail: NavigationRail(
          selectedIndex: 0,
          minWidth: 56.0,
          destinations: _destinations(),
        ),
      );

      // Since the rail is icon only, its preferred width should not be affected
      // by textScaleFactor.
      final RenderBox renderBox = tester.renderObject(find.byType(NavigationRail));
      expect(renderBox.size.width, compactWidth);

      // The first destination below the rail top by some padding.
      double nextDestinationY = topPadding + destinationSpacing / 2;
      final RenderBox firstIconRenderBox = _iconRenderBox(tester, Icons.favorite);
      expect(
        firstIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (compactWidth - firstIconRenderBox.size.width) / 2.0,
            nextDestinationY + (destinationHeight - firstIconRenderBox.size.height) / 2.0,
          ),
        ),
      );

      // The second destination is row below the first destination.
      nextDestinationY += destinationHeight + destinationSpacing;
      final RenderBox secondIconRenderBox = _iconRenderBox(tester, Icons.bookmark_border);
      expect(
        secondIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (compactWidth - secondIconRenderBox.size.width) / 2.0,
            nextDestinationY + (destinationHeight - secondIconRenderBox.size.height) / 2.0,
          ),
        ),
      );

      // The third destination is a row below the second destination.
      nextDestinationY += destinationHeight + destinationSpacing;
      final RenderBox thirdIconRenderBox = _iconRenderBox(tester, Icons.star_border);
      expect(
        thirdIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (compactWidth - thirdIconRenderBox.size.width) / 2.0,
            nextDestinationY + (destinationHeight - thirdIconRenderBox.size.height) / 2.0,
          ),
        ),
      );

      // The fourth destination is a row below the third destination.
      nextDestinationY += destinationHeight + destinationSpacing;
      final RenderBox fourthIconRenderBox = _iconRenderBox(tester, Icons.hotel);
      expect(
        fourthIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (compactWidth - fourthIconRenderBox.size.width) / 2.0,
            nextDestinationY + (destinationHeight - fourthIconRenderBox.size.height) / 2.0,
          ),
        ),
      );
    },
  );

  testWidgets('Group alignment works - [groupAlignment]=-1.0 (default)', (
    WidgetTester tester,
  ) async {
    // Padding at the top of the rail.
    const double topPadding = 8.0;
    // Width of a destination.
    const double destinationWidth = 80.0;
    // Height of a destination indicator with icon.
    const double destinationHeight = 32.0;
    // Space between destinations.
    const double destinationPadding = 12.0;

    await _pumpNavigationRail(
      tester,
      navigationRail: NavigationRail(selectedIndex: 0, destinations: _destinations()),
    );

    final RenderBox renderBox = tester.renderObject(find.byType(NavigationRail));
    expect(renderBox.size.width, destinationWidth);

    // The first destination below the rail top by some padding.
    double nextDestinationY = topPadding + destinationPadding / 2;
    final RenderBox firstIconRenderBox = _iconRenderBox(tester, Icons.favorite);
    expect(
      firstIconRenderBox.localToGlobal(Offset.zero),
      equals(
        Offset(
          (destinationWidth - firstIconRenderBox.size.width) / 2.0,
          nextDestinationY + (destinationHeight - firstIconRenderBox.size.height) / 2.0,
        ),
      ),
    );

    // The second destination is one height below the first destination.
    nextDestinationY += destinationHeight + destinationPadding;
    final RenderBox secondIconRenderBox = _iconRenderBox(tester, Icons.bookmark_border);
    expect(
      secondIconRenderBox.localToGlobal(Offset.zero),
      equals(
        Offset(
          (destinationWidth - secondIconRenderBox.size.width) / 2.0,
          nextDestinationY + (destinationHeight - secondIconRenderBox.size.height) / 2.0,
        ),
      ),
    );

    // The third destination is one height below the second destination.
    nextDestinationY += destinationHeight + destinationPadding;
    final RenderBox thirdIconRenderBox = _iconRenderBox(tester, Icons.star_border);
    expect(
      thirdIconRenderBox.localToGlobal(Offset.zero),
      equals(
        Offset(
          (destinationWidth - thirdIconRenderBox.size.width) / 2.0,
          nextDestinationY + (destinationHeight - thirdIconRenderBox.size.height) / 2.0,
        ),
      ),
    );

    // The fourth destination is one height below the third destination.
    nextDestinationY += destinationHeight + destinationPadding;
    final RenderBox fourthIconRenderBox = _iconRenderBox(tester, Icons.hotel);
    expect(
      fourthIconRenderBox.localToGlobal(Offset.zero),
      equals(
        Offset(
          (destinationWidth - fourthIconRenderBox.size.width) / 2.0,
          nextDestinationY + (destinationHeight - fourthIconRenderBox.size.height) / 2.0,
        ),
      ),
    );
  });

  testWidgets('Group alignment works - [groupAlignment]=0.0', (WidgetTester tester) async {
    // Padding at the top of the rail.
    const double topPadding = 8.0;
    // Width of a destination.
    const double destinationWidth = 80.0;
    // Height of a destination indicator with icon.
    const double destinationHeight = 32.0;
    // Space between destinations.
    const double destinationPadding = 12.0;

    await _pumpNavigationRail(
      tester,
      navigationRail: NavigationRail(
        selectedIndex: 0,
        groupAlignment: 0.0,
        destinations: _destinations(),
      ),
    );

    final RenderBox renderBox = tester.renderObject(find.byType(NavigationRail));
    expect(renderBox.size.width, destinationWidth);

    // The first destination below the rail top by some padding with an offset for the alignment.
    double nextDestinationY = topPadding + destinationPadding / 2 + 208;
    final RenderBox firstIconRenderBox = _iconRenderBox(tester, Icons.favorite);
    expect(
      firstIconRenderBox.localToGlobal(Offset.zero),
      equals(
        Offset(
          (destinationWidth - firstIconRenderBox.size.width) / 2.0,
          nextDestinationY + (destinationHeight - firstIconRenderBox.size.height) / 2.0,
        ),
      ),
    );

    // The second destination is one height below the first destination.
    nextDestinationY += destinationHeight + destinationPadding;
    final RenderBox secondIconRenderBox = _iconRenderBox(tester, Icons.bookmark_border);
    expect(
      secondIconRenderBox.localToGlobal(Offset.zero),
      equals(
        Offset(
          (destinationWidth - secondIconRenderBox.size.width) / 2.0,
          nextDestinationY + (destinationHeight - secondIconRenderBox.size.height) / 2.0,
        ),
      ),
    );

    // The third destination is one height below the second destination.
    nextDestinationY += destinationHeight + destinationPadding;
    final RenderBox thirdIconRenderBox = _iconRenderBox(tester, Icons.star_border);
    expect(
      thirdIconRenderBox.localToGlobal(Offset.zero),
      equals(
        Offset(
          (destinationWidth - thirdIconRenderBox.size.width) / 2.0,
          nextDestinationY + (destinationHeight - thirdIconRenderBox.size.height) / 2.0,
        ),
      ),
    );

    // The fourth destination is one height below the third destination.
    nextDestinationY += destinationHeight + destinationPadding;
    final RenderBox fourthIconRenderBox = _iconRenderBox(tester, Icons.hotel);
    expect(
      fourthIconRenderBox.localToGlobal(Offset.zero),
      equals(
        Offset(
          (destinationWidth - fourthIconRenderBox.size.width) / 2.0,
          nextDestinationY + (destinationHeight - fourthIconRenderBox.size.height) / 2.0,
        ),
      ),
    );
  });

  testWidgets('Group alignment works - [groupAlignment]=1.0', (WidgetTester tester) async {
    // Padding at the top of the rail.
    const double topPadding = 8.0;
    // Width of a destination.
    const double destinationWidth = 80.0;
    // Height of a destination indicator with icon.
    const double destinationHeight = 32.0;
    // Space between destinations.
    const double destinationPadding = 12.0;

    await _pumpNavigationRail(
      tester,
      navigationRail: NavigationRail(
        selectedIndex: 0,
        groupAlignment: 1.0,
        destinations: _destinations(),
      ),
    );

    final RenderBox renderBox = tester.renderObject(find.byType(NavigationRail));
    expect(renderBox.size.width, destinationWidth);

    // The first destination below the rail top by some padding with an offset for the alignment.
    double nextDestinationY = topPadding + destinationPadding / 2 + 416;
    final RenderBox firstIconRenderBox = _iconRenderBox(tester, Icons.favorite);
    expect(
      firstIconRenderBox.localToGlobal(Offset.zero),
      equals(
        Offset(
          (destinationWidth - firstIconRenderBox.size.width) / 2.0,
          nextDestinationY + (destinationHeight - firstIconRenderBox.size.height) / 2.0,
        ),
      ),
    );

    // The second destination is one height below the first destination.
    nextDestinationY += destinationHeight + destinationPadding;
    final RenderBox secondIconRenderBox = _iconRenderBox(tester, Icons.bookmark_border);
    expect(
      secondIconRenderBox.localToGlobal(Offset.zero),
      equals(
        Offset(
          (destinationWidth - secondIconRenderBox.size.width) / 2.0,
          nextDestinationY + (destinationHeight - secondIconRenderBox.size.height) / 2.0,
        ),
      ),
    );

    // The third destination is one height below the second destination.
    nextDestinationY += destinationHeight + destinationPadding;
    final RenderBox thirdIconRenderBox = _iconRenderBox(tester, Icons.star_border);
    expect(
      thirdIconRenderBox.localToGlobal(Offset.zero),
      equals(
        Offset(
          (destinationWidth - thirdIconRenderBox.size.width) / 2.0,
          nextDestinationY + (destinationHeight - thirdIconRenderBox.size.height) / 2.0,
        ),
      ),
    );

    // The fourth destination is one height below the third destination.
    nextDestinationY += destinationHeight + destinationPadding;
    final RenderBox fourthIconRenderBox = _iconRenderBox(tester, Icons.hotel);
    expect(
      fourthIconRenderBox.localToGlobal(Offset.zero),
      equals(
        Offset(
          (destinationWidth - fourthIconRenderBox.size.width) / 2.0,
          nextDestinationY + (destinationHeight - fourthIconRenderBox.size.height) / 2.0,
        ),
      ),
    );
  });

  testWidgets('Leading and trailing appear in the correct places', (WidgetTester tester) async {
    await _pumpNavigationRail(
      tester,
      navigationRail: NavigationRail(
        selectedIndex: 0,
        leading: FloatingActionButton(onPressed: () {}),
        trailing: FloatingActionButton(onPressed: () {}),
        destinations: _destinations(),
      ),
    );

    final RenderBox leading = tester.renderObject<RenderBox>(
      find.byType(FloatingActionButton).at(0),
    );
    final RenderBox trailing = tester.renderObject<RenderBox>(
      find.byType(FloatingActionButton).at(1),
    );
    expect(leading.localToGlobal(Offset.zero), Offset((80 - leading.size.width) / 2, 8.0));
    expect(trailing.localToGlobal(Offset.zero), Offset((80 - trailing.size.width) / 2, 248.0));
  });

  testWidgets('Extended rail animates the width and labels appear - [textDirection]=LTR', (
    WidgetTester tester,
  ) async {
    // Padding at the top of the rail.
    const double topPadding = 8.0;
    // Width of a destination.
    const double destinationWidth = 80.0;
    // Height of a destination indicator with icon.
    const double destinationHeight = 32.0;
    // Space between destinations.
    const double destinationPadding = 12.0;

    bool extended = false;
    late StateSetter stateSetter;

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: true),
        home: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            stateSetter = setState;
            return Scaffold(
              body: Row(
                children: <Widget>[
                  NavigationRail(
                    selectedIndex: 0,
                    destinations: _destinations(),
                    extended: extended,
                  ),
                  const Expanded(child: Text('body')),
                ],
              ),
            );
          },
        ),
      ),
    );

    final RenderBox rail = tester.firstRenderObject<RenderBox>(find.byType(NavigationRail));

    expect(rail.size.width, destinationWidth);

    stateSetter(() {
      extended = true;
    });

    await tester.pump();
    await tester.pump(const Duration(milliseconds: 100));
    expect(rail.size.width, equals(168.0));

    await tester.pumpAndSettle();
    expect(rail.size.width, equals(256.0));

    // The first destination below the rail top by some padding.
    double nextDestinationY = topPadding + destinationPadding / 2;
    final RenderBox firstIconRenderBox = _iconRenderBox(tester, Icons.favorite);
    final RenderBox firstLabelRenderBox = _labelRenderBox(tester, 'Abc');
    expect(
      firstIconRenderBox.localToGlobal(Offset.zero),
      equals(
        Offset(
          (destinationWidth - firstIconRenderBox.size.width) / 2.0,
          nextDestinationY + (destinationHeight - firstIconRenderBox.size.height) / 2.0,
        ),
      ),
    );
    expect(
      firstLabelRenderBox.localToGlobal(Offset.zero),
      equals(
        Offset(
          destinationWidth,
          nextDestinationY + (destinationHeight - firstLabelRenderBox.size.height) / 2.0,
        ),
      ),
    );

    // The second destination is one height below the first destination.
    nextDestinationY += destinationHeight + destinationPadding;
    final RenderBox secondIconRenderBox = _iconRenderBox(tester, Icons.bookmark_border);
    final RenderBox secondLabelRenderBox = _labelRenderBox(tester, 'Def');
    expect(
      secondIconRenderBox.localToGlobal(Offset.zero),
      equals(
        Offset(
          (destinationWidth - secondIconRenderBox.size.width) / 2.0,
          nextDestinationY + (destinationHeight - secondIconRenderBox.size.height) / 2.0,
        ),
      ),
    );
    expect(
      secondLabelRenderBox.localToGlobal(Offset.zero),
      equals(
        Offset(
          destinationWidth,
          nextDestinationY + (destinationHeight - secondLabelRenderBox.size.height) / 2.0,
        ),
      ),
    );

    // The third destination is one height below the second destination.
    nextDestinationY += destinationHeight + destinationPadding;
    final RenderBox thirdIconRenderBox = _iconRenderBox(tester, Icons.star_border);
    final RenderBox thirdLabelRenderBox = _labelRenderBox(tester, 'Ghi');
    expect(
      thirdIconRenderBox.localToGlobal(Offset.zero),
      equals(
        Offset(
          (destinationWidth - thirdIconRenderBox.size.width) / 2.0,
          nextDestinationY + (destinationHeight - thirdIconRenderBox.size.height) / 2.0,
        ),
      ),
    );
    expect(
      thirdLabelRenderBox.localToGlobal(Offset.zero),
      equals(
        Offset(
          destinationWidth,
          nextDestinationY + (destinationHeight - thirdLabelRenderBox.size.height) / 2.0,
        ),
      ),
    );

    // The fourth destination is one height below the third destination.
    nextDestinationY += destinationHeight + destinationPadding;
    final RenderBox fourthIconRenderBox = _iconRenderBox(tester, Icons.hotel);
    final RenderBox fourthLabelRenderBox = _labelRenderBox(tester, 'Jkl');
    expect(
      fourthIconRenderBox.localToGlobal(Offset.zero),
      equals(
        Offset(
          (destinationWidth - fourthIconRenderBox.size.width) / 2.0,
          nextDestinationY + (destinationHeight - fourthIconRenderBox.size.height) / 2.0,
        ),
      ),
    );
    expect(
      fourthLabelRenderBox.localToGlobal(Offset.zero),
      equals(
        Offset(
          destinationWidth,
          nextDestinationY + (destinationHeight - fourthLabelRenderBox.size.height) / 2.0,
        ),
      ),
    );
  });

  testWidgets('Extended rail animates the width and labels appear - [textDirection]=RTL', (
    WidgetTester tester,
  ) async {
    // Padding at the top of the rail.
    const double topPadding = 8.0;
    // Width of a destination.
    const double destinationWidth = 80.0;
    // Height of a destination indicator with icon.
    const double destinationHeight = 32.0;
    // Space between destinations.
    const double destinationPadding = 12.0;

    bool extended = false;
    late StateSetter stateSetter;

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: true),
        home: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            stateSetter = setState;
            return Directionality(
              textDirection: TextDirection.rtl,
              child: Scaffold(
                body: Row(
                  textDirection: TextDirection.rtl,
                  children: <Widget>[
                    NavigationRail(
                      selectedIndex: 0,
                      destinations: _destinations(),
                      extended: extended,
                    ),
                    const Expanded(child: Text('body')),
                  ],
                ),
              ),
            );
          },
        ),
      ),
    );

    final RenderBox rail = tester.firstRenderObject<RenderBox>(find.byType(NavigationRail));

    expect(rail.size.width, equals(destinationWidth));
    expect(rail.localToGlobal(Offset.zero), equals(const Offset(720.0, 0.0)));

    stateSetter(() {
      extended = true;
    });

    await tester.pump();
    await tester.pump(const Duration(milliseconds: 100));
    expect(rail.size.width, equals(168.0));
    expect(rail.localToGlobal(Offset.zero), equals(const Offset(632.0, 0.0)));

    await tester.pumpAndSettle();
    expect(rail.size.width, equals(256.0));
    expect(rail.localToGlobal(Offset.zero), equals(const Offset(544.0, 0.0)));

    // The first destination below the rail top by some padding.
    double nextDestinationY = topPadding + destinationPadding / 2;
    final RenderBox firstIconRenderBox = _iconRenderBox(tester, Icons.favorite);
    final RenderBox firstLabelRenderBox = _labelRenderBox(tester, 'Abc');
    expect(
      firstIconRenderBox.localToGlobal(Offset.zero),
      equals(
        Offset(
          800.0 - (destinationWidth + firstIconRenderBox.size.width) / 2.0,
          nextDestinationY + (destinationHeight - firstIconRenderBox.size.height) / 2.0,
        ),
      ),
    );
    expect(
      firstLabelRenderBox.localToGlobal(Offset.zero),
      equals(
        Offset(
          800.0 - destinationWidth - firstLabelRenderBox.size.width,
          nextDestinationY + (destinationHeight - firstLabelRenderBox.size.height) / 2.0,
        ),
      ),
    );

    // The second destination is one height below the first destination.
    nextDestinationY += destinationHeight + destinationPadding;
    final RenderBox secondIconRenderBox = _iconRenderBox(tester, Icons.bookmark_border);
    final RenderBox secondLabelRenderBox = _labelRenderBox(tester, 'Def');
    expect(
      secondIconRenderBox.localToGlobal(Offset.zero),
      equals(
        Offset(
          800.0 - (destinationWidth + secondIconRenderBox.size.width) / 2.0,
          nextDestinationY + (destinationHeight - secondIconRenderBox.size.height) / 2.0,
        ),
      ),
    );
    expect(
      secondLabelRenderBox.localToGlobal(Offset.zero),
      equals(
        Offset(
          800.0 - destinationWidth - secondLabelRenderBox.size.width,
          nextDestinationY + (destinationHeight - secondLabelRenderBox.size.height) / 2.0,
        ),
      ),
    );

    // The third destination is one height below the second destination.
    nextDestinationY += destinationHeight + destinationPadding;
    final RenderBox thirdIconRenderBox = _iconRenderBox(tester, Icons.star_border);
    final RenderBox thirdLabelRenderBox = _labelRenderBox(tester, 'Ghi');
    expect(
      thirdIconRenderBox.localToGlobal(Offset.zero),
      equals(
        Offset(
          800.0 - (destinationWidth + thirdIconRenderBox.size.width) / 2.0,
          nextDestinationY + (destinationHeight - thirdIconRenderBox.size.height) / 2.0,
        ),
      ),
    );
    expect(
      thirdLabelRenderBox.localToGlobal(Offset.zero),
      equals(
        Offset(
          800.0 - destinationWidth - thirdLabelRenderBox.size.width,
          nextDestinationY + (destinationHeight - thirdLabelRenderBox.size.height) / 2.0,
        ),
      ),
    );

    // The fourth destination is one height below the third destination.
    nextDestinationY += destinationHeight + destinationPadding;
    final RenderBox fourthIconRenderBox = _iconRenderBox(tester, Icons.hotel);
    final RenderBox fourthLabelRenderBox = _labelRenderBox(tester, 'Jkl');
    expect(
      fourthIconRenderBox.localToGlobal(Offset.zero),
      equals(
        Offset(
          800 - (destinationWidth + fourthIconRenderBox.size.width) / 2.0,
          nextDestinationY + (destinationHeight - fourthIconRenderBox.size.height) / 2.0,
        ),
      ),
    );
    expect(
      fourthLabelRenderBox.localToGlobal(Offset.zero),
      equals(
        Offset(
          800.0 - destinationWidth - fourthLabelRenderBox.size.width,
          nextDestinationY + (destinationHeight - fourthLabelRenderBox.size.height) / 2.0,
        ),
      ),
    );
  });

  testWidgets('Extended rail gets wider with longer labels are larger text scale', (
    WidgetTester tester,
  ) async {
    bool extended = false;
    late StateSetter stateSetter;

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: true),
        home: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            stateSetter = setState;
            return Scaffold(
              body: Row(
                children: <Widget>[
                  MediaQuery.withClampedTextScaling(
                    minScaleFactor: 3.0,
                    maxScaleFactor: 3.0,
                    child: NavigationRail(
                      selectedIndex: 0,
                      destinations: const <NavigationRailDestination>[
                        NavigationRailDestination(
                          icon: Icon(Icons.favorite_border),
                          selectedIcon: Icon(Icons.favorite),
                          label: Text('Abc'),
                        ),
                        NavigationRailDestination(
                          icon: Icon(Icons.bookmark_border),
                          selectedIcon: Icon(Icons.bookmark),
                          label: Text('Longer Label'),
                        ),
                      ],
                      extended: extended,
                    ),
                  ),
                  const Expanded(child: Text('body')),
                ],
              ),
            );
          },
        ),
      ),
    );

    final RenderBox rail = tester.firstRenderObject<RenderBox>(find.byType(NavigationRail));

    expect(rail.size.width, equals(80.0));

    stateSetter(() {
      extended = true;
    });

    await tester.pump();
    await tester.pump(const Duration(milliseconds: 100));
    expect(rail.size.width, equals(303.0));

    await tester.pumpAndSettle();
    expect(rail.size.width, equals(526.0));
  });

  testWidgets('Extended rail final width can be changed', (WidgetTester tester) async {
    bool extended = false;
    late StateSetter stateSetter;

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: true),
        home: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            stateSetter = setState;
            return Scaffold(
              body: Row(
                children: <Widget>[
                  NavigationRail(
                    selectedIndex: 0,
                    minExtendedWidth: 300,
                    destinations: _destinations(),
                    extended: extended,
                  ),
                  const Expanded(child: Text('body')),
                ],
              ),
            );
          },
        ),
      ),
    );

    final RenderBox rail = tester.firstRenderObject<RenderBox>(find.byType(NavigationRail));

    expect(rail.size.width, equals(80.0));

    stateSetter(() {
      extended = true;
    });

    await tester.pumpAndSettle();
    expect(rail.size.width, equals(300.0));
  });

  /// Regression test for https://github.com/flutter/flutter/issues/65657
  testWidgets('Extended rail transition does not jump from the beginning', (
    WidgetTester tester,
  ) async {
    bool extended = false;
    late StateSetter stateSetter;

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: true),
        home: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            stateSetter = setState;
            return Scaffold(
              body: Row(
                children: <Widget>[
                  NavigationRail(
                    selectedIndex: 0,
                    destinations: const <NavigationRailDestination>[
                      NavigationRailDestination(
                        icon: Icon(Icons.favorite_border),
                        selectedIcon: Icon(Icons.favorite),
                        label: Text('Abc'),
                      ),
                      NavigationRailDestination(
                        icon: Icon(Icons.bookmark_border),
                        selectedIcon: Icon(Icons.bookmark),
                        label: Text('Longer Label'),
                      ),
                    ],
                    extended: extended,
                  ),
                  const Expanded(child: Text('body')),
                ],
              ),
            );
          },
        ),
      ),
    );

    final Finder rail = find.byType(NavigationRail);

    // Before starting the animation, the rail has a width of 80.
    expect(tester.getSize(rail).width, 80.0);

    stateSetter(() {
      extended = true;
    });

    await tester.pump();
    // Create very close to 0, but non-zero, animation value.
    await tester.pump(const Duration(milliseconds: 1));
    // Expect that it has started to extend.
    expect(tester.getSize(rail).width, greaterThan(80.0));
    // Expect that it has only extended by a small amount, or that the first
    // frame does not jump. This helps verify that it is a smooth animation.
    expect(tester.getSize(rail).width, closeTo(80.0, 1.0));
  });

  testWidgets('Extended rail animation can be consumed', (WidgetTester tester) async {
    bool extended = false;
    late Animation<double> animation;
    late StateSetter stateSetter;

    await tester.pumpWidget(
      MaterialApp(
        home: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            stateSetter = setState;
            return Scaffold(
              body: Row(
                children: <Widget>[
                  NavigationRail(
                    selectedIndex: 0,
                    leading: Builder(
                      builder: (BuildContext context) {
                        animation = NavigationRail.extendedAnimation(context);
                        return FloatingActionButton(onPressed: () {});
                      },
                    ),
                    destinations: _destinations(),
                    extended: extended,
                  ),
                  const Expanded(child: Text('body')),
                ],
              ),
            );
          },
        ),
      ),
    );

    expect(animation.isDismissed, isTrue);

    stateSetter(() {
      extended = true;
    });
    await tester.pumpAndSettle();

    expect(animation.isCompleted, isTrue);
  });

  testWidgets('onDestinationSelected is called', (WidgetTester tester) async {
    late int selectedIndex;

    await _pumpNavigationRail(
      tester,
      navigationRail: NavigationRail(
        selectedIndex: 0,
        destinations: _destinations(),
        onDestinationSelected: (int index) {
          selectedIndex = index;
        },
        labelType: NavigationRailLabelType.all,
      ),
    );

    await tester.tap(find.text('Def'));
    expect(selectedIndex, 1);

    await tester.tap(find.text('Ghi'));
    expect(selectedIndex, 2);

    // Wait for any pending shader compilation.
    await tester.pumpAndSettle();
  });

  testWidgets('onDestinationSelected is not called if null', (WidgetTester tester) async {
    const int selectedIndex = 0;
    await _pumpNavigationRail(
      tester,
      navigationRail: NavigationRail(
        selectedIndex: selectedIndex,
        destinations: _destinations(),
        labelType: NavigationRailLabelType.all,
      ),
    );

    await tester.tap(find.text('Def'));
    expect(selectedIndex, 0);

    // Wait for any pending shader compilation.
    await tester.pumpAndSettle();
  });

  testWidgets('Changing destinations animate when [labelType]=selected', (
    WidgetTester tester,
  ) async {
    int selectedIndex = 0;

    await tester.pumpWidget(
      MaterialApp(
        home: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return Scaffold(
              body: Row(
                children: <Widget>[
                  NavigationRail(
                    destinations: _destinations(),
                    selectedIndex: selectedIndex,
                    labelType: NavigationRailLabelType.selected,
                    onDestinationSelected: (int index) {
                      setState(() {
                        selectedIndex = index;
                      });
                    },
                  ),
                  const Expanded(child: Text('body')),
                ],
              ),
            );
          },
        ),
      ),
    );

    // Tap the second destination.
    await tester.tap(find.byIcon(Icons.bookmark_border));
    expect(selectedIndex, 1);

    // The second destination animates in.
    expect(_labelOpacity(tester, 'Def'), equals(0.0));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 100));
    expect(_labelOpacity(tester, 'Def'), equals(0.5));
    await tester.pumpAndSettle();
    expect(_labelOpacity(tester, 'Def'), equals(1.0));

    // Tap the third destination.
    await tester.tap(find.byIcon(Icons.star_border));
    expect(selectedIndex, 2);

    // The second destination animates out quickly and the third destination
    // animates in.
    expect(_labelOpacity(tester, 'Ghi'), equals(0.0));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 25));
    expect(_labelOpacity(tester, 'Def'), equals(0.5));
    expect(_labelOpacity(tester, 'Ghi'), equals(0.0));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 25));
    expect(_labelOpacity(tester, 'Def'), equals(0.0));
    expect(_labelOpacity(tester, 'Ghi'), equals(0.0));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 50));
    expect(_labelOpacity(tester, 'Ghi'), equals(0.5));
    await tester.pumpAndSettle();
    expect(_labelOpacity(tester, 'Ghi'), equals(1.0));
  });

  testWidgets('Changing destinations animate for selectedIndex=null', (WidgetTester tester) async {
    int? selectedIndex = 0;
    late StateSetter stateSetter;

    await tester.pumpWidget(
      MaterialApp(
        home: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            stateSetter = setState;
            return Scaffold(
              body: Row(
                children: <Widget>[
                  NavigationRail(
                    destinations: _destinations(),
                    selectedIndex: selectedIndex,
                    labelType: NavigationRailLabelType.selected,
                  ),
                  const Expanded(child: Text('body')),
                ],
              ),
            );
          },
        ),
      ),
    );

    // Unset the selected index.
    stateSetter(() {
      selectedIndex = null;
    });

    // The first destination animates out.
    expect(_labelOpacity(tester, 'Abc'), equals(1.0));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 25));
    expect(_labelOpacity(tester, 'Abc'), equals(0.5));
    await tester.pumpAndSettle();
    expect(_labelOpacity(tester, 'Abc'), equals(0.0));

    // Set the selected index to the first destination.
    stateSetter(() {
      selectedIndex = 0;
    });

    // The first destination animates in.
    expect(_labelOpacity(tester, 'Abc'), equals(0.0));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 100));
    expect(_labelOpacity(tester, 'Abc'), equals(0.5));
    await tester.pumpAndSettle();
    expect(_labelOpacity(tester, 'Abc'), equals(1.0));
  });

  testWidgets('Changing destinations animate when selectedIndex=null during transition', (
    WidgetTester tester,
  ) async {
    int? selectedIndex = 0;
    late StateSetter stateSetter;

    await tester.pumpWidget(
      MaterialApp(
        home: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            stateSetter = setState;
            return Scaffold(
              body: Row(
                children: <Widget>[
                  NavigationRail(
                    destinations: _destinations(),
                    selectedIndex: selectedIndex,
                    labelType: NavigationRailLabelType.selected,
                  ),
                  const Expanded(child: Text('body')),
                ],
              ),
            );
          },
        ),
      ),
    );

    stateSetter(() {
      selectedIndex = 1;
    });

    await tester.pump();
    await tester.pump(const Duration(milliseconds: 175));

    // Interrupt while animating from index 0 to 1.
    stateSetter(() {
      selectedIndex = null;
    });

    expect(_labelOpacity(tester, 'Abc'), equals(0));
    expect(_labelOpacity(tester, 'Def'), equals(1));

    await tester.pump();
    // Create very close to 0, but non-zero, animation value.
    await tester.pump(const Duration(milliseconds: 1));
    // Ensure the opacity is animated back towards 0.
    expect(_labelOpacity(tester, 'Def'), lessThan(0.5));
    expect(_labelOpacity(tester, 'Def'), closeTo(0.5, 0.03));

    await tester.pumpAndSettle();
    expect(_labelOpacity(tester, 'Abc'), equals(0.0));
    expect(_labelOpacity(tester, 'Def'), equals(0.0));
  });

  testWidgets('Semantics - labelType=[none]', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    await _pumpLocalizedTestRail(tester, labelType: NavigationRailLabelType.none);

    expect(
      semantics,
      hasSemantics(_expectedSemantics(), ignoreId: true, ignoreTransform: true, ignoreRect: true),
    );

    semantics.dispose();
  });

  testWidgets('Semantics - labelType=[selected]', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    await _pumpLocalizedTestRail(tester, labelType: NavigationRailLabelType.selected);

    expect(
      semantics,
      hasSemantics(_expectedSemantics(), ignoreId: true, ignoreTransform: true, ignoreRect: true),
    );

    semantics.dispose();
  });

  testWidgets('Semantics - labelType=[all]', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    await _pumpLocalizedTestRail(tester, labelType: NavigationRailLabelType.all);

    expect(
      semantics,
      hasSemantics(_expectedSemantics(), ignoreId: true, ignoreTransform: true, ignoreRect: true),
    );

    semantics.dispose();
  });

  testWidgets('Semantics - extended', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    await _pumpLocalizedTestRail(tester, extended: true);

    expect(
      semantics,
      hasSemantics(_expectedSemantics(), ignoreId: true, ignoreTransform: true, ignoreRect: true),
    );

    semantics.dispose();
  });

  testWidgets('NavigationRailDestination padding properly applied - NavigationRailLabelType.all', (
    WidgetTester tester,
  ) async {
    const EdgeInsets defaultPadding = EdgeInsets.symmetric(horizontal: 8.0);
    const EdgeInsets secondItemPadding = EdgeInsets.symmetric(vertical: 30.0);
    const EdgeInsets thirdItemPadding = EdgeInsets.symmetric(horizontal: 10.0);

    await _pumpNavigationRail(
      tester,
      navigationRail: NavigationRail(
        labelType: NavigationRailLabelType.all,
        selectedIndex: 0,
        destinations: const <NavigationRailDestination>[
          NavigationRailDestination(
            icon: Icon(Icons.favorite_border),
            selectedIcon: Icon(Icons.favorite),
            label: Text('Abc'),
          ),
          NavigationRailDestination(
            icon: Icon(Icons.bookmark_border),
            selectedIcon: Icon(Icons.bookmark),
            label: Text('Def'),
            padding: secondItemPadding,
          ),
          NavigationRailDestination(
            icon: Icon(Icons.star_border),
            selectedIcon: Icon(Icons.star),
            label: Text('Ghi'),
            padding: thirdItemPadding,
          ),
        ],
      ),
    );

    final Iterable<Widget> indicatorInkWells = tester.allWidgets.where(
      (Widget object) => object.runtimeType.toString() == '_IndicatorInkWell',
    );
    final Padding firstItem = tester.widget<Padding>(
      find.descendant(
        of: find.widgetWithText(indicatorInkWells.elementAt(0).runtimeType, 'Abc'),
        matching: find.widgetWithText(Padding, 'Abc'),
      ),
    );
    final Padding secondItem = tester.widget<Padding>(
      find.descendant(
        of: find.widgetWithText(indicatorInkWells.elementAt(1).runtimeType, 'Def'),
        matching: find.widgetWithText(Padding, 'Def'),
      ),
    );
    final Padding thirdItem = tester.widget<Padding>(
      find.descendant(
        of: find.widgetWithText(indicatorInkWells.elementAt(2).runtimeType, 'Ghi'),
        matching: find.widgetWithText(Padding, 'Ghi'),
      ),
    );

    expect(firstItem.padding, defaultPadding);
    expect(secondItem.padding, secondItemPadding);
    expect(thirdItem.padding, thirdItemPadding);
  });

  testWidgets(
    'NavigationRailDestination padding properly applied - NavigationRailLabelType.selected',
    (WidgetTester tester) async {
      const EdgeInsets defaultPadding = EdgeInsets.symmetric(horizontal: 8.0);
      const EdgeInsets secondItemPadding = EdgeInsets.symmetric(vertical: 30.0);
      const EdgeInsets thirdItemPadding = EdgeInsets.symmetric(horizontal: 10.0);

      await _pumpNavigationRail(
        tester,
        navigationRail: NavigationRail(
          labelType: NavigationRailLabelType.selected,
          selectedIndex: 0,
          destinations: const <NavigationRailDestination>[
            NavigationRailDestination(
              icon: Icon(Icons.favorite_border),
              selectedIcon: Icon(Icons.favorite),
              label: Text('Abc'),
            ),
            NavigationRailDestination(
              icon: Icon(Icons.bookmark_border),
              selectedIcon: Icon(Icons.bookmark),
              label: Text('Def'),
              padding: secondItemPadding,
            ),
            NavigationRailDestination(
              icon: Icon(Icons.star_border),
              selectedIcon: Icon(Icons.star),
              label: Text('Ghi'),
              padding: thirdItemPadding,
            ),
          ],
        ),
      );

      final Iterable<Widget> indicatorInkWells = tester.allWidgets.where(
        (Widget object) => object.runtimeType.toString() == '_IndicatorInkWell',
      );
      final Padding firstItem = tester.widget<Padding>(
        find.descendant(
          of: find.widgetWithText(indicatorInkWells.elementAt(0).runtimeType, 'Abc'),
          matching: find.widgetWithText(Padding, 'Abc'),
        ),
      );
      final Padding secondItem = tester.widget<Padding>(
        find.descendant(
          of: find.widgetWithText(indicatorInkWells.elementAt(1).runtimeType, 'Def'),
          matching: find.widgetWithText(Padding, 'Def'),
        ),
      );
      final Padding thirdItem = tester.widget<Padding>(
        find.descendant(
          of: find.widgetWithText(indicatorInkWells.elementAt(2).runtimeType, 'Ghi'),
          matching: find.widgetWithText(Padding, 'Ghi'),
        ),
      );

      expect(firstItem.padding, defaultPadding);
      expect(secondItem.padding, secondItemPadding);
      expect(thirdItem.padding, thirdItemPadding);
    },
  );

  testWidgets('NavigationRailDestination padding properly applied - NavigationRailLabelType.none', (
    WidgetTester tester,
  ) async {
    const EdgeInsets defaultPadding = EdgeInsets.zero;
    const EdgeInsets secondItemPadding = EdgeInsets.symmetric(vertical: 30.0);
    const EdgeInsets thirdItemPadding = EdgeInsets.symmetric(horizontal: 10.0);

    await _pumpNavigationRail(
      tester,
      navigationRail: NavigationRail(
        labelType: NavigationRailLabelType.none,
        selectedIndex: 0,
        destinations: const <NavigationRailDestination>[
          NavigationRailDestination(
            icon: Icon(Icons.favorite_border),
            selectedIcon: Icon(Icons.favorite),
            label: Text('Abc'),
          ),
          NavigationRailDestination(
            icon: Icon(Icons.bookmark_border),
            selectedIcon: Icon(Icons.bookmark),
            label: Text('Def'),
            padding: secondItemPadding,
          ),
          NavigationRailDestination(
            icon: Icon(Icons.star_border),
            selectedIcon: Icon(Icons.star),
            label: Text('Ghi'),
            padding: thirdItemPadding,
          ),
        ],
      ),
    );

    final Iterable<Widget> indicatorInkWells = tester.allWidgets.where(
      (Widget object) => object.runtimeType.toString() == '_IndicatorInkWell',
    );
    final Padding firstItem = tester.widget<Padding>(
      find.descendant(
        of: find.widgetWithText(indicatorInkWells.elementAt(0).runtimeType, 'Abc'),
        matching: find.widgetWithText(Padding, 'Abc'),
      ),
    );
    final Padding secondItem = tester.widget<Padding>(
      find.descendant(
        of: find.widgetWithText(indicatorInkWells.elementAt(1).runtimeType, 'Def'),
        matching: find.widgetWithText(Padding, 'Def'),
      ),
    );
    final Padding thirdItem = tester.widget<Padding>(
      find.descendant(
        of: find.widgetWithText(indicatorInkWells.elementAt(2).runtimeType, 'Ghi'),
        matching: find.widgetWithText(Padding, 'Ghi'),
      ),
    );

    expect(firstItem.padding, defaultPadding);
    expect(secondItem.padding, secondItemPadding);
    expect(thirdItem.padding, thirdItemPadding);
  });

  testWidgets(
    'NavigationRailDestination adds indicator by default when ThemeData.useMaterial3 is true',
    (WidgetTester tester) async {
      await _pumpNavigationRail(
        tester,
        navigationRail: NavigationRail(
          labelType: NavigationRailLabelType.selected,
          selectedIndex: 0,
          destinations: const <NavigationRailDestination>[
            NavigationRailDestination(
              icon: Icon(Icons.favorite_border),
              selectedIcon: Icon(Icons.favorite),
              label: Text('Abc'),
            ),
            NavigationRailDestination(
              icon: Icon(Icons.bookmark_border),
              selectedIcon: Icon(Icons.bookmark),
              label: Text('Def'),
            ),
            NavigationRailDestination(
              icon: Icon(Icons.star_border),
              selectedIcon: Icon(Icons.star),
              label: Text('Ghi'),
            ),
          ],
        ),
      );

      expect(find.byType(NavigationIndicator), findsWidgets);
    },
  );

  testWidgets('NavigationRailDestination adds indicator when useIndicator is true', (
    WidgetTester tester,
  ) async {
    await _pumpNavigationRail(
      tester,
      navigationRail: NavigationRail(
        useIndicator: true,
        labelType: NavigationRailLabelType.selected,
        selectedIndex: 0,
        destinations: const <NavigationRailDestination>[
          NavigationRailDestination(
            icon: Icon(Icons.favorite_border),
            selectedIcon: Icon(Icons.favorite),
            label: Text('Abc'),
          ),
          NavigationRailDestination(
            icon: Icon(Icons.bookmark_border),
            selectedIcon: Icon(Icons.bookmark),
            label: Text('Def'),
          ),
          NavigationRailDestination(
            icon: Icon(Icons.star_border),
            selectedIcon: Icon(Icons.star),
            label: Text('Ghi'),
          ),
        ],
      ),
    );

    expect(find.byType(NavigationIndicator), findsWidgets);
  });

  testWidgets('NavigationRailDestination does not add indicator when useIndicator is false', (
    WidgetTester tester,
  ) async {
    await _pumpNavigationRail(
      tester,
      navigationRail: NavigationRail(
        useIndicator: false,
        labelType: NavigationRailLabelType.selected,
        selectedIndex: 0,
        destinations: const <NavigationRailDestination>[
          NavigationRailDestination(
            icon: Icon(Icons.favorite_border),
            selectedIcon: Icon(Icons.favorite),
            label: Text('Abc'),
          ),
          NavigationRailDestination(
            icon: Icon(Icons.bookmark_border),
            selectedIcon: Icon(Icons.bookmark),
            label: Text('Def'),
          ),
          NavigationRailDestination(
            icon: Icon(Icons.star_border),
            selectedIcon: Icon(Icons.star),
            label: Text('Ghi'),
          ),
        ],
      ),
    );

    expect(find.byType(NavigationIndicator), findsNothing);
  });

  testWidgets('NavigationRailDestination adds an oval indicator when no labels are present', (
    WidgetTester tester,
  ) async {
    await _pumpNavigationRail(
      tester,
      navigationRail: NavigationRail(
        useIndicator: true,
        labelType: NavigationRailLabelType.none,
        selectedIndex: 0,
        destinations: const <NavigationRailDestination>[
          NavigationRailDestination(
            icon: Icon(Icons.favorite_border),
            selectedIcon: Icon(Icons.favorite),
            label: Text('Abc'),
          ),
          NavigationRailDestination(
            icon: Icon(Icons.bookmark_border),
            selectedIcon: Icon(Icons.bookmark),
            label: Text('Def'),
          ),
          NavigationRailDestination(
            icon: Icon(Icons.star_border),
            selectedIcon: Icon(Icons.star),
            label: Text('Ghi'),
          ),
        ],
      ),
    );

    final NavigationIndicator indicator = tester.widget<NavigationIndicator>(
      find.byType(NavigationIndicator).first,
    );

    expect(indicator.width, 56);
    expect(indicator.height, 32);
  });

  testWidgets('NavigationRailDestination adds an oval indicator when selected labels are present', (
    WidgetTester tester,
  ) async {
    await _pumpNavigationRail(
      tester,
      navigationRail: NavigationRail(
        useIndicator: true,
        labelType: NavigationRailLabelType.selected,
        selectedIndex: 0,
        destinations: const <NavigationRailDestination>[
          NavigationRailDestination(
            icon: Icon(Icons.favorite_border),
            selectedIcon: Icon(Icons.favorite),
            label: Text('Abc'),
          ),
          NavigationRailDestination(
            icon: Icon(Icons.bookmark_border),
            selectedIcon: Icon(Icons.bookmark),
            label: Text('Def'),
          ),
          NavigationRailDestination(
            icon: Icon(Icons.star_border),
            selectedIcon: Icon(Icons.star),
            label: Text('Ghi'),
          ),
        ],
      ),
    );

    final NavigationIndicator indicator = tester.widget<NavigationIndicator>(
      find.byType(NavigationIndicator).first,
    );

    expect(indicator.width, 56);
    expect(indicator.height, 32);
  });

  testWidgets('NavigationRailDestination adds an oval indicator when all labels are present', (
    WidgetTester tester,
  ) async {
    await _pumpNavigationRail(
      tester,
      navigationRail: NavigationRail(
        useIndicator: true,
        labelType: NavigationRailLabelType.all,
        selectedIndex: 0,
        destinations: const <NavigationRailDestination>[
          NavigationRailDestination(
            icon: Icon(Icons.favorite_border),
            selectedIcon: Icon(Icons.favorite),
            label: Text('Abc'),
          ),
          NavigationRailDestination(
            icon: Icon(Icons.bookmark_border),
            selectedIcon: Icon(Icons.bookmark),
            label: Text('Def'),
          ),
          NavigationRailDestination(
            icon: Icon(Icons.star_border),
            selectedIcon: Icon(Icons.star),
            label: Text('Ghi'),
          ),
        ],
      ),
    );

    final NavigationIndicator indicator = tester.widget<NavigationIndicator>(
      find.byType(NavigationIndicator).first,
    );

    expect(indicator.width, 56);
    expect(indicator.height, 32);
  });

  testWidgets('NavigationRailDestination has center aligned indicator - [labelType]=none', (
    WidgetTester tester,
  ) async {
    // This is a regression test for
    // https://github.com/flutter/flutter/issues/97753
    await _pumpNavigationRail(
      tester,
      navigationRail: NavigationRail(
        labelType: NavigationRailLabelType.none,
        selectedIndex: 0,
        destinations: const <NavigationRailDestination>[
          NavigationRailDestination(
            icon: Stack(
              children: <Widget>[
                Icon(Icons.umbrella),
                Positioned(
                  top: 0,
                  right: 0,
                  child: Text('Text', style: TextStyle(fontSize: 10, color: Colors.red)),
                ),
              ],
            ),
            label: Text('Abc'),
          ),
          NavigationRailDestination(icon: Icon(Icons.umbrella), label: Text('Def')),
          NavigationRailDestination(icon: Icon(Icons.bookmark_border), label: Text('Ghi')),
        ],
      ),
    );
    // Indicator with Stack widget
    final RenderBox firstIndicator = tester.renderObject(find.byType(Icon).first);
    expect(firstIndicator.localToGlobal(Offset.zero).dx, 28.0);
    // Indicator without Stack widget
    final RenderBox lastIndicator = tester.renderObject(find.byType(Icon).last);
    expect(lastIndicator.localToGlobal(Offset.zero).dx, 28.0);
  });

  testWidgets('NavigationRail respects the notch/system navigation bar in landscape mode', (
    WidgetTester tester,
  ) async {
    const double safeAreaPadding = 40.0;
    NavigationRail navigationRail() {
      return NavigationRail(
        selectedIndex: 0,
        destinations: const <NavigationRailDestination>[
          NavigationRailDestination(
            icon: Icon(Icons.favorite_border),
            selectedIcon: Icon(Icons.favorite),
            label: Text('Abc'),
          ),
          NavigationRailDestination(
            icon: Icon(Icons.bookmark_border),
            selectedIcon: Icon(Icons.bookmark),
            label: Text('Def'),
          ),
        ],
      );
    }

    await tester.pumpWidget(_buildWidget(navigationRail()));
    final double defaultWidth = tester.getSize(find.byType(NavigationRail)).width;
    expect(defaultWidth, 80);

    await tester.pumpWidget(
      _buildWidget(
        MediaQuery(
          data: const MediaQueryData(padding: EdgeInsets.only(left: safeAreaPadding)),
          child: navigationRail(),
        ),
      ),
    );
    final double updatedWidth = tester.getSize(find.byType(NavigationRail)).width;
    expect(updatedWidth, defaultWidth + safeAreaPadding);

    // test width when text direction is RTL.
    await tester.pumpWidget(
      _buildWidget(
        MediaQuery(
          data: const MediaQueryData(padding: EdgeInsets.only(right: safeAreaPadding)),
          child: navigationRail(),
        ),
        isRTL: true,
      ),
    );
    final double updatedWidthRTL = tester.getSize(find.byType(NavigationRail)).width;
    expect(updatedWidthRTL, defaultWidth + safeAreaPadding);
  });

  testWidgets('NavigationRail indicator renders ripple', (WidgetTester tester) async {
    await _pumpNavigationRail(
      tester,
      navigationRail: NavigationRail(
        selectedIndex: 1,
        destinations: const <NavigationRailDestination>[
          NavigationRailDestination(
            icon: Icon(Icons.favorite_border),
            selectedIcon: Icon(Icons.favorite),
            label: Text('Abc'),
          ),
          NavigationRailDestination(
            icon: Icon(Icons.bookmark_border),
            selectedIcon: Icon(Icons.bookmark),
            label: Text('Def'),
          ),
        ],
        labelType: NavigationRailLabelType.all,
      ),
    );

    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(tester.getCenter(find.byIcon(Icons.favorite_border)));
    await tester.pumpAndSettle();

    final RenderObject inkFeatures = tester.allRenderObjects.firstWhere(
      (RenderObject object) => object.runtimeType.toString() == '_RenderInkFeatures',
    );
    const Rect indicatorRect = Rect.fromLTRB(12.0, 0.0, 68.0, 32.0);
    const Rect includedRect = indicatorRect;
    final Rect excludedRect = includedRect.inflate(10);

    expect(
      inkFeatures,
      paints
        ..clipPath(
          pathMatcher: isPathThat(
            includes: <Offset>[
              includedRect.centerLeft,
              includedRect.topCenter,
              includedRect.centerRight,
              includedRect.bottomCenter,
            ],
            excludes: <Offset>[
              excludedRect.centerLeft,
              excludedRect.topCenter,
              excludedRect.centerRight,
              excludedRect.bottomCenter,
            ],
          ),
        )
        ..rect(rect: indicatorRect, color: const Color(0x0a6750a4))
        ..rrect(
          rrect: RRect.fromLTRBR(12.0, 72.0, 68.0, 104.0, const Radius.circular(16)),
          color: const Color(0xffe8def8),
        ),
    );
  }, skip: kIsWeb && !isSkiaWeb); // https://github.com/flutter/flutter/issues/99933

  testWidgets('NavigationRail indicator renders ripple - extended', (WidgetTester tester) async {
    // This is a regression test for https://github.com/flutter/flutter/issues/117126
    await _pumpNavigationRail(
      tester,
      navigationRail: NavigationRail(
        selectedIndex: 1,
        extended: true,
        destinations: const <NavigationRailDestination>[
          NavigationRailDestination(
            icon: Icon(Icons.favorite_border),
            selectedIcon: Icon(Icons.favorite),
            label: Text('Abc'),
          ),
          NavigationRailDestination(
            icon: Icon(Icons.bookmark_border),
            selectedIcon: Icon(Icons.bookmark),
            label: Text('Def'),
          ),
        ],
        labelType: NavigationRailLabelType.none,
      ),
    );

    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(tester.getCenter(find.byIcon(Icons.favorite_border)));
    await tester.pumpAndSettle();

    final RenderObject inkFeatures = tester.allRenderObjects.firstWhere(
      (RenderObject object) => object.runtimeType.toString() == '_RenderInkFeatures',
    );
    const Rect indicatorRect = Rect.fromLTRB(12.0, 6.0, 68.0, 38.0);
    const Rect includedRect = indicatorRect;
    final Rect excludedRect = includedRect.inflate(10);

    expect(
      inkFeatures,
      paints
        ..clipPath(
          pathMatcher: isPathThat(
            includes: <Offset>[
              includedRect.centerLeft,
              includedRect.topCenter,
              includedRect.centerRight,
              includedRect.bottomCenter,
            ],
            excludes: <Offset>[
              excludedRect.centerLeft,
              excludedRect.topCenter,
              excludedRect.centerRight,
              excludedRect.bottomCenter,
            ],
          ),
        )
        ..rect(rect: indicatorRect, color: const Color(0x0a6750a4))
        ..rrect(
          rrect: RRect.fromLTRBR(12.0, 58.0, 68.0, 90.0, const Radius.circular(16)),
          color: const Color(0xffe8def8),
        ),
    );
  });

  testWidgets('NavigationRail indicator renders properly when padding is applied', (
    WidgetTester tester,
  ) async {
    // This is a regression test for https://github.com/flutter/flutter/issues/117126
    await _pumpNavigationRail(
      tester,
      navigationRail: NavigationRail(
        selectedIndex: 1,
        extended: true,
        destinations: const <NavigationRailDestination>[
          NavigationRailDestination(
            padding: EdgeInsets.all(10),
            icon: Icon(Icons.favorite_border),
            selectedIcon: Icon(Icons.favorite),
            label: Text('Abc'),
          ),
          NavigationRailDestination(
            padding: EdgeInsets.all(18),
            icon: Icon(Icons.bookmark_border),
            selectedIcon: Icon(Icons.bookmark),
            label: Text('Def'),
          ),
        ],
        labelType: NavigationRailLabelType.none,
      ),
    );

    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(tester.getCenter(find.byIcon(Icons.favorite_border)));
    await tester.pumpAndSettle();

    final RenderObject inkFeatures = tester.allRenderObjects.firstWhere(
      (RenderObject object) => object.runtimeType.toString() == '_RenderInkFeatures',
    );
    const Rect indicatorRect = Rect.fromLTRB(22.0, 16.0, 78.0, 48.0);
    const Rect includedRect = indicatorRect;
    final Rect excludedRect = includedRect.inflate(10);

    expect(
      inkFeatures,
      paints
        ..clipPath(
          pathMatcher: isPathThat(
            includes: <Offset>[
              includedRect.centerLeft,
              includedRect.topCenter,
              includedRect.centerRight,
              includedRect.bottomCenter,
            ],
            excludes: <Offset>[
              excludedRect.centerLeft,
              excludedRect.topCenter,
              excludedRect.centerRight,
              excludedRect.bottomCenter,
            ],
          ),
        )
        ..rect(rect: indicatorRect, color: const Color(0x0a6750a4))
        ..rrect(
          rrect: RRect.fromLTRBR(30.0, 96.0, 86.0, 128.0, const Radius.circular(16)),
          color: const Color(0xffe8def8),
        ),
    );
  });

  testWidgets('Indicator renders properly when NavigationRai.minWidth < default minWidth', (
    WidgetTester tester,
  ) async {
    // This is a regression test for https://github.com/flutter/flutter/issues/117126
    await _pumpNavigationRail(
      tester,
      navigationRail: NavigationRail(
        minWidth: 50,
        selectedIndex: 1,
        extended: true,
        destinations: const <NavigationRailDestination>[
          NavigationRailDestination(
            icon: Icon(Icons.favorite_border),
            selectedIcon: Icon(Icons.favorite),
            label: Text('Abc'),
          ),
          NavigationRailDestination(
            icon: Icon(Icons.bookmark_border),
            selectedIcon: Icon(Icons.bookmark),
            label: Text('Def'),
          ),
        ],
        labelType: NavigationRailLabelType.none,
      ),
    );

    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(tester.getCenter(find.byIcon(Icons.favorite_border)));
    await tester.pumpAndSettle();

    final RenderObject inkFeatures = tester.allRenderObjects.firstWhere(
      (RenderObject object) => object.runtimeType.toString() == '_RenderInkFeatures',
    );
    const Rect indicatorRect = Rect.fromLTRB(-3.0, 6.0, 53.0, 38.0);
    const Rect includedRect = indicatorRect;
    final Rect excludedRect = includedRect.inflate(10);

    expect(
      inkFeatures,
      paints
        ..clipPath(
          pathMatcher: isPathThat(
            includes: <Offset>[
              includedRect.centerLeft,
              includedRect.topCenter,
              includedRect.centerRight,
              includedRect.bottomCenter,
            ],
            excludes: <Offset>[
              excludedRect.centerLeft,
              excludedRect.topCenter,
              excludedRect.centerRight,
              excludedRect.bottomCenter,
            ],
          ),
        )
        ..rect(rect: indicatorRect, color: const Color(0x0a6750a4))
        ..rrect(
          rrect: RRect.fromLTRBR(0.0, 58.0, 50.0, 90.0, const Radius.circular(16)),
          color: const Color(0xffe8def8),
        ),
    );
  });

  testWidgets('NavigationRail indicator renders properly with custom padding and minWidth', (
    WidgetTester tester,
  ) async {
    // This is a regression test for https://github.com/flutter/flutter/issues/117126
    await _pumpNavigationRail(
      tester,
      navigationRail: NavigationRail(
        minWidth: 300,
        selectedIndex: 1,
        extended: true,
        destinations: const <NavigationRailDestination>[
          NavigationRailDestination(
            padding: EdgeInsets.all(10),
            icon: Icon(Icons.favorite_border),
            selectedIcon: Icon(Icons.favorite),
            label: Text('Abc'),
          ),
          NavigationRailDestination(
            padding: EdgeInsets.all(18),
            icon: Icon(Icons.bookmark_border),
            selectedIcon: Icon(Icons.bookmark),
            label: Text('Def'),
          ),
        ],
        labelType: NavigationRailLabelType.none,
      ),
    );

    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(tester.getCenter(find.byIcon(Icons.favorite_border)));
    await tester.pumpAndSettle();

    final RenderObject inkFeatures = tester.allRenderObjects.firstWhere(
      (RenderObject object) => object.runtimeType.toString() == '_RenderInkFeatures',
    );
    const Rect indicatorRect = Rect.fromLTRB(132.0, 16.0, 188.0, 48.0);
    const Rect includedRect = indicatorRect;
    final Rect excludedRect = includedRect.inflate(10);

    expect(
      inkFeatures,
      paints
        ..clipPath(
          pathMatcher: isPathThat(
            includes: <Offset>[
              includedRect.centerLeft,
              includedRect.topCenter,
              includedRect.centerRight,
              includedRect.bottomCenter,
            ],
            excludes: <Offset>[
              excludedRect.centerLeft,
              excludedRect.topCenter,
              excludedRect.centerRight,
              excludedRect.bottomCenter,
            ],
          ),
        )
        ..rect(rect: indicatorRect, color: const Color(0x0a6750a4))
        ..rrect(
          rrect: RRect.fromLTRBR(140.0, 96.0, 196.0, 128.0, const Radius.circular(16)),
          color: const Color(0xffe8def8),
        ),
    );
  });

  testWidgets('NavigationRail indicator renders properly with long labels', (
    WidgetTester tester,
  ) async {
    // This is a regression test for https://github.com/flutter/flutter/issues/128005.
    await _pumpNavigationRail(
      tester,
      navigationRail: NavigationRail(
        selectedIndex: 1,
        destinations: const <NavigationRailDestination>[
          NavigationRailDestination(
            icon: Icon(Icons.favorite_border),
            selectedIcon: Icon(Icons.favorite),
            label: Text('ABCDEFGHIJKLMNOPQRSTUVWXYZ'),
          ),
          NavigationRailDestination(
            icon: Icon(Icons.bookmark_border),
            selectedIcon: Icon(Icons.bookmark),
            label: Text('ABC'),
          ),
        ],
        labelType: NavigationRailLabelType.all,
      ),
    );

    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(tester.getCenter(find.byIcon(Icons.favorite_border)));
    await tester.pumpAndSettle();

    final RenderObject inkFeatures = tester.allRenderObjects.firstWhere(
      (RenderObject object) => object.runtimeType.toString() == '_RenderInkFeatures',
    );

    // Default values from M3 specification.
    const double indicatorHeight = 32.0;
    const double destinationWidth = 72.0;
    const double destinationHorizontalPadding = 8.0;
    const double indicatorWidth = destinationWidth - 2 * destinationHorizontalPadding; // 56.0
    const double verticalSpacer = 8.0;
    const double verticalIconLabelSpacing = 4.0;
    const double verticalDestinationSpacing = 12.0;

    // The navigation rail width is larger than default because of the first destination long label.
    final double railWidth = tester.getSize(find.byType(NavigationRail)).width;

    // Expected indicator position.
    final double indicatorLeft = (railWidth - indicatorWidth) / 2;
    final double indicatorRight = (railWidth + indicatorWidth) / 2;
    final Rect indicatorRect = Rect.fromLTRB(indicatorLeft, 0.0, indicatorRight, indicatorHeight);
    final Rect includedRect = indicatorRect;
    final Rect excludedRect = includedRect.inflate(10);

    // Compute the vertical position for the selected destination (the one with 'bookmark' icon).
    const double labelHeight = 16; // fontSize is 12 and height is 1.3.
    const double destinationHeight =
        indicatorHeight + verticalIconLabelSpacing + labelHeight + verticalDestinationSpacing;
    const double secondDestinationVerticalOffset = verticalSpacer + destinationHeight;
    const double secondIndicatorVerticalOffset = secondDestinationVerticalOffset;

    expect(
      inkFeatures,
      paints
        ..clipPath(
          pathMatcher: isPathThat(
            includes: <Offset>[
              includedRect.centerLeft,
              includedRect.topCenter,
              includedRect.centerRight,
              includedRect.bottomCenter,
            ],
            excludes: <Offset>[
              excludedRect.centerLeft,
              excludedRect.topCenter,
              excludedRect.centerRight,
              excludedRect.bottomCenter,
            ],
          ),
        )
        ..rect(rect: indicatorRect, color: const Color(0x0a6750a4))
        ..rrect(
          rrect: RRect.fromLTRBR(
            indicatorLeft,
            secondIndicatorVerticalOffset,
            indicatorRight,
            secondIndicatorVerticalOffset + indicatorHeight,
            const Radius.circular(16),
          ),
          color: const Color(0xffe8def8),
        ),
    );
  }, skip: kIsWeb && !isSkiaWeb); // https://github.com/flutter/flutter/issues/99933

  testWidgets('NavigationRail indicator renders properly with large icon', (
    WidgetTester tester,
  ) async {
    // This is a regression test for https://github.com/flutter/flutter/issues/133799.
    const double iconSize = 50;
    await _pumpNavigationRail(
      tester,
      navigationRailTheme: const NavigationRailThemeData(
        selectedIconTheme: IconThemeData(size: iconSize),
        unselectedIconTheme: IconThemeData(size: iconSize),
      ),
      navigationRail: NavigationRail(
        selectedIndex: 1,
        destinations: const <NavigationRailDestination>[
          NavigationRailDestination(
            icon: Icon(Icons.favorite_border),
            selectedIcon: Icon(Icons.favorite),
            label: Text('ABC'),
          ),
          NavigationRailDestination(
            icon: Icon(Icons.bookmark_border),
            selectedIcon: Icon(Icons.bookmark),
            label: Text('DEF'),
          ),
        ],
        labelType: NavigationRailLabelType.all,
      ),
    );

    // Hover the first destination.
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(tester.getCenter(find.byIcon(Icons.favorite_border)));
    await tester.pumpAndSettle();

    final RenderObject inkFeatures = tester.allRenderObjects.firstWhere(
      (RenderObject object) => object.runtimeType.toString() == '_RenderInkFeatures',
    );

    // Default values from M3 specification.
    const double railMinWidth = 80.0;
    const double indicatorHeight = 32.0;
    const double destinationWidth = 72.0;
    const double destinationHorizontalPadding = 8.0;
    const double indicatorWidth = destinationWidth - 2 * destinationHorizontalPadding; // 56.0
    const double verticalSpacer = 8.0;
    const double verticalIconLabelSpacing = 4.0;
    const double verticalDestinationSpacing = 12.0;

    // The navigation rail width is the default one because labels are short.
    final double railWidth = tester.getSize(find.byType(NavigationRail)).width;
    expect(railWidth, railMinWidth);

    // Expected indicator position.
    final double indicatorLeft = (railWidth - indicatorWidth) / 2;
    final double indicatorRight = (railWidth + indicatorWidth) / 2;
    const double indicatorTop = (iconSize - indicatorHeight) / 2;
    const double indicatorBottom = (iconSize + indicatorHeight) / 2;
    final Rect indicatorRect = Rect.fromLTRB(
      indicatorLeft,
      indicatorTop,
      indicatorRight,
      indicatorBottom,
    );
    final Rect includedRect = indicatorRect;
    final Rect excludedRect = includedRect.inflate(10);

    // Compute the vertical position for the selected destination (the one with 'bookmark' icon).
    const double labelHeight = 16; // fontSize is 12 and height is 1.3.
    const double destinationHeight =
        iconSize + verticalIconLabelSpacing + labelHeight + verticalDestinationSpacing;
    const double secondDestinationVerticalOffset = verticalSpacer + destinationHeight;
    const double indicatorOffset = (iconSize - indicatorHeight) / 2;
    const double secondIndicatorVerticalOffset = secondDestinationVerticalOffset + indicatorOffset;

    expect(
      inkFeatures,
      paints
        ..clipPath(
          pathMatcher: isPathThat(
            includes: <Offset>[
              includedRect.centerLeft,
              includedRect.topCenter,
              includedRect.centerRight,
              includedRect.bottomCenter,
            ],
            excludes: <Offset>[
              excludedRect.centerLeft,
              excludedRect.topCenter,
              excludedRect.centerRight,
              excludedRect.bottomCenter,
            ],
          ),
        )
        // Hover highlight for the hovered destination (the one with 'favorite' icon).
        ..rect(rect: indicatorRect, color: const Color(0x0a6750a4))
        // Indicator for the selected destination (the one with 'bookmark' icon).
        ..rrect(
          rrect: RRect.fromLTRBR(
            indicatorLeft,
            secondIndicatorVerticalOffset,
            indicatorRight,
            secondIndicatorVerticalOffset + indicatorHeight,
            const Radius.circular(16),
          ),
          color: const Color(0xffe8def8),
        ),
    );
  }, skip: kIsWeb && !isSkiaWeb); // https://github.com/flutter/flutter/issues/99933

  testWidgets('NavigationRail indicator renders properly when text direction is rtl', (
    WidgetTester tester,
  ) async {
    // This is a regression test for https://github.com/flutter/flutter/issues/134361.
    await tester.pumpWidget(
      _buildWidget(
        NavigationRail(
          selectedIndex: 1,
          extended: true,
          destinations: const <NavigationRailDestination>[
            NavigationRailDestination(
              icon: Icon(Icons.favorite_border),
              selectedIcon: Icon(Icons.favorite),
              label: Text('ABC'),
            ),
            NavigationRailDestination(
              icon: Icon(Icons.bookmark_border),
              selectedIcon: Icon(Icons.bookmark),
              label: Text('DEF'),
            ),
          ],
        ),
        isRTL: true,
      ),
    );

    // Hover the first destination.
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(tester.getCenter(find.byIcon(Icons.favorite_border)));
    await tester.pumpAndSettle();

    final RenderObject inkFeatures = tester.allRenderObjects.firstWhere(
      (RenderObject object) => object.runtimeType.toString() == '_RenderInkFeatures',
    );

    // Default values from M3 specification.
    const double railMinExtendedWidth = 256.0;
    const double indicatorHeight = 32.0;
    const double destinationWidth = 72.0;
    const double destinationHorizontalPadding = 8.0;
    const double indicatorWidth = destinationWidth - 2 * destinationHorizontalPadding; // 56.0
    const double verticalSpacer = 8.0;
    const double verticalDestinationSpacingM3 = 12.0;

    // The navigation rail width is the default one because labels are short.
    final double railWidth = tester.getSize(find.byType(NavigationRail)).width;
    expect(railWidth, railMinExtendedWidth);

    // Expected indicator position.
    final double indicatorLeft = railWidth - (destinationWidth - destinationHorizontalPadding / 2);
    final double indicatorRight = indicatorLeft + indicatorWidth;
    final Rect indicatorRect = Rect.fromLTRB(
      indicatorLeft,
      verticalDestinationSpacingM3 / 2,
      indicatorRight,
      verticalDestinationSpacingM3 / 2 + indicatorHeight,
    );
    final Rect includedRect = indicatorRect;
    final Rect excludedRect = includedRect.inflate(10);

    // Compute the vertical position for the selected destination (the one with 'bookmark' icon).
    const double destinationHeight = indicatorHeight + verticalDestinationSpacingM3;
    const double secondDestinationVerticalOffset = verticalSpacer + destinationHeight;
    const double secondIndicatorVerticalOffset =
        secondDestinationVerticalOffset + verticalDestinationSpacingM3 / 2;
    const double secondDestinationHorizontalOffset = 800 - railMinExtendedWidth; // RTL.

    expect(
      inkFeatures,
      paints
        ..clipPath(
          pathMatcher: isPathThat(
            includes: <Offset>[
              includedRect.centerLeft,
              includedRect.topCenter,
              includedRect.centerRight,
              includedRect.bottomCenter,
            ],
            excludes: <Offset>[
              excludedRect.centerLeft,
              excludedRect.topCenter,
              excludedRect.centerRight,
              excludedRect.bottomCenter,
            ],
          ),
        )
        // Hover highlight for the hovered destination (the one with 'favorite' icon).
        ..rect(rect: indicatorRect, color: const Color(0x0a6750a4))
        // Indicator for the selected destination (the one with 'bookmark' icon).
        ..rrect(
          rrect: RRect.fromLTRBR(
            secondDestinationHorizontalOffset + indicatorLeft,
            secondIndicatorVerticalOffset,
            secondDestinationHorizontalOffset + indicatorRight,
            secondIndicatorVerticalOffset + indicatorHeight,
            const Radius.circular(16),
          ),
          color: const Color(0xffe8def8),
        ),
    );
  }, skip: kIsWeb && !isSkiaWeb); // https://github.com/flutter/flutter/issues/99933

  testWidgets('NavigationRail indicator scale transform', (WidgetTester tester) async {
    int selectedIndex = 0;
    Future<void> buildWidget() async {
      await _pumpNavigationRail(
        tester,
        navigationRail: NavigationRail(
          selectedIndex: selectedIndex,
          destinations: const <NavigationRailDestination>[
            NavigationRailDestination(
              icon: Icon(Icons.favorite_border),
              selectedIcon: Icon(Icons.favorite),
              label: Text('Abc'),
            ),
            NavigationRailDestination(
              icon: Icon(Icons.bookmark_border),
              selectedIcon: Icon(Icons.bookmark),
              label: Text('Def'),
            ),
          ],
          labelType: NavigationRailLabelType.all,
        ),
      );
    }

    await buildWidget();
    await tester.pumpAndSettle();
    final Finder transformFinder =
        find
            .descendant(of: find.byType(NavigationIndicator), matching: find.byType(Transform))
            .last;
    Matrix4 transform = tester.widget<Transform>(transformFinder).transform;
    expect(transform.getColumn(0)[0], 0.0);

    selectedIndex = 1;
    await buildWidget();
    await tester.pump(const Duration(milliseconds: 100));
    transform = tester.widget<Transform>(transformFinder).transform;
    expect(transform.getColumn(0)[0], closeTo(0.9705023956298828, precisionErrorTolerance));

    await tester.pump(const Duration(milliseconds: 100));
    transform = tester.widget<Transform>(transformFinder).transform;
    expect(transform.getColumn(0)[0], 1.0);
  });

  testWidgets('Navigation destination updates indicator color and shape', (
    WidgetTester tester,
  ) async {
    final ThemeData theme = ThemeData(useMaterial3: true);
    const Color color = Color(0xff0000ff);
    const ShapeBorder shape = RoundedRectangleBorder();

    Widget buildNavigationRail({Color? indicatorColor, ShapeBorder? indicatorShape}) {
      return MaterialApp(
        theme: theme,
        home: Builder(
          builder: (BuildContext context) {
            return Scaffold(
              body: Row(
                children: <Widget>[
                  NavigationRail(
                    useIndicator: true,
                    indicatorColor: indicatorColor,
                    indicatorShape: indicatorShape,
                    selectedIndex: 0,
                    destinations: _destinations(),
                  ),
                  const Expanded(child: Text('body')),
                ],
              ),
            );
          },
        ),
      );
    }

    await tester.pumpWidget(buildNavigationRail());

    // Test default indicator color and shape.
    expect(_getIndicatorDecoration(tester)?.color, theme.colorScheme.secondaryContainer);
    expect(_getIndicatorDecoration(tester)?.shape, const StadiumBorder());

    await tester.pumpWidget(buildNavigationRail(indicatorColor: color, indicatorShape: shape));

    // Test custom indicator color and shape.
    expect(_getIndicatorDecoration(tester)?.color, color);
    expect(_getIndicatorDecoration(tester)?.shape, shape);
  });

  testWidgets("Destination's respect their disabled state", (WidgetTester tester) async {
    late int selectedIndex;
    await _pumpNavigationRail(
      tester,
      navigationRail: NavigationRail(
        selectedIndex: 0,
        destinations: const <NavigationRailDestination>[
          NavigationRailDestination(
            icon: Icon(Icons.favorite_border),
            selectedIcon: Icon(Icons.favorite),
            label: Text('Abc'),
          ),
          NavigationRailDestination(
            icon: Icon(Icons.star_border),
            selectedIcon: Icon(Icons.star),
            label: Text('Bcd'),
          ),
          NavigationRailDestination(
            icon: Icon(Icons.bookmark_border),
            selectedIcon: Icon(Icons.bookmark),
            label: Text('Cde'),
            disabled: true,
          ),
        ],
        onDestinationSelected: (int index) {
          selectedIndex = index;
        },
        labelType: NavigationRailLabelType.all,
      ),
    );

    await tester.tap(find.text('Abc'));
    expect(selectedIndex, 0);

    await tester.tap(find.text('Bcd'));
    expect(selectedIndex, 1);

    await tester.tap(find.text('Cde'));
    expect(selectedIndex, 1);

    // Wait for any pending shader compilation.
    await tester.pumpAndSettle();
  });

  testWidgets("Destination's label with the right opacity while disabled", (
    WidgetTester tester,
  ) async {
    await _pumpNavigationRail(
      tester,
      navigationRail: NavigationRail(
        selectedIndex: 0,
        destinations: const <NavigationRailDestination>[
          NavigationRailDestination(
            icon: Icon(Icons.favorite_border),
            selectedIcon: Icon(Icons.favorite),
            label: Text('Abc'),
          ),
          NavigationRailDestination(
            icon: Icon(Icons.bookmark_border),
            selectedIcon: Icon(Icons.bookmark),
            label: Text('Bcd'),
            disabled: true,
          ),
        ],
        onDestinationSelected: (int index) {},
        labelType: NavigationRailLabelType.all,
      ),
    );

    await tester.pumpAndSettle();

    double? defaultTextStyleOpacity(String text) {
      return tester
          .widget<DefaultTextStyle>(
            find.ancestor(of: find.text(text), matching: find.byType(DefaultTextStyle)).first,
          )
          .style
          .color
          ?.opacity;
    }

    final double? abcLabelOpacity = defaultTextStyleOpacity('Abc');
    final double? bcdLabelOpacity = defaultTextStyleOpacity('Bcd');

    expect(abcLabelOpacity, 1.0);
    expect(bcdLabelOpacity, closeTo(0.38, 0.01));
  });

  testWidgets('NavigationRail indicator inkwell can be transparent', (WidgetTester tester) async {
    // This is a regression test for https://github.com/flutter/flutter/issues/135866.
    final ThemeData theme = ThemeData(
      colorScheme: const ColorScheme.light().copyWith(primary: Colors.transparent),
      // Material 3 defaults to InkSparkle which is not testable using paints.
      splashFactory: InkSplash.splashFactory,
    );
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Builder(
          builder: (BuildContext context) {
            return Scaffold(
              body: Row(
                children: <Widget>[
                  NavigationRail(
                    selectedIndex: 1,
                    destinations: const <NavigationRailDestination>[
                      NavigationRailDestination(
                        icon: Icon(Icons.favorite_border),
                        selectedIcon: Icon(Icons.favorite),
                        label: Text('ABC'),
                      ),
                      NavigationRailDestination(
                        icon: Icon(Icons.bookmark_border),
                        selectedIcon: Icon(Icons.bookmark),
                        label: Text('DEF'),
                      ),
                    ],
                    labelType: NavigationRailLabelType.all,
                  ),
                  const Expanded(child: Text('body')),
                ],
              ),
            );
          },
        ),
      ),
    );

    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(tester.getCenter(find.byIcon(Icons.favorite_border)));
    await tester.pumpAndSettle();
    RenderObject inkFeatures = tester.allRenderObjects.firstWhere(
      (RenderObject object) => object.runtimeType.toString() == '_RenderInkFeatures',
    );

    expect(inkFeatures, paints..rect(color: Colors.transparent));

    await gesture.down(tester.getCenter(find.byIcon(Icons.favorite_border)));
    await tester.pump(); // Start the splash and highlight animations.
    await tester.pump(
      const Duration(milliseconds: 800),
    ); // Wait for splash and highlight to be well under way.

    inkFeatures = tester.allRenderObjects.firstWhere(
      (RenderObject object) => object.runtimeType.toString() == '_RenderInkFeatures',
    );
    expect(inkFeatures, paints..circle(color: Colors.transparent));
  }, skip: kIsWeb && !isSkiaWeb); // https://github.com/flutter/flutter/issues/99933

  testWidgets('Navigation rail can have expanded widgets inside', (WidgetTester tester) async {
    await _pumpNavigationRail(
      tester,
      navigationRail: NavigationRail(
        selectedIndex: 0,
        destinations: const <NavigationRailDestination>[
          NavigationRailDestination(icon: Icon(Icons.favorite_border), label: Text('Abc')),
          NavigationRailDestination(icon: Icon(Icons.bookmark_border), label: Text('Bcd')),
        ],
        trailing: const Expanded(child: Icon(Icons.search)),
      ),
    );

    await tester.pumpAndSettle();

    expect(tester.takeException(), isNull);
  });

  testWidgets('NavigationRail labels shall not overflow if longer texts provided - extended', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/110901.
    // The navigation rail has a narrow width constraint. The text should wrap.
    const String normalLabel = 'Abc';
    const String longLabel = 'Very long bookmark text for navigation destination';
    await tester.pumpWidget(
      MaterialApp(
        home: Builder(
          builder: (BuildContext context) {
            return Scaffold(
              body: Row(
                children: <Widget>[
                  SizedBox(
                    width: 140.0,
                    child: NavigationRail(
                      selectedIndex: 1,
                      extended: true,
                      destinations: const <NavigationRailDestination>[
                        NavigationRailDestination(
                          icon: Icon(Icons.favorite_border),
                          selectedIcon: Icon(Icons.favorite),
                          label: Text(normalLabel),
                        ),
                        NavigationRailDestination(
                          icon: Icon(Icons.bookmark_border),
                          selectedIcon: Icon(Icons.bookmark),
                          label: Text(longLabel),
                        ),
                      ],
                    ),
                  ),
                  const Expanded(child: Text('body')),
                ],
              ),
            );
          },
        ),
      ),
    );

    expect(find.byType(NavigationRail), findsOneWidget);
    expect(find.text(normalLabel), findsOneWidget);
    expect(find.text(longLabel), findsOneWidget);

    // If the widget manages to layout without throwing an overflow exception,
    // the test passes.
    expect(tester.takeException(), isNull);
  });

  group('Material 2', () {
    // These tests are only relevant for Material 2. Once Material 2
    // support is deprecated and the APIs are removed, these tests
    // can be deleted.

    testWidgets('Renders at the correct default width - [labelType]=none (default)', (
      WidgetTester tester,
    ) async {
      await _pumpNavigationRail(
        tester,
        useMaterial3: false,
        navigationRail: NavigationRail(selectedIndex: 0, destinations: _destinations()),
      );

      final RenderBox renderBox = tester.renderObject(find.byType(NavigationRail));
      expect(renderBox.size.width, 72.0);
    });

    testWidgets('Renders at the correct default width - [labelType]=selected', (
      WidgetTester tester,
    ) async {
      await _pumpNavigationRail(
        tester,
        useMaterial3: false,
        navigationRail: NavigationRail(
          selectedIndex: 0,
          labelType: NavigationRailLabelType.selected,
          destinations: _destinations(),
        ),
      );

      final RenderBox renderBox = tester.renderObject(find.byType(NavigationRail));
      expect(renderBox.size.width, 72.0);
    });

    testWidgets('Renders at the correct default width - [labelType]=all', (
      WidgetTester tester,
    ) async {
      await _pumpNavigationRail(
        tester,
        useMaterial3: false,
        navigationRail: NavigationRail(
          selectedIndex: 0,
          labelType: NavigationRailLabelType.all,
          destinations: _destinations(),
        ),
      );

      final RenderBox renderBox = tester.renderObject(find.byType(NavigationRail));
      expect(renderBox.size.width, 72.0);
    });

    testWidgets('Leading and trailing spacing is correct with 0~2 destinations', (
      WidgetTester tester,
    ) async {
      late StateSetter stateSetter;
      List<NavigationRailDestination> destinations = const <NavigationRailDestination>[];
      Widget? leadingWidget;
      Widget? trailingWidget;

      const Key leadingWidgetKey = Key('leadingWidget');
      const Key trailingWidgetKey = Key('trailingWidget');

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(useMaterial3: false),
          home: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              stateSetter = setState;
              return Scaffold(
                body: Row(
                  children: <Widget>[
                    NavigationRail(
                      destinations: destinations,
                      selectedIndex: null,
                      leading: leadingWidget,
                      trailing: trailingWidget,
                    ),
                    const Expanded(child: Text('body')),
                  ],
                ),
              );
            },
          ),
        ),
      );

      // empty destinations and leading widget
      stateSetter(() {
        destinations = const <NavigationRailDestination>[];
        leadingWidget = FloatingActionButton(key: leadingWidgetKey, onPressed: () {});
        trailingWidget = null;
      });
      await tester.pumpAndSettle();
      expect(
        tester.renderObject<RenderBox>(find.byKey(leadingWidgetKey)).localToGlobal(Offset.zero),
        const Offset(0, 8.0),
      );

      // one destination and leading widget
      stateSetter(() {
        destinations = const <NavigationRailDestination>[
          NavigationRailDestination(
            icon: Icon(Icons.favorite_border),
            selectedIcon: Icon(Icons.favorite),
            label: Text('Abc'),
          ),
        ];
      });
      await tester.pumpAndSettle();
      expect(
        _iconRenderBox(tester, Icons.favorite_border).localToGlobal(Offset.zero),
        const Offset(24.0, 96.0),
      );
      expect(_labelRenderBox(tester, 'Abc').localToGlobal(Offset.zero), const Offset(0.0, 72.0));
      expect(
        tester.renderObject<RenderBox>(find.byKey(leadingWidgetKey)).localToGlobal(Offset.zero),
        const Offset(8.0, 8.0),
      );

      // two destinations and leading widget
      stateSetter(() {
        destinations = const <NavigationRailDestination>[
          NavigationRailDestination(
            icon: Icon(Icons.favorite_border),
            selectedIcon: Icon(Icons.favorite),
            label: Text('Abc'),
          ),
          NavigationRailDestination(
            icon: Icon(Icons.bookmark_border),
            selectedIcon: Icon(Icons.bookmark),
            label: Text('Longer Label'),
          ),
        ];
      });
      await tester.pumpAndSettle();
      expect(
        _iconRenderBox(tester, Icons.favorite_border).localToGlobal(Offset.zero),
        const Offset(24.0, 96.0),
      );
      expect(_labelRenderBox(tester, 'Abc').localToGlobal(Offset.zero), const Offset(0.0, 72.0));
      expect(
        _iconRenderBox(tester, Icons.bookmark_border).localToGlobal(Offset.zero),
        const Offset(24.0, 168.0),
      );
      expect(
        _labelRenderBox(tester, 'Longer Label').localToGlobal(Offset.zero),
        const Offset(0.0, 144.0),
      );
      expect(
        tester.renderObject<RenderBox>(find.byKey(leadingWidgetKey)).localToGlobal(Offset.zero),
        const Offset(8.0, 8.0),
      );

      // empty destinations and trailing widget
      stateSetter(() {
        destinations = const <NavigationRailDestination>[];
        leadingWidget = null;
        trailingWidget = FloatingActionButton(key: trailingWidgetKey, onPressed: () {});
      });
      await tester.pumpAndSettle();
      expect(
        tester.renderObject<RenderBox>(find.byKey(trailingWidgetKey)).localToGlobal(Offset.zero),
        const Offset(0, 8.0),
      );

      // one destination and trailing widget
      stateSetter(() {
        destinations = const <NavigationRailDestination>[
          NavigationRailDestination(
            icon: Icon(Icons.favorite_border),
            selectedIcon: Icon(Icons.favorite),
            label: Text('Abc'),
          ),
        ];
      });
      await tester.pumpAndSettle();
      expect(
        _iconRenderBox(tester, Icons.favorite_border).localToGlobal(Offset.zero),
        const Offset(24.0, 32.0),
      );
      expect(_labelRenderBox(tester, 'Abc').localToGlobal(Offset.zero), const Offset(0.0, 8.0));
      expect(
        tester.renderObject<RenderBox>(find.byKey(trailingWidgetKey)).localToGlobal(Offset.zero),
        const Offset(8.0, 80.0),
      );

      // two destinations and trailing widget
      stateSetter(() {
        destinations = const <NavigationRailDestination>[
          NavigationRailDestination(
            icon: Icon(Icons.favorite_border),
            selectedIcon: Icon(Icons.favorite),
            label: Text('Abc'),
          ),
          NavigationRailDestination(
            icon: Icon(Icons.bookmark_border),
            selectedIcon: Icon(Icons.bookmark),
            label: Text('Longer Label'),
          ),
        ];
      });
      await tester.pumpAndSettle();
      expect(
        _iconRenderBox(tester, Icons.favorite_border).localToGlobal(Offset.zero),
        const Offset(24.0, 32.0),
      );
      expect(_labelRenderBox(tester, 'Abc').localToGlobal(Offset.zero), const Offset(0.0, 8.0));
      expect(
        _iconRenderBox(tester, Icons.bookmark_border).localToGlobal(Offset.zero),
        const Offset(24.0, 104.0),
      );
      expect(
        _labelRenderBox(tester, 'Longer Label').localToGlobal(Offset.zero),
        const Offset(0.0, 80.0),
      );
      expect(
        tester.renderObject<RenderBox>(find.byKey(trailingWidgetKey)).localToGlobal(Offset.zero),
        const Offset(8.0, 152.0),
      );
    });

    testWidgets('Change destinations and selectedIndex', (WidgetTester tester) async {
      late StateSetter stateSetter;
      int? selectedIndex;
      List<NavigationRailDestination> destinations = const <NavigationRailDestination>[];

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(useMaterial3: false),
          home: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              stateSetter = setState;
              return Scaffold(
                body: Row(
                  children: <Widget>[
                    NavigationRail(selectedIndex: selectedIndex, destinations: destinations),
                    const Expanded(child: Text('body')),
                  ],
                ),
              );
            },
          ),
        ),
      );

      expect(tester.widget<NavigationRail>(find.byType(NavigationRail)).destinations.length, 0);
      expect(tester.widget<NavigationRail>(find.byType(NavigationRail)).selectedIndex, isNull);

      stateSetter(() {
        destinations = const <NavigationRailDestination>[
          NavigationRailDestination(
            icon: Icon(Icons.favorite_border),
            selectedIcon: Icon(Icons.favorite),
            label: Text('Abc'),
          ),
          NavigationRailDestination(
            icon: Icon(Icons.bookmark_border),
            selectedIcon: Icon(Icons.bookmark),
            label: Text('Longer Label'),
          ),
        ];
      });

      await tester.pumpAndSettle();

      expect(tester.widget<NavigationRail>(find.byType(NavigationRail)).destinations.length, 2);
      expect(tester.widget<NavigationRail>(find.byType(NavigationRail)).selectedIndex, isNull);

      stateSetter(() {
        selectedIndex = 0;
      });

      await tester.pumpAndSettle();

      expect(tester.widget<NavigationRail>(find.byType(NavigationRail)).destinations.length, 2);
      expect(tester.widget<NavigationRail>(find.byType(NavigationRail)).selectedIndex, 0);

      stateSetter(() {
        destinations = const <NavigationRailDestination>[
          NavigationRailDestination(
            icon: Icon(Icons.favorite_border),
            selectedIcon: Icon(Icons.favorite),
            label: Text('Abc'),
          ),
        ];
      });

      await tester.pumpAndSettle();

      expect(tester.widget<NavigationRail>(find.byType(NavigationRail)).destinations.length, 1);
      expect(tester.widget<NavigationRail>(find.byType(NavigationRail)).selectedIndex, 0);
    });

    testWidgets(
      'Destination spacing is correct - [labelType]=none (default), [textScaleFactor]=1.0 (default)',
      (WidgetTester tester) async {
        await _pumpNavigationRail(
          tester,
          useMaterial3: false,
          navigationRail: NavigationRail(selectedIndex: 0, destinations: _destinations()),
        );

        final RenderBox renderBox = tester.renderObject(find.byType(NavigationRail));
        expect(renderBox.size.width, 72.0);

        // The first destination is 8 from the top because of the default vertical
        // padding at the to of the rail.
        double nextDestinationY = 8.0;
        final RenderBox firstIconRenderBox = _iconRenderBox(tester, Icons.favorite);
        expect(
          firstIconRenderBox.localToGlobal(Offset.zero),
          equals(
            Offset(
              (72.0 - firstIconRenderBox.size.width) / 2.0,
              nextDestinationY + (72.0 - firstIconRenderBox.size.height) / 2.0,
            ),
          ),
        );

        // The second destination is 72 below the first destination.
        nextDestinationY += 72.0;
        final RenderBox secondIconRenderBox = _iconRenderBox(tester, Icons.bookmark_border);
        expect(
          secondIconRenderBox.localToGlobal(Offset.zero),
          equals(
            Offset(
              (72.0 - secondIconRenderBox.size.width) / 2.0,
              nextDestinationY + (72.0 - secondIconRenderBox.size.height) / 2.0,
            ),
          ),
        );

        // The third destination is 72 below the second destination.
        nextDestinationY += 72.0;
        final RenderBox thirdIconRenderBox = _iconRenderBox(tester, Icons.star_border);
        expect(
          thirdIconRenderBox.localToGlobal(Offset.zero),
          equals(
            Offset(
              (72.0 - thirdIconRenderBox.size.width) / 2.0,
              nextDestinationY + (72.0 - thirdIconRenderBox.size.height) / 2.0,
            ),
          ),
        );

        // The fourth destination is 72 below the third destination.
        nextDestinationY += 72.0;
        final RenderBox fourthIconRenderBox = _iconRenderBox(tester, Icons.hotel);
        expect(
          fourthIconRenderBox.localToGlobal(Offset.zero),
          equals(
            Offset(
              (72.0 - fourthIconRenderBox.size.width) / 2.0,
              nextDestinationY + (72.0 - fourthIconRenderBox.size.height) / 2.0,
            ),
          ),
        );
      },
      skip: kIsWeb && !isSkiaWeb, // https://github.com/flutter/flutter/issues/99933
    );

    testWidgets(
      'Destination spacing is correct - [labelType]=none (default), [textScaleFactor]=3.0',
      (WidgetTester tester) async {
        // Since the rail is icon only, its destinations should not be affected by
        // textScaleFactor.
        await _pumpNavigationRail(
          tester,
          useMaterial3: false,
          textScaleFactor: 3.0,
          navigationRail: NavigationRail(selectedIndex: 0, destinations: _destinations()),
        );

        final RenderBox renderBox = tester.renderObject(find.byType(NavigationRail));
        expect(renderBox.size.width, 72.0);

        // The first destination is 8 from the top because of the default vertical
        // padding at the to of the rail.
        double nextDestinationY = 8.0;
        final RenderBox firstIconRenderBox = _iconRenderBox(tester, Icons.favorite);
        expect(
          firstIconRenderBox.localToGlobal(Offset.zero),
          equals(
            Offset(
              (72.0 - firstIconRenderBox.size.width) / 2.0,
              nextDestinationY + (72.0 - firstIconRenderBox.size.height) / 2.0,
            ),
          ),
        );

        // The second destination is 72 below the first destination.
        nextDestinationY += 72.0;
        final RenderBox secondIconRenderBox = _iconRenderBox(tester, Icons.bookmark_border);
        expect(
          secondIconRenderBox.localToGlobal(Offset.zero),
          equals(
            Offset(
              (72.0 - secondIconRenderBox.size.width) / 2.0,
              nextDestinationY + (72.0 - secondIconRenderBox.size.height) / 2.0,
            ),
          ),
        );

        // The third destination is 72 below the second destination.
        nextDestinationY += 72.0;
        final RenderBox thirdIconRenderBox = _iconRenderBox(tester, Icons.star_border);
        expect(
          thirdIconRenderBox.localToGlobal(Offset.zero),
          equals(
            Offset(
              (72.0 - thirdIconRenderBox.size.width) / 2.0,
              nextDestinationY + (72.0 - thirdIconRenderBox.size.height) / 2.0,
            ),
          ),
        );

        // The fourth destination is 72 below the third destination.
        nextDestinationY += 72.0;
        final RenderBox fourthIconRenderBox = _iconRenderBox(tester, Icons.hotel);
        expect(
          fourthIconRenderBox.localToGlobal(Offset.zero),
          equals(
            Offset(
              (72.0 - fourthIconRenderBox.size.width) / 2.0,
              nextDestinationY + (72.0 - fourthIconRenderBox.size.height) / 2.0,
            ),
          ),
        );
      },
      skip: kIsWeb && !isSkiaWeb, // https://github.com/flutter/flutter/issues/99933
    );

    testWidgets(
      'Destination spacing is correct - [labelType]=none (default), [textScaleFactor]=0.75',
      (WidgetTester tester) async {
        // Since the rail is icon only, its destinations should not be affected by
        // textScaleFactor.
        await _pumpNavigationRail(
          tester,
          useMaterial3: false,
          textScaleFactor: 0.75,
          navigationRail: NavigationRail(selectedIndex: 0, destinations: _destinations()),
        );

        final RenderBox renderBox = tester.renderObject(find.byType(NavigationRail));
        expect(renderBox.size.width, 72.0);

        // The first destination is 8 from the top because of the default vertical
        // padding at the to of the rail.
        double nextDestinationY = 8.0;
        final RenderBox firstIconRenderBox = _iconRenderBox(tester, Icons.favorite);
        expect(
          firstIconRenderBox.localToGlobal(Offset.zero),
          equals(
            Offset(
              (72.0 - firstIconRenderBox.size.width) / 2.0,
              nextDestinationY + (72.0 - firstIconRenderBox.size.height) / 2.0,
            ),
          ),
        );

        // The second destination is 72 below the first destination.
        nextDestinationY += 72.0;
        final RenderBox secondIconRenderBox = _iconRenderBox(tester, Icons.bookmark_border);
        expect(
          secondIconRenderBox.localToGlobal(Offset.zero),
          equals(
            Offset(
              (72.0 - secondIconRenderBox.size.width) / 2.0,
              nextDestinationY + (72.0 - secondIconRenderBox.size.height) / 2.0,
            ),
          ),
        );

        // The third destination is 72 below the second destination.
        nextDestinationY += 72.0;
        final RenderBox thirdIconRenderBox = _iconRenderBox(tester, Icons.star_border);
        expect(
          thirdIconRenderBox.localToGlobal(Offset.zero),
          equals(
            Offset(
              (72.0 - thirdIconRenderBox.size.width) / 2.0,
              nextDestinationY + (72.0 - thirdIconRenderBox.size.height) / 2.0,
            ),
          ),
        );

        // The fourth destination is 72 below the third destination.
        nextDestinationY += 72.0;
        final RenderBox fourthIconRenderBox = _iconRenderBox(tester, Icons.hotel);
        expect(
          fourthIconRenderBox.localToGlobal(Offset.zero),
          equals(
            Offset(
              (72.0 - fourthIconRenderBox.size.width) / 2.0,
              nextDestinationY + (72.0 - fourthIconRenderBox.size.height) / 2.0,
            ),
          ),
        );
      },
      skip: kIsWeb && !isSkiaWeb, // https://github.com/flutter/flutter/issues/99933
    );

    testWidgets(
      'Destination spacing is correct - [labelType]=selected, [textScaleFactor]=1.0 (default)',
      (WidgetTester tester) async {
        await _pumpNavigationRail(
          tester,
          useMaterial3: false,
          navigationRail: NavigationRail(
            selectedIndex: 0,
            destinations: _destinations(),
            labelType: NavigationRailLabelType.selected,
          ),
        );

        final RenderBox renderBox = tester.renderObject(find.byType(NavigationRail));
        expect(renderBox.size.width, 72.0);

        // The first destination is 8 from the top because of the default vertical
        // padding at the to of the rail.
        double nextDestinationY = 8.0;
        final RenderBox firstIconRenderBox = _iconRenderBox(tester, Icons.favorite);
        final RenderBox firstLabelRenderBox = _labelRenderBox(tester, 'Abc');
        expect(
          firstIconRenderBox.localToGlobal(Offset.zero),
          equals(
            Offset(
              (72.0 - firstIconRenderBox.size.width) / 2.0,
              nextDestinationY +
                  (72.0 - firstIconRenderBox.size.height - firstLabelRenderBox.size.height) / 2.0,
            ),
          ),
        );
        expect(
          firstLabelRenderBox.localToGlobal(Offset.zero),
          equals(
            Offset(
              (72.0 - firstLabelRenderBox.size.width) / 2.0,
              nextDestinationY +
                  (72.0 + firstIconRenderBox.size.height - firstLabelRenderBox.size.height) / 2.0,
            ),
          ),
        );

        // The second destination is 72 below the first destination.
        nextDestinationY += 72.0;
        final RenderBox secondIconRenderBox = _iconRenderBox(tester, Icons.bookmark_border);
        expect(
          secondIconRenderBox.localToGlobal(Offset.zero),
          equals(
            Offset(
              (72.0 - secondIconRenderBox.size.width) / 2.0,
              nextDestinationY + (72.0 - secondIconRenderBox.size.height) / 2.0,
            ),
          ),
        );

        // The third destination is 72 below the second destination.
        nextDestinationY += 72.0;
        final RenderBox thirdIconRenderBox = _iconRenderBox(tester, Icons.star_border);
        expect(
          thirdIconRenderBox.localToGlobal(Offset.zero),
          equals(
            Offset(
              (72.0 - thirdIconRenderBox.size.width) / 2.0,
              nextDestinationY + (72.0 - thirdIconRenderBox.size.height) / 2.0,
            ),
          ),
        );

        // The fourth destination is 72 below the third destination.
        nextDestinationY += 72.0;
        final RenderBox fourthIconRenderBox = _iconRenderBox(tester, Icons.hotel);
        expect(
          fourthIconRenderBox.localToGlobal(Offset.zero),
          equals(
            Offset(
              (72.0 - fourthIconRenderBox.size.width) / 2.0,
              nextDestinationY + (72.0 - fourthIconRenderBox.size.height) / 2.0,
            ),
          ),
        );
      },
    );

    testWidgets('Destination spacing is correct - [labelType]=selected, [textScaleFactor]=3.0', (
      WidgetTester tester,
    ) async {
      await _pumpNavigationRail(
        tester,
        useMaterial3: false,
        textScaleFactor: 3.0,
        navigationRail: NavigationRail(
          selectedIndex: 0,
          destinations: _destinations(),
          labelType: NavigationRailLabelType.selected,
        ),
      );

      // The rail and destinations sizes grow to fit the larger text labels.
      final RenderBox renderBox = tester.renderObject(find.byType(NavigationRail));
      expect(renderBox.size.width, 142.0);

      // The first destination is 8 from the top because of the default vertical
      // padding at the to of the rail.
      double nextDestinationY = 8.0;
      final RenderBox firstIconRenderBox = _iconRenderBox(tester, Icons.favorite);
      expect(
        firstIconRenderBox.localToGlobal(Offset.zero),
        equals(Offset((142.0 - firstIconRenderBox.size.width) / 2.0, nextDestinationY + 16.0)),
      );

      // The first label sits right below the first icon.
      final RenderBox firstLabelRenderBox = _labelRenderBox(tester, 'Abc');
      expect(
        firstLabelRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (142.0 - firstLabelRenderBox.size.width) / 2.0,
            nextDestinationY + 16.0 + firstIconRenderBox.size.height,
          ),
        ),
      );

      nextDestinationY +=
          16.0 + firstIconRenderBox.size.height + firstLabelRenderBox.size.height + 16.0;
      final RenderBox secondIconRenderBox = _iconRenderBox(tester, Icons.bookmark_border);
      expect(
        secondIconRenderBox.localToGlobal(Offset.zero),
        equals(Offset((142.0 - secondIconRenderBox.size.width) / 2.0, nextDestinationY + 24.0)),
      );

      nextDestinationY += 72.0;
      final RenderBox thirdIconRenderBox = _iconRenderBox(tester, Icons.star_border);
      expect(
        thirdIconRenderBox.localToGlobal(Offset.zero),
        equals(Offset((142.0 - thirdIconRenderBox.size.width) / 2.0, nextDestinationY + 24.0)),
      );

      nextDestinationY += 72.0;
      final RenderBox fourthIconRenderBox = _iconRenderBox(tester, Icons.hotel);
      expect(
        fourthIconRenderBox.localToGlobal(Offset.zero),
        equals(Offset((142.0 - fourthIconRenderBox.size.width) / 2.0, nextDestinationY + 24.0)),
      );
    });

    testWidgets('Destination spacing is correct - [labelType]=selected, [textScaleFactor]=0.75', (
      WidgetTester tester,
    ) async {
      await _pumpNavigationRail(
        tester,
        useMaterial3: false,
        textScaleFactor: 0.75,
        navigationRail: NavigationRail(
          selectedIndex: 0,
          destinations: _destinations(),
          labelType: NavigationRailLabelType.selected,
        ),
      );

      // A smaller textScaleFactor will not reduce the default width of the rail
      // since there is a minWidth.
      final RenderBox renderBox = tester.renderObject(find.byType(NavigationRail));
      expect(renderBox.size.width, 72.0);

      // The first destination is 8 from the top because of the default vertical
      // padding at the to of the rail.
      double nextDestinationY = 8.0;
      final RenderBox firstIconRenderBox = _iconRenderBox(tester, Icons.favorite);
      final RenderBox firstLabelRenderBox = _labelRenderBox(tester, 'Abc');
      expect(
        firstIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (72.0 - firstIconRenderBox.size.width) / 2.0,
            nextDestinationY +
                (72.0 - firstIconRenderBox.size.height - firstLabelRenderBox.size.height) / 2.0,
          ),
        ),
      );
      expect(
        firstLabelRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (72.0 - firstLabelRenderBox.size.width) / 2.0,
            nextDestinationY +
                (72.0 + firstIconRenderBox.size.height - firstLabelRenderBox.size.height) / 2.0,
          ),
        ),
      );

      // The second destination is 72 below the first destination.
      nextDestinationY += 72.0;
      final RenderBox secondIconRenderBox = _iconRenderBox(tester, Icons.bookmark_border);
      expect(
        secondIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (72.0 - secondIconRenderBox.size.width) / 2.0,
            nextDestinationY + (72.0 - secondIconRenderBox.size.height) / 2.0,
          ),
        ),
      );

      // The third destination is 72 below the second destination.
      nextDestinationY += 72.0;
      final RenderBox thirdIconRenderBox = _iconRenderBox(tester, Icons.star_border);
      expect(
        thirdIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (72.0 - thirdIconRenderBox.size.width) / 2.0,
            nextDestinationY + (72.0 - thirdIconRenderBox.size.height) / 2.0,
          ),
        ),
      );

      // The fourth destination is 72 below the third destination.
      nextDestinationY += 72.0;
      final RenderBox fourthIconRenderBox = _iconRenderBox(tester, Icons.hotel);
      expect(
        fourthIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (72.0 - fourthIconRenderBox.size.width) / 2.0,
            nextDestinationY + (72.0 - fourthIconRenderBox.size.height) / 2.0,
          ),
        ),
      );
    });

    testWidgets(
      'Destination spacing is correct - [labelType]=all, [textScaleFactor]=1.0 (default)',
      (WidgetTester tester) async {
        await _pumpNavigationRail(
          tester,
          useMaterial3: false,
          navigationRail: NavigationRail(
            selectedIndex: 0,
            destinations: _destinations(),
            labelType: NavigationRailLabelType.all,
          ),
        );

        final RenderBox renderBox = tester.renderObject(find.byType(NavigationRail));
        expect(renderBox.size.width, 72.0);

        // The first destination is 8 from the top because of the default vertical
        // padding at the to of the rail.
        double nextDestinationY = 8.0;
        final RenderBox firstIconRenderBox = _iconRenderBox(tester, Icons.favorite);
        final RenderBox firstLabelRenderBox = _labelRenderBox(tester, 'Abc');
        expect(
          firstIconRenderBox.localToGlobal(Offset.zero),
          equals(Offset((72.0 - firstIconRenderBox.size.width) / 2.0, nextDestinationY + 16.0)),
        );
        expect(
          firstLabelRenderBox.localToGlobal(Offset.zero),
          equals(
            Offset(
              (72.0 - firstLabelRenderBox.size.width) / 2.0,
              nextDestinationY + 16.0 + firstIconRenderBox.size.height,
            ),
          ),
        );

        // The second destination is 72 below the first destination.
        nextDestinationY += 72.0;
        final RenderBox secondIconRenderBox = _iconRenderBox(tester, Icons.bookmark_border);
        final RenderBox secondLabelRenderBox = _labelRenderBox(tester, 'Def');
        expect(
          secondIconRenderBox.localToGlobal(Offset.zero),
          equals(Offset((72.0 - secondIconRenderBox.size.width) / 2.0, nextDestinationY + 16.0)),
        );
        expect(
          secondLabelRenderBox.localToGlobal(Offset.zero),
          equals(
            Offset(
              (72.0 - secondLabelRenderBox.size.width) / 2.0,
              nextDestinationY + 16.0 + secondIconRenderBox.size.height,
            ),
          ),
        );

        // The third destination is 72 below the second destination.
        nextDestinationY += 72.0;
        final RenderBox thirdIconRenderBox = _iconRenderBox(tester, Icons.star_border);
        final RenderBox thirdLabelRenderBox = _labelRenderBox(tester, 'Ghi');
        expect(
          thirdIconRenderBox.localToGlobal(Offset.zero),
          equals(Offset((72.0 - thirdIconRenderBox.size.width) / 2.0, nextDestinationY + 16.0)),
        );
        expect(
          thirdLabelRenderBox.localToGlobal(Offset.zero),
          equals(
            Offset(
              (72.0 - thirdLabelRenderBox.size.width) / 2.0,
              nextDestinationY + 16.0 + thirdIconRenderBox.size.height,
            ),
          ),
        );

        // The fourth destination is 72 below the third destination.
        nextDestinationY += 72.0;
        final RenderBox fourthIconRenderBox = _iconRenderBox(tester, Icons.hotel);
        final RenderBox fourthLabelRenderBox = _labelRenderBox(tester, 'Jkl');
        expect(
          fourthIconRenderBox.localToGlobal(Offset.zero),
          equals(Offset((72.0 - fourthIconRenderBox.size.width) / 2.0, nextDestinationY + 16.0)),
        );
        expect(
          fourthLabelRenderBox.localToGlobal(Offset.zero),
          equals(
            Offset(
              (72.0 - fourthLabelRenderBox.size.width) / 2.0,
              nextDestinationY + 16.0 + fourthIconRenderBox.size.height,
            ),
          ),
        );
      },
    );

    testWidgets('Destination spacing is correct - [labelType]=all, [textScaleFactor]=3.0', (
      WidgetTester tester,
    ) async {
      await _pumpNavigationRail(
        tester,
        useMaterial3: false,
        textScaleFactor: 3.0,
        navigationRail: NavigationRail(
          selectedIndex: 0,
          destinations: _destinations(),
          labelType: NavigationRailLabelType.all,
        ),
      );

      // The rail and destinations sizes grow to fit the larger text labels.
      final RenderBox renderBox = tester.renderObject(find.byType(NavigationRail));
      expect(renderBox.size.width, 142.0);

      // The first destination is 8 from the top because of the default vertical
      // padding at the to of the rail.
      double nextDestinationY = 8.0;
      final RenderBox firstIconRenderBox = _iconRenderBox(tester, Icons.favorite);
      final RenderBox firstLabelRenderBox = _labelRenderBox(tester, 'Abc');
      expect(
        firstIconRenderBox.localToGlobal(Offset.zero),
        equals(Offset((142.0 - firstIconRenderBox.size.width) / 2.0, nextDestinationY + 16.0)),
      );
      expect(
        firstLabelRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (142.0 - firstLabelRenderBox.size.width) / 2.0,
            nextDestinationY + 16.0 + firstIconRenderBox.size.height,
          ),
        ),
      );

      nextDestinationY +=
          16.0 + firstIconRenderBox.size.height + firstLabelRenderBox.size.height + 16.0;
      final RenderBox secondIconRenderBox = _iconRenderBox(tester, Icons.bookmark_border);
      final RenderBox secondLabelRenderBox = _labelRenderBox(tester, 'Def');
      expect(
        secondIconRenderBox.localToGlobal(Offset.zero),
        equals(Offset((142.0 - secondIconRenderBox.size.width) / 2.0, nextDestinationY + 16.0)),
      );
      expect(
        secondLabelRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (142.0 - secondLabelRenderBox.size.width) / 2.0,
            nextDestinationY + 16.0 + secondIconRenderBox.size.height,
          ),
        ),
      );

      nextDestinationY +=
          16.0 + secondIconRenderBox.size.height + secondLabelRenderBox.size.height + 16.0;
      final RenderBox thirdIconRenderBox = _iconRenderBox(tester, Icons.star_border);
      final RenderBox thirdLabelRenderBox = _labelRenderBox(tester, 'Ghi');
      expect(
        thirdIconRenderBox.localToGlobal(Offset.zero),
        equals(Offset((142.0 - thirdIconRenderBox.size.width) / 2.0, nextDestinationY + 16.0)),
      );
      expect(
        thirdLabelRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (142.0 - thirdLabelRenderBox.size.width) / 2.0,
            nextDestinationY + 16.0 + thirdIconRenderBox.size.height,
          ),
        ),
      );

      nextDestinationY +=
          16.0 + thirdIconRenderBox.size.height + thirdLabelRenderBox.size.height + 16.0;
      final RenderBox fourthIconRenderBox = _iconRenderBox(tester, Icons.hotel);
      final RenderBox fourthLabelRenderBox = _labelRenderBox(tester, 'Jkl');
      expect(
        fourthIconRenderBox.localToGlobal(Offset.zero),
        equals(Offset((142.0 - fourthIconRenderBox.size.width) / 2.0, nextDestinationY + 16.0)),
      );
      expect(
        fourthLabelRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (142.0 - fourthLabelRenderBox.size.width) / 2.0,
            nextDestinationY + 16.0 + fourthIconRenderBox.size.height,
          ),
        ),
      );
    });

    testWidgets('Destination spacing is correct - [labelType]=all, [textScaleFactor]=0.75', (
      WidgetTester tester,
    ) async {
      await _pumpNavigationRail(
        tester,
        useMaterial3: false,
        textScaleFactor: 0.75,
        navigationRail: NavigationRail(
          selectedIndex: 0,
          destinations: _destinations(),
          labelType: NavigationRailLabelType.all,
        ),
      );

      // A smaller textScaleFactor will not reduce the default size of the rail.
      final RenderBox renderBox = tester.renderObject(find.byType(NavigationRail));
      expect(renderBox.size.width, 72.0);

      // The first destination is 8 from the top because of the default vertical
      // padding at the to of the rail.
      double nextDestinationY = 8.0;
      final RenderBox firstIconRenderBox = _iconRenderBox(tester, Icons.favorite);
      final RenderBox firstLabelRenderBox = _labelRenderBox(tester, 'Abc');
      expect(
        firstIconRenderBox.localToGlobal(Offset.zero),
        equals(Offset((72.0 - firstIconRenderBox.size.width) / 2.0, nextDestinationY + 16.0)),
      );
      expect(
        firstLabelRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (72.0 - firstLabelRenderBox.size.width) / 2.0,
            nextDestinationY + 16.0 + firstIconRenderBox.size.height,
          ),
        ),
      );

      // The second destination is 72 below the first destination.
      nextDestinationY += 72.0;
      final RenderBox secondIconRenderBox = _iconRenderBox(tester, Icons.bookmark_border);
      final RenderBox secondLabelRenderBox = _labelRenderBox(tester, 'Def');
      expect(
        secondIconRenderBox.localToGlobal(Offset.zero),
        equals(Offset((72.0 - secondIconRenderBox.size.width) / 2.0, nextDestinationY + 16.0)),
      );
      expect(
        secondLabelRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (72.0 - secondLabelRenderBox.size.width) / 2.0,
            nextDestinationY + 16.0 + secondIconRenderBox.size.height,
          ),
        ),
      );

      // The third destination is 72 below the second destination.
      nextDestinationY += 72.0;
      final RenderBox thirdIconRenderBox = _iconRenderBox(tester, Icons.star_border);
      final RenderBox thirdLabelRenderBox = _labelRenderBox(tester, 'Ghi');
      expect(
        thirdIconRenderBox.localToGlobal(Offset.zero),
        equals(Offset((72.0 - thirdIconRenderBox.size.width) / 2.0, nextDestinationY + 16.0)),
      );
      expect(
        thirdLabelRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (72.0 - thirdLabelRenderBox.size.width) / 2.0,
            nextDestinationY + 16.0 + thirdIconRenderBox.size.height,
          ),
        ),
      );

      // The fourth destination is 72 below the third destination.
      nextDestinationY += 72.0;
      final RenderBox fourthIconRenderBox = _iconRenderBox(tester, Icons.hotel);
      final RenderBox fourthLabelRenderBox = _labelRenderBox(tester, 'Jkl');
      expect(
        fourthIconRenderBox.localToGlobal(Offset.zero),
        equals(Offset((72.0 - fourthIconRenderBox.size.width) / 2.0, nextDestinationY + 16.0)),
      );
      expect(
        fourthLabelRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (72.0 - fourthLabelRenderBox.size.width) / 2.0,
            nextDestinationY + 16.0 + fourthIconRenderBox.size.height,
          ),
        ),
      );
    });

    testWidgets(
      'Destination spacing is correct for a compact rail - [preferredWidth]=56, [textScaleFactor]=1.0 (default)',
      (WidgetTester tester) async {
        await _pumpNavigationRail(
          tester,
          useMaterial3: false,
          navigationRail: NavigationRail(
            selectedIndex: 0,
            minWidth: 56.0,
            destinations: _destinations(),
          ),
        );

        final RenderBox renderBox = tester.renderObject(find.byType(NavigationRail));
        expect(renderBox.size.width, 56.0);

        // The first destination is 8 from the top because of the default vertical
        // padding at the to of the rail.
        double nextDestinationY = 8.0;
        final RenderBox firstIconRenderBox = _iconRenderBox(tester, Icons.favorite);
        expect(
          firstIconRenderBox.localToGlobal(Offset.zero),
          equals(
            Offset(
              (56.0 - firstIconRenderBox.size.width) / 2.0,
              nextDestinationY + (56.0 - firstIconRenderBox.size.height) / 2.0,
            ),
          ),
        );

        // The second destination is 56 below the first destination.
        nextDestinationY += 56.0;
        final RenderBox secondIconRenderBox = _iconRenderBox(tester, Icons.bookmark_border);
        expect(
          secondIconRenderBox.localToGlobal(Offset.zero),
          equals(
            Offset(
              (56.0 - secondIconRenderBox.size.width) / 2.0,
              nextDestinationY + (56.0 - secondIconRenderBox.size.height) / 2.0,
            ),
          ),
        );

        // The third destination is 56 below the second destination.
        nextDestinationY += 56.0;
        final RenderBox thirdIconRenderBox = _iconRenderBox(tester, Icons.star_border);
        expect(
          thirdIconRenderBox.localToGlobal(Offset.zero),
          equals(
            Offset(
              (56.0 - thirdIconRenderBox.size.width) / 2.0,
              nextDestinationY + (56.0 - thirdIconRenderBox.size.height) / 2.0,
            ),
          ),
        );

        // The fourth destination is 56 below the third destination.
        nextDestinationY += 56.0;
        final RenderBox fourthIconRenderBox = _iconRenderBox(tester, Icons.hotel);
        expect(
          fourthIconRenderBox.localToGlobal(Offset.zero),
          equals(
            Offset(
              (56.0 - fourthIconRenderBox.size.width) / 2.0,
              nextDestinationY + (56.0 - fourthIconRenderBox.size.height) / 2.0,
            ),
          ),
        );
      },
    );

    testWidgets(
      'Destination spacing is correct for a compact rail - [preferredWidth]=56, [textScaleFactor]=3.0',
      (WidgetTester tester) async {
        await _pumpNavigationRail(
          tester,
          useMaterial3: false,
          textScaleFactor: 3.0,
          navigationRail: NavigationRail(
            selectedIndex: 0,
            minWidth: 56.0,
            destinations: _destinations(),
          ),
        );

        // Since the rail is icon only, its preferred width should not be affected
        // by textScaleFactor.
        final RenderBox renderBox = tester.renderObject(find.byType(NavigationRail));
        expect(renderBox.size.width, 56.0);

        // The first destination is 8 from the top because of the default vertical
        // padding at the to of the rail.
        double nextDestinationY = 8.0;
        final RenderBox firstIconRenderBox = _iconRenderBox(tester, Icons.favorite);
        expect(
          firstIconRenderBox.localToGlobal(Offset.zero),
          equals(
            Offset(
              (56.0 - firstIconRenderBox.size.width) / 2.0,
              nextDestinationY + (56.0 - firstIconRenderBox.size.height) / 2.0,
            ),
          ),
        );

        // The second destination is 56 below the first destination.
        nextDestinationY += 56.0;
        final RenderBox secondIconRenderBox = _iconRenderBox(tester, Icons.bookmark_border);
        expect(
          secondIconRenderBox.localToGlobal(Offset.zero),
          equals(
            Offset(
              (56.0 - secondIconRenderBox.size.width) / 2.0,
              nextDestinationY + (56.0 - secondIconRenderBox.size.height) / 2.0,
            ),
          ),
        );

        // The third destination is 56 below the second destination.
        nextDestinationY += 56.0;
        final RenderBox thirdIconRenderBox = _iconRenderBox(tester, Icons.star_border);
        expect(
          thirdIconRenderBox.localToGlobal(Offset.zero),
          equals(
            Offset(
              (56.0 - thirdIconRenderBox.size.width) / 2.0,
              nextDestinationY + (56.0 - thirdIconRenderBox.size.height) / 2.0,
            ),
          ),
        );

        // The fourth destination is 56 below the third destination.
        nextDestinationY += 56.0;
        final RenderBox fourthIconRenderBox = _iconRenderBox(tester, Icons.hotel);
        expect(
          fourthIconRenderBox.localToGlobal(Offset.zero),
          equals(
            Offset(
              (56.0 - fourthIconRenderBox.size.width) / 2.0,
              nextDestinationY + (56.0 - fourthIconRenderBox.size.height) / 2.0,
            ),
          ),
        );
      },
    );

    testWidgets(
      'Destination spacing is correct for a compact rail - [preferredWidth]=56, [textScaleFactor]=0.75',
      (WidgetTester tester) async {
        await _pumpNavigationRail(
          tester,
          useMaterial3: false,
          textScaleFactor: 3.0,
          navigationRail: NavigationRail(
            selectedIndex: 0,
            minWidth: 56.0,
            destinations: _destinations(),
          ),
        );

        // Since the rail is icon only, its preferred width should not be affected
        // by textScaleFactor.
        final RenderBox renderBox = tester.renderObject(find.byType(NavigationRail));
        expect(renderBox.size.width, 56.0);

        // The first destination is 8 from the top because of the default vertical
        // padding at the to of the rail.
        double nextDestinationY = 8.0;
        final RenderBox firstIconRenderBox = _iconRenderBox(tester, Icons.favorite);
        expect(
          firstIconRenderBox.localToGlobal(Offset.zero),
          equals(
            Offset(
              (56.0 - firstIconRenderBox.size.width) / 2.0,
              nextDestinationY + (56.0 - firstIconRenderBox.size.height) / 2.0,
            ),
          ),
        );

        // The second destination is 56 below the first destination.
        nextDestinationY += 56.0;
        final RenderBox secondIconRenderBox = _iconRenderBox(tester, Icons.bookmark_border);
        expect(
          secondIconRenderBox.localToGlobal(Offset.zero),
          equals(
            Offset(
              (56.0 - secondIconRenderBox.size.width) / 2.0,
              nextDestinationY + (56.0 - secondIconRenderBox.size.height) / 2.0,
            ),
          ),
        );

        // The third destination is 56 below the second destination.
        nextDestinationY += 56.0;
        final RenderBox thirdIconRenderBox = _iconRenderBox(tester, Icons.star_border);
        expect(
          thirdIconRenderBox.localToGlobal(Offset.zero),
          equals(
            Offset(
              (56.0 - thirdIconRenderBox.size.width) / 2.0,
              nextDestinationY + (56.0 - thirdIconRenderBox.size.height) / 2.0,
            ),
          ),
        );

        // The fourth destination is 56 below the third destination.
        nextDestinationY += 56.0;
        final RenderBox fourthIconRenderBox = _iconRenderBox(tester, Icons.hotel);
        expect(
          fourthIconRenderBox.localToGlobal(Offset.zero),
          equals(
            Offset(
              (56.0 - fourthIconRenderBox.size.width) / 2.0,
              nextDestinationY + (56.0 - fourthIconRenderBox.size.height) / 2.0,
            ),
          ),
        );
      },
    );

    testWidgets('Group alignment works - [groupAlignment]=-1.0 (default)', (
      WidgetTester tester,
    ) async {
      await _pumpNavigationRail(
        tester,
        useMaterial3: false,
        navigationRail: NavigationRail(selectedIndex: 0, destinations: _destinations()),
      );

      // The first destination is 8 from the top because of the default vertical
      // padding at the to of the rail.
      double nextDestinationY = 8.0;
      final RenderBox firstIconRenderBox = _iconRenderBox(tester, Icons.favorite);
      expect(
        firstIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (72.0 - firstIconRenderBox.size.width) / 2.0,
            nextDestinationY + (72.0 - firstIconRenderBox.size.height) / 2.0,
          ),
        ),
      );

      // The second destination is 72 below the first destination.
      nextDestinationY += 72.0;
      final RenderBox secondIconRenderBox = _iconRenderBox(tester, Icons.bookmark_border);
      expect(
        secondIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (72.0 - secondIconRenderBox.size.width) / 2.0,
            nextDestinationY + (72.0 - secondIconRenderBox.size.height) / 2.0,
          ),
        ),
      );

      // The third destination is 72 below the second destination.
      nextDestinationY += 72.0;
      final RenderBox thirdIconRenderBox = _iconRenderBox(tester, Icons.star_border);
      expect(
        thirdIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (72.0 - thirdIconRenderBox.size.width) / 2.0,
            nextDestinationY + (72.0 - thirdIconRenderBox.size.height) / 2.0,
          ),
        ),
      );

      // The fourth destination is 72 below the third destination.
      nextDestinationY += 72.0;
      final RenderBox fourthIconRenderBox = _iconRenderBox(tester, Icons.hotel);
      expect(
        fourthIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (72.0 - fourthIconRenderBox.size.width) / 2.0,
            nextDestinationY + (72.0 - fourthIconRenderBox.size.height) / 2.0,
          ),
        ),
      );
    });

    testWidgets('Group alignment works - [groupAlignment]=0.0', (WidgetTester tester) async {
      await _pumpNavigationRail(
        tester,
        useMaterial3: false,
        navigationRail: NavigationRail(
          selectedIndex: 0,
          groupAlignment: 0.0,
          destinations: _destinations(),
        ),
      );

      double nextDestinationY = 160.0;
      final RenderBox firstIconRenderBox = _iconRenderBox(tester, Icons.favorite);
      expect(
        firstIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (72.0 - firstIconRenderBox.size.width) / 2.0,
            nextDestinationY + (72.0 - firstIconRenderBox.size.height) / 2.0,
          ),
        ),
      );

      // The second destination is 72 below the first destination.
      nextDestinationY += 72.0;
      final RenderBox secondIconRenderBox = _iconRenderBox(tester, Icons.bookmark_border);
      expect(
        secondIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (72.0 - secondIconRenderBox.size.width) / 2.0,
            nextDestinationY + (72.0 - secondIconRenderBox.size.height) / 2.0,
          ),
        ),
      );

      // The third destination is 72 below the second destination.
      nextDestinationY += 72.0;
      final RenderBox thirdIconRenderBox = _iconRenderBox(tester, Icons.star_border);
      expect(
        thirdIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (72.0 - thirdIconRenderBox.size.width) / 2.0,
            nextDestinationY + (72.0 - thirdIconRenderBox.size.height) / 2.0,
          ),
        ),
      );

      // The fourth destination is 72 below the third destination.
      nextDestinationY += 72.0;
      final RenderBox fourthIconRenderBox = _iconRenderBox(tester, Icons.hotel);
      expect(
        fourthIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (72.0 - fourthIconRenderBox.size.width) / 2.0,
            nextDestinationY + (72.0 - fourthIconRenderBox.size.height) / 2.0,
          ),
        ),
      );
    });

    testWidgets('Group alignment works - [groupAlignment]=1.0', (WidgetTester tester) async {
      await _pumpNavigationRail(
        tester,
        useMaterial3: false,
        navigationRail: NavigationRail(
          selectedIndex: 0,
          groupAlignment: 1.0,
          destinations: _destinations(),
        ),
      );

      double nextDestinationY = 312.0;
      final RenderBox firstIconRenderBox = _iconRenderBox(tester, Icons.favorite);
      expect(
        firstIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (72.0 - firstIconRenderBox.size.width) / 2.0,
            nextDestinationY + (72.0 - firstIconRenderBox.size.height) / 2.0,
          ),
        ),
      );

      // The second destination is 72 below the first destination.
      nextDestinationY += 72.0;
      final RenderBox secondIconRenderBox = _iconRenderBox(tester, Icons.bookmark_border);
      expect(
        secondIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (72.0 - secondIconRenderBox.size.width) / 2.0,
            nextDestinationY + (72.0 - secondIconRenderBox.size.height) / 2.0,
          ),
        ),
      );

      // The third destination is 72 below the second destination.
      nextDestinationY += 72.0;
      final RenderBox thirdIconRenderBox = _iconRenderBox(tester, Icons.star_border);
      expect(
        thirdIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (72.0 - thirdIconRenderBox.size.width) / 2.0,
            nextDestinationY + (72.0 - thirdIconRenderBox.size.height) / 2.0,
          ),
        ),
      );

      // The fourth destination is 72 below the third destination.
      nextDestinationY += 72.0;
      final RenderBox fourthIconRenderBox = _iconRenderBox(tester, Icons.hotel);
      expect(
        fourthIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (72.0 - fourthIconRenderBox.size.width) / 2.0,
            nextDestinationY + (72.0 - fourthIconRenderBox.size.height) / 2.0,
          ),
        ),
      );
    });

    testWidgets('Leading and trailing appear in the correct places', (WidgetTester tester) async {
      await _pumpNavigationRail(
        tester,
        useMaterial3: false,
        navigationRail: NavigationRail(
          selectedIndex: 0,
          leading: FloatingActionButton(onPressed: () {}),
          trailing: FloatingActionButton(onPressed: () {}),
          destinations: _destinations(),
        ),
      );

      final RenderBox leading = tester.renderObject<RenderBox>(
        find.byType(FloatingActionButton).at(0),
      );
      final RenderBox trailing = tester.renderObject<RenderBox>(
        find.byType(FloatingActionButton).at(1),
      );
      expect(leading.localToGlobal(Offset.zero), Offset((72 - leading.size.width) / 2.0, 8.0));
      expect(trailing.localToGlobal(Offset.zero), Offset((72 - trailing.size.width) / 2.0, 360.0));
    });

    testWidgets('Extended rail animates the width and labels appear - [textDirection]=LTR', (
      WidgetTester tester,
    ) async {
      bool extended = false;
      late StateSetter stateSetter;

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(useMaterial3: false),
          home: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              stateSetter = setState;
              return Scaffold(
                body: Row(
                  children: <Widget>[
                    NavigationRail(
                      selectedIndex: 0,
                      destinations: _destinations(),
                      extended: extended,
                    ),
                    const Expanded(child: Text('body')),
                  ],
                ),
              );
            },
          ),
        ),
      );

      final RenderBox rail = tester.firstRenderObject<RenderBox>(find.byType(NavigationRail));

      expect(rail.size.width, equals(72.0));

      stateSetter(() {
        extended = true;
      });

      await tester.pump();
      await tester.pump(const Duration(milliseconds: 100));
      expect(rail.size.width, equals(164.0));

      await tester.pumpAndSettle();
      expect(rail.size.width, equals(256.0));

      // The first destination is 8 from the top because of the default vertical
      // padding at the to of the rail.
      double nextDestinationY = 8.0;
      final RenderBox firstIconRenderBox = _iconRenderBox(tester, Icons.favorite);
      final RenderBox firstLabelRenderBox = _labelRenderBox(tester, 'Abc');
      expect(
        firstIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (72.0 - firstIconRenderBox.size.width) / 2.0,
            nextDestinationY + (72.0 - firstIconRenderBox.size.height) / 2.0,
          ),
        ),
      );
      expect(
        firstLabelRenderBox.localToGlobal(Offset.zero),
        equals(Offset(72.0, nextDestinationY + (72.0 - firstLabelRenderBox.size.height) / 2.0)),
      );

      // The second destination is 72 below the first destination.
      nextDestinationY += 72.0;
      final RenderBox secondIconRenderBox = _iconRenderBox(tester, Icons.bookmark_border);
      final RenderBox secondLabelRenderBox = _labelRenderBox(tester, 'Def');
      expect(
        secondIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (72.0 - secondIconRenderBox.size.width) / 2.0,
            nextDestinationY + (72.0 - secondIconRenderBox.size.height) / 2.0,
          ),
        ),
      );
      expect(
        secondLabelRenderBox.localToGlobal(Offset.zero),
        equals(Offset(72.0, nextDestinationY + (72.0 - secondLabelRenderBox.size.height) / 2.0)),
      );

      // The third destination is 72 below the second destination.
      nextDestinationY += 72.0;
      final RenderBox thirdIconRenderBox = _iconRenderBox(tester, Icons.star_border);
      final RenderBox thirdLabelRenderBox = _labelRenderBox(tester, 'Ghi');
      expect(
        thirdIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (72.0 - thirdIconRenderBox.size.width) / 2.0,
            nextDestinationY + (72.0 - thirdIconRenderBox.size.height) / 2.0,
          ),
        ),
      );
      expect(
        thirdLabelRenderBox.localToGlobal(Offset.zero),
        equals(Offset(72.0, nextDestinationY + (72.0 - thirdLabelRenderBox.size.height) / 2.0)),
      );

      // The fourth destination is 72 below the third destination.
      nextDestinationY += 72.0;
      final RenderBox fourthIconRenderBox = _iconRenderBox(tester, Icons.hotel);
      final RenderBox fourthLabelRenderBox = _labelRenderBox(tester, 'Jkl');
      expect(
        fourthIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            (72.0 - fourthIconRenderBox.size.width) / 2.0,
            nextDestinationY + (72.0 - fourthIconRenderBox.size.height) / 2.0,
          ),
        ),
      );
      expect(
        fourthLabelRenderBox.localToGlobal(Offset.zero),
        equals(Offset(72.0, nextDestinationY + (72.0 - fourthLabelRenderBox.size.height) / 2.0)),
      );
    });

    testWidgets('Extended rail animates the width and labels appear - [textDirection]=RTL', (
      WidgetTester tester,
    ) async {
      bool extended = false;
      late StateSetter stateSetter;

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(useMaterial3: false),
          home: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              stateSetter = setState;
              return Directionality(
                textDirection: TextDirection.rtl,
                child: Scaffold(
                  body: Row(
                    textDirection: TextDirection.rtl,
                    children: <Widget>[
                      NavigationRail(
                        selectedIndex: 0,
                        destinations: _destinations(),
                        extended: extended,
                      ),
                      const Expanded(child: Text('body')),
                    ],
                  ),
                ),
              );
            },
          ),
        ),
      );

      final RenderBox rail = tester.firstRenderObject<RenderBox>(find.byType(NavigationRail));

      expect(rail.size.width, equals(72.0));
      expect(rail.localToGlobal(Offset.zero), equals(const Offset(728.0, 0.0)));

      stateSetter(() {
        extended = true;
      });

      await tester.pump();
      await tester.pump(const Duration(milliseconds: 100));
      expect(rail.size.width, equals(164.0));
      expect(rail.localToGlobal(Offset.zero), equals(const Offset(636.0, 0.0)));

      await tester.pumpAndSettle();
      expect(rail.size.width, equals(256.0));
      expect(rail.localToGlobal(Offset.zero), equals(const Offset(544.0, 0.0)));

      // The first destination is 8 from the top because of the default vertical
      // padding at the to of the rail.
      double nextDestinationY = 8.0;
      final RenderBox firstIconRenderBox = _iconRenderBox(tester, Icons.favorite);
      final RenderBox firstLabelRenderBox = _labelRenderBox(tester, 'Abc');
      expect(
        firstIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            800.0 - (72.0 + firstIconRenderBox.size.width) / 2.0,
            nextDestinationY + (72.0 - firstIconRenderBox.size.height) / 2.0,
          ),
        ),
      );
      expect(
        firstLabelRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            800.0 - 72.0 - firstLabelRenderBox.size.width,
            nextDestinationY + (72.0 - firstLabelRenderBox.size.height) / 2.0,
          ),
        ),
      );

      // The second destination is 72 below the first destination.
      nextDestinationY += 72.0;
      final RenderBox secondIconRenderBox = _iconRenderBox(tester, Icons.bookmark_border);
      final RenderBox secondLabelRenderBox = _labelRenderBox(tester, 'Def');
      expect(
        secondIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            800.0 - (72.0 + secondIconRenderBox.size.width) / 2.0,
            nextDestinationY + (72.0 - secondIconRenderBox.size.height) / 2.0,
          ),
        ),
      );
      expect(
        secondLabelRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            800.0 - 72.0 - secondLabelRenderBox.size.width,
            nextDestinationY + (72.0 - secondLabelRenderBox.size.height) / 2.0,
          ),
        ),
      );

      // The third destination is 72 below the second destination.
      nextDestinationY += 72.0;
      final RenderBox thirdIconRenderBox = _iconRenderBox(tester, Icons.star_border);
      final RenderBox thirdLabelRenderBox = _labelRenderBox(tester, 'Ghi');
      expect(
        thirdIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            800.0 - (72.0 + thirdIconRenderBox.size.width) / 2.0,
            nextDestinationY + (72.0 - thirdIconRenderBox.size.height) / 2.0,
          ),
        ),
      );
      expect(
        thirdLabelRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            800.0 - 72.0 - thirdLabelRenderBox.size.width,
            nextDestinationY + (72.0 - thirdLabelRenderBox.size.height) / 2.0,
          ),
        ),
      );

      // The fourth destination is 72 below the third destination.
      nextDestinationY += 72.0;
      final RenderBox fourthIconRenderBox = _iconRenderBox(tester, Icons.hotel);
      final RenderBox fourthLabelRenderBox = _labelRenderBox(tester, 'Jkl');
      expect(
        fourthIconRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            800.0 - (72.0 + fourthIconRenderBox.size.width) / 2.0,
            nextDestinationY + (72.0 - fourthIconRenderBox.size.height) / 2.0,
          ),
        ),
      );
      expect(
        fourthLabelRenderBox.localToGlobal(Offset.zero),
        equals(
          Offset(
            800.0 - 72.0 - fourthLabelRenderBox.size.width,
            nextDestinationY + (72.0 - fourthLabelRenderBox.size.height) / 2.0,
          ),
        ),
      );
    });

    testWidgets('Extended rail gets wider with longer labels are larger text scale', (
      WidgetTester tester,
    ) async {
      bool extended = false;
      late StateSetter stateSetter;

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(useMaterial3: false),
          home: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              stateSetter = setState;
              return Scaffold(
                body: Row(
                  children: <Widget>[
                    MediaQuery.withClampedTextScaling(
                      minScaleFactor: 3.0,
                      maxScaleFactor: 3.0,
                      child: NavigationRail(
                        selectedIndex: 0,
                        destinations: const <NavigationRailDestination>[
                          NavigationRailDestination(
                            icon: Icon(Icons.favorite_border),
                            selectedIcon: Icon(Icons.favorite),
                            label: Text('Abc'),
                          ),
                          NavigationRailDestination(
                            icon: Icon(Icons.bookmark_border),
                            selectedIcon: Icon(Icons.bookmark),
                            label: Text('Longer Label'),
                          ),
                        ],
                        extended: extended,
                      ),
                    ),
                    const Expanded(child: Text('body')),
                  ],
                ),
              );
            },
          ),
        ),
      );

      final RenderBox rail = tester.firstRenderObject<RenderBox>(find.byType(NavigationRail));

      expect(rail.size.width, equals(72.0));

      stateSetter(() {
        extended = true;
      });

      await tester.pump();
      await tester.pump(const Duration(milliseconds: 100));
      expect(rail.size.width, equals(328.0));

      await tester.pumpAndSettle();
      expect(rail.size.width, equals(584.0));
    });

    testWidgets('Extended rail final width can be changed', (WidgetTester tester) async {
      bool extended = false;
      late StateSetter stateSetter;

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(useMaterial3: false),
          home: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              stateSetter = setState;
              return Scaffold(
                body: Row(
                  children: <Widget>[
                    NavigationRail(
                      selectedIndex: 0,
                      minExtendedWidth: 300,
                      destinations: _destinations(),
                      extended: extended,
                    ),
                    const Expanded(child: Text('body')),
                  ],
                ),
              );
            },
          ),
        ),
      );

      final RenderBox rail = tester.firstRenderObject<RenderBox>(find.byType(NavigationRail));

      expect(rail.size.width, equals(72.0));

      stateSetter(() {
        extended = true;
      });

      await tester.pumpAndSettle();
      expect(rail.size.width, equals(300.0));
    });

    /// Regression test for https://github.com/flutter/flutter/issues/65657
    testWidgets('Extended rail transition does not jump from the beginning', (
      WidgetTester tester,
    ) async {
      bool extended = false;
      late StateSetter stateSetter;

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(useMaterial3: false),
          home: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              stateSetter = setState;
              return Scaffold(
                body: Row(
                  children: <Widget>[
                    NavigationRail(
                      selectedIndex: 0,
                      destinations: const <NavigationRailDestination>[
                        NavigationRailDestination(
                          icon: Icon(Icons.favorite_border),
                          selectedIcon: Icon(Icons.favorite),
                          label: Text('Abc'),
                        ),
                        NavigationRailDestination(
                          icon: Icon(Icons.bookmark_border),
                          selectedIcon: Icon(Icons.bookmark),
                          label: Text('Longer Label'),
                        ),
                      ],
                      extended: extended,
                    ),
                    const Expanded(child: Text('body')),
                  ],
                ),
              );
            },
          ),
        ),
      );

      final Finder rail = find.byType(NavigationRail);

      // Before starting the animation, the rail has a width of 72.
      expect(tester.getSize(rail).width, 72.0);

      stateSetter(() {
        extended = true;
      });

      await tester.pump();
      // Create very close to 0, but non-zero, animation value.
      await tester.pump(const Duration(milliseconds: 1));
      // Expect that it has started to extend.
      expect(tester.getSize(rail).width, greaterThan(72.0));
      // Expect that it has only extended by a small amount, or that the first
      // frame does not jump. This helps verify that it is a smooth animation.
      expect(tester.getSize(rail).width, closeTo(72.0, 1.0));
    });

    testWidgets('NavigationRailDestination adds circular indicator when no labels are present', (
      WidgetTester tester,
    ) async {
      await _pumpNavigationRail(
        tester,
        useMaterial3: false,
        navigationRail: NavigationRail(
          useIndicator: true,
          labelType: NavigationRailLabelType.none,
          selectedIndex: 0,
          destinations: const <NavigationRailDestination>[
            NavigationRailDestination(
              icon: Icon(Icons.favorite_border),
              selectedIcon: Icon(Icons.favorite),
              label: Text('Abc'),
            ),
            NavigationRailDestination(
              icon: Icon(Icons.bookmark_border),
              selectedIcon: Icon(Icons.bookmark),
              label: Text('Def'),
            ),
            NavigationRailDestination(
              icon: Icon(Icons.star_border),
              selectedIcon: Icon(Icons.star),
              label: Text('Ghi'),
            ),
          ],
        ),
      );

      final NavigationIndicator indicator = tester.widget<NavigationIndicator>(
        find.byType(NavigationIndicator).first,
      );

      expect(indicator.width, 56);
      expect(indicator.height, 56);
    });

    testWidgets('NavigationRailDestination has center aligned indicator - [labelType]=none', (
      WidgetTester tester,
    ) async {
      // This is a regression test for
      // https://github.com/flutter/flutter/issues/97753
      await _pumpNavigationRail(
        tester,
        useMaterial3: false,
        navigationRail: NavigationRail(
          labelType: NavigationRailLabelType.none,
          selectedIndex: 0,
          destinations: const <NavigationRailDestination>[
            NavigationRailDestination(
              icon: Stack(
                children: <Widget>[
                  Icon(Icons.umbrella),
                  Positioned(
                    top: 0,
                    right: 0,
                    child: Text('Text', style: TextStyle(fontSize: 10, color: Colors.red)),
                  ),
                ],
              ),
              label: Text('Abc'),
            ),
            NavigationRailDestination(icon: Icon(Icons.umbrella), label: Text('Def')),
            NavigationRailDestination(icon: Icon(Icons.bookmark_border), label: Text('Ghi')),
          ],
        ),
      );
      // Indicator with Stack widget
      final RenderBox firstIndicator = tester.renderObject(find.byType(Icon).first);
      expect(firstIndicator.localToGlobal(Offset.zero).dx, 24.0);
      // Indicator without Stack widget
      final RenderBox lastIndicator = tester.renderObject(find.byType(Icon).last);
      expect(lastIndicator.localToGlobal(Offset.zero).dx, 24.0);
    });

    testWidgets('NavigationRail respects the notch/system navigation bar in landscape mode', (
      WidgetTester tester,
    ) async {
      const double safeAreaPadding = 40.0;
      NavigationRail navigationRail() {
        return NavigationRail(
          selectedIndex: 0,
          destinations: const <NavigationRailDestination>[
            NavigationRailDestination(
              icon: Icon(Icons.favorite_border),
              selectedIcon: Icon(Icons.favorite),
              label: Text('Abc'),
            ),
            NavigationRailDestination(
              icon: Icon(Icons.bookmark_border),
              selectedIcon: Icon(Icons.bookmark),
              label: Text('Def'),
            ),
          ],
        );
      }

      await tester.pumpWidget(_buildWidget(navigationRail(), useMaterial3: false));
      final double defaultWidth = tester.getSize(find.byType(NavigationRail)).width;
      expect(defaultWidth, 72);

      await tester.pumpWidget(
        _buildWidget(
          MediaQuery(
            data: const MediaQueryData(padding: EdgeInsets.only(left: safeAreaPadding)),
            child: navigationRail(),
          ),
          useMaterial3: false,
        ),
      );
      final double updatedWidth = tester.getSize(find.byType(NavigationRail)).width;
      expect(updatedWidth, defaultWidth + safeAreaPadding);

      // test width when text direction is RTL.
      await tester.pumpWidget(
        _buildWidget(
          MediaQuery(
            data: const MediaQueryData(padding: EdgeInsets.only(right: safeAreaPadding)),
            child: navigationRail(),
          ),
          useMaterial3: false,
          isRTL: true,
        ),
      );
      final double updatedWidthRTL = tester.getSize(find.byType(NavigationRail)).width;
      expect(updatedWidthRTL, defaultWidth + safeAreaPadding);
    });
  }); // End Material 2 group
}

TestSemantics _expectedSemantics() {
  return TestSemantics.root(
    children: <TestSemantics>[
      TestSemantics(
        textDirection: TextDirection.ltr,
        children: <TestSemantics>[
          TestSemantics(
            children: <TestSemantics>[
              TestSemantics(
                flags: <SemanticsFlag>[SemanticsFlag.scopesRoute],
                children: <TestSemantics>[
                  TestSemantics(
                    flags: <SemanticsFlag>[
                      SemanticsFlag.hasSelectedState,
                      SemanticsFlag.isSelected,
                      SemanticsFlag.isFocusable,
                    ],
                    actions: <SemanticsAction>[SemanticsAction.tap, SemanticsAction.focus],
                    label: 'Abc\nTab 1 of 4',
                    textDirection: TextDirection.ltr,
                  ),
                  TestSemantics(
                    flags: <SemanticsFlag>[
                      SemanticsFlag.isFocusable,
                      SemanticsFlag.hasSelectedState,
                    ],
                    actions: <SemanticsAction>[SemanticsAction.tap, SemanticsAction.focus],
                    label: 'Def\nTab 2 of 4',
                    textDirection: TextDirection.ltr,
                  ),
                  TestSemantics(
                    flags: <SemanticsFlag>[
                      SemanticsFlag.isFocusable,
                      SemanticsFlag.hasSelectedState,
                    ],
                    actions: <SemanticsAction>[SemanticsAction.tap, SemanticsAction.focus],
                    label: 'Ghi\nTab 3 of 4',
                    textDirection: TextDirection.ltr,
                  ),
                  TestSemantics(
                    flags: <SemanticsFlag>[
                      SemanticsFlag.isFocusable,
                      SemanticsFlag.hasSelectedState,
                    ],
                    actions: <SemanticsAction>[SemanticsAction.tap, SemanticsAction.focus],
                    label: 'Jkl\nTab 4 of 4',
                    textDirection: TextDirection.ltr,
                  ),
                  TestSemantics(label: 'body', textDirection: TextDirection.ltr),
                ],
              ),
            ],
          ),
        ],
      ),
    ],
  );
}

List<NavigationRailDestination> _destinations() {
  return const <NavigationRailDestination>[
    NavigationRailDestination(
      icon: Icon(Icons.favorite_border),
      selectedIcon: Icon(Icons.favorite),
      label: Text('Abc'),
    ),
    NavigationRailDestination(
      icon: Icon(Icons.bookmark_border),
      selectedIcon: Icon(Icons.bookmark),
      label: Text('Def'),
    ),
    NavigationRailDestination(
      icon: Icon(Icons.star_border),
      selectedIcon: Icon(Icons.star),
      label: Text('Ghi'),
    ),
    NavigationRailDestination(
      icon: Icon(Icons.hotel),
      selectedIcon: Icon(Icons.home),
      label: Text('Jkl'),
    ),
  ];
}

Future<void> _pumpNavigationRail(
  WidgetTester tester, {
  double textScaleFactor = 1.0,
  required NavigationRail navigationRail,
  bool useMaterial3 = true,
  NavigationRailThemeData? navigationRailTheme,
}) async {
  await tester.pumpWidget(
    MaterialApp(
      theme: ThemeData(useMaterial3: useMaterial3, navigationRailTheme: navigationRailTheme),
      home: Builder(
        builder: (BuildContext context) {
          return MediaQuery.withClampedTextScaling(
            minScaleFactor: textScaleFactor,
            maxScaleFactor: textScaleFactor,
            child: Scaffold(
              body: Row(children: <Widget>[navigationRail, const Expanded(child: Text('body'))]),
            ),
          );
        },
      ),
    ),
  );
}

Future<void> _pumpLocalizedTestRail(
  WidgetTester tester, {
  NavigationRailLabelType? labelType,
  bool extended = false,
}) async {
  await tester.pumpWidget(
    Localizations(
      locale: const Locale('en', 'US'),
      delegates: const <LocalizationsDelegate<dynamic>>[
        DefaultMaterialLocalizations.delegate,
        DefaultWidgetsLocalizations.delegate,
      ],
      child: MaterialApp(
        home: Scaffold(
          body: Row(
            children: <Widget>[
              NavigationRail(
                selectedIndex: 0,
                extended: extended,
                destinations: _destinations(),
                labelType: labelType,
              ),
              const Expanded(child: Text('body')),
            ],
          ),
        ),
      ),
    ),
  );
}

RenderBox _iconRenderBox(WidgetTester tester, IconData iconData) {
  return tester.firstRenderObject<RenderBox>(
    find.descendant(of: find.byIcon(iconData), matching: find.byType(RichText)),
  );
}

RenderBox _labelRenderBox(WidgetTester tester, String text) {
  return tester.firstRenderObject<RenderBox>(
    find.descendant(of: find.text(text), matching: find.byType(RichText)),
  );
}

TextStyle _iconStyle(WidgetTester tester, IconData icon) {
  return tester
      .widget<RichText>(find.descendant(of: find.byIcon(icon), matching: find.byType(RichText)))
      .text
      .style!;
}

Finder _opacityAboveLabel(String text) {
  return find.ancestor(of: find.text(text), matching: find.byType(Opacity));
}

// Only valid when labelType != all.
double? _labelOpacity(WidgetTester tester, String text) {
  // We search for both Visibility and FadeTransition since in some
  // cases opacity is animated, in other it's not.
  final Iterable<Visibility> visibilityWidgets = tester.widgetList<Visibility>(
    find.ancestor(of: find.text(text), matching: find.byType(Visibility)),
  );
  if (visibilityWidgets.isNotEmpty) {
    return visibilityWidgets.single.visible ? 1.0 : 0.0;
  }

  final FadeTransition fadeTransitionWidget = tester.widget<FadeTransition>(
    find
        .ancestor(of: find.text(text), matching: find.byType(FadeTransition))
        .first, // first because there's also a FadeTransition from the MaterialPageRoute, which is up the tree
  );
  return fadeTransitionWidget.opacity.value;
}

Material _railMaterial(WidgetTester tester) {
  // The first material is for the rail, and the rest are for the destinations.
  return tester.firstWidget<Material>(
    find.descendant(of: find.byType(NavigationRail), matching: find.byType(Material)),
  );
}

Widget _buildWidget(Widget child, {bool useMaterial3 = true, bool isRTL = false}) {
  return MaterialApp(
    theme: ThemeData(useMaterial3: useMaterial3),
    home: Directionality(
      textDirection: isRTL ? TextDirection.rtl : TextDirection.ltr,
      child: Scaffold(body: Row(children: <Widget>[child, const Expanded(child: Text('body'))])),
    ),
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
