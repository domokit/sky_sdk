// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart' show DragStartBehavior;
import 'package:flutter/material.dart';
import 'package:flutter/services.dart' show LogicalKeyboardKey;
import 'package:flutter_test/flutter_test.dart';

import 'states.dart';

class MaterialLocalizationsDelegate extends LocalizationsDelegate<MaterialLocalizations> {
  @override
  bool isSupported(Locale locale) => true;

  @override
  Future<MaterialLocalizations> load(Locale locale) => DefaultMaterialLocalizations.load(locale);

  @override
  bool shouldReload(MaterialLocalizationsDelegate old) => false;
}

class WidgetsLocalizationsDelegate extends LocalizationsDelegate<WidgetsLocalizations> {
  @override
  bool isSupported(Locale locale) => true;

  @override
  Future<WidgetsLocalizations> load(Locale locale) => DefaultWidgetsLocalizations.load(locale);

  @override
  bool shouldReload(WidgetsLocalizationsDelegate old) => false;
}

Widget textFieldBoilerplate({required Widget child}) {
  return MaterialApp(
    home: Localizations(
      locale: const Locale('en', 'US'),
      delegates: <LocalizationsDelegate<dynamic>>[
        WidgetsLocalizationsDelegate(),
        MaterialLocalizationsDelegate(),
      ],
      child: Directionality(
        textDirection: TextDirection.ltr,
        child: MediaQuery(
          data: const MediaQueryData(size: Size(800.0, 600.0)),
          child: Center(child: Material(child: child)),
        ),
      ),
    ),
  );
}

Widget primaryScrollControllerBoilerplate({
  required Widget child,
  required ScrollController controller,
}) {
  return Directionality(
    textDirection: TextDirection.ltr,
    child: MediaQuery(
      data: const MediaQueryData(),
      child: PrimaryScrollController(controller: controller, child: child),
    ),
  );
}

void main() {
  testWidgets('ListView control test', (WidgetTester tester) async {
    final List<String> log = <String>[];

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: ListView(
          dragStartBehavior: DragStartBehavior.down,
          children:
              kStates.map<Widget>((String state) {
                return GestureDetector(
                  onTap: () {
                    log.add(state);
                  },
                  dragStartBehavior: DragStartBehavior.down,
                  child: Container(
                    height: 200.0,
                    color: const Color(0xFF0000FF),
                    child: Text(state),
                  ),
                );
              }).toList(),
        ),
      ),
    );

    await tester.tap(find.text('Alabama'));
    expect(log, equals(<String>['Alabama']));
    log.clear();

    expect(find.text('Nevada'), findsNothing);

    await tester.drag(find.text('Alabama'), const Offset(0.0, -4000.0));
    await tester.pump();

    expect(find.text('Alabama'), findsNothing);
    expect(tester.getCenter(find.text('Massachusetts')), equals(const Offset(400.0, 100.0)));

    await tester.tap(find.text('Massachusetts'));
    expect(log, equals(<String>['Massachusetts']));
    log.clear();
  });

  testWidgets('ListView dismiss keyboard onDrag test', (WidgetTester tester) async {
    final List<FocusNode> focusNodes = List<FocusNode>.generate(50, (int i) => FocusNode());
    addTearDown(() {
      for (final FocusNode node in focusNodes) {
        node.dispose();
      }
    });

    await tester.pumpWidget(
      textFieldBoilerplate(
        child: ListView(
          padding: EdgeInsets.zero,
          keyboardDismissBehavior: ScrollViewKeyboardDismissBehavior.onDrag,
          children:
              focusNodes.map((FocusNode focusNode) {
                return Container(
                  height: 50,
                  color: Colors.green,
                  child: TextField(
                    focusNode: focusNode,
                    style: const TextStyle(fontSize: 24, fontWeight: FontWeight.bold),
                  ),
                );
              }).toList(),
        ),
      ),
    );

    final Finder finder = find.byType(TextField).first;
    final TextField textField = tester.widget(finder);
    await tester.showKeyboard(finder);
    expect(textField.focusNode!.hasFocus, isTrue);

    await tester.drag(finder, const Offset(0.0, -40.0));
    await tester.pumpAndSettle();
    expect(textField.focusNode!.hasFocus, isFalse);
  });

  testWidgets('GridView.builder supports null items', (WidgetTester tester) async {
    await tester.pumpWidget(
      textFieldBoilerplate(
        child: GridView.builder(
          gridDelegate: const SliverGridDelegateWithFixedCrossAxisCount(crossAxisCount: 42),
          itemCount: 42,
          itemBuilder: (BuildContext context, int index) {
            if (index == 5) {
              return null;
            }

            return const Text('item');
          },
        ),
      ),
    );

    expect(find.text('item'), findsNWidgets(5));
  });

  testWidgets('ListView.builder supports null items', (WidgetTester tester) async {
    await tester.pumpWidget(
      textFieldBoilerplate(
        child: ListView.builder(
          itemCount: 42,
          itemBuilder: (BuildContext context, int index) {
            if (index == 5) {
              return null;
            }

            return const Text('item');
          },
        ),
      ),
    );

    expect(find.text('item'), findsNWidgets(5));
  });

  testWidgets('PageView supports null items in itemBuilder', (WidgetTester tester) async {
    final PageController controller = PageController(viewportFraction: 1 / 5);
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      textFieldBoilerplate(
        child: PageView.builder(
          itemCount: 5,
          controller: controller,
          itemBuilder: (BuildContext context, int index) {
            if (index == 2) {
              return null;
            }

            return const Text('item');
          },
        ),
      ),
    );

    expect(find.text('item'), findsNWidgets(2));
  });

  testWidgets('ListView.separated supports null items in itemBuilder', (WidgetTester tester) async {
    await tester.pumpWidget(
      textFieldBoilerplate(
        child: ListView.separated(
          itemCount: 42,
          separatorBuilder: (BuildContext context, int index) {
            return const Text('separator');
          },
          itemBuilder: (BuildContext context, int index) {
            if (index == 5) {
              return null;
            }

            return const Text('item');
          },
        ),
      ),
    );

    expect(find.text('item'), findsNWidgets(5));
    expect(find.text('separator'), findsNWidgets(5));
  });

  testWidgets('ListView.builder dismiss keyboard onDrag test', (WidgetTester tester) async {
    final List<FocusNode> focusNodes = List<FocusNode>.generate(50, (int i) => FocusNode());
    addTearDown(() {
      for (final FocusNode node in focusNodes) {
        node.dispose();
      }
    });

    await tester.pumpWidget(
      textFieldBoilerplate(
        child: ListView.builder(
          padding: EdgeInsets.zero,
          keyboardDismissBehavior: ScrollViewKeyboardDismissBehavior.onDrag,
          itemCount: focusNodes.length,
          itemBuilder: (BuildContext context, int index) {
            return Container(
              height: 50,
              color: Colors.green,
              child: TextField(
                focusNode: focusNodes[index],
                style: const TextStyle(fontSize: 24, fontWeight: FontWeight.bold),
              ),
            );
          },
        ),
      ),
    );

    final Finder finder = find.byType(TextField).first;
    final TextField textField = tester.widget(finder);
    await tester.showKeyboard(finder);
    expect(textField.focusNode!.hasFocus, isTrue);

    await tester.drag(finder, const Offset(0.0, -40.0));
    await tester.pumpAndSettle();
    expect(textField.focusNode!.hasFocus, isFalse);
  });

  testWidgets('ListView.custom dismiss keyboard onDrag test', (WidgetTester tester) async {
    final List<FocusNode> focusNodes = List<FocusNode>.generate(50, (int i) => FocusNode());
    addTearDown(() {
      for (final FocusNode node in focusNodes) {
        node.dispose();
      }
    });

    await tester.pumpWidget(
      textFieldBoilerplate(
        child: ListView.custom(
          padding: EdgeInsets.zero,
          keyboardDismissBehavior: ScrollViewKeyboardDismissBehavior.onDrag,
          childrenDelegate: SliverChildBuilderDelegate((BuildContext context, int index) {
            return Container(
              height: 50,
              color: Colors.green,
              child: TextField(
                focusNode: focusNodes[index],
                style: const TextStyle(fontSize: 24, fontWeight: FontWeight.bold),
              ),
            );
          }, childCount: focusNodes.length),
        ),
      ),
    );

    final Finder finder = find.byType(TextField).first;
    final TextField textField = tester.widget(finder);
    await tester.showKeyboard(finder);
    expect(textField.focusNode!.hasFocus, isTrue);

    await tester.drag(finder, const Offset(0.0, -40.0));
    await tester.pumpAndSettle();
    expect(textField.focusNode!.hasFocus, isFalse);
  });

  testWidgets('ListView.separated dismiss keyboard onDrag test', (WidgetTester tester) async {
    final List<FocusNode> focusNodes = List<FocusNode>.generate(50, (int i) => FocusNode());
    addTearDown(() {
      for (final FocusNode node in focusNodes) {
        node.dispose();
      }
    });

    await tester.pumpWidget(
      textFieldBoilerplate(
        child: ListView.separated(
          padding: EdgeInsets.zero,
          keyboardDismissBehavior: ScrollViewKeyboardDismissBehavior.onDrag,
          itemCount: focusNodes.length,
          separatorBuilder: (BuildContext context, int index) => const Divider(),
          itemBuilder: (BuildContext context, int index) {
            return Container(
              height: 50,
              color: Colors.green,
              child: TextField(
                focusNode: focusNodes[index],
                style: const TextStyle(fontSize: 24, fontWeight: FontWeight.bold),
              ),
            );
          },
        ),
      ),
    );

    final Finder finder = find.byType(TextField).first;
    final TextField textField = tester.widget(finder);
    await tester.showKeyboard(finder);
    expect(textField.focusNode!.hasFocus, isTrue);

    await tester.drag(finder, const Offset(0.0, -40.0));
    await tester.pumpAndSettle();
    expect(textField.focusNode!.hasFocus, isFalse);
  });

  testWidgets('GridView dismiss keyboard onDrag test', (WidgetTester tester) async {
    final List<FocusNode> focusNodes = List<FocusNode>.generate(50, (int i) => FocusNode());
    addTearDown(() {
      for (final FocusNode node in focusNodes) {
        node.dispose();
      }
    });

    await tester.pumpWidget(
      textFieldBoilerplate(
        child: GridView(
          padding: EdgeInsets.zero,
          gridDelegate: const SliverGridDelegateWithFixedCrossAxisCount(crossAxisCount: 2),
          keyboardDismissBehavior: ScrollViewKeyboardDismissBehavior.onDrag,
          children:
              focusNodes.map((FocusNode focusNode) {
                return Container(
                  height: 50,
                  color: Colors.green,
                  child: TextField(
                    focusNode: focusNode,
                    style: const TextStyle(fontSize: 24, fontWeight: FontWeight.bold),
                  ),
                );
              }).toList(),
        ),
      ),
    );

    final Finder finder = find.byType(TextField).first;
    final TextField textField = tester.widget(finder);
    await tester.showKeyboard(finder);
    expect(textField.focusNode!.hasFocus, isTrue);

    await tester.drag(finder, const Offset(0.0, -40.0));
    await tester.pumpAndSettle();
    expect(textField.focusNode!.hasFocus, isFalse);
  });

  testWidgets('GridView.builder dismiss keyboard onDrag test', (WidgetTester tester) async {
    final List<FocusNode> focusNodes = List<FocusNode>.generate(50, (int i) => FocusNode());
    addTearDown(() {
      for (final FocusNode node in focusNodes) {
        node.dispose();
      }
    });

    await tester.pumpWidget(
      textFieldBoilerplate(
        child: GridView.builder(
          padding: EdgeInsets.zero,
          gridDelegate: const SliverGridDelegateWithFixedCrossAxisCount(crossAxisCount: 2),
          keyboardDismissBehavior: ScrollViewKeyboardDismissBehavior.onDrag,
          itemCount: focusNodes.length,
          itemBuilder: (BuildContext context, int index) {
            return Container(
              height: 50,
              color: Colors.green,
              child: TextField(
                focusNode: focusNodes[index],
                style: const TextStyle(fontSize: 24, fontWeight: FontWeight.bold),
              ),
            );
          },
        ),
      ),
    );

    final Finder finder = find.byType(TextField).first;
    final TextField textField = tester.widget(finder);
    await tester.showKeyboard(finder);
    expect(textField.focusNode!.hasFocus, isTrue);

    await tester.drag(finder, const Offset(0.0, -40.0));
    await tester.pumpAndSettle();
    expect(textField.focusNode!.hasFocus, isFalse);
  });

  testWidgets('GridView.count dismiss keyboard onDrag test', (WidgetTester tester) async {
    final List<FocusNode> focusNodes = List<FocusNode>.generate(50, (int i) => FocusNode());
    addTearDown(() {
      for (final FocusNode node in focusNodes) {
        node.dispose();
      }
    });

    await tester.pumpWidget(
      textFieldBoilerplate(
        child: GridView.count(
          padding: EdgeInsets.zero,
          crossAxisCount: 2,
          keyboardDismissBehavior: ScrollViewKeyboardDismissBehavior.onDrag,
          children:
              focusNodes.map((FocusNode focusNode) {
                return Container(
                  height: 50,
                  color: Colors.green,
                  child: TextField(
                    focusNode: focusNode,
                    style: const TextStyle(fontSize: 24, fontWeight: FontWeight.bold),
                  ),
                );
              }).toList(),
        ),
      ),
    );

    final Finder finder = find.byType(TextField).first;
    final TextField textField = tester.widget(finder);
    await tester.showKeyboard(finder);
    expect(textField.focusNode!.hasFocus, isTrue);

    await tester.drag(finder, const Offset(0.0, -40.0));
    await tester.pumpAndSettle();
    expect(textField.focusNode!.hasFocus, isFalse);
  });

  testWidgets('GridView.extent dismiss keyboard onDrag test', (WidgetTester tester) async {
    final List<FocusNode> focusNodes = List<FocusNode>.generate(50, (int i) => FocusNode());
    addTearDown(() {
      for (final FocusNode node in focusNodes) {
        node.dispose();
      }
    });

    await tester.pumpWidget(
      textFieldBoilerplate(
        child: GridView.extent(
          padding: EdgeInsets.zero,
          maxCrossAxisExtent: 300,
          keyboardDismissBehavior: ScrollViewKeyboardDismissBehavior.onDrag,
          children:
              focusNodes.map((FocusNode focusNode) {
                return Container(
                  height: 50,
                  color: Colors.green,
                  child: TextField(
                    focusNode: focusNode,
                    style: const TextStyle(fontSize: 24, fontWeight: FontWeight.bold),
                  ),
                );
              }).toList(),
        ),
      ),
    );

    final Finder finder = find.byType(TextField).first;
    final TextField textField = tester.widget(finder);
    await tester.showKeyboard(finder);
    expect(textField.focusNode!.hasFocus, isTrue);

    await tester.drag(finder, const Offset(0.0, -40.0));
    await tester.pumpAndSettle();
    expect(textField.focusNode!.hasFocus, isFalse);
  });

  testWidgets('GridView.custom dismiss keyboard onDrag test', (WidgetTester tester) async {
    final List<FocusNode> focusNodes = List<FocusNode>.generate(50, (int i) => FocusNode());
    addTearDown(() {
      for (final FocusNode node in focusNodes) {
        node.dispose();
      }
    });

    await tester.pumpWidget(
      textFieldBoilerplate(
        child: GridView.custom(
          padding: EdgeInsets.zero,
          gridDelegate: const SliverGridDelegateWithFixedCrossAxisCount(crossAxisCount: 2),
          keyboardDismissBehavior: ScrollViewKeyboardDismissBehavior.onDrag,
          childrenDelegate: SliverChildBuilderDelegate((BuildContext context, int index) {
            return Container(
              height: 50,
              color: Colors.green,
              child: TextField(
                focusNode: focusNodes[index],
                style: const TextStyle(fontSize: 24, fontWeight: FontWeight.bold),
              ),
            );
          }, childCount: focusNodes.length),
        ),
      ),
    );

    final Finder finder = find.byType(TextField).first;
    final TextField textField = tester.widget(finder);
    await tester.showKeyboard(finder);
    expect(textField.focusNode!.hasFocus, isTrue);

    await tester.drag(finder, const Offset(0.0, -40.0));
    await tester.pumpAndSettle();
    expect(textField.focusNode!.hasFocus, isFalse);
  });

  testWidgets('ListView dismiss keyboard manual test', (WidgetTester tester) async {
    final List<FocusNode> focusNodes = List<FocusNode>.generate(50, (int i) => FocusNode());
    addTearDown(() {
      for (final FocusNode node in focusNodes) {
        node.dispose();
      }
    });

    await tester.pumpWidget(
      textFieldBoilerplate(
        child: ListView(
          padding: EdgeInsets.zero,
          children:
              focusNodes.map((FocusNode focusNode) {
                return Container(
                  height: 50,
                  color: Colors.green,
                  child: TextField(
                    focusNode: focusNode,
                    style: const TextStyle(fontSize: 24, fontWeight: FontWeight.bold),
                  ),
                );
              }).toList(),
        ),
      ),
    );

    final Finder finder = find.byType(TextField).first;
    final TextField textField = tester.widget(finder);
    await tester.showKeyboard(finder);
    expect(textField.focusNode!.hasFocus, isTrue);

    await tester.drag(finder, const Offset(0.0, -40.0));
    await tester.pumpAndSettle();
    expect(textField.focusNode!.hasFocus, isTrue);
  });

  testWidgets('ListView.builder dismiss keyboard manual test', (WidgetTester tester) async {
    final List<FocusNode> focusNodes = List<FocusNode>.generate(50, (int i) => FocusNode());
    addTearDown(() {
      for (final FocusNode node in focusNodes) {
        node.dispose();
      }
    });

    await tester.pumpWidget(
      textFieldBoilerplate(
        child: ListView.builder(
          padding: EdgeInsets.zero,
          itemCount: focusNodes.length,
          itemBuilder: (BuildContext context, int index) {
            return Container(
              height: 50,
              color: Colors.green,
              child: TextField(
                focusNode: focusNodes[index],
                style: const TextStyle(fontSize: 24, fontWeight: FontWeight.bold),
              ),
            );
          },
        ),
      ),
    );

    final Finder finder = find.byType(TextField).first;
    final TextField textField = tester.widget(finder);
    await tester.showKeyboard(finder);
    expect(textField.focusNode!.hasFocus, isTrue);

    await tester.drag(finder, const Offset(0.0, -40.0));
    await tester.pumpAndSettle();
    expect(textField.focusNode!.hasFocus, isTrue);
  });

  testWidgets('ListView.custom dismiss keyboard manual test', (WidgetTester tester) async {
    final List<FocusNode> focusNodes = List<FocusNode>.generate(50, (int i) => FocusNode());
    addTearDown(() {
      for (final FocusNode node in focusNodes) {
        node.dispose();
      }
    });

    await tester.pumpWidget(
      textFieldBoilerplate(
        child: ListView.custom(
          padding: EdgeInsets.zero,
          childrenDelegate: SliverChildBuilderDelegate((BuildContext context, int index) {
            return Container(
              height: 50,
              color: Colors.green,
              child: TextField(
                focusNode: focusNodes[index],
                style: const TextStyle(fontSize: 24, fontWeight: FontWeight.bold),
              ),
            );
          }, childCount: focusNodes.length),
        ),
      ),
    );

    final Finder finder = find.byType(TextField).first;
    final TextField textField = tester.widget(finder);
    await tester.showKeyboard(finder);
    expect(textField.focusNode!.hasFocus, isTrue);

    await tester.drag(finder, const Offset(0.0, -40.0));
    await tester.pumpAndSettle();
    expect(textField.focusNode!.hasFocus, isTrue);
  });

  testWidgets('ListView.separated dismiss keyboard manual test', (WidgetTester tester) async {
    final List<FocusNode> focusNodes = List<FocusNode>.generate(50, (int i) => FocusNode());
    addTearDown(() {
      for (final FocusNode node in focusNodes) {
        node.dispose();
      }
    });

    await tester.pumpWidget(
      textFieldBoilerplate(
        child: ListView.separated(
          padding: EdgeInsets.zero,
          itemCount: focusNodes.length,
          separatorBuilder: (BuildContext context, int index) => const Divider(),
          itemBuilder: (BuildContext context, int index) {
            return Container(
              height: 50,
              color: Colors.green,
              child: TextField(
                focusNode: focusNodes[index],
                style: const TextStyle(fontSize: 24, fontWeight: FontWeight.bold),
              ),
            );
          },
        ),
      ),
    );

    final Finder finder = find.byType(TextField).first;
    final TextField textField = tester.widget(finder);
    await tester.showKeyboard(finder);
    expect(textField.focusNode!.hasFocus, isTrue);

    await tester.drag(finder, const Offset(0.0, -40.0));
    await tester.pumpAndSettle();
    expect(textField.focusNode!.hasFocus, isTrue);
  });

  testWidgets('GridView dismiss keyboard manual test', (WidgetTester tester) async {
    final List<FocusNode> focusNodes = List<FocusNode>.generate(50, (int i) => FocusNode());
    addTearDown(() {
      for (final FocusNode node in focusNodes) {
        node.dispose();
      }
    });

    await tester.pumpWidget(
      textFieldBoilerplate(
        child: GridView(
          padding: EdgeInsets.zero,
          gridDelegate: const SliverGridDelegateWithFixedCrossAxisCount(crossAxisCount: 2),
          children:
              focusNodes.map((FocusNode focusNode) {
                return Container(
                  height: 50,
                  color: Colors.green,
                  child: TextField(
                    focusNode: focusNode,
                    style: const TextStyle(fontSize: 24, fontWeight: FontWeight.bold),
                  ),
                );
              }).toList(),
        ),
      ),
    );

    final Finder finder = find.byType(TextField).first;
    final TextField textField = tester.widget(finder);
    await tester.showKeyboard(finder);
    expect(textField.focusNode!.hasFocus, isTrue);

    await tester.drag(finder, const Offset(0.0, -40.0));
    await tester.pumpAndSettle();
    expect(textField.focusNode!.hasFocus, isTrue);
  });

  testWidgets('GridView.builder dismiss keyboard manual test', (WidgetTester tester) async {
    final List<FocusNode> focusNodes = List<FocusNode>.generate(50, (int i) => FocusNode());
    addTearDown(() {
      for (final FocusNode node in focusNodes) {
        node.dispose();
      }
    });

    await tester.pumpWidget(
      textFieldBoilerplate(
        child: GridView.builder(
          padding: EdgeInsets.zero,
          gridDelegate: const SliverGridDelegateWithFixedCrossAxisCount(crossAxisCount: 2),
          itemCount: focusNodes.length,
          itemBuilder: (BuildContext context, int index) {
            return Container(
              height: 50,
              color: Colors.green,
              child: TextField(
                focusNode: focusNodes[index],
                style: const TextStyle(fontSize: 24, fontWeight: FontWeight.bold),
              ),
            );
          },
        ),
      ),
    );

    final Finder finder = find.byType(TextField).first;
    final TextField textField = tester.widget(finder);
    await tester.showKeyboard(finder);
    expect(textField.focusNode!.hasFocus, isTrue);

    await tester.drag(finder, const Offset(0.0, -40.0));
    await tester.pumpAndSettle();
    expect(textField.focusNode!.hasFocus, isTrue);
  });

  testWidgets('GridView.count dismiss keyboard manual test', (WidgetTester tester) async {
    final List<FocusNode> focusNodes = List<FocusNode>.generate(50, (int i) => FocusNode());
    addTearDown(() {
      for (final FocusNode node in focusNodes) {
        node.dispose();
      }
    });

    await tester.pumpWidget(
      textFieldBoilerplate(
        child: GridView.count(
          padding: EdgeInsets.zero,
          crossAxisCount: 2,
          children:
              focusNodes.map((FocusNode focusNode) {
                return Container(
                  height: 50,
                  color: Colors.green,
                  child: TextField(
                    focusNode: focusNode,
                    style: const TextStyle(fontSize: 24, fontWeight: FontWeight.bold),
                  ),
                );
              }).toList(),
        ),
      ),
    );

    final Finder finder = find.byType(TextField).first;
    final TextField textField = tester.widget(finder);
    await tester.showKeyboard(finder);
    expect(textField.focusNode!.hasFocus, isTrue);

    await tester.drag(finder, const Offset(0.0, -40.0));
    await tester.pumpAndSettle();
    expect(textField.focusNode!.hasFocus, isTrue);
  });

  testWidgets('GridView.extent dismiss keyboard manual test', (WidgetTester tester) async {
    final List<FocusNode> focusNodes = List<FocusNode>.generate(50, (int i) => FocusNode());
    addTearDown(() {
      for (final FocusNode node in focusNodes) {
        node.dispose();
      }
    });

    await tester.pumpWidget(
      textFieldBoilerplate(
        child: GridView.extent(
          padding: EdgeInsets.zero,
          maxCrossAxisExtent: 300,
          children:
              focusNodes.map((FocusNode focusNode) {
                return Container(
                  height: 50,
                  color: Colors.green,
                  child: TextField(
                    focusNode: focusNode,
                    style: const TextStyle(fontSize: 24, fontWeight: FontWeight.bold),
                  ),
                );
              }).toList(),
        ),
      ),
    );

    final Finder finder = find.byType(TextField).first;
    final TextField textField = tester.widget(finder);
    await tester.showKeyboard(finder);
    expect(textField.focusNode!.hasFocus, isTrue);

    await tester.drag(finder, const Offset(0.0, -40.0));
    await tester.pumpAndSettle();
    expect(textField.focusNode!.hasFocus, isTrue);
  });

  testWidgets('GridView.custom dismiss keyboard manual test', (WidgetTester tester) async {
    final List<FocusNode> focusNodes = List<FocusNode>.generate(50, (int i) => FocusNode());
    addTearDown(() {
      for (final FocusNode node in focusNodes) {
        node.dispose();
      }
    });

    await tester.pumpWidget(
      textFieldBoilerplate(
        child: GridView.custom(
          padding: EdgeInsets.zero,
          gridDelegate: const SliverGridDelegateWithFixedCrossAxisCount(crossAxisCount: 2),
          childrenDelegate: SliverChildBuilderDelegate((BuildContext context, int index) {
            return Container(
              height: 50,
              color: Colors.green,
              child: TextField(
                focusNode: focusNodes[index],
                style: const TextStyle(fontSize: 24, fontWeight: FontWeight.bold),
              ),
            );
          }, childCount: focusNodes.length),
        ),
      ),
    );

    final Finder finder = find.byType(TextField).first;
    final TextField textField = tester.widget(finder);
    await tester.showKeyboard(finder);
    expect(textField.focusNode!.hasFocus, isTrue);

    await tester.drag(finder, const Offset(0.0, -40.0));
    await tester.pumpAndSettle();
    expect(textField.focusNode!.hasFocus, isTrue);
  });

  testWidgets('ListView restart ballistic activity out of range', (WidgetTester tester) async {
    Widget buildListView(int n) {
      return Directionality(
        textDirection: TextDirection.ltr,
        child: ListView(
          dragStartBehavior: DragStartBehavior.down,
          children:
              kStates.take(n).map<Widget>((String state) {
                return Container(height: 200.0, color: const Color(0xFF0000FF), child: Text(state));
              }).toList(),
        ),
      );
    }

    await tester.pumpWidget(buildListView(30));
    await tester.fling(find.byType(ListView), const Offset(0.0, -4000.0), 4000.0);
    await tester.pumpWidget(buildListView(15));
    await tester.pump(const Duration(milliseconds: 10));
    await tester.pump(const Duration(milliseconds: 10));
    await tester.pump(const Duration(milliseconds: 10));
    await tester.pump(const Duration(milliseconds: 10));
    await tester.pumpAndSettle();

    final Viewport viewport = tester.widget(find.byType(Viewport));
    expect(viewport.offset.pixels, equals(2400.0));
  });

  testWidgets('CustomScrollView control test', (WidgetTester tester) async {
    final List<String> log = <String>[];

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: CustomScrollView(
          dragStartBehavior: DragStartBehavior.down,
          slivers: <Widget>[
            SliverList(
              delegate: SliverChildListDelegate(
                kStates.map<Widget>((String state) {
                  return GestureDetector(
                    dragStartBehavior: DragStartBehavior.down,
                    onTap: () {
                      log.add(state);
                    },
                    child: Container(
                      height: 200.0,
                      color: const Color(0xFF0000FF),
                      child: Text(state),
                    ),
                  );
                }).toList(),
              ),
            ),
          ],
        ),
      ),
    );

    await tester.tap(find.text('Alabama'));
    expect(log, equals(<String>['Alabama']));
    log.clear();

    expect(find.text('Nevada'), findsNothing);

    await tester.drag(find.text('Alabama'), const Offset(0.0, -4000.0));
    await tester.pump();

    expect(find.text('Alabama'), findsNothing);
    expect(tester.getCenter(find.text('Massachusetts')), equals(const Offset(400.0, 100.0)));

    await tester.tap(find.text('Massachusetts'));
    expect(log, equals(<String>['Massachusetts']));
    log.clear();
  });

  testWidgets('CustomScrollView dismiss keyboard onDrag test', (WidgetTester tester) async {
    final List<FocusNode> focusNodes = List<FocusNode>.generate(50, (int i) => FocusNode());
    addTearDown(() {
      for (final FocusNode node in focusNodes) {
        node.dispose();
      }
    });

    await tester.pumpWidget(
      textFieldBoilerplate(
        child: CustomScrollView(
          dragStartBehavior: DragStartBehavior.down,
          keyboardDismissBehavior: ScrollViewKeyboardDismissBehavior.onDrag,
          slivers: <Widget>[
            SliverList(
              delegate: SliverChildListDelegate(
                focusNodes.map((FocusNode focusNode) {
                  return Container(
                    height: 50,
                    color: Colors.green,
                    child: TextField(
                      focusNode: focusNode,
                      style: const TextStyle(fontSize: 24, fontWeight: FontWeight.bold),
                    ),
                  );
                }).toList(),
              ),
            ),
          ],
        ),
      ),
    );

    final Finder finder = find.byType(TextField).first;
    final TextField textField = tester.widget(finder);
    await tester.showKeyboard(finder);
    expect(textField.focusNode!.hasFocus, isTrue);

    await tester.drag(finder, const Offset(0.0, -40.0));
    await tester.pumpAndSettle();
    expect(textField.focusNode!.hasFocus, isFalse);
  });

  testWidgets('Can jumpTo during drag', (WidgetTester tester) async {
    final List<Type> log = <Type>[];
    final ScrollController controller = ScrollController();
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: NotificationListener<ScrollNotification>(
          onNotification: (ScrollNotification notification) {
            log.add(notification.runtimeType);
            return false;
          },
          child: ListView(
            controller: controller,
            children:
                kStates.map<Widget>((String state) {
                  return SizedBox(height: 200.0, child: Text(state));
                }).toList(),
          ),
        ),
      ),
    );

    expect(log, isEmpty);

    final TestGesture gesture = await tester.startGesture(const Offset(100.0, 100.0));
    await gesture.moveBy(const Offset(0.0, -100.0));

    expect(
      log,
      equals(<Type>[ScrollStartNotification, UserScrollNotification, ScrollUpdateNotification]),
    );
    log.clear();

    await tester.pump();

    controller.jumpTo(550.0);

    expect(controller.offset, equals(550.0));
    expect(
      log,
      equals(<Type>[
        ScrollEndNotification,
        UserScrollNotification,
        ScrollStartNotification,
        ScrollUpdateNotification,
        ScrollEndNotification,
      ]),
    );
    log.clear();

    await tester.pump();
    await gesture.moveBy(const Offset(0.0, -100.0));

    expect(controller.offset, equals(550.0));
    expect(log, isEmpty);
  });

  test(
    'PrimaryScrollController.automaticallyInheritOnPlatforms defaults to all mobile platforms',
    () {
      final ScrollController controller = ScrollController();
      addTearDown(controller.dispose);
      final PrimaryScrollController primaryScrollController = PrimaryScrollController(
        controller: controller,
        child: const SizedBox(),
      );
      expect(
        primaryScrollController.automaticallyInheritForPlatforms,
        TargetPlatformVariant.mobile().values,
      );
    },
  );

  testWidgets('Vertical CustomScrollViews are not primary by default', (WidgetTester tester) async {
    const CustomScrollView view = CustomScrollView();
    expect(view.primary, isNull);
  });

  testWidgets(
    'Vertical CustomScrollViews use PrimaryScrollController by default on mobile',
    (WidgetTester tester) async {
      final ScrollController controller = ScrollController();
      addTearDown(controller.dispose);
      await tester.pumpWidget(
        primaryScrollControllerBoilerplate(child: const CustomScrollView(), controller: controller),
      );
      expect(controller.hasClients, isTrue);
    },
    variant: TargetPlatformVariant.mobile(),
  );

  testWidgets(
    "Vertical CustomScrollViews don't use PrimaryScrollController by default on desktop",
    (WidgetTester tester) async {
      final ScrollController controller = ScrollController();
      addTearDown(controller.dispose);
      await tester.pumpWidget(
        primaryScrollControllerBoilerplate(child: const CustomScrollView(), controller: controller),
      );
      expect(controller.hasClients, isFalse);
    },
    variant: TargetPlatformVariant.desktop(),
  );

  testWidgets('Vertical ListViews are not primary by default', (WidgetTester tester) async {
    final ListView view = ListView();
    expect(view.primary, isNull);
  });

  testWidgets(
    'Vertical ListViews use PrimaryScrollController by default on mobile',
    (WidgetTester tester) async {
      final ScrollController controller = ScrollController();
      addTearDown(controller.dispose);
      await tester.pumpWidget(
        primaryScrollControllerBoilerplate(child: ListView(), controller: controller),
      );
      expect(controller.hasClients, isTrue);
    },
    variant: TargetPlatformVariant.mobile(),
  );

  testWidgets(
    "Vertical ListViews don't use PrimaryScrollController by default on desktop",
    (WidgetTester tester) async {
      final ScrollController controller = ScrollController();
      addTearDown(controller.dispose);
      await tester.pumpWidget(
        primaryScrollControllerBoilerplate(child: ListView(), controller: controller),
      );
      expect(controller.hasClients, isFalse);
    },
    variant: TargetPlatformVariant.desktop(),
  );

  testWidgets('Vertical GridViews are not primary by default', (WidgetTester tester) async {
    final GridView view = GridView.count(crossAxisCount: 1);
    expect(view.primary, isNull);
  });

  testWidgets(
    'Vertical GridViews use PrimaryScrollController by default on mobile',
    (WidgetTester tester) async {
      final ScrollController controller = ScrollController();
      addTearDown(controller.dispose);
      await tester.pumpWidget(
        primaryScrollControllerBoilerplate(
          child: GridView.count(crossAxisCount: 1),
          controller: controller,
        ),
      );
      expect(controller.hasClients, isTrue);
    },
    variant: TargetPlatformVariant.mobile(),
  );

  testWidgets(
    "Vertical GridViews don't use PrimaryScrollController by default on desktop",
    (WidgetTester tester) async {
      final ScrollController controller = ScrollController();
      addTearDown(controller.dispose);
      await tester.pumpWidget(
        primaryScrollControllerBoilerplate(
          child: GridView.count(crossAxisCount: 1),
          controller: controller,
        ),
      );
      expect(controller.hasClients, isFalse);
    },
    variant: TargetPlatformVariant.desktop(),
  );

  testWidgets('Horizontal CustomScrollViews are non-primary by default', (
    WidgetTester tester,
  ) async {
    final ScrollController controller1 = ScrollController();
    addTearDown(controller1.dispose);
    final ScrollController controller2 = ScrollController();
    addTearDown(controller2.dispose);
    await tester.pumpWidget(
      primaryScrollControllerBoilerplate(
        child: CustomScrollView(scrollDirection: Axis.horizontal, controller: controller2),
        controller: controller1,
      ),
    );
    expect(controller1.hasClients, isFalse);
  });

  testWidgets('Horizontal ListViews are non-primary by default', (WidgetTester tester) async {
    final ScrollController controller1 = ScrollController();
    addTearDown(controller1.dispose);
    final ScrollController controller2 = ScrollController();
    addTearDown(controller2.dispose);
    await tester.pumpWidget(
      primaryScrollControllerBoilerplate(
        child: ListView(scrollDirection: Axis.horizontal, controller: controller2),
        controller: controller1,
      ),
    );
    expect(controller1.hasClients, isFalse);
  });

  testWidgets('Horizontal GridViews are non-primary by default', (WidgetTester tester) async {
    final ScrollController controller1 = ScrollController();
    addTearDown(controller1.dispose);
    final ScrollController controller2 = ScrollController();
    addTearDown(controller2.dispose);
    await tester.pumpWidget(
      primaryScrollControllerBoilerplate(
        child: GridView.count(
          scrollDirection: Axis.horizontal,
          controller: controller2,
          crossAxisCount: 1,
        ),
        controller: controller1,
      ),
    );
    expect(controller1.hasClients, isFalse);
  });

  testWidgets('CustomScrollViews with controllers are non-primary by default', (
    WidgetTester tester,
  ) async {
    final ScrollController controller1 = ScrollController();
    addTearDown(controller1.dispose);
    final ScrollController controller2 = ScrollController();
    addTearDown(controller2.dispose);
    await tester.pumpWidget(
      primaryScrollControllerBoilerplate(
        child: CustomScrollView(controller: controller2),
        controller: controller1,
      ),
    );
    expect(controller1.hasClients, isFalse);
  });

  testWidgets('ListViews with controllers are non-primary by default', (WidgetTester tester) async {
    final ScrollController controller1 = ScrollController();
    addTearDown(controller1.dispose);
    final ScrollController controller2 = ScrollController();
    addTearDown(controller2.dispose);
    await tester.pumpWidget(
      primaryScrollControllerBoilerplate(
        child: ListView(controller: controller2),
        controller: controller1,
      ),
    );
    expect(controller1.hasClients, isFalse);
  });

  testWidgets('GridViews with controllers are non-primary by default', (WidgetTester tester) async {
    final ScrollController controller1 = ScrollController();
    addTearDown(controller1.dispose);
    final ScrollController controller2 = ScrollController();
    addTearDown(controller2.dispose);
    await tester.pumpWidget(
      primaryScrollControllerBoilerplate(
        child: GridView.count(controller: controller2, crossAxisCount: 1),
        controller: controller1,
      ),
    );
    expect(controller1.hasClients, isFalse);
  });

  testWidgets('CustomScrollView sets PrimaryScrollController when primary', (
    WidgetTester tester,
  ) async {
    final ScrollController primaryScrollController = ScrollController();
    addTearDown(primaryScrollController.dispose);
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: PrimaryScrollController(
          controller: primaryScrollController,
          child: const CustomScrollView(primary: true),
        ),
      ),
    );
    final Scrollable scrollable = tester.widget(find.byType(Scrollable));
    expect(scrollable.controller, primaryScrollController);
  });

  testWidgets('ListView sets PrimaryScrollController when primary', (WidgetTester tester) async {
    final ScrollController primaryScrollController = ScrollController();
    addTearDown(primaryScrollController.dispose);
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: PrimaryScrollController(
          controller: primaryScrollController,
          child: ListView(primary: true),
        ),
      ),
    );
    final Scrollable scrollable = tester.widget(find.byType(Scrollable));
    expect(scrollable.controller, primaryScrollController);
  });

  testWidgets('GridView sets PrimaryScrollController when primary', (WidgetTester tester) async {
    final ScrollController primaryScrollController = ScrollController();
    addTearDown(primaryScrollController.dispose);
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: PrimaryScrollController(
          controller: primaryScrollController,
          child: GridView.count(primary: true, crossAxisCount: 1),
        ),
      ),
    );
    final Scrollable scrollable = tester.widget(find.byType(Scrollable));
    expect(scrollable.controller, primaryScrollController);
  });

  testWidgets('Nested scrollables have a null PrimaryScrollController', (
    WidgetTester tester,
  ) async {
    const Key innerKey = Key('inner');
    final ScrollController primaryScrollController = ScrollController();
    addTearDown(primaryScrollController.dispose);
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: PrimaryScrollController(
          controller: primaryScrollController,
          child: ListView(
            primary: true,
            children: <Widget>[
              Container(
                constraints: const BoxConstraints(maxHeight: 200.0),
                child: ListView(key: innerKey, primary: true),
              ),
            ],
          ),
        ),
      ),
    );

    final Scrollable innerScrollable = tester.widget(
      find.descendant(of: find.byKey(innerKey), matching: find.byType(Scrollable)),
    );
    expect(innerScrollable.controller, isNull);
  });

  testWidgets('Primary ListViews are always scrollable', (WidgetTester tester) async {
    final ListView view = ListView(primary: true);
    expect(view.physics, isA<AlwaysScrollableScrollPhysics>());
  });

  testWidgets('Non-primary ListViews are not always scrollable', (WidgetTester tester) async {
    final ListView view = ListView(primary: false);
    expect(view.physics, isNot(isA<AlwaysScrollableScrollPhysics>()));
  });

  testWidgets('Defaulting-to-primary ListViews are always scrollable', (WidgetTester tester) async {
    final ListView view = ListView();
    expect(view.physics, isA<AlwaysScrollableScrollPhysics>());
  });

  testWidgets('Defaulting-to-not-primary ListViews are not always scrollable', (
    WidgetTester tester,
  ) async {
    final ListView view = ListView(scrollDirection: Axis.horizontal);
    expect(view.physics, isNot(isA<AlwaysScrollableScrollPhysics>()));
  });

  testWidgets('primary:true leads to scrolling', (WidgetTester tester) async {
    bool scrolled = false;
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: NotificationListener<OverscrollNotification>(
          onNotification: (OverscrollNotification message) {
            scrolled = true;
            return false;
          },
          child: ListView(primary: true),
        ),
      ),
    );
    await tester.dragFrom(const Offset(100.0, 100.0), const Offset(0.0, 100.0));
    expect(scrolled, isTrue);
  });

  testWidgets('primary:false leads to no scrolling', (WidgetTester tester) async {
    bool scrolled = false;
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: NotificationListener<OverscrollNotification>(
          onNotification: (OverscrollNotification message) {
            scrolled = true;
            return false;
          },
          child: ListView(primary: false),
        ),
      ),
    );
    await tester.dragFrom(const Offset(100.0, 100.0), const Offset(0.0, 100.0));
    expect(scrolled, isFalse);
  });

  testWidgets(
    'physics:AlwaysScrollableScrollPhysics actually overrides primary:false default behavior',
    (WidgetTester tester) async {
      bool scrolled = false;
      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: NotificationListener<OverscrollNotification>(
            onNotification: (OverscrollNotification message) {
              scrolled = true;
              return false;
            },
            child: ListView(primary: false, physics: const AlwaysScrollableScrollPhysics()),
          ),
        ),
      );
      await tester.dragFrom(const Offset(100.0, 100.0), const Offset(0.0, 100.0));
      expect(scrolled, isTrue);
    },
  );

  testWidgets('physics:ScrollPhysics actually overrides primary:true default behavior', (
    WidgetTester tester,
  ) async {
    bool scrolled = false;
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: NotificationListener<OverscrollNotification>(
          onNotification: (OverscrollNotification message) {
            scrolled = true;
            return false;
          },
          child: ListView(primary: true, physics: const ScrollPhysics()),
        ),
      ),
    );
    await tester.dragFrom(const Offset(100.0, 100.0), const Offset(0.0, 100.0));
    expect(scrolled, isFalse);
  });

  testWidgets('separatorBuilder must return something', (WidgetTester tester) async {
    const List<String> listOfValues = <String>['ALPHA', 'BETA', 'GAMMA', 'DELTA'];

    Widget buildFrame(Widget firstSeparator) {
      return MaterialApp(
        home: Material(
          child: ListView.separated(
            itemBuilder: (BuildContext context, int index) {
              return Text(listOfValues[index]);
            },
            separatorBuilder: (BuildContext context, int index) {
              if (index == 0) {
                return firstSeparator;
              } else {
                return const Divider();
              }
            },
            itemCount: listOfValues.length,
          ),
        ),
      );
    }

    // A separatorBuilder that always returns a Divider is fine
    await tester.pumpWidget(buildFrame(const Divider()));
    expect(tester.takeException(), isNull);
  });

  testWidgets('when itemBuilder throws, creates Error Widget', (WidgetTester tester) async {
    const List<String> listOfValues = <String>['ALPHA', 'BETA', 'GAMMA', 'DELTA'];

    Widget buildFrame(bool throwOnFirstItem) {
      return MaterialApp(
        home: Material(
          child: ListView.builder(
            itemBuilder: (BuildContext context, int index) {
              if (index == 0 && throwOnFirstItem) {
                throw Exception('itemBuilder fail');
              }
              return Text(listOfValues[index]);
            },
            itemCount: listOfValues.length,
          ),
        ),
      );
    }

    // When itemBuilder doesn't throw, no ErrorWidget
    await tester.pumpWidget(buildFrame(false));
    expect(tester.takeException(), isNull);
    final Finder finder = find.byType(ErrorWidget);
    expect(find.byType(ErrorWidget), findsNothing);

    // When it does throw, one error widget is rendered in the item's place
    await tester.pumpWidget(buildFrame(true));
    expect(tester.takeException(), isA<Exception>());
    expect(finder, findsOneWidget);
  });

  testWidgets('when separatorBuilder throws, creates ErrorWidget', (WidgetTester tester) async {
    const List<String> listOfValues = <String>['ALPHA', 'BETA', 'GAMMA', 'DELTA'];
    const Key key = Key('list');

    Widget buildFrame(bool throwOnFirstSeparator) {
      return MaterialApp(
        home: Material(
          child: ListView.separated(
            key: key,
            itemBuilder: (BuildContext context, int index) {
              return Text(listOfValues[index]);
            },
            separatorBuilder: (BuildContext context, int index) {
              if (index == 0 && throwOnFirstSeparator) {
                throw Exception('separatorBuilder fail');
              }
              return const Divider();
            },
            itemCount: listOfValues.length,
          ),
        ),
      );
    }

    // When separatorBuilder doesn't throw, no ErrorWidget
    await tester.pumpWidget(buildFrame(false));
    expect(tester.takeException(), isNull);
    final Finder finder = find.byType(ErrorWidget);
    expect(find.byType(ErrorWidget), findsNothing);

    // When it does throw, one error widget is rendered in the separator's place
    await tester.pumpWidget(buildFrame(true));
    expect(tester.takeException(), isA<Exception>());
    expect(finder, findsOneWidget);
  });

  testWidgets('ListView asserts on both non-null itemExtent and prototypeItem', (
    WidgetTester tester,
  ) async {
    expect(() => ListView(itemExtent: 100, prototypeItem: const SizedBox()), throwsAssertionError);
  });

  testWidgets('ListView.builder asserts on negative childCount', (WidgetTester tester) async {
    expect(
      () => ListView.builder(
        itemBuilder: (BuildContext context, int index) {
          return const SizedBox();
        },
        itemCount: -1,
      ),
      throwsAssertionError,
    );
  });

  testWidgets('ListView.builder asserts on negative semanticChildCount', (
    WidgetTester tester,
  ) async {
    expect(
      () => ListView.builder(
        itemBuilder: (BuildContext context, int index) {
          return const SizedBox();
        },
        itemCount: 1,
        semanticChildCount: -1,
      ),
      throwsAssertionError,
    );
  });

  testWidgets('ListView.builder asserts on nonsensical childCount/semanticChildCount', (
    WidgetTester tester,
  ) async {
    expect(
      () => ListView.builder(
        itemBuilder: (BuildContext context, int index) {
          return const SizedBox();
        },
        itemCount: 1,
        semanticChildCount: 4,
      ),
      throwsAssertionError,
    );
  });

  testWidgets('ListView.builder asserts on both non-null itemExtent and prototypeItem', (
    WidgetTester tester,
  ) async {
    expect(
      () => ListView.builder(
        itemBuilder: (BuildContext context, int index) {
          return const SizedBox();
        },
        itemExtent: 100,
        prototypeItem: const SizedBox(),
      ),
      throwsAssertionError,
    );
  });

  testWidgets('ListView.custom asserts on both non-null itemExtent and prototypeItem', (
    WidgetTester tester,
  ) async {
    expect(
      () => ListView.custom(
        childrenDelegate: SliverChildBuilderDelegate((BuildContext context, int index) {
          return const SizedBox();
        }),
        itemExtent: 100,
        prototypeItem: const SizedBox(),
      ),
      throwsAssertionError,
    );
  });

  testWidgets('PrimaryScrollController provides fallback ScrollActions', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        home: CustomScrollView(
          primary: true,
          slivers: List<Widget>.generate(20, (int index) {
            return SliverToBoxAdapter(
              child: Focus(
                autofocus: index == 0,
                child: SizedBox(key: ValueKey<String>('Box $index'), height: 50.0),
              ),
            );
          }),
        ),
      ),
    );
    final ScrollController controller = PrimaryScrollController.of(
      tester.element(find.byType(CustomScrollView)),
    );
    await tester.pumpAndSettle();
    expect(controller.position.pixels, equals(0.0));
    expect(
      tester.getRect(find.byKey(const ValueKey<String>('Box 0'), skipOffstage: false)),
      equals(const Rect.fromLTRB(0.0, 0.0, 800.0, 50.0)),
    );
    await tester.sendKeyEvent(LogicalKeyboardKey.pageDown);
    await tester.pumpAndSettle();
    expect(controller.position.pixels, equals(400.0));
    expect(
      tester.getRect(find.byKey(const ValueKey<String>('Box 0'), skipOffstage: false)),
      equals(const Rect.fromLTRB(0.0, -400.0, 800.0, -350.0)),
    );
    await tester.sendKeyEvent(LogicalKeyboardKey.pageUp);
    await tester.pumpAndSettle();
    expect(controller.position.pixels, equals(0.0));
    expect(
      tester.getRect(find.byKey(const ValueKey<String>('Box 0'), skipOffstage: false)),
      equals(const Rect.fromLTRB(0.0, 0.0, 800.0, 50.0)),
    );
  });

  testWidgets('Fallback ScrollActions handle too many positions with error message', (
    WidgetTester tester,
  ) async {
    Widget getScrollView() {
      return SizedBox(
        width: 400.0,
        child: CustomScrollView(
          primary: true,
          slivers: List<Widget>.generate(20, (int index) {
            return SliverToBoxAdapter(
              child: Focus(child: SizedBox(key: ValueKey<String>('Box $index'), height: 50.0)),
            );
          }),
        ),
      );
    }

    await tester.pumpWidget(
      MaterialApp(home: Row(children: <Widget>[getScrollView(), getScrollView()])),
    );
    await tester.pumpAndSettle();
    expect(
      tester.getRect(find.byKey(const ValueKey<String>('Box 0'), skipOffstage: false).first),
      equals(const Rect.fromLTRB(0.0, 0.0, 400.0, 50.0)),
    );
    await tester.sendKeyEvent(LogicalKeyboardKey.pageDown);
    final AssertionError exception = tester.takeException() as AssertionError;
    expect(exception, isAssertionError);
    expect(
      exception.message,
      contains(
        'A ScrollAction was invoked with the PrimaryScrollController, but '
        'more than one ScrollPosition is attached.',
      ),
    );
  });

  testWidgets('if itemExtent is non-null, children have same extent in the scroll direction', (
    WidgetTester tester,
  ) async {
    final List<int> numbers = <int>[0, 1, 2];

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return ListView.builder(
                itemBuilder: (BuildContext context, int index) {
                  return SizedBox(
                    key: ValueKey<int>(numbers[index]),
                    // children with different heights
                    height: 20 + numbers[index] * 10,
                    child: ReorderableDragStartListener(
                      index: index,
                      child: Text(numbers[index].toString()),
                    ),
                  );
                },
                itemCount: numbers.length,
                itemExtent: 30,
              );
            },
          ),
        ),
      ),
    );

    final double item0Height = tester.getSize(find.text('0').hitTestable()).height;
    final double item1Height = tester.getSize(find.text('1').hitTestable()).height;
    final double item2Height = tester.getSize(find.text('2').hitTestable()).height;

    expect(item0Height, 30.0);
    expect(item1Height, 30.0);
    expect(item2Height, 30.0);
  });

  testWidgets('if prototypeItem is non-null, children have same extent in the scroll direction', (
    WidgetTester tester,
  ) async {
    final List<int> numbers = <int>[0, 1, 2];

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return ListView.builder(
                itemBuilder: (BuildContext context, int index) {
                  return SizedBox(
                    key: ValueKey<int>(numbers[index]),
                    // children with different heights
                    height: 20 + numbers[index] * 10,
                    child: ReorderableDragStartListener(
                      index: index,
                      child: Text(numbers[index].toString()),
                    ),
                  );
                },
                itemCount: numbers.length,
                prototypeItem: const SizedBox(height: 30, child: Text('3')),
              );
            },
          ),
        ),
      ),
    );

    final double item0Height = tester.getSize(find.text('0').hitTestable()).height;
    final double item1Height = tester.getSize(find.text('1').hitTestable()).height;
    final double item2Height = tester.getSize(find.text('2').hitTestable()).height;

    expect(item0Height, 30.0);
    expect(item1Height, 30.0);
    expect(item2Height, 30.0);
  });

  testWidgets('ListView dismiss keyboard onDrag and keep dismissed on drawer opened test', (
    WidgetTester tester,
  ) async {
    final List<int> list = List<int>.generate(50, (int i) => i);
    final GlobalKey<ScaffoldState> scaffoldKey = GlobalKey<ScaffoldState>();

    await tester.pumpWidget(
      textFieldBoilerplate(
        child: Scaffold(
          key: scaffoldKey,
          drawer: Container(),
          body: Column(
            children: <Widget>[
              const TextField(),
              Expanded(
                child: ListView(
                  keyboardDismissBehavior: ScrollViewKeyboardDismissBehavior.onDrag,
                  children:
                      list.map((int i) {
                        return Container(height: 50);
                      }).toList(),
                ),
              ),
            ],
          ),
        ),
      ),
    );

    expect(tester.testTextInput.isVisible, isFalse);
    final Finder finder = find.byType(TextField).first;
    await tester.tap(finder);
    expect(tester.testTextInput.isVisible, isTrue);

    await tester.drag(find.byType(ListView).first, const Offset(0.0, -40.0));
    await tester.pumpAndSettle();

    expect(tester.testTextInput.isVisible, isFalse);
    scaffoldKey.currentState!.openDrawer();
    await tester.pumpAndSettle();

    expect(tester.testTextInput.isVisible, isFalse);
  });
}
