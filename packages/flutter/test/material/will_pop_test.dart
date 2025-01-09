// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

bool willPopValue = false;

class SamplePage extends StatefulWidget {
  const SamplePage({super.key});
  @override
  SamplePageState createState() => SamplePageState();
}

class SamplePageState extends State<SamplePage> {
  ModalRoute<void>? _route;

  Future<bool> _callback() async => willPopValue;

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    _route?.removeScopedWillPopCallback(_callback);
    _route = ModalRoute.of(context);
    _route?.addScopedWillPopCallback(_callback);
  }

  @override
  void dispose() {
    super.dispose();
    _route?.removeScopedWillPopCallback(_callback);
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(appBar: AppBar(title: const Text('Sample Page')));
  }
}

int willPopCount = 0;

class SampleForm extends StatelessWidget {
  const SampleForm({super.key, required this.callback});

  final WillPopCallback callback;

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('Sample Form')),
      body: SizedBox.expand(
        child: Form(
          onWillPop: () {
            willPopCount += 1;
            return callback();
          },
          child: const TextField(),
        ),
      ),
    );
  }
}

// Expose the protected hasScopedWillPopCallback getter
class _TestPageRoute<T> extends MaterialPageRoute<T> {
  _TestPageRoute({super.settings, required super.builder}) : super(maintainState: true);

  bool get hasCallback => super.hasScopedWillPopCallback;
}

class _TestPage extends Page<dynamic> {
  _TestPage({required this.builder, required LocalKey key}) : _key = GlobalKey(), super(key: key);

  final WidgetBuilder builder;
  final GlobalKey<dynamic> _key;

  @override
  Route<dynamic> createRoute(BuildContext context) {
    return _TestPageRoute<dynamic>(
      settings: this,
      builder: (BuildContext context) {
        // keep state during move to another location in tree
        return KeyedSubtree(key: _key, child: builder.call(context));
      },
    );
  }
}

void main() {
  testWidgets('ModalRoute scopedWillPopupCallback can inhibit back button', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          appBar: AppBar(title: const Text('Home')),
          body: Builder(
            builder: (BuildContext context) {
              return Center(
                child: TextButton(
                  child: const Text('X'),
                  onPressed: () {
                    showDialog<void>(
                      context: context,
                      builder: (BuildContext context) => const SamplePage(),
                    );
                  },
                ),
              );
            },
          ),
        ),
      ),
    );

    expect(find.byTooltip('Back'), findsNothing);
    expect(find.text('Sample Page'), findsNothing);

    await tester.tap(find.text('X'));
    await tester.pump();
    await tester.pump(const Duration(seconds: 1));

    expect(find.text('Sample Page'), findsOneWidget);

    willPopValue = false;
    await tester.tap(find.byTooltip('Back'));
    await tester.pump();
    await tester.pump();
    await tester.pump(const Duration(seconds: 1));
    expect(find.text('Sample Page'), findsOneWidget);

    // Use didPopRoute() to simulate the system back button. Check that
    // didPopRoute() indicates that the notification was handled.
    final dynamic widgetsAppState = tester.state(find.byType(WidgetsApp));
    // ignore: avoid_dynamic_calls
    expect(await widgetsAppState.didPopRoute(), isTrue);
    expect(find.text('Sample Page'), findsOneWidget);

    willPopValue = true;
    await tester.tap(find.byTooltip('Back'));
    await tester.pump();
    await tester.pump();
    await tester.pump(const Duration(seconds: 1));
    expect(find.text('Sample Page'), findsNothing);
  });

  testWidgets('willPop will only pop if the callback returns true', (WidgetTester tester) async {
    Widget buildFrame() {
      return MaterialApp(
        home: Scaffold(
          appBar: AppBar(title: const Text('Home')),
          body: Builder(
            builder: (BuildContext context) {
              return Center(
                child: TextButton(
                  child: const Text('X'),
                  onPressed: () {
                    Navigator.of(context).push(
                      MaterialPageRoute<void>(
                        builder: (BuildContext context) {
                          return SampleForm(callback: () => Future<bool>.value(willPopValue));
                        },
                      ),
                    );
                  },
                ),
              );
            },
          ),
        ),
      );
    }

    await tester.pumpWidget(buildFrame());
    await tester.tap(find.text('X'));
    await tester.pumpAndSettle();
    expect(find.text('Sample Form'), findsOneWidget);

    // Should pop if callback returns true
    willPopValue = true;
    await tester.tap(find.byTooltip('Back'));
    await tester.pumpAndSettle();
    expect(find.text('Sample Form'), findsNothing);
  });

  testWidgets('Form.willPop can inhibit back button', (WidgetTester tester) async {
    Widget buildFrame() {
      return MaterialApp(
        home: Scaffold(
          appBar: AppBar(title: const Text('Home')),
          body: Builder(
            builder: (BuildContext context) {
              return Center(
                child: TextButton(
                  child: const Text('X'),
                  onPressed: () {
                    Navigator.of(context).push(
                      MaterialPageRoute<void>(
                        builder: (BuildContext context) {
                          return SampleForm(callback: () => Future<bool>.value(willPopValue));
                        },
                      ),
                    );
                  },
                ),
              );
            },
          ),
        ),
      );
    }

    await tester.pumpWidget(buildFrame());

    await tester.tap(find.text('X'));
    await tester.pump();
    await tester.pump(const Duration(seconds: 1));

    expect(find.text('Sample Form'), findsOneWidget);

    willPopValue = false;
    willPopCount = 0;
    await tester.tap(find.byTooltip('Back'));
    await tester.pump(); // Start the pop "back" operation.
    await tester.pump(); // Complete the willPop() Future.
    await tester.pump(const Duration(seconds: 1)); // Wait until it has finished.
    expect(find.text('Sample Form'), findsOneWidget);
    expect(willPopCount, 1);

    willPopValue = true;
    willPopCount = 0;
    await tester.tap(find.byTooltip('Back'));
    await tester.pump(); // Start the pop "back" operation.
    await tester.pump(); // Complete the willPop() Future.
    await tester.pump(const Duration(seconds: 1)); // Wait until it has finished.
    expect(find.text('Sample Form'), findsNothing);
    expect(willPopCount, 1);
  });

  testWidgets('Form.willPop callbacks do not accumulate', (WidgetTester tester) async {
    Future<bool> showYesNoAlert(BuildContext context) async {
      return (await showDialog<bool>(
        context: context,
        builder: (BuildContext context) {
          return AlertDialog(
            actions: <Widget>[
              TextButton(
                child: const Text('YES'),
                onPressed: () {
                  Navigator.of(context).pop(true);
                },
              ),
              TextButton(
                child: const Text('NO'),
                onPressed: () {
                  Navigator.of(context).pop(false);
                },
              ),
            ],
          );
        },
      ))!;
    }

    Widget buildFrame() {
      return MaterialApp(
        home: Scaffold(
          appBar: AppBar(title: const Text('Home')),
          body: Builder(
            builder: (BuildContext context) {
              return Center(
                child: TextButton(
                  child: const Text('X'),
                  onPressed: () {
                    Navigator.of(context).push(
                      MaterialPageRoute<void>(
                        builder: (BuildContext context) {
                          return SampleForm(callback: () => showYesNoAlert(context));
                        },
                      ),
                    );
                  },
                ),
              );
            },
          ),
        ),
      );
    }

    await tester.pumpWidget(buildFrame());

    await tester.tap(find.text('X'));
    await tester.pump();
    await tester.pump(const Duration(seconds: 1));

    expect(find.text('Sample Form'), findsOneWidget);

    // Press the Scaffold's back button. This causes the willPop callback
    // to run, which shows the YES/NO Alert Dialog. Veto the back operation
    // by pressing the Alert's NO button.
    await tester.tap(find.byTooltip('Back'));
    await tester.pump(); // Start the pop "back" operation.
    await tester.pump(); // Call willPop which will show an Alert.
    await tester.tap(find.text('NO'));
    await tester.pump(); // Start the dismiss animation.
    await tester.pump(); // Resolve the willPop callback.
    await tester.pump(const Duration(seconds: 1)); // Wait until it has finished.
    expect(find.text('Sample Form'), findsOneWidget);

    // Do it again.
    // Each time the Alert is shown and dismissed the FormState's
    // didChangeDependencies() method runs. We're making sure that the
    // didChangeDependencies() method doesn't add an extra willPop callback.
    await tester.tap(find.byTooltip('Back'));
    await tester.pump(); // Start the pop "back" operation.
    await tester.pump(); // Call willPop which will show an Alert.
    await tester.tap(find.text('NO'));
    await tester.pump(); // Start the dismiss animation.
    await tester.pump(); // Resolve the willPop callback.
    await tester.pump(const Duration(seconds: 1)); // Wait until it has finished.
    expect(find.text('Sample Form'), findsOneWidget);

    // This time really dismiss the SampleForm by pressing the Alert's
    // YES button.
    await tester.tap(find.byTooltip('Back'));
    await tester.pump(); // Start the pop "back" operation.
    await tester.pump(); // Call willPop which will show an Alert.
    await tester.tap(find.text('YES'));
    await tester.pump(); // Start the dismiss animation.
    await tester.pump(); // Resolve the willPop callback.
    await tester.pump(const Duration(seconds: 1)); // Wait until it has finished.
    expect(find.text('Sample Form'), findsNothing);
  });

  testWidgets('Route.scopedWillPop callbacks do not accumulate', (WidgetTester tester) async {
    late StateSetter contentsSetState; // call this to rebuild the route's SampleForm contents
    bool contentsEmpty = false; // when true, don't include the SampleForm in the route

    final _TestPageRoute<void> route = _TestPageRoute<void>(
      builder: (BuildContext context) {
        return StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            contentsSetState = setState;
            return contentsEmpty
                ? Container()
                : SampleForm(key: UniqueKey(), callback: () async => false);
          },
        );
      },
    );

    Widget buildFrame() {
      return MaterialApp(
        home: Scaffold(
          appBar: AppBar(title: const Text('Home')),
          body: Builder(
            builder: (BuildContext context) {
              return Center(
                child: TextButton(
                  child: const Text('X'),
                  onPressed: () {
                    Navigator.of(context).push(route);
                  },
                ),
              );
            },
          ),
        ),
      );
    }

    await tester.pumpWidget(buildFrame());

    await tester.tap(find.text('X'));
    await tester.pump();
    await tester.pump(const Duration(seconds: 1));

    expect(find.text('Sample Form'), findsOneWidget);
    expect(route.hasCallback, isTrue);

    // Rebuild the route's SampleForm child an additional 3x for good measure.
    contentsSetState(() {});
    await tester.pump();
    contentsSetState(() {});
    await tester.pump();
    contentsSetState(() {});
    await tester.pump();

    // Now build the route's contents without the sample form.
    contentsEmpty = true;
    contentsSetState(() {});
    await tester.pump();

    expect(route.hasCallback, isFalse);
  });

  testWidgets('should handle new route if page moved from one navigator to another', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/89133
    late StateSetter contentsSetState;
    bool moveToAnotherNavigator = false;

    final List<Page<dynamic>> pages = <Page<dynamic>>[
      _TestPage(
        key: UniqueKey(),
        builder: (BuildContext context) {
          return WillPopScope(onWillPop: () async => true, child: const Text('anchor'));
        },
      ),
    ];

    Widget buildNavigator(Key? key, List<Page<dynamic>> pages) {
      return Navigator(
        key: key,
        pages: pages,
        onPopPage: (Route<dynamic> route, dynamic result) {
          return route.didPop(result);
        },
      );
    }

    Widget buildFrame() {
      return MaterialApp(
        home: Scaffold(
          body: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              contentsSetState = setState;
              if (moveToAnotherNavigator) {
                return buildNavigator(const ValueKey<int>(1), pages);
              }
              return buildNavigator(const ValueKey<int>(2), pages);
            },
          ),
        ),
      );
    }

    await tester.pumpWidget(buildFrame());
    await tester.pump();
    final _TestPageRoute<dynamic> route1 =
        ModalRoute.of(tester.element(find.text('anchor')))! as _TestPageRoute<dynamic>;
    expect(route1.hasCallback, isTrue);
    moveToAnotherNavigator = true;
    contentsSetState(() {});

    await tester.pump();
    final _TestPageRoute<dynamic> route2 =
        ModalRoute.of(tester.element(find.text('anchor')))! as _TestPageRoute<dynamic>;

    expect(route1.hasCallback, isFalse);
    expect(route2.hasCallback, isTrue);
  });
}
