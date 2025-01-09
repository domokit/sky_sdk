// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/rendering.dart';
import 'package:flutter/services.dart';
import 'package:flutter/src/widgets/basic.dart';
import 'package:flutter/src/widgets/framework.dart';
import 'package:flutter/src/widgets/layout_builder.dart';
import 'package:flutter/src/widgets/media_query.dart';
import 'package:flutter/src/widgets/scroll_view.dart';
import 'package:flutter/src/widgets/sliver_layout_builder.dart';
import 'package:flutter_test/flutter_test.dart';

class Wrapper extends StatelessWidget {
  const Wrapper({super.key, required this.child});

  final Widget child;

  @override
  Widget build(BuildContext context) => child;
}

void main() {
  testWidgets('Moving a global key from another LayoutBuilder at layout time', (
    WidgetTester tester,
  ) async {
    final GlobalKey victimKey = GlobalKey();

    await tester.pumpWidget(
      Row(
        textDirection: TextDirection.ltr,
        children: <Widget>[
          Wrapper(
            child: LayoutBuilder(
              builder: (BuildContext context, BoxConstraints constraints) {
                return const SizedBox();
              },
            ),
          ),
          Wrapper(
            child: Wrapper(
              child: LayoutBuilder(
                builder: (BuildContext context, BoxConstraints constraints) {
                  return Wrapper(child: SizedBox(key: victimKey));
                },
              ),
            ),
          ),
        ],
      ),
    );

    await tester.pumpWidget(
      Row(
        textDirection: TextDirection.ltr,
        children: <Widget>[
          Wrapper(
            child: LayoutBuilder(
              builder: (BuildContext context, BoxConstraints constraints) {
                return Wrapper(child: SizedBox(key: victimKey));
              },
            ),
          ),
          Wrapper(
            child: Wrapper(
              child: LayoutBuilder(
                builder: (BuildContext context, BoxConstraints constraints) {
                  return const SizedBox();
                },
              ),
            ),
          ),
        ],
      ),
    );

    expect(tester.takeException(), null);
  });

  testWidgets('Moving a global key from another SliverLayoutBuilder at layout time', (
    WidgetTester tester,
  ) async {
    final GlobalKey victimKey1 = GlobalKey();
    final GlobalKey victimKey2 = GlobalKey();

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: CustomScrollView(
          slivers: <Widget>[
            SliverLayoutBuilder(
              builder: (BuildContext context, SliverConstraints constraint) {
                return SliverPadding(
                  key: victimKey1,
                  padding: const EdgeInsets.fromLTRB(1, 2, 3, 4),
                );
              },
            ),
            SliverLayoutBuilder(
              builder: (BuildContext context, SliverConstraints constraint) {
                return SliverPadding(
                  key: victimKey2,
                  padding: const EdgeInsets.fromLTRB(5, 7, 11, 13),
                );
              },
            ),
            SliverLayoutBuilder(
              builder: (BuildContext context, SliverConstraints constraint) {
                return const SliverPadding(padding: EdgeInsets.fromLTRB(5, 7, 11, 13));
              },
            ),
          ],
        ),
      ),
    );

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: CustomScrollView(
          slivers: <Widget>[
            SliverLayoutBuilder(
              builder: (BuildContext context, SliverConstraints constraint) {
                return SliverPadding(
                  key: victimKey2,
                  padding: const EdgeInsets.fromLTRB(1, 2, 3, 4),
                );
              },
            ),
            SliverLayoutBuilder(
              builder: (BuildContext context, SliverConstraints constraint) {
                return const SliverPadding(padding: EdgeInsets.fromLTRB(5, 7, 11, 13));
              },
            ),
            SliverLayoutBuilder(
              builder: (BuildContext context, SliverConstraints constraint) {
                return SliverPadding(
                  key: victimKey1,
                  padding: const EdgeInsets.fromLTRB(5, 7, 11, 13),
                );
              },
            ),
          ],
        ),
      ),
    );

    expect(tester.takeException(), null);
  });

  testWidgets('LayoutBuilder does not layout twice', (WidgetTester tester) async {
    // This widget marks itself dirty when the closest MediaQuery changes.
    final _LayoutCount widget = _LayoutCount();
    late StateSetter setState;
    bool updated = false;

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: StatefulBuilder(
          builder: (BuildContext context, StateSetter setter) {
            setState = setter;
            return MediaQuery(
              data:
                  updated
                      ? const MediaQueryData(platformBrightness: Brightness.dark)
                      : const MediaQueryData(),
              child: LayoutBuilder(
                builder: (BuildContext context, BoxConstraints constraints) {
                  return Center(
                    child: SizedBox.square(
                      dimension: 20,
                      child: Center(
                        child: SizedBox.square(dimension: updated ? 10 : 20, child: widget),
                      ),
                    ),
                  );
                },
              ),
            );
          },
        ),
      ),
    );

    assert(widget._renderObject.layoutCount == 1);
    setState(() {
      updated = true;
    });

    await tester.pump();
    expect(widget._renderObject.layoutCount, 2);
  });
}

class _LayoutCount extends LeafRenderObjectWidget {
  late final _RenderLayoutCount _renderObject;

  @override
  RenderObject createRenderObject(BuildContext context) {
    return _renderObject = _RenderLayoutCount(MediaQuery.of(context));
  }

  @override
  void updateRenderObject(BuildContext context, _RenderLayoutCount renderObject) {
    renderObject.mediaQuery = MediaQuery.of(context);
  }
}

class _RenderLayoutCount extends RenderProxyBox {
  _RenderLayoutCount(this._mediaQuery);
  int layoutCount = 0;

  MediaQueryData get mediaQuery => _mediaQuery;
  MediaQueryData _mediaQuery;
  set mediaQuery(MediaQueryData newValue) {
    if (newValue != _mediaQuery) {
      _mediaQuery = newValue;
      markNeedsLayout();
    }
  }

  @override
  bool get sizedByParent => true;

  @override
  void performLayout() {
    layoutCount += 1;
  }
}
