// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@TestOn('chrome || safari || firefox')
library;

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine/dom.dart';

import '../common/matchers.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

void testMain() {
  group('expectDom', () {
    _expectDomTests();
  });
}

void _expectDomTests() {
  test('trivial equal elements', () {
    expectDom('<div></div>', hasHtml('<div></div>'));
  });

  test('trivial unequal elements', () {
    expectDom(
      '<div></div>',
      expectMismatch(hasHtml('<span></span>'), '''
The following DOM structure did not match the expected pattern:
<div></div>

Specifically:
 - @span: unexpected tag name <div> (expected <span>).'''),
    );
  });

  test('trivial equal text content', () {
    expectDom('<div>hello</div>', hasHtml('<div>hello</div>'));
  });

  test('trivial unequal text content', () {
    expectDom(
      '<div>hello</div>',
      expectMismatch(hasHtml('<div>world</div>'), '''
The following DOM structure did not match the expected pattern:
<div>hello</div>

Specifically:
 - @div: expected text content "world", but found "hello".'''),
    );
  });

  test('white space between elements', () {
    expectDom('<a> <b> </b> </a>', hasHtml('<a><b> </b></a>'));

    expectDom('<a><b> </b></a>', hasHtml('<a> <b> </b> </a>'));

    expectDom(
      '<a><b> </b></a>',
      expectMismatch(hasHtml('<a><b>   </b></a>'), '''
The following DOM structure did not match the expected pattern:
<a><b> </b></a>

Specifically:
 - @a > b: expected text content "   ", but found " ".'''),
    );
  });

  test('trivial equal attributes', () {
    expectDom('<div id="hello"></div>', hasHtml('<div id="hello"></div>'));
  });

  test('trivial out-of-order equal attributes', () {
    expectDom(
      '<div hello="brave" new="world"></div>',
      hasHtml('<div new="world" hello="brave"></div>'),
    );
  });

  test('trivial unequal attributes', () {
    expectDom(
      '<div id="hello"></div>',
      expectMismatch(hasHtml('<div id="world"></div>'), '''
The following DOM structure did not match the expected pattern:
<div id="hello"></div>

Specifically:
 - @div#id: expected attribute value id="world", but found id="hello".'''),
    );
  });

  test('trivial missing attributes', () {
    expectDom(
      '<div></div>',
      expectMismatch(hasHtml('<div id="hello"></div>'), '''
The following DOM structure did not match the expected pattern:
<div></div>

Specifically:
 - @div#id: attribute id="hello" missing.'''),
    );
  });

  test('trivial additional attributes', () {
    expectDom('<div id="hello"></div>', hasHtml('<div></div>'));

    expectDom('<div id="hello" foo="bar"></div>', hasHtml('<div id="hello"></div>'));
  });

  test('trivial equal style', () {
    expectDom(
      '<div style="width: 10px; height: 40px"></div>',
      hasHtml('<div style="width: 10px; height: 40px"></div>'),
    );
  });

  test('trivial additional style attribute', () {
    expectDom(
      '<div style="width: 10px; transform: scale(2, 3); height: 40px"></div>',
      hasHtml('<div style="width: 10px; height: 40px"></div>'),
    );
  });

  test('out of order equal style', () {
    expectDom(
      '<div style="width: 10px; height: 40px"></div>',
      hasHtml('<div style="height: 40px; width: 10px"></div>'),
    );
  });

  test('trivial unequal style attributes', () {
    expectDom(
      '<div style="width: 10px"></div>',
      expectMismatch(hasHtml('<div style="width: 12px"></div>'), '''
The following DOM structure did not match the expected pattern:
<div style="width: 10px"></div>

Specifically:
 - @div#style(width): expected style property width="12px", but found width="10px".'''),
    );
  });

  test('trivial missing style attribute', () {
    expectDom(
      '<div style="width: 12px"></div>',
      expectMismatch(hasHtml('<div style="width: 12px; height: 20px"></div>'), '''
The following DOM structure did not match the expected pattern:
<div style="width: 12px"></div>

Specifically:
 - @div#style(height): style property height="20px" missing.'''),
    );
  });

  test('multiple attribute mismatches', () {
    expectDom(
      '<div id="other" style="width: 12px; transform: scale(2)"></div>',
      expectMismatch(
        hasHtml(
          '<div id="this" foo="bar" style="width: 12px; transform: scale(2); height: 20px"></div>',
        ),
        '''
The following DOM structure did not match the expected pattern:
<div id="other" style="width: 12px; transform: scale(2)"></div>

Specifically:
 - @div#id: expected attribute value id="this", but found id="other".
 - @div#foo: attribute foo="bar" missing.
 - @div#style(height): style property height="20px" missing.''',
      ),
    );
  });

  test('trivial child elements', () {
    expectDom('<div><span></span><p></p></div>', hasHtml('<div><span></span><p></p></div>'));
  });

  test('trivial nested child elements', () {
    expectDom('<div><p><span></span></p></div>', hasHtml('<div><p><span></span></p></div>'));
  });

  test('missing child elements', () {
    expectDom(
      '<div><span></span><p></p></div>',
      expectMismatch(hasHtml('<div><span></span><waldo></waldo><p></p></div>'), '''
The following DOM structure did not match the expected pattern:
<div><span></span><p></p></div>

Specifically:
 - @div: expected 3 child nodes, but found 2.'''),
    );
  });

  test('additional child elements', () {
    expectDom(
      '<div><span></span><waldo></waldo><p></p></div>',
      expectMismatch(hasHtml('<div><span></span><p></p></div>'), '''
The following DOM structure did not match the expected pattern:
<div><span></span><waldo></waldo><p></p></div>

Specifically:
 - @div: expected 2 child nodes, but found 3.'''),
    );
  });

  test('deep breadcrumbs', () {
    expectDom(
      '<a><b><c><d style="width: 1px"></d></c></b></a>',
      expectMismatch(hasHtml('<a><b><c><d style="width: 2px"></d></c></b></a>'), '''
The following DOM structure did not match the expected pattern:
<a><b><c><d style="width: 1px"></d></c></b></a>

Specifically:
 - @a > b > c > d#style(width): expected style property width="2px", but found width="1px".'''),
    );
  });
}

void expectDom(String domHtml, Matcher matcher) {
  final DomElement root = createDomElement('div');
  root.innerHTML = domHtml;
  expect(root.children.single, matcher);
}

Matcher expectMismatch(Matcher matcher, String expectedMismatchDescription) {
  return _ExpectMismatch(matcher, expectedMismatchDescription);
}

class _ExpectMismatch extends Matcher {
  const _ExpectMismatch(this._matcher, this.expectedMismatchDescription);

  final Matcher _matcher;
  final String expectedMismatchDescription;

  @override
  bool matches(Object? item, Map<Object?, Object?> matchState) {
    if (_matcher.matches(item, matchState)) {
      matchState['matched'] = true;
      return false;
    }

    final _TestDescription description = _TestDescription();
    _matcher.describeMismatch(item, description, matchState, false);
    final String mismatchDescription = description.items.join();

    if (mismatchDescription.trim() != expectedMismatchDescription.trim()) {
      matchState['mismatchDescription'] = mismatchDescription;
      matchState['expectedMismatchDescription'] = expectedMismatchDescription;
      return false;
    }

    return true;
  }

  @override
  Description describe(Description description) =>
      description.add('not ').addDescriptionOf(_matcher);

  @override
  Description describeMismatch(
    Object? object,
    Description mismatchDescription,
    Map<Object?, Object?> matchState,
    bool verbose,
  ) {
    if (matchState.containsKey('matched')) {
      mismatchDescription.add('Expected a mismatch, but the HTML pattern matched.');
    }
    if (matchState.containsKey('mismatchDescription')) {
      mismatchDescription.add('Mismatch description was wrong.\n');
      mismatchDescription.add('  Expected: ${matchState['expectedMismatchDescription']}\n');
      mismatchDescription.add('  Actual  : ${matchState['mismatchDescription']}\n');
    }

    return mismatchDescription;
  }
}

class _TestDescription implements Description {
  final List<String> items = <String>[];

  @override
  int get length => items.length;

  @override
  Description add(String text) {
    items.add(text);
    return this;
  }

  @override
  Description addAll(String start, String separator, String end, Iterable<Object?> list) {
    throw UnimplementedError();
  }

  @override
  Description addDescriptionOf(Object? value) {
    throw UnimplementedError();
  }

  @override
  Description replace(String text) {
    throw UnimplementedError();
  }
}
