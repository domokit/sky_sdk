// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

import 'logic.dart';

class Calculator extends StatefulWidget {
  const Calculator({super.key});

  @override
  State<Calculator> createState() => CalculatorState();
}

class CalculatorState extends State<Calculator> {
  /// As the user taps keys we update the current `_expression` and we also
  /// keep a stack of previous expressions so we can return to earlier states
  /// when the user hits the DEL key.
  final List<CalcExpression> _expressionStack = <CalcExpression>[];
  CalcExpression _expression = CalcExpression.empty();

  // Make `expression` the current expression and push the previous current
  // expression onto the stack.
  void pushExpression(CalcExpression expression) {
    _expressionStack.add(_expression);
    _expression = expression;
  }

  /// Pop the top expression off of the stack and make it the current expression.
  void popCalcExpression() {
    if (_expressionStack.isNotEmpty) {
      _expression = _expressionStack.removeLast();
    } else {
      _expression = CalcExpression.empty();
    }
  }

  /// Set `resultExpression` to the current expression and clear the stack.
  void setResult(CalcExpression resultExpression) {
    _expressionStack.clear();
    _expression = resultExpression;
  }

  void handleNumberTap(int n) {
    final CalcExpression? expression = _expression.appendDigit(n);
    if (expression != null) {
      setState(() {
        pushExpression(expression);
      });
    }
  }

  void handlePointTap() {
    final CalcExpression? expression = _expression.appendPoint();
    if (expression != null) {
      setState(() {
        pushExpression(expression);
      });
    }
  }

  void handlePlusTap() {
    final CalcExpression? expression = _expression.appendOperation(Operation.Addition);
    if (expression != null) {
      setState(() {
        pushExpression(expression);
      });
    }
  }

  void handleMinusTap() {
    final CalcExpression? expression = _expression.appendMinus();
    if (expression != null) {
      setState(() {
        pushExpression(expression);
      });
    }
  }

  void handleMultTap() {
    final CalcExpression? expression = _expression.appendOperation(Operation.Multiplication);
    if (expression != null) {
      setState(() {
        pushExpression(expression);
      });
    }
  }

  void handleDivTap() {
    final CalcExpression? expression = _expression.appendOperation(Operation.Division);
    if (expression != null) {
      setState(() {
        pushExpression(expression);
      });
    }
  }

  void handleEqualsTap() {
    final CalcExpression? resultExpression = _expression.computeResult();
    if (resultExpression != null) {
      setState(() {
        setResult(resultExpression);
      });
    }
  }

  void handleDelTap() {
    setState(() {
      popCalcExpression();
    });
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(backgroundColor: Theme.of(context).canvasColor, elevation: 0.0),
      body: Column(
        crossAxisAlignment: CrossAxisAlignment.stretch,
        children: <Widget>[
          // Give the key-pad 3/5 of the vertical space and the display 2/5.
          Expanded(flex: 2, child: CalcDisplay(content: _expression.toString())),
          const Divider(height: 1.0),
          Expanded(flex: 3, child: KeyPad(calcState: this)),
        ],
      ),
    );
  }
}

class CalcDisplay extends StatelessWidget {
  const CalcDisplay({super.key, this.content});

  final String? content;

  @override
  Widget build(BuildContext context) {
    return Center(child: Text(content!, style: const TextStyle(fontSize: 24.0)));
  }
}

class KeyPad extends StatelessWidget {
  const KeyPad({super.key, this.calcState});

  final CalculatorState? calcState;

  @override
  Widget build(BuildContext context) {
    final ThemeData themeData = ThemeData(
      primarySwatch: Colors.purple,
      brightness: Brightness.dark,
      platform: Theme.of(context).platform,
    );
    return Theme(
      data: themeData,
      child: Material(
        child: Row(
          children: <Widget>[
            Expanded(
              // We set flex equal to the number of columns so that the main keypad
              // and the op keypad have sizes proportional to their number of
              // columns.
              flex: 3,
              child: Column(
                children: <Widget>[
                  KeyRow(<Widget>[
                    NumberKey(7, calcState),
                    NumberKey(8, calcState),
                    NumberKey(9, calcState),
                  ]),
                  KeyRow(<Widget>[
                    NumberKey(4, calcState),
                    NumberKey(5, calcState),
                    NumberKey(6, calcState),
                  ]),
                  KeyRow(<Widget>[
                    NumberKey(1, calcState),
                    NumberKey(2, calcState),
                    NumberKey(3, calcState),
                  ]),
                  KeyRow(<Widget>[
                    CalcKey('.', calcState!.handlePointTap),
                    NumberKey(0, calcState),
                    CalcKey('=', calcState!.handleEqualsTap),
                  ]),
                ],
              ),
            ),
            Expanded(
              child: Material(
                color: themeData.colorScheme.surface,
                child: Column(
                  children: <Widget>[
                    CalcKey('\u232B', calcState!.handleDelTap),
                    CalcKey('\u00F7', calcState!.handleDivTap),
                    CalcKey('\u00D7', calcState!.handleMultTap),
                    CalcKey('-', calcState!.handleMinusTap),
                    CalcKey('+', calcState!.handlePlusTap),
                  ],
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }
}

class KeyRow extends StatelessWidget {
  const KeyRow(this.keys, {super.key});

  final List<Widget> keys;

  @override
  Widget build(BuildContext context) {
    return Expanded(child: Row(mainAxisAlignment: MainAxisAlignment.center, children: keys));
  }
}

class CalcKey extends StatelessWidget {
  const CalcKey(this.text, this.onTap, {super.key});

  final String text;
  final GestureTapCallback onTap;

  @override
  Widget build(BuildContext context) {
    final Orientation orientation = MediaQuery.of(context).orientation;
    return Expanded(
      child: InkResponse(
        onTap: onTap,
        child: Center(
          child: Text(
            text,
            style: TextStyle(
              // This line is used as a sentinel in the hot reload tests: hot_mode_test.dart
              // in the devicelab.
              fontSize: (orientation == Orientation.portrait) ? 32.0 : 24.0,
            ),
          ),
        ),
      ),
    );
  }
}

class NumberKey extends CalcKey {
  NumberKey(int value, CalculatorState? calcState, {Key? key})
    : super('$value', () {
        calcState!.handleNumberTap(value);
      }, key: key);
}
