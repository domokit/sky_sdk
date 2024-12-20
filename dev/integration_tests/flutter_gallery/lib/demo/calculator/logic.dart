// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// A token that composes an expression. There are several kinds of tokens
/// that represent arithmetic operation symbols, numbers and pieces of numbers.
/// We need to represent pieces of numbers because the user may have only
/// entered a partial expression so far.
class ExpressionToken {
  ExpressionToken(this.stringRep);

  final String? stringRep;

  @override
  String toString() => stringRep!;
}

/// A token that represents a number.
class NumberToken extends ExpressionToken {
  NumberToken(String super.stringRep, this.number);

  NumberToken.fromNumber(num number) : this('$number', number);

  final num number;
}

/// A token that represents an integer.
class IntToken extends NumberToken {
  IntToken(String stringRep) : super(stringRep, int.parse(stringRep));
}

/// A token that represents a floating point number.
class FloatToken extends NumberToken {
  FloatToken(String stringRep) : super(stringRep, _parse(stringRep));

  static double _parse(String stringRep) {
    String toParse = stringRep;
    if (toParse.startsWith('.')) {
      toParse = '0$toParse';
    }
    if (toParse.endsWith('.')) {
      toParse = '${toParse}0';
    }
    return double.parse(toParse);
  }
}

/// A token that represents a number that is the result of a computation.
class ResultToken extends NumberToken {
  ResultToken(num number) : super.fromNumber(round(number));

  /// rounds `number` to 14 digits of precision. A double precision
  /// floating point number is guaranteed to have at least this many
  /// decimal digits of precision.
  static num round(num number) {
    if (number is int) {
      return number;
    }
    return double.parse(number.toStringAsPrecision(14));
  }
}

/// A token that represents the unary minus prefix.
class LeadingNegToken extends ExpressionToken {
  LeadingNegToken() : super('-');
}

enum Operation { Addition, Subtraction, Multiplication, Division }

/// A token that represents an arithmetic operation symbol.
class OperationToken extends ExpressionToken {
  OperationToken(this.operation) : super(opString(operation));

  Operation operation;

  static String? opString(Operation operation) {
    return switch (operation) {
      Operation.Addition => ' + ',
      Operation.Subtraction => ' - ',
      Operation.Multiplication => '  \u00D7  ',
      Operation.Division => '  \u00F7  ',
    };
  }
}

/// As the user taps different keys the current expression can be in one
/// of several states.
enum ExpressionState {
  /// The expression is empty or an operation symbol was just entered.
  /// A new number must be started now.
  Start,

  /// A minus sign was entered as a leading negative prefix.
  LeadingNeg,

  /// We are in the midst of a number without a point.
  Number,

  /// A point was just entered.
  Point,

  /// We are in the midst of a number with a point.
  NumberWithPoint,

  /// A result is being displayed
  Result,
}

/// An expression that can be displayed in a calculator. It is the result
/// of a sequence of user entries. It is represented by a sequence of tokens.
///
/// The tokens are not in one to one correspondence with the key taps because we
/// use one token per number, not one token per digit. A [CalcExpression] is
/// immutable. The `append*` methods return a new [CalcExpression] that
/// represents the appropriate expression when one additional key tap occurs.
class CalcExpression {
  CalcExpression(this._list, this.state);

  CalcExpression.empty() : this(<ExpressionToken>[], ExpressionState.Start);

  CalcExpression.result(FloatToken result)
    : _list = <ExpressionToken?>[result],
      state = ExpressionState.Result;

  /// The tokens comprising the expression.
  final List<ExpressionToken?> _list;

  /// The state of the expression.
  final ExpressionState state;

  /// The string representation of the expression. This will be displayed
  /// in the calculator's display panel.
  @override
  String toString() {
    final StringBuffer buffer = StringBuffer();
    buffer.writeAll(_list);
    return buffer.toString();
  }

  /// Append a digit to the current expression and return a new expression
  /// representing the result. Returns null to indicate that it is not legal
  /// to append a digit in the current state.
  CalcExpression? appendDigit(int digit) {
    ExpressionState newState = ExpressionState.Number;
    ExpressionToken? newToken;
    final List<ExpressionToken?> outList = _list.toList();
    switch (state) {
      case ExpressionState.Start:
        // Start a new number with digit.
        newToken = IntToken('$digit');
      case ExpressionState.LeadingNeg:
        // Replace the leading neg with a negative number starting with digit.
        outList.removeLast();
        newToken = IntToken('-$digit');
      case ExpressionState.Number:
        final ExpressionToken last = outList.removeLast()!;
        newToken = IntToken('${last.stringRep}$digit');
      case ExpressionState.Point:
      case ExpressionState.NumberWithPoint:
        final ExpressionToken last = outList.removeLast()!;
        newState = ExpressionState.NumberWithPoint;
        newToken = FloatToken('${last.stringRep}$digit');
      case ExpressionState.Result:
        // Cannot enter a number now
        return null;
    }
    outList.add(newToken);
    return CalcExpression(outList, newState);
  }

  /// Append a point to the current expression and return a new expression
  /// representing the result. Returns null to indicate that it is not legal
  /// to append a point in the current state.
  CalcExpression? appendPoint() {
    ExpressionToken? newToken;
    final List<ExpressionToken?> outList = _list.toList();
    switch (state) {
      case ExpressionState.Start:
        newToken = FloatToken('.');
      case ExpressionState.LeadingNeg:
      case ExpressionState.Number:
        final ExpressionToken last = outList.removeLast()!;
        final String value = last.stringRep!;
        newToken = FloatToken('$value.');
      case ExpressionState.Point:
      case ExpressionState.NumberWithPoint:
      case ExpressionState.Result:
        // Cannot enter a point now
        return null;
    }
    outList.add(newToken);
    return CalcExpression(outList, ExpressionState.Point);
  }

  /// Append an operation symbol to the current expression and return a new
  /// expression representing the result. Returns null to indicate that it is not
  /// legal to append an operation symbol in the current state.
  CalcExpression? appendOperation(Operation op) {
    switch (state) {
      case ExpressionState.Start:
      case ExpressionState.LeadingNeg:
      case ExpressionState.Point:
        // Cannot enter operation now.
        return null;
      case ExpressionState.Number:
      case ExpressionState.NumberWithPoint:
      case ExpressionState.Result:
        break;
    }
    final List<ExpressionToken?> outList = _list.toList();
    outList.add(OperationToken(op));
    return CalcExpression(outList, ExpressionState.Start);
  }

  /// Append a leading minus sign to the current expression and return a new
  /// expression representing the result. Returns null to indicate that it is not
  /// legal to append a leading minus sign in the current state.
  CalcExpression? appendLeadingNeg() {
    switch (state) {
      case ExpressionState.Start:
        break;
      case ExpressionState.LeadingNeg:
      case ExpressionState.Point:
      case ExpressionState.Number:
      case ExpressionState.NumberWithPoint:
      case ExpressionState.Result:
        // Cannot enter leading neg now.
        return null;
    }
    final List<ExpressionToken?> outList = _list.toList();
    outList.add(LeadingNegToken());
    return CalcExpression(outList, ExpressionState.LeadingNeg);
  }

  /// Append a minus sign to the current expression and return a new expression
  /// representing the result. Returns null to indicate that it is not legal
  /// to append a minus sign in the current state. Depending on the current
  /// state the minus sign will be interpreted as either a leading negative
  /// sign or a subtraction operation.
  CalcExpression? appendMinus() {
    switch (state) {
      case ExpressionState.Start:
        return appendLeadingNeg();
      case ExpressionState.LeadingNeg:
      case ExpressionState.Point:
      case ExpressionState.Number:
      case ExpressionState.NumberWithPoint:
      case ExpressionState.Result:
        return appendOperation(Operation.Subtraction);
    }
  }

  /// Computes the result of the current expression and returns a new
  /// ResultExpression containing the result. Returns null to indicate that
  /// it is not legal to compute a result in the current state.
  CalcExpression? computeResult() {
    switch (state) {
      case ExpressionState.Start:
      case ExpressionState.LeadingNeg:
      case ExpressionState.Point:
      case ExpressionState.Result:
        // Cannot compute result now.
        return null;
      case ExpressionState.Number:
      case ExpressionState.NumberWithPoint:
        break;
    }

    // We make a copy of _list because CalcExpressions are supposed to
    // be immutable.
    final List<ExpressionToken?> list = _list.toList();
    // We obey order-of-operations by computing the sum of the 'terms',
    // where a "term" is defined to be a sequence of numbers separated by
    // multiplication or division symbols.
    num currentTermValue = removeNextTerm(list);
    while (list.isNotEmpty) {
      final OperationToken opToken = list.removeAt(0)! as OperationToken;
      final num nextTermValue = removeNextTerm(list);
      switch (opToken.operation) {
        case Operation.Addition:
          currentTermValue += nextTermValue;
        case Operation.Subtraction:
          currentTermValue -= nextTermValue;
        case Operation.Multiplication:
        case Operation.Division:
          // Logic error.
          assert(false);
      }
    }
    final List<ExpressionToken> outList = <ExpressionToken>[ResultToken(currentTermValue)];
    return CalcExpression(outList, ExpressionState.Result);
  }

  /// Removes the next "term" from `list` and returns its numeric value.
  /// A "term" is a sequence of number tokens separated by multiplication
  /// and division symbols.
  static num removeNextTerm(List<ExpressionToken?> list) {
    assert(list.isNotEmpty);
    final NumberToken firstNumToken = list.removeAt(0)! as NumberToken;
    num currentValue = firstNumToken.number;
    while (list.isNotEmpty) {
      bool isDivision = false;
      final OperationToken nextOpToken = list.first! as OperationToken;
      switch (nextOpToken.operation) {
        case Operation.Addition:
        case Operation.Subtraction:
          // We have reached the end of the current term
          return currentValue;
        case Operation.Multiplication:
          break;
        case Operation.Division:
          isDivision = true;
      }
      // Remove the operation token.
      list.removeAt(0);
      // Remove the next number token.
      final NumberToken nextNumToken = list.removeAt(0)! as NumberToken;
      final num nextNumber = nextNumToken.number;
      if (isDivision) {
        currentValue /= nextNumber;
      } else {
        currentValue *= nextNumber;
      }
    }
    return currentValue;
  }
}
