// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// The whole design for the lexing and parsing step can be found in this design doc.
// See https://flutter.dev/go/icu-message-parser.

// Symbol Types
import '../base/logger.dart';
import 'gen_l10n_types.dart';

enum ST {
  // Terminal Types
  openBrace,
  closeBrace,
  comma,
  equalSign,
  other,
  plural,
  select,
  string,
  number,
  identifier,
  empty,
  colon,
  date,
  time,
  // Nonterminal Types
  message,

  placeholderExpr,

  argumentExpr,

  pluralExpr,
  pluralParts,
  pluralPart,

  selectExpr,
  selectParts,
  selectPart,

  argType,
}

// The grammar of the syntax.
Map<ST, List<List<ST>>> grammar = <ST, List<List<ST>>>{
  ST.message: <List<ST>>[
    <ST>[ST.string, ST.message],
    <ST>[ST.placeholderExpr, ST.message],
    <ST>[ST.pluralExpr, ST.message],
    <ST>[ST.selectExpr, ST.message],
    <ST>[ST.argumentExpr, ST.message],
    <ST>[ST.empty],
  ],
  ST.placeholderExpr: <List<ST>>[
    <ST>[ST.openBrace, ST.identifier, ST.closeBrace],
  ],
  ST.pluralExpr: <List<ST>>[
    <ST>[ST.openBrace, ST.identifier, ST.comma, ST.plural, ST.comma, ST.pluralParts, ST.closeBrace],
  ],
  ST.pluralParts: <List<ST>>[
    <ST>[ST.pluralPart, ST.pluralParts],
    <ST>[ST.empty],
  ],
  ST.pluralPart: <List<ST>>[
    <ST>[ST.identifier, ST.openBrace, ST.message, ST.closeBrace],
    <ST>[ST.equalSign, ST.number, ST.openBrace, ST.message, ST.closeBrace],
    <ST>[ST.other, ST.openBrace, ST.message, ST.closeBrace],
  ],
  ST.selectExpr: <List<ST>>[
    <ST>[ST.openBrace, ST.identifier, ST.comma, ST.select, ST.comma, ST.selectParts, ST.closeBrace],
    <ST>[ST.other, ST.openBrace, ST.message, ST.closeBrace],
  ],
  ST.selectParts: <List<ST>>[
    <ST>[ST.selectPart, ST.selectParts],
    <ST>[ST.empty],
  ],
  ST.selectPart: <List<ST>>[
    <ST>[ST.identifier, ST.openBrace, ST.message, ST.closeBrace],
    <ST>[ST.number, ST.openBrace, ST.message, ST.closeBrace],
    <ST>[ST.other, ST.openBrace, ST.message, ST.closeBrace],
  ],
  ST.argumentExpr: <List<ST>>[
    <ST>[
      ST.openBrace,
      ST.identifier,
      ST.comma,
      ST.argType,
      ST.comma,
      ST.colon,
      ST.colon,
      ST.identifier,
      ST.closeBrace,
    ],
  ],
  ST.argType: <List<ST>>[
    <ST>[ST.date],
    <ST>[ST.time],
  ],
};

class Node {
  Node(
    this.type,
    this.positionInMessage, {
    this.expectedSymbolCount = 0,
    this.value,
    List<Node>? children,
  }) : children = children ?? <Node>[];

  // Token constructors.
  Node.openBrace(this.positionInMessage) : type = ST.openBrace, value = '{';
  Node.closeBrace(this.positionInMessage) : type = ST.closeBrace, value = '}';
  Node.brace(this.positionInMessage, String this.value)
    : type = switch (value) {
        '{' => ST.openBrace,
        '}' => ST.closeBrace,
        _ => throw L10nException('Provided value $value is not a brace.'),
      };
  Node.equalSign(this.positionInMessage) : type = ST.equalSign, value = '=';
  Node.comma(this.positionInMessage) : type = ST.comma, value = ',';
  Node.string(this.positionInMessage, String this.value) : type = ST.string;
  Node.number(this.positionInMessage, String this.value) : type = ST.number;
  Node.identifier(this.positionInMessage, String this.value) : type = ST.identifier;
  Node.pluralKeyword(this.positionInMessage) : type = ST.plural, value = 'plural';
  Node.selectKeyword(this.positionInMessage) : type = ST.select, value = 'select';
  Node.otherKeyword(this.positionInMessage) : type = ST.other, value = 'other';
  Node.empty(this.positionInMessage) : type = ST.empty, value = '';
  Node.dateKeyword(this.positionInMessage) : type = ST.date, value = 'date';
  Node.timeKeyword(this.positionInMessage) : type = ST.time, value = 'time';

  String? value;
  late ST type;
  List<Node> children = <Node>[];
  int positionInMessage;
  int expectedSymbolCount = 0;

  @override
  String toString() {
    return _toStringHelper(0);
  }

  String _toStringHelper(int indentLevel) {
    final String indent = List<String>.filled(indentLevel, '  ').join();
    if (children.isEmpty) {
      return '''
${indent}Node($type, $positionInMessage${value == null ? '' : ", value: '$value'"})''';
    }
    final String childrenString = children
        .map((Node child) => child._toStringHelper(indentLevel + 1))
        .join(',\n');
    return '''
${indent}Node($type, $positionInMessage${value == null ? '' : ", value: '$value'"}, children: <Node>[
$childrenString,
$indent])''';
  }

  // Only used for testing. We don't compare expectedSymbolCount because
  // it is an auxiliary member used during the parse function but doesn't
  // have meaning after calling compress.
  @override
  // ignore: avoid_equals_and_hash_code_on_mutable_classes, hash_and_equals
  bool operator ==(covariant Node other) {
    if (value != other.value ||
        type != other.type ||
        positionInMessage != other.positionInMessage ||
        children.length != other.children.length) {
      return false;
    }
    for (int i = 0; i < children.length; i++) {
      if (children[i] != other.children[i]) {
        return false;
      }
    }
    return true;
  }

  bool get isFull {
    return children.length >= expectedSymbolCount;
  }
}

RegExp escapedString = RegExp(r"'[^']*'");
RegExp unescapedString = RegExp(r"[^{}']+");
RegExp normalString = RegExp(r'[^{}]+');

RegExp brace = RegExp(r'{|}');

RegExp whitespace = RegExp(r'\s+');
RegExp numeric = RegExp(r'[0-9]+');
RegExp alphanumeric = RegExp(r'[a-zA-Z0-9|_]+');
RegExp comma = RegExp(r',');
RegExp equalSign = RegExp(r'=');
RegExp colon = RegExp(r':');

// List of token matchers ordered by precedence
Map<ST, RegExp> matchers = <ST, RegExp>{
  ST.empty: whitespace,
  ST.number: numeric,
  ST.comma: comma,
  ST.equalSign: equalSign,
  ST.colon: colon,
  ST.identifier: alphanumeric,
};

class Parser {
  Parser(
    this.messageId,
    this.filename,
    this.messageString, {
    this.useEscaping = false,
    this.logger,
    this.placeholders,
  });

  final String messageId;
  final String messageString;
  final String filename;
  final bool useEscaping;
  final Logger? logger;
  final List<String>? placeholders;

  static String indentForError(int position) {
    return '${List<String>.filled(position, ' ').join()}^';
  }

  // Lexes the message into a list of typed tokens. General idea is that
  // every instance of "{" and "}" toggles the isString boolean and every
  // instance of "'" toggles the isEscaped boolean (and treats a double
  // single quote "''" as a single quote "'"). When !isString and !isEscaped
  // delimit tokens by whitespace and special characters. When placeholders
  // is passed, relax the syntax so that "{" and "}" can be used as strings in
  // certain cases.
  List<Node> lexIntoTokens() {
    final bool useRelaxedLexer = placeholders != null;
    final List<Node> tokens = <Node>[];
    bool isString = true;
    // Index specifying where to match from
    int startIndex = 0;
    int depth = 0;

    // At every iteration, we should be able to match a new token until we
    // reach the end of the string. If for some reason we don't match a
    // token in any iteration of the loop, throw an error.
    while (startIndex < messageString.length) {
      Match? match;
      if (isString) {
        if (useEscaping) {
          // This case is slightly involved. Essentially, wrapping any syntax in
          // single quotes escapes the syntax except when there are consecutive pair of single
          // quotes. For example, "Hello! 'Flutter''s amazing'. { unescapedPlaceholder }"
          // converts the '' in "Flutter's" to a single quote for convenience, since technically,
          // we should interpret this as two strings 'Flutter' and 's amazing'. To get around this,
          // we also check if the previous character is a ', and if so, add a single quote at the beginning
          // of the token.
          match = escapedString.matchAsPrefix(messageString, startIndex);
          if (match != null) {
            final String string = match.group(0)!;
            if (string == "''") {
              tokens.add(Node.string(startIndex, "'"));
            } else if (startIndex > 1 && messageString[startIndex - 1] == "'") {
              // Include a single quote in the beginning of the token.
              tokens.add(Node.string(startIndex, string.substring(0, string.length - 1)));
            } else {
              tokens.add(Node.string(startIndex, string.substring(1, string.length - 1)));
            }
            startIndex = match.end;
            continue;
          }
          match = unescapedString.matchAsPrefix(messageString, startIndex);
          if (match != null) {
            tokens.add(Node.string(startIndex, match.group(0)!));
            startIndex = match.end;
            continue;
          }
        } else {
          match = normalString.matchAsPrefix(messageString, startIndex);
          if (match != null) {
            tokens.add(Node.string(startIndex, match.group(0)!));
            startIndex = match.end;
            continue;
          }
        }
        match = brace.matchAsPrefix(messageString, startIndex);
        if (match != null) {
          final String matchedBrace = match.group(0)!;
          if (useRelaxedLexer) {
            final Match? whitespaceMatch = whitespace.matchAsPrefix(messageString, match.end);
            final int endOfWhitespace =
                whitespaceMatch?.group(0) == null ? match.end : whitespaceMatch!.end;
            final Match? identifierMatch = alphanumeric.matchAsPrefix(
              messageString,
              endOfWhitespace,
            );
            // If we match a "}" and the depth is 0, treat it as a string.
            // If we match a "{" and the next token is not a valid placeholder, treat it as a string.
            if (matchedBrace == '}' && depth == 0) {
              tokens.add(Node.string(startIndex, matchedBrace));
              startIndex = match.end;
              continue;
            }
            if (matchedBrace == '{' &&
                (identifierMatch == null || !placeholders!.contains(identifierMatch.group(0)))) {
              tokens.add(Node.string(startIndex, matchedBrace));
              startIndex = match.end;
              continue;
            }
          }
          tokens.add(Node.brace(startIndex, match.group(0)!));
          isString = false;
          startIndex = match.end;
          depth += 1;
          continue;
        }
        // Theoretically, we only reach this point because of unmatched single quotes because
        // 1. If it begins with single quotes, then we match the longest string contained in single quotes.
        // 2. If it begins with braces, then we match those braces.
        // 3. Else the first character is neither single quote or brace so it is matched by RegExp "unescapedString"
        throw L10nParserException(
          'ICU Lexing Error: Unmatched single quotes.',
          filename,
          messageId,
          messageString,
          startIndex,
        );
      } else {
        RegExp matcher;
        ST? matchedType;

        // Try to match tokens until we succeed
        for (matchedType in matchers.keys) {
          matcher = matchers[matchedType]!;
          match = matcher.matchAsPrefix(messageString, startIndex);
          if (match != null) {
            break;
          }
        }

        if (match == null) {
          match = brace.matchAsPrefix(messageString, startIndex);
          if (match != null) {
            final String matchedBrace = match.group(0)!;
            tokens.add(Node.brace(startIndex, matchedBrace));
            isString = true;
            startIndex = match.end;
            if (matchedBrace == '{') {
              depth += 1;
            } else {
              depth -= 1;
            }
            continue;
          }
          // This should only happen when there are special characters we are unable to match.
          throw L10nParserException(
            'ICU Lexing Error: Unexpected character.',
            filename,
            messageId,
            messageString,
            startIndex,
          );
        } else if (matchedType == ST.empty) {
          // Do not add whitespace as a token.
          startIndex = match.end;
          continue;
        } else if (<ST>[ST.identifier].contains(matchedType) && tokens.last.type == ST.openBrace) {
          // Treat any token as identifier if it comes right after an open brace, whether it's a keyword or not.
          tokens.add(Node(ST.identifier, startIndex, value: match.group(0)));
          startIndex = match.end;
          continue;
        } else {
          // Handle keywords separately. Otherwise, lexer will assume parts of identifiers may be keywords.
          final String tokenStr = match.group(0)!;
          switch (tokenStr) {
            case 'plural':
              matchedType = ST.plural;
            case 'select':
              matchedType = ST.select;
            case 'other':
              matchedType = ST.other;
            case 'date':
              matchedType = ST.date;
            case 'time':
              matchedType = ST.time;
          }
          tokens.add(Node(matchedType!, startIndex, value: match.group(0)));
          startIndex = match.end;
          continue;
        }
      }
    }
    return tokens;
  }

  Node parseIntoTree() {
    final List<Node> tokens = lexIntoTokens();
    final List<ST> parsingStack = <ST>[ST.message];
    final Node syntaxTree = Node(ST.empty, 0, expectedSymbolCount: 1);
    final List<Node> treeTraversalStack = <Node>[syntaxTree];

    // Helper function for parsing and constructing tree.
    void parseAndConstructNode(ST nonterminal, int ruleIndex) {
      final Node parent = treeTraversalStack.last;
      final List<ST> grammarRule = grammar[nonterminal]![ruleIndex];

      // When we run out of tokens, just use -1 to represent the last index.
      final int positionInMessage = tokens.isNotEmpty ? tokens.first.positionInMessage : -1;
      final Node node = Node(
        nonterminal,
        positionInMessage,
        expectedSymbolCount: grammarRule.length,
      );
      parsingStack.addAll(grammarRule.reversed);

      // For tree construction, add nodes to the parent until the parent has all
      // the children it is expecting.
      parent.children.add(node);
      if (parent.isFull) {
        treeTraversalStack.removeLast();
      }
      treeTraversalStack.add(node);
    }

    while (parsingStack.isNotEmpty) {
      final ST symbol = parsingStack.removeLast();

      // Figure out which production rule to use.
      switch (symbol) {
        case ST.message:
          if (tokens.isEmpty) {
            parseAndConstructNode(ST.message, 5);
          } else if (tokens[0].type == ST.closeBrace) {
            parseAndConstructNode(ST.message, 5);
          } else if (tokens[0].type == ST.string) {
            parseAndConstructNode(ST.message, 0);
          } else if (tokens[0].type == ST.openBrace) {
            if (3 < tokens.length && tokens[3].type == ST.plural) {
              parseAndConstructNode(ST.message, 2);
            } else if (3 < tokens.length && tokens[3].type == ST.select) {
              parseAndConstructNode(ST.message, 3);
            } else if (3 < tokens.length &&
                (tokens[3].type == ST.date || tokens[3].type == ST.time)) {
              parseAndConstructNode(ST.message, 4);
            } else {
              parseAndConstructNode(ST.message, 1);
            }
          } else {
            // Theoretically, we can never get here.
            throw L10nException('ICU Syntax Error.');
          }
        case ST.placeholderExpr:
          parseAndConstructNode(ST.placeholderExpr, 0);
        case ST.argumentExpr:
          parseAndConstructNode(ST.argumentExpr, 0);
        case ST.argType:
          if (tokens.isNotEmpty && tokens[0].type == ST.date) {
            parseAndConstructNode(ST.argType, 0);
          } else if (tokens.isNotEmpty && tokens[0].type == ST.time) {
            parseAndConstructNode(ST.argType, 1);
          } else {
            throw L10nException('ICU Syntax Error. Found unknown argument type.');
          }
        case ST.pluralExpr:
          parseAndConstructNode(ST.pluralExpr, 0);
        case ST.pluralParts:
          if (tokens.isNotEmpty &&
              (tokens[0].type == ST.identifier ||
                  tokens[0].type == ST.other ||
                  tokens[0].type == ST.equalSign)) {
            parseAndConstructNode(ST.pluralParts, 0);
          } else {
            parseAndConstructNode(ST.pluralParts, 1);
          }
        case ST.pluralPart:
          if (tokens.isNotEmpty && tokens[0].type == ST.identifier) {
            parseAndConstructNode(ST.pluralPart, 0);
          } else if (tokens.isNotEmpty && tokens[0].type == ST.equalSign) {
            parseAndConstructNode(ST.pluralPart, 1);
          } else if (tokens.isNotEmpty && tokens[0].type == ST.other) {
            parseAndConstructNode(ST.pluralPart, 2);
          } else {
            throw L10nParserException(
              'ICU Syntax Error: Plural parts must be of the form "identifier { message }" or "= number { message }"',
              filename,
              messageId,
              messageString,
              tokens[0].positionInMessage,
            );
          }
        case ST.selectExpr:
          parseAndConstructNode(ST.selectExpr, 0);
        case ST.selectParts:
          if (tokens.isNotEmpty &&
              (tokens[0].type == ST.identifier ||
                  tokens[0].type == ST.number ||
                  tokens[0].type == ST.other)) {
            parseAndConstructNode(ST.selectParts, 0);
          } else {
            parseAndConstructNode(ST.selectParts, 1);
          }
        case ST.selectPart:
          if (tokens.isNotEmpty && tokens[0].type == ST.identifier) {
            parseAndConstructNode(ST.selectPart, 0);
          } else if (tokens.isNotEmpty && tokens[0].type == ST.number) {
            parseAndConstructNode(ST.selectPart, 1);
          } else if (tokens.isNotEmpty && tokens[0].type == ST.other) {
            parseAndConstructNode(ST.selectPart, 2);
          } else {
            throw L10nParserException(
              'ICU Syntax Error: Select parts must be of the form "identifier { message }"',
              filename,
              messageId,
              messageString,
              tokens[0].positionInMessage,
            );
          }
        // At this point, we are only handling terminal symbols.
        // ignore: no_default_cases
        default:
          final Node parent = treeTraversalStack.last;
          // If we match a terminal symbol, then remove it from tokens and
          // add it to the tree.
          if (symbol == ST.empty) {
            parent.children.add(Node.empty(-1));
          } else if (tokens.isEmpty) {
            throw L10nParserException(
              'ICU Syntax Error: Expected "${terminalTypeToString[symbol]}" but found no tokens.',
              filename,
              messageId,
              messageString,
              messageString.length + 1,
            );
          } else if (symbol == tokens[0].type) {
            final Node token = tokens.removeAt(0);
            parent.children.add(token);
          } else {
            throw L10nParserException(
              'ICU Syntax Error: Expected "${terminalTypeToString[symbol]}" but found "${tokens[0].value}".',
              filename,
              messageId,
              messageString,
              tokens[0].positionInMessage,
            );
          }

          if (parent.isFull) {
            treeTraversalStack.removeLast();
          }
      }
    }

    return syntaxTree.children[0];
  }

  final Map<ST, String> terminalTypeToString = <ST, String>{
    ST.openBrace: '{',
    ST.closeBrace: '}',
    ST.comma: ',',
    ST.empty: '',
    ST.identifier: 'identifier',
    ST.number: 'number',
    ST.plural: 'plural',
    ST.select: 'select',
    ST.equalSign: '=',
    ST.other: 'other',
  };

  // Compress the syntax tree.
  //
  // After `parse(lex(message))`, the individual parts (`ST.string`,
  // `ST.placeholderExpr`, `ST.pluralExpr`, and `ST.selectExpr`) are structured
  // as a linked list (see diagram below). This function compresses these parts
  // into a single children array (and does this for `ST.pluralParts` and
  // `ST.selectParts` as well). Then it checks extra syntax rules. Essentially, it
  // converts:
  //
  //             Message
  //             /     \
  //     PluralExpr  Message
  //                 /     \
  //             String  Message
  //                     /     \
  //             SelectExpr   ...
  //
  // ...to:
  //
  //                Message
  //               /   |   \
  //     PluralExpr String SelectExpr ...
  //
  // Keep in mind that this modifies the tree in place and the values of
  // expectedSymbolCount and isFull is no longer useful after this operation.
  Node compress(Node syntaxTree) {
    Node node = syntaxTree;
    final List<Node> children = <Node>[];
    switch (syntaxTree.type) {
      case ST.message:
      case ST.pluralParts:
      case ST.selectParts:
        while (node.children.length == 2) {
          children.add(node.children[0]);
          compress(node.children[0]);
          node = node.children[1];
        }
        syntaxTree.children = children;
      // ignore: no_default_cases
      default:
        node.children.forEach(compress);
    }
    return syntaxTree;
  }

  // Takes in a compressed syntax tree and checks extra rules on
  // plural parts and select parts.
  void checkExtraRules(Node syntaxTree) {
    final List<Node> children = syntaxTree.children;
    switch (syntaxTree.type) {
      case ST.pluralParts:
        // Must have an "other" case.
        if (children.every((Node node) => node.children[0].type != ST.other)) {
          throw L10nParserException(
            'ICU Syntax Error: Plural expressions must have an "other" case.',
            filename,
            messageId,
            messageString,
            syntaxTree.positionInMessage,
          );
        }
        // Identifier must be one of "zero", "one", "two", "few", "many".
        for (final Node node in children) {
          final Node pluralPartFirstToken = node.children[0];
          const List<String> validIdentifiers = <String>['zero', 'one', 'two', 'few', 'many'];
          if (pluralPartFirstToken.type == ST.identifier &&
              !validIdentifiers.contains(pluralPartFirstToken.value)) {
            throw L10nParserException(
              'ICU Syntax Error: Plural expressions case must be one of "zero", "one", "two", "few", "many", or "other".',
              filename,
              messageId,
              messageString,
              node.positionInMessage,
            );
          }
        }
      case ST.selectParts:
        if (children.every((Node node) => node.children[0].type != ST.other)) {
          throw L10nParserException(
            'ICU Syntax Error: Select expressions must have an "other" case.',
            filename,
            messageId,
            messageString,
            syntaxTree.positionInMessage,
          );
        }
      // ignore: no_default_cases
      default:
        break;
    }
    children.forEach(checkExtraRules);
  }

  Node parse() {
    final Node syntaxTree = compress(parseIntoTree());
    checkExtraRules(syntaxTree);
    return syntaxTree;
  }
}
