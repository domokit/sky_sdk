// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// ignore_for_file: avoid_print

import 'dart:io';

import 'package:analyzer/dart/analysis/features.dart';
import 'package:analyzer/dart/analysis/results.dart';
import 'package:analyzer/dart/analysis/utilities.dart';
import 'package:analyzer/dart/ast/ast.dart';

// Ignore members defined on Object.
const Set<String> _kObjectMembers = <String>{'==', 'toString', 'hashCode'};

CompilationUnit _parseAndCheckDart(String path) {
  final FeatureSet analyzerFeatures = FeatureSet.latestLanguageVersion();
  if (!analyzerFeatures.isEnabled(Feature.non_nullable)) {
    throw Exception('non-nullable feature is disabled.');
  }
  final ParseStringResult result = parseFile(
    path: path,
    featureSet: analyzerFeatures,
    throwIfDiagnostics: false,
  );
  if (result.errors.isNotEmpty) {
    result.errors.forEach(stderr.writeln);
    stderr.writeln('Failure!');
    exit(1);
  }
  return result.unit;
}

void main() {
  final String flutterDir = Platform.environment['FLUTTER_DIR']!;
  // These files just contain imports to the part files;
  final CompilationUnit uiUnit = _parseAndCheckDart('$flutterDir/lib/ui/ui.dart');
  final CompilationUnit webUnit = _parseAndCheckDart('$flutterDir/lib/web_ui/lib/ui.dart');
  final Map<String, ClassDeclaration> uiClasses = <String, ClassDeclaration>{};
  final Map<String, ClassDeclaration> webClasses = <String, ClassDeclaration>{};

  final Map<String, GenericTypeAlias> uiTypeDefs = <String, GenericTypeAlias>{};
  final Map<String, GenericTypeAlias> webTypeDefs = <String, GenericTypeAlias>{};

  // Gather all public classes from each library. For now we are skipping
  // other top level members.
  _collectPublicClasses(uiUnit, uiClasses, '$flutterDir/lib/ui/');
  _collectPublicClasses(webUnit, webClasses, '$flutterDir/lib/web_ui/lib/');

  _collectPublicTypeDefs(uiUnit, uiTypeDefs, '$flutterDir/lib/ui/');
  _collectPublicTypeDefs(webUnit, webTypeDefs, '$flutterDir/lib/web_ui/lib/');

  if (uiClasses.isEmpty || webClasses.isEmpty) {
    print('Warning: did not resolve any classes.');
  }

  if (uiTypeDefs.isEmpty || webTypeDefs.isEmpty) {
    print('Warning: did not resolve any typedefs.');
  }

  bool failed = false;
  print('Checking ${uiClasses.length} public classes.');
  for (final String className in uiClasses.keys) {
    final ClassDeclaration uiClass = uiClasses[className]!;
    final ClassDeclaration? webClass = webClasses[className];
    // If the web class is missing there isn't much left to do here. Print a
    // warning and move along.
    if (webClass == null) {
      failed = true;
      print(
        'Warning: lib/ui/ui.dart contained public class $className, but '
        'this was missing from lib/web_ui/ui.dart.',
      );
      continue;
    }
    // Next will check that the public methods exposed in each library are
    // identical.
    final Map<String, MethodDeclaration> uiMethods = <String, MethodDeclaration>{};
    final Map<String, MethodDeclaration> webMethods = <String, MethodDeclaration>{};
    final Map<String, ConstructorDeclaration> uiConstructors = <String, ConstructorDeclaration>{};
    final Map<String, ConstructorDeclaration> webConstructors = <String, ConstructorDeclaration>{};
    _collectPublicMethods(uiClass, uiMethods);
    _collectPublicMethods(webClass, webMethods);
    _collectPublicConstructors(uiClass, uiConstructors);
    _collectPublicConstructors(webClass, webConstructors);

    for (final String name in uiConstructors.keys) {
      final ConstructorDeclaration uiConstructor = uiConstructors[name]!;
      final ConstructorDeclaration? webConstructor = webConstructors[name];
      if (webConstructor == null) {
        failed = true;
        print('Warning: lib/ui/ui.dart $className.$name is missing from lib/web_ui/ui.dart.');
        continue;
      }

      if (uiConstructor.parameters.parameters.length !=
          webConstructor.parameters.parameters.length) {
        failed = true;
        print(
          'Warning: lib/ui/ui.dart $className.$name has a different parameter '
          'length than in lib/web_ui/ui.dart.',
        );
      }

      for (
        int i = 0;
        i < uiConstructor.parameters.parameters.length &&
            i < uiConstructor.parameters.parameters.length;
        i++
      ) {
        // Technically you could re-order named parameters and still be valid,
        // but we enforce that they are identical.
        for (
          int i = 0;
          i < uiConstructor.parameters.parameters.length &&
              i < webConstructor.parameters.parameters.length;
          i++
        ) {
          final FormalParameter uiParam = uiConstructor.parameters.parameters[i];
          final FormalParameter webParam = webConstructor.parameters.parameters[i];
          if (webParam.name!.lexeme != uiParam.name!.lexeme) {
            failed = true;
            print(
              'Warning: lib/ui/ui.dart $className.$name parameter $i'
              ' ${uiParam.name!.lexeme} has a different name in lib/web_ui/ui.dart.',
            );
          }
          if (uiParam.isPositional != webParam.isPositional) {
            failed = true;
            print(
              'Warning: lib/ui/ui.dart $className.$name parameter $i'
              '${uiParam.name!.lexeme} is positional, but not in lib/web_ui/ui.dart.',
            );
          }
          if (uiParam.isNamed != webParam.isNamed) {
            failed = true;
            print(
              'Warning: lib/ui/ui.dart $className.$name parameter $i'
              '${uiParam.name!.lexeme} is named, but not in lib/web_ui/ui.dart.',
            );
          }
        }
      }
    }

    for (final String methodName in uiMethods.keys) {
      if (_kObjectMembers.contains(methodName)) {
        continue;
      }
      final MethodDeclaration uiMethod = uiMethods[methodName]!;
      final MethodDeclaration? webMethod = webMethods[methodName];
      if (webMethod == null) {
        failed = true;
        print('Warning: lib/ui/ui.dart $className.$methodName is missing from lib/web_ui/ui.dart.');
        continue;
      }
      if (uiMethod.parameters == null || webMethod.parameters == null) {
        continue;
      }
      if (uiMethod.parameters!.parameters.length != webMethod.parameters!.parameters.length) {
        failed = true;
        print(
          'Warning: lib/ui/ui.dart $className.$methodName has a different parameter '
          'length than in lib/web_ui/ui.dart.',
        );
      }
      // Technically you could re-order named parameters and still be valid,
      // but we enforce that they are identical.
      for (
        int i = 0;
        i < uiMethod.parameters!.parameters.length && i < webMethod.parameters!.parameters.length;
        i++
      ) {
        final FormalParameter uiParam = uiMethod.parameters!.parameters[i];
        final FormalParameter webParam = webMethod.parameters!.parameters[i];
        if (webParam.name!.lexeme != uiParam.name!.lexeme) {
          failed = true;
          print(
            'Warning: lib/ui/ui.dart $className.$methodName parameter $i'
            ' ${uiParam.name!.lexeme} has a different name in lib/web_ui/ui.dart.',
          );
        }
        if (uiParam.isPositional != webParam.isPositional) {
          failed = true;
          print(
            'Warning: lib/ui/ui.dart $className.$methodName parameter $i'
            '${uiParam.name!.lexeme} is positional, but not in lib/web_ui/ui.dart.',
          );
        }
        if (uiParam.isNamed != webParam.isNamed) {
          failed = true;
          print(
            'Warning: lib/ui/ui.dart $className.$methodName parameter $i'
            '${uiParam.name!.lexeme} is named, but not in lib/web_ui/ui.dart.',
          );
        }
        // check nullability
        if (uiParam is SimpleFormalParameter && webParam is SimpleFormalParameter) {
          final bool isUiNullable = uiParam.type?.question != null;
          final bool isWebNullable = webParam.type?.question != null;
          if (isUiNullable != isWebNullable) {
            failed = true;
            print(
              'Warning: lib/ui/ui.dart $className.$methodName parameter $i '
              '${uiParam.name} has a different nullability than in lib/web_ui/ui.dart.',
            );
          }
        }
      }
      // check return type.
      if (uiMethod.returnType?.toString() != webMethod.returnType?.toString()) {
        // allow dynamic in web implementation.
        if (webMethod.returnType?.toString() != 'dynamic') {
          failed = true;
          print(
            'Warning: $className.$methodName return type mismatch:\n'
            '  lib/ui/ui.dart     : ${uiMethod.returnType?.toSource()}\n'
            '  lib/web_ui/ui.dart : ${webMethod.returnType?.toSource()}',
          );
        }
      }
    }
  }
  print('Checking ${uiTypeDefs.length} typedefs.');
  for (final String typeDefName in uiTypeDefs.keys) {
    final GenericTypeAlias uiTypeDef = uiTypeDefs[typeDefName]!;
    final GenericTypeAlias? webTypeDef = webTypeDefs[typeDefName];
    // If the web typedef is missing there isn't much left to do here. Print a
    // warning and move along.
    if (webTypeDef == null) {
      failed = true;
      print(
        'Warning: lib/ui/ui.dart contained typedef $typeDefName, but '
        'this was missing from lib/web_ui/ui.dart.',
      );
      continue;
    }

    // uiTypeDef.functionType.parameters
    if (uiTypeDef.functionType?.parameters.parameters == null ||
        webTypeDef.functionType?.parameters.parameters == null) {
      continue;
    }
    if (uiTypeDef.functionType?.parameters.parameters.length !=
        webTypeDef.functionType?.parameters.parameters.length) {
      failed = true;
      print(
        'Warning: lib/ui/ui.dart $typeDefName has a different parameter '
        'length than in lib/web_ui/ui.dart.',
      );
    }
    // Technically you could re-order named parameters and still be valid,
    // but we enforce that they are identical.

    for (
      int i = 0;
      i < uiTypeDef.functionType!.parameters.parameters.length &&
          i < webTypeDef.functionType!.parameters.parameters.length;
      i++
    ) {
      final FormalParameter uiFormalParam =
          (uiTypeDef.type as GenericFunctionType).parameters.parameters[i];
      final FormalParameter webFormalParam =
          (webTypeDef.type as GenericFunctionType).parameters.parameters[i];

      if (uiFormalParam.runtimeType != webFormalParam.runtimeType) {
        failed = true;
        print(
          'Warning: lib/ui/ui.dart $typeDefName parameter $i '
          '${uiFormalParam.name!.lexeme}} is of type ${uiFormalParam.runtimeType}, but of ${webFormalParam.runtimeType} in lib/web_ui/ui.dart.',
        );
      }

      // This is not entirely true and can break, but this way we can support both positional and named params
      // (The assumption that the parameter of a DefaultFormalParameter is a SimpleFormalParameter is a stretch)
      final SimpleFormalParameter uiParam =
          ((uiFormalParam is DefaultFormalParameter) ? uiFormalParam.parameter : uiFormalParam)
              as SimpleFormalParameter;
      final SimpleFormalParameter webParam =
          ((webFormalParam is DefaultFormalParameter) ? webFormalParam.parameter : uiFormalParam)
              as SimpleFormalParameter;

      if (webParam.name == null) {
        failed = true;
        print('Warning: lib/web_ui/ui.dart $typeDefName parameter $i should have name.');
      }
      if (uiParam.name == null) {
        failed = true;
        print('Warning: lib/ui/ui.dart $typeDefName parameter $i should have name.');
      }
      if (webParam.name?.lexeme != uiParam.name?.lexeme) {
        failed = true;
        print(
          'Warning: lib/ui/ui.dart $typeDefName parameter $i '
          '${uiParam.name!.lexeme} has a different name in lib/web_ui/ui.dart.',
        );
      }
      if (uiParam.isPositional != webParam.isPositional) {
        failed = true;
        print(
          'Warning: lib/ui/ui.dart $typeDefName parameter $i '
          '${uiParam.name!.lexeme} is positional, but not in lib/web_ui/ui.dart.',
        );
      }
      if (uiParam.isNamed != webParam.isNamed) {
        failed = true;
        print(
          'Warning: lib/ui/ui.dart $typeDefName parameter $i '
          '${uiParam.name!.lexeme}} is named, but not in lib/web_ui/ui.dart.',
        );
      }

      final bool isUiNullable = uiParam.type?.question != null;
      final bool isWebNullable = webParam.type?.question != null;
      if (isUiNullable != isWebNullable) {
        failed = true;
        print(
          'Warning: lib/ui/ui.dart $typeDefName parameter $i '
          '${uiParam.name} has a different nullability than in lib/web_ui/ui.dart.',
        );
      }
    }

    // check return type.
    if (uiTypeDef.functionType?.returnType?.toString() !=
        webTypeDef.functionType?.returnType?.toString()) {
      // allow dynamic in web implementation.
      if (webTypeDef.functionType?.returnType?.toString() != 'dynamic') {
        failed = true;
        print(
          'Warning: $typeDefName return type mismatch:\n'
          '  lib/ui/ui.dart     : ${uiTypeDef.functionType?.returnType?.toSource()}\n'
          '  lib/web_ui/ui.dart : ${webTypeDef.functionType?.returnType?.toSource()}',
        );
      }
    }
  }
  if (failed) {
    print('Failure!');
    exit(1);
  }
  print('Success!');
  exit(0);
}

// Collects all public classes defined by the part files of [unit].
void _collectPublicClasses(
  CompilationUnit unit,
  Map<String, ClassDeclaration> destination,
  String root,
) {
  for (final Directive directive in unit.directives) {
    if (directive is! PartDirective) {
      continue;
    }
    final PartDirective partDirective = directive;
    final String literalUri = partDirective.uri.toString();
    final CompilationUnit subUnit = _parseAndCheckDart(
      '$root${literalUri.substring(1, literalUri.length - 1)}',
    );
    for (final CompilationUnitMember member in subUnit.declarations) {
      if (member is! ClassDeclaration) {
        continue;
      }
      final ClassDeclaration classDeclaration = member;
      if (classDeclaration.name.lexeme.startsWith('_')) {
        continue;
      }
      destination[classDeclaration.name.lexeme] = classDeclaration;
    }
  }
}

void _collectPublicConstructors(
  ClassDeclaration classDeclaration,
  Map<String, ConstructorDeclaration> destination,
) {
  for (final ClassMember member in classDeclaration.members) {
    if (member is! ConstructorDeclaration) {
      continue;
    }
    final String? methodName = member.name?.lexeme;
    if (methodName == null) {
      destination['Unnamed Constructor'] = member;
      continue;
    }
    if (methodName.startsWith('_')) {
      continue;
    }
    destination[methodName] = member;
  }
}

void _collectPublicMethods(
  ClassDeclaration classDeclaration,
  Map<String, MethodDeclaration> destination,
) {
  for (final ClassMember member in classDeclaration.members) {
    if (member is! MethodDeclaration) {
      continue;
    }
    if (member.name.lexeme.startsWith('_')) {
      continue;
    }
    destination[member.name.lexeme] = member;
  }
}

void _collectPublicTypeDefs(
  CompilationUnit unit,
  Map<String, GenericTypeAlias> destination,
  String root,
) {
  for (final Directive directive in unit.directives) {
    if (directive is! PartDirective) {
      continue;
    }
    final PartDirective partDirective = directive;
    final String literalUri = partDirective.uri.toString();
    final CompilationUnit subUnit = _parseAndCheckDart(
      '$root${literalUri.substring(1, literalUri.length - 1)}',
    );
    for (final CompilationUnitMember member in subUnit.declarations) {
      if (member is! GenericTypeAlias) {
        continue;
      }
      final GenericTypeAlias typeDeclaration = member;
      if (typeDeclaration.name.lexeme.startsWith('_')) {
        continue;
      }
      destination[typeDeclaration.name.lexeme] = typeDeclaration;
    }
  }
}
