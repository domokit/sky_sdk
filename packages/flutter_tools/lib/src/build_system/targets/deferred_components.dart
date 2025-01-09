// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:meta/meta.dart';

import '../../android/deferred_components_gen_snapshot_validator.dart';
import '../../base/deferred_component.dart';
import '../../build_info.dart';
import '../../project.dart';
import '../build_system.dart';
import '../depfile.dart';
import 'android.dart';

/// Creates a [DeferredComponentsGenSnapshotValidator], runs the checks, and displays the validator
/// output to the developer if changes are recommended.
class DeferredComponentsGenSnapshotValidatorTarget extends Target {
  /// Create an [AndroidAotDeferredComponentsBundle] implementation for a given [targetPlatform] and [buildMode].
  DeferredComponentsGenSnapshotValidatorTarget({
    required this.deferredComponentsDependencies,
    required this.nonDeferredComponentsDependencies,
    this.title,
    this.exitOnFail = true,
  });

  /// The [AndroidAotDeferredComponentsBundle] derived target instances this rule depends on.
  final List<AndroidAotDeferredComponentsBundle> deferredComponentsDependencies;
  final List<Target> nonDeferredComponentsDependencies;

  /// The title of the [DeferredComponentsGenSnapshotValidator] that is
  /// displayed to the developer when logging results.
  final String? title;

  /// Whether to exit the tool if a recommended change is found by the
  /// [DeferredComponentsGenSnapshotValidator].
  final bool exitOnFail;

  /// The abis to validate.
  List<String> get _abis {
    return <String>[
      for (final AndroidAotDeferredComponentsBundle target in deferredComponentsDependencies)
        if (deferredComponentsTargets.contains(target.name))
          getAndroidArchForName(
            getNameForTargetPlatform(target.dependency.targetPlatform),
          ).archName,
    ];
  }

  @override
  String get name => 'deferred_components_gen_snapshot_validator';

  @override
  List<Source> get inputs => const <Source>[];

  @override
  List<Source> get outputs => const <Source>[];

  @override
  List<String> get depfiles => <String>['flutter_$name.d'];

  @override
  List<Target> get dependencies {
    final List<Target> deps = <Target>[CompositeTarget(deferredComponentsDependencies)];
    deps.addAll(nonDeferredComponentsDependencies);
    return deps;
  }

  @visibleForTesting
  DeferredComponentsGenSnapshotValidator? validator;

  @override
  Future<void> build(Environment environment) async {
    validator = DeferredComponentsGenSnapshotValidator(
      environment,
      title: title,
      exitOnFail: exitOnFail,
    );

    final List<LoadingUnit> generatedLoadingUnits = LoadingUnit.parseGeneratedLoadingUnits(
      environment.outputDir,
      environment.logger,
      abis: _abis,
    );

    validator!
      ..checkAppAndroidManifestComponentLoadingUnitMapping(
        FlutterProject.current().manifest.deferredComponents ?? <DeferredComponent>[],
        generatedLoadingUnits,
      )
      ..checkAgainstLoadingUnitsCache(generatedLoadingUnits)
      ..writeLoadingUnitsCache(generatedLoadingUnits);

    validator!.handleResults();

    environment.depFileService.writeToFile(
      Depfile(validator!.inputs, validator!.outputs),
      environment.buildDir.childFile('flutter_$name.d'),
    );
  }
}
