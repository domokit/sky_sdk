// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/memory.dart';
import 'package:flutter_tools/src/project.dart';
import 'package:flutter_tools/src/project_validator.dart';
import 'package:flutter_tools/src/project_validator_result.dart';

import '../src/common.dart';

class ProjectValidatorTaskImpl extends ProjectValidator {
  @override
  Future<List<ProjectValidatorResult>> start(FlutterProject project) async {
    const ProjectValidatorResult error = ProjectValidatorResult(
      name: 'result_1',
      value: 'this is an error',
      status: StatusProjectValidator.error,
    );

    const ProjectValidatorResult success = ProjectValidatorResult(
      name: 'result_2',
      value: 'correct',
      status: StatusProjectValidator.success,
    );

    const ProjectValidatorResult warning = ProjectValidatorResult(
      name: 'result_3',
      value: 'this passed',
      status: StatusProjectValidator.success,
      warning: 'with a warning',
    );

    return <ProjectValidatorResult>[error, success, warning];
  }

  @override
  bool supportsProject(FlutterProject project) {
    return true;
  }

  @override
  String get title => 'Impl';
}

void main() {
  group('ProjectValidatorResult', () {
    testWithoutContext('success status', () {
      const ProjectValidatorResult result = ProjectValidatorResult(
        name: 'name',
        value: 'value',
        status: StatusProjectValidator.success,
      );
      expect(result.toString(), 'name: value');
      expect(result.status, StatusProjectValidator.success);
    });

    testWithoutContext('success status with warning', () {
      const ProjectValidatorResult result = ProjectValidatorResult(
        name: 'name',
        value: 'value',
        status: StatusProjectValidator.success,
        warning: 'my warning',
      );
      expect(result.toString(), 'name: value (warning: my warning)');
      expect(result.status, StatusProjectValidator.success);
    });

    testWithoutContext('error status', () {
      const ProjectValidatorResult result = ProjectValidatorResult(
        name: 'name',
        value: 'my error',
        status: StatusProjectValidator.error,
      );
      expect(result.toString(), 'name: my error');
      expect(result.status, StatusProjectValidator.error);
    });
  });

  group('ProjectValidatorTask', () {
    late ProjectValidatorTaskImpl task;

    setUp(() {
      task = ProjectValidatorTaskImpl();
    });

    testWithoutContext('error status', () async {
      final MemoryFileSystem fs = MemoryFileSystem.test();
      final FlutterProject project = FlutterProject.fromDirectoryTest(fs.currentDirectory);
      final List<ProjectValidatorResult> results = await task.start(project);
      expect(results.length, 3);
      expect(results[0].toString(), 'result_1: this is an error');
      expect(results[1].toString(), 'result_2: correct');
      expect(results[2].toString(), 'result_3: this passed (warning: with a warning)');
    });
  });
}
