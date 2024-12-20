// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

const String buildConfigJson = '''
{
  "builds": [
    {
      "archives": [
        {
          "name": "build_name",
          "base_path": "base/path",
          "type": "gcs",
          "include_paths": ["include/path"],
          "realm": "archive_realm"
        }
      ],
      "drone_dimensions": [
        "os=Linux"
      ],
      "gclient_variables": {
        "variable": false
      },
      "gn": ["--gn-arg", "--lto", "--no-rbe"],
      "name": "build_name",
      "description": "build_description",
      "ninja": {
        "config": "build_name",
        "targets": ["ninja_target"]
      },
      "tests": [
        {
          "language": "python3",
          "name": "build_name tests",
          "parameters": ["--test-params"],
          "script": "test/script.py",
          "contexts": ["context"]
        }
      ],
      "generators": {
        "tasks": [
          {
            "name": "generator_task",
            "language": "python",
            "parameters": ["--gen-param"],
            "scripts": ["gen/script.py"]
          }
        ]
      }
    }
  ],
  "generators": {
    "tasks": [
      {
        "name": "global generator task",
        "parameters": ["--global-gen-param"],
        "script": "global/gen_script.dart",
        "language": "dart"
      }
    ]
  },
  "tests": [
    {
      "name": "global test",
      "recipe": "engine_v2/tester_engine",
      "drone_dimensions": [
        "os=Linux"
      ],
      "gclient_variables": {
        "variable": false
      },
      "dependencies": ["dependency"],
      "test_dependencies": [
        {
          "dependency": "test_dependency",
          "version": "git_revision:3a77d0b12c697a840ca0c7705208e8622dc94603"
        }
      ],
      "tasks": [
        {
          "name": "global test task",
          "parameters": ["--test-parameter"],
          "script": "global/test/script.py"
        }
      ]
    }
  ]
}
''';
