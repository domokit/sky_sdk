// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:yaml/yaml.dart';
import 'package:yaml_edit/yaml_edit.dart' show YamlEditor;

/// Converts a [YamlNode] to a valid YAML-formatted [String].
String encodeYamlAsString(YamlNode contents) {
  final YamlEditor editor = YamlEditor('');
  editor.update(const <String>[], contents);
  return editor.toString();
}
