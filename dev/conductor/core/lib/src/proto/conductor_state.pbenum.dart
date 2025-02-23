// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

//
//  Generated code. Do not modify.
//  source: conductor_state.proto
//
// @dart = 2.12

// ignore_for_file: annotate_overrides, camel_case_types, comment_references
// ignore_for_file: constant_identifier_names, library_prefixes
// ignore_for_file: non_constant_identifier_names, prefer_final_fields
// ignore_for_file: unnecessary_import, unnecessary_this, unused_import

import 'dart:core' as $core;

import 'package:protobuf/protobuf.dart' as $pb;

class ReleasePhase extends $pb.ProtobufEnum {
  static const ReleasePhase APPLY_FRAMEWORK_CHERRYPICKS =
      ReleasePhase._(0, _omitEnumNames ? '' : 'APPLY_FRAMEWORK_CHERRYPICKS');
  static const ReleasePhase UPDATE_ENGINE_VERSION =
      ReleasePhase._(1, _omitEnumNames ? '' : 'UPDATE_ENGINE_VERSION');
  static const ReleasePhase PUBLISH_VERSION =
      ReleasePhase._(2, _omitEnumNames ? '' : 'PUBLISH_VERSION');
  static const ReleasePhase VERIFY_RELEASE =
      ReleasePhase._(3, _omitEnumNames ? '' : 'VERIFY_RELEASE');
  static const ReleasePhase RELEASE_COMPLETED =
      ReleasePhase._(4, _omitEnumNames ? '' : 'RELEASE_COMPLETED');

  static const $core.List<ReleasePhase> values = <ReleasePhase>[
    APPLY_FRAMEWORK_CHERRYPICKS,
    UPDATE_ENGINE_VERSION,
    PUBLISH_VERSION,
    VERIFY_RELEASE,
    RELEASE_COMPLETED,
  ];

  static final $core.Map<$core.int, ReleasePhase> _byValue = $pb.ProtobufEnum.initByValue(values);
  static ReleasePhase? valueOf($core.int value) => _byValue[value];

  const ReleasePhase._($core.int v, $core.String n) : super(v, n);
}

class CherrypickState extends $pb.ProtobufEnum {
  static const CherrypickState PENDING = CherrypickState._(0, _omitEnumNames ? '' : 'PENDING');
  static const CherrypickState PENDING_WITH_CONFLICT =
      CherrypickState._(1, _omitEnumNames ? '' : 'PENDING_WITH_CONFLICT');
  static const CherrypickState COMPLETED = CherrypickState._(2, _omitEnumNames ? '' : 'COMPLETED');
  static const CherrypickState ABANDONED = CherrypickState._(3, _omitEnumNames ? '' : 'ABANDONED');

  static const $core.List<CherrypickState> values = <CherrypickState>[
    PENDING,
    PENDING_WITH_CONFLICT,
    COMPLETED,
    ABANDONED,
  ];

  static final $core.Map<$core.int, CherrypickState> _byValue =
      $pb.ProtobufEnum.initByValue(values);
  static CherrypickState? valueOf($core.int value) => _byValue[value];

  const CherrypickState._($core.int v, $core.String n) : super(v, n);
}

///  The type of release that is being created.
///
///  This determines how the version will be calculated.
class ReleaseType extends $pb.ProtobufEnum {
  static const ReleaseType STABLE_INITIAL =
      ReleaseType._(0, _omitEnumNames ? '' : 'STABLE_INITIAL');
  static const ReleaseType STABLE_HOTFIX = ReleaseType._(1, _omitEnumNames ? '' : 'STABLE_HOTFIX');
  static const ReleaseType BETA_INITIAL = ReleaseType._(2, _omitEnumNames ? '' : 'BETA_INITIAL');
  static const ReleaseType BETA_HOTFIX = ReleaseType._(3, _omitEnumNames ? '' : 'BETA_HOTFIX');

  static const $core.List<ReleaseType> values = <ReleaseType>[
    STABLE_INITIAL,
    STABLE_HOTFIX,
    BETA_INITIAL,
    BETA_HOTFIX,
  ];

  static final $core.Map<$core.int, ReleaseType> _byValue = $pb.ProtobufEnum.initByValue(values);
  static ReleaseType? valueOf($core.int value) => _byValue[value];

  const ReleaseType._($core.int v, $core.String n) : super(v, n);
}

const _omitEnumNames = $core.bool.fromEnvironment('protobuf.omit_enum_names');
