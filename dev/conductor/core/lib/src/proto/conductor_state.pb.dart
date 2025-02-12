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

import 'package:fixnum/fixnum.dart' as $fixnum;
import 'package:protobuf/protobuf.dart' as $pb;

import 'conductor_state.pbenum.dart';

export 'conductor_state.pbenum.dart';

/// A git remote
class Remote extends $pb.GeneratedMessage {
  factory Remote({
    $core.String? name,
    $core.String? url,
  }) {
    final $result = create();
    if (name != null) {
      $result.name = name;
    }
    if (url != null) {
      $result.url = url;
    }
    return $result;
  }
  Remote._() : super();
  factory Remote.fromBuffer($core.List<$core.int> i,
          [$pb.ExtensionRegistry r = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromBuffer(i, r);
  factory Remote.fromJson($core.String i,
          [$pb.ExtensionRegistry r = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromJson(i, r);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(_omitMessageNames ? '' : 'Remote',
      package: const $pb.PackageName(_omitMessageNames ? '' : 'conductor_state'),
      createEmptyInstance: create)
    ..aOS(1, _omitFieldNames ? '' : 'name')
    ..aOS(2, _omitFieldNames ? '' : 'url')
    ..hasRequiredFields = false;

  @$core.Deprecated('Using this can add significant overhead to your binary. '
      'Use [GeneratedMessageGenericExtensions.deepCopy] instead. '
      'Will be removed in next major version')
  Remote clone() => Remote()..mergeFromMessage(this);
  @$core.Deprecated('Using this can add significant overhead to your binary. '
      'Use [GeneratedMessageGenericExtensions.rebuild] instead. '
      'Will be removed in next major version')
  Remote copyWith(void Function(Remote) updates) =>
      super.copyWith((message) => updates(message as Remote)) as Remote;

  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  static Remote create() => Remote._();
  Remote createEmptyInstance() => create();
  static $pb.PbList<Remote> createRepeated() => $pb.PbList<Remote>();
  @$core.pragma('dart2js:noInline')
  static Remote getDefault() =>
      _defaultInstance ??= $pb.GeneratedMessage.$_defaultFor<Remote>(create);
  static Remote? _defaultInstance;

  @$pb.TagNumber(1)
  $core.String get name => $_getSZ(0);
  @$pb.TagNumber(1)
  set name($core.String v) {
    $_setString(0, v);
  }

  @$pb.TagNumber(1)
  $core.bool hasName() => $_has(0);
  @$pb.TagNumber(1)
  void clearName() => clearField(1);

  @$pb.TagNumber(2)
  $core.String get url => $_getSZ(1);
  @$pb.TagNumber(2)
  set url($core.String v) {
    $_setString(1, v);
  }

  @$pb.TagNumber(2)
  $core.bool hasUrl() => $_has(1);
  @$pb.TagNumber(2)
  void clearUrl() => clearField(2);
}

class Cherrypick extends $pb.GeneratedMessage {
  factory Cherrypick({
    $core.String? trunkRevision,
    $core.String? appliedRevision,
    CherrypickState? state,
  }) {
    final $result = create();
    if (trunkRevision != null) {
      $result.trunkRevision = trunkRevision;
    }
    if (appliedRevision != null) {
      $result.appliedRevision = appliedRevision;
    }
    if (state != null) {
      $result.state = state;
    }
    return $result;
  }
  Cherrypick._() : super();
  factory Cherrypick.fromBuffer($core.List<$core.int> i,
          [$pb.ExtensionRegistry r = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromBuffer(i, r);
  factory Cherrypick.fromJson($core.String i,
          [$pb.ExtensionRegistry r = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromJson(i, r);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(_omitMessageNames ? '' : 'Cherrypick',
      package: const $pb.PackageName(_omitMessageNames ? '' : 'conductor_state'),
      createEmptyInstance: create)
    ..aOS(1, _omitFieldNames ? '' : 'trunkRevision', protoName: 'trunkRevision')
    ..aOS(2, _omitFieldNames ? '' : 'appliedRevision', protoName: 'appliedRevision')
    ..e<CherrypickState>(3, _omitFieldNames ? '' : 'state', $pb.PbFieldType.OE,
        defaultOrMaker: CherrypickState.PENDING,
        valueOf: CherrypickState.valueOf,
        enumValues: CherrypickState.values)
    ..hasRequiredFields = false;

  @$core.Deprecated('Using this can add significant overhead to your binary. '
      'Use [GeneratedMessageGenericExtensions.deepCopy] instead. '
      'Will be removed in next major version')
  Cherrypick clone() => Cherrypick()..mergeFromMessage(this);
  @$core.Deprecated('Using this can add significant overhead to your binary. '
      'Use [GeneratedMessageGenericExtensions.rebuild] instead. '
      'Will be removed in next major version')
  Cherrypick copyWith(void Function(Cherrypick) updates) =>
      super.copyWith((message) => updates(message as Cherrypick)) as Cherrypick;

  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  static Cherrypick create() => Cherrypick._();
  Cherrypick createEmptyInstance() => create();
  static $pb.PbList<Cherrypick> createRepeated() => $pb.PbList<Cherrypick>();
  @$core.pragma('dart2js:noInline')
  static Cherrypick getDefault() =>
      _defaultInstance ??= $pb.GeneratedMessage.$_defaultFor<Cherrypick>(create);
  static Cherrypick? _defaultInstance;

  /// The revision on trunk to cherrypick.
  @$pb.TagNumber(1)
  $core.String get trunkRevision => $_getSZ(0);
  @$pb.TagNumber(1)
  set trunkRevision($core.String v) {
    $_setString(0, v);
  }

  @$pb.TagNumber(1)
  $core.bool hasTrunkRevision() => $_has(0);
  @$pb.TagNumber(1)
  void clearTrunkRevision() => clearField(1);

  /// Once applied, the actual commit revision of the cherrypick.
  @$pb.TagNumber(2)
  $core.String get appliedRevision => $_getSZ(1);
  @$pb.TagNumber(2)
  set appliedRevision($core.String v) {
    $_setString(1, v);
  }

  @$pb.TagNumber(2)
  $core.bool hasAppliedRevision() => $_has(1);
  @$pb.TagNumber(2)
  void clearAppliedRevision() => clearField(2);

  @$pb.TagNumber(3)
  CherrypickState get state => $_getN(2);
  @$pb.TagNumber(3)
  set state(CherrypickState v) {
    setField(3, v);
  }

  @$pb.TagNumber(3)
  $core.bool hasState() => $_has(2);
  @$pb.TagNumber(3)
  void clearState() => clearField(3);
}

class Repository extends $pb.GeneratedMessage {
  factory Repository({
    $core.String? candidateBranch,
    $core.String? startingGitHead,
    $core.String? currentGitHead,
    $core.String? checkoutPath,
    Remote? upstream,
    Remote? mirror,
    $core.Iterable<Cherrypick>? cherrypicks,
    $core.String? dartRevision,
    $core.String? workingBranch,
  }) {
    final $result = create();
    if (candidateBranch != null) {
      $result.candidateBranch = candidateBranch;
    }
    if (startingGitHead != null) {
      $result.startingGitHead = startingGitHead;
    }
    if (currentGitHead != null) {
      $result.currentGitHead = currentGitHead;
    }
    if (checkoutPath != null) {
      $result.checkoutPath = checkoutPath;
    }
    if (upstream != null) {
      $result.upstream = upstream;
    }
    if (mirror != null) {
      $result.mirror = mirror;
    }
    if (cherrypicks != null) {
      $result.cherrypicks.addAll(cherrypicks);
    }
    if (dartRevision != null) {
      $result.dartRevision = dartRevision;
    }
    if (workingBranch != null) {
      $result.workingBranch = workingBranch;
    }
    return $result;
  }
  Repository._() : super();
  factory Repository.fromBuffer($core.List<$core.int> i,
          [$pb.ExtensionRegistry r = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromBuffer(i, r);
  factory Repository.fromJson($core.String i,
          [$pb.ExtensionRegistry r = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromJson(i, r);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(_omitMessageNames ? '' : 'Repository',
      package: const $pb.PackageName(_omitMessageNames ? '' : 'conductor_state'),
      createEmptyInstance: create)
    ..aOS(1, _omitFieldNames ? '' : 'candidateBranch', protoName: 'candidateBranch')
    ..aOS(2, _omitFieldNames ? '' : 'startingGitHead', protoName: 'startingGitHead')
    ..aOS(3, _omitFieldNames ? '' : 'currentGitHead', protoName: 'currentGitHead')
    ..aOS(4, _omitFieldNames ? '' : 'checkoutPath', protoName: 'checkoutPath')
    ..aOM<Remote>(5, _omitFieldNames ? '' : 'upstream', subBuilder: Remote.create)
    ..aOM<Remote>(6, _omitFieldNames ? '' : 'mirror', subBuilder: Remote.create)
    ..pc<Cherrypick>(7, _omitFieldNames ? '' : 'cherrypicks', $pb.PbFieldType.PM,
        subBuilder: Cherrypick.create)
    ..aOS(8, _omitFieldNames ? '' : 'dartRevision', protoName: 'dartRevision')
    ..aOS(9, _omitFieldNames ? '' : 'workingBranch', protoName: 'workingBranch')
    ..hasRequiredFields = false;

  @$core.Deprecated('Using this can add significant overhead to your binary. '
      'Use [GeneratedMessageGenericExtensions.deepCopy] instead. '
      'Will be removed in next major version')
  Repository clone() => Repository()..mergeFromMessage(this);
  @$core.Deprecated('Using this can add significant overhead to your binary. '
      'Use [GeneratedMessageGenericExtensions.rebuild] instead. '
      'Will be removed in next major version')
  Repository copyWith(void Function(Repository) updates) =>
      super.copyWith((message) => updates(message as Repository)) as Repository;

  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  static Repository create() => Repository._();
  Repository createEmptyInstance() => create();
  static $pb.PbList<Repository> createRepeated() => $pb.PbList<Repository>();
  @$core.pragma('dart2js:noInline')
  static Repository getDefault() =>
      _defaultInstance ??= $pb.GeneratedMessage.$_defaultFor<Repository>(create);
  static Repository? _defaultInstance;

  ///  The development git branch the release is based on.
  ///
  ///  Must be of the form /flutter-(\d+)\.(\d+)-candidate\.(\d+)/
  @$pb.TagNumber(1)
  $core.String get candidateBranch => $_getSZ(0);
  @$pb.TagNumber(1)
  set candidateBranch($core.String v) {
    $_setString(0, v);
  }

  @$pb.TagNumber(1)
  $core.bool hasCandidateBranch() => $_has(0);
  @$pb.TagNumber(1)
  void clearCandidateBranch() => clearField(1);

  /// The commit hash at the tip of the branch before cherrypicks were applied.
  @$pb.TagNumber(2)
  $core.String get startingGitHead => $_getSZ(1);
  @$pb.TagNumber(2)
  set startingGitHead($core.String v) {
    $_setString(1, v);
  }

  @$pb.TagNumber(2)
  $core.bool hasStartingGitHead() => $_has(1);
  @$pb.TagNumber(2)
  void clearStartingGitHead() => clearField(2);

  /// The difference in commits between this and [startingGitHead] is the number
  /// of cherrypicks that have been currently applied.
  @$pb.TagNumber(3)
  $core.String get currentGitHead => $_getSZ(2);
  @$pb.TagNumber(3)
  set currentGitHead($core.String v) {
    $_setString(2, v);
  }

  @$pb.TagNumber(3)
  $core.bool hasCurrentGitHead() => $_has(2);
  @$pb.TagNumber(3)
  void clearCurrentGitHead() => clearField(3);

  /// Path to the git checkout on local disk.
  @$pb.TagNumber(4)
  $core.String get checkoutPath => $_getSZ(3);
  @$pb.TagNumber(4)
  set checkoutPath($core.String v) {
    $_setString(3, v);
  }

  @$pb.TagNumber(4)
  $core.bool hasCheckoutPath() => $_has(3);
  @$pb.TagNumber(4)
  void clearCheckoutPath() => clearField(4);

  /// The remote commits will be fetched from.
  @$pb.TagNumber(5)
  Remote get upstream => $_getN(4);
  @$pb.TagNumber(5)
  set upstream(Remote v) {
    setField(5, v);
  }

  @$pb.TagNumber(5)
  $core.bool hasUpstream() => $_has(4);
  @$pb.TagNumber(5)
  void clearUpstream() => clearField(5);
  @$pb.TagNumber(5)
  Remote ensureUpstream() => $_ensure(4);

  ///  The remote cherrypicks will be pushed to create a Pull Request.
  ///
  ///  This should be a mirror owned by the user conducting the release.
  @$pb.TagNumber(6)
  Remote get mirror => $_getN(5);
  @$pb.TagNumber(6)
  set mirror(Remote v) {
    setField(6, v);
  }

  @$pb.TagNumber(6)
  $core.bool hasMirror() => $_has(5);
  @$pb.TagNumber(6)
  void clearMirror() => clearField(6);
  @$pb.TagNumber(6)
  Remote ensureMirror() => $_ensure(5);

  /// Desired cherrypicks.
  @$pb.TagNumber(7)
  $core.List<Cherrypick> get cherrypicks => $_getList(6);

  /// For the repository that has a dart_revision in a DEPS file.
  @$pb.TagNumber(8)
  $core.String get dartRevision => $_getSZ(7);
  @$pb.TagNumber(8)
  set dartRevision($core.String v) {
    $_setString(7, v);
  }

  @$pb.TagNumber(8)
  $core.bool hasDartRevision() => $_has(7);
  @$pb.TagNumber(8)
  void clearDartRevision() => clearField(8);

  ///  Name of local and remote branch for applying cherrypicks.
  ///
  ///  When the pull request is merged, all commits here will be squashed to a
  ///  single commit on the [candidateBranch].
  @$pb.TagNumber(9)
  $core.String get workingBranch => $_getSZ(8);
  @$pb.TagNumber(9)
  set workingBranch($core.String v) {
    $_setString(8, v);
  }

  @$pb.TagNumber(9)
  $core.bool hasWorkingBranch() => $_has(8);
  @$pb.TagNumber(9)
  void clearWorkingBranch() => clearField(9);
}

class ConductorState extends $pb.GeneratedMessage {
  factory ConductorState({
    $core.String? releaseChannel,
    $core.String? releaseVersion,
    Repository? engine,
    Repository? framework,
    $fixnum.Int64? createdDate,
    $fixnum.Int64? lastUpdatedDate,
    $core.Iterable<$core.String>? logs,
    ReleasePhase? currentPhase,
    $core.String? conductorVersion,
    ReleaseType? releaseType,
  }) {
    final $result = create();
    if (releaseChannel != null) {
      $result.releaseChannel = releaseChannel;
    }
    if (releaseVersion != null) {
      $result.releaseVersion = releaseVersion;
    }
    if (engine != null) {
      $result.engine = engine;
    }
    if (framework != null) {
      $result.framework = framework;
    }
    if (createdDate != null) {
      $result.createdDate = createdDate;
    }
    if (lastUpdatedDate != null) {
      $result.lastUpdatedDate = lastUpdatedDate;
    }
    if (logs != null) {
      $result.logs.addAll(logs);
    }
    if (currentPhase != null) {
      $result.currentPhase = currentPhase;
    }
    if (conductorVersion != null) {
      $result.conductorVersion = conductorVersion;
    }
    if (releaseType != null) {
      $result.releaseType = releaseType;
    }
    return $result;
  }
  ConductorState._() : super();
  factory ConductorState.fromBuffer($core.List<$core.int> i,
          [$pb.ExtensionRegistry r = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromBuffer(i, r);
  factory ConductorState.fromJson($core.String i,
          [$pb.ExtensionRegistry r = $pb.ExtensionRegistry.EMPTY]) =>
      create()..mergeFromJson(i, r);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(_omitMessageNames ? '' : 'ConductorState',
      package: const $pb.PackageName(_omitMessageNames ? '' : 'conductor_state'),
      createEmptyInstance: create)
    ..aOS(1, _omitFieldNames ? '' : 'releaseChannel', protoName: 'releaseChannel')
    ..aOS(2, _omitFieldNames ? '' : 'releaseVersion', protoName: 'releaseVersion')
    ..aOM<Repository>(4, _omitFieldNames ? '' : 'engine', subBuilder: Repository.create)
    ..aOM<Repository>(5, _omitFieldNames ? '' : 'framework', subBuilder: Repository.create)
    ..aInt64(6, _omitFieldNames ? '' : 'createdDate', protoName: 'createdDate')
    ..aInt64(7, _omitFieldNames ? '' : 'lastUpdatedDate', protoName: 'lastUpdatedDate')
    ..pPS(8, _omitFieldNames ? '' : 'logs')
    ..e<ReleasePhase>(9, _omitFieldNames ? '' : 'currentPhase', $pb.PbFieldType.OE,
        protoName: 'currentPhase',
        defaultOrMaker: ReleasePhase.APPLY_FRAMEWORK_CHERRYPICKS,
        valueOf: ReleasePhase.valueOf,
        enumValues: ReleasePhase.values)
    ..aOS(10, _omitFieldNames ? '' : 'conductorVersion', protoName: 'conductorVersion')
    ..e<ReleaseType>(11, _omitFieldNames ? '' : 'releaseType', $pb.PbFieldType.OE,
        protoName: 'releaseType',
        defaultOrMaker: ReleaseType.STABLE_INITIAL,
        valueOf: ReleaseType.valueOf,
        enumValues: ReleaseType.values)
    ..hasRequiredFields = false;

  @$core.Deprecated('Using this can add significant overhead to your binary. '
      'Use [GeneratedMessageGenericExtensions.deepCopy] instead. '
      'Will be removed in next major version')
  ConductorState clone() => ConductorState()..mergeFromMessage(this);
  @$core.Deprecated('Using this can add significant overhead to your binary. '
      'Use [GeneratedMessageGenericExtensions.rebuild] instead. '
      'Will be removed in next major version')
  ConductorState copyWith(void Function(ConductorState) updates) =>
      super.copyWith((message) => updates(message as ConductorState)) as ConductorState;

  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  static ConductorState create() => ConductorState._();
  ConductorState createEmptyInstance() => create();
  static $pb.PbList<ConductorState> createRepeated() => $pb.PbList<ConductorState>();
  @$core.pragma('dart2js:noInline')
  static ConductorState getDefault() =>
      _defaultInstance ??= $pb.GeneratedMessage.$_defaultFor<ConductorState>(create);
  static ConductorState? _defaultInstance;

  /// One of 'stable', 'beta', or 'dev'
  @$pb.TagNumber(1)
  $core.String get releaseChannel => $_getSZ(0);
  @$pb.TagNumber(1)
  set releaseChannel($core.String v) {
    $_setString(0, v);
  }

  @$pb.TagNumber(1)
  $core.bool hasReleaseChannel() => $_has(0);
  @$pb.TagNumber(1)
  void clearReleaseChannel() => clearField(1);

  /// The name of the release.
  @$pb.TagNumber(2)
  $core.String get releaseVersion => $_getSZ(1);
  @$pb.TagNumber(2)
  set releaseVersion($core.String v) {
    $_setString(1, v);
  }

  @$pb.TagNumber(2)
  $core.bool hasReleaseVersion() => $_has(1);
  @$pb.TagNumber(2)
  void clearReleaseVersion() => clearField(2);

  @$pb.TagNumber(4)
  Repository get engine => $_getN(2);
  @$pb.TagNumber(4)
  set engine(Repository v) {
    setField(4, v);
  }

  @$pb.TagNumber(4)
  $core.bool hasEngine() => $_has(2);
  @$pb.TagNumber(4)
  void clearEngine() => clearField(4);
  @$pb.TagNumber(4)
  Repository ensureEngine() => $_ensure(2);

  @$pb.TagNumber(5)
  Repository get framework => $_getN(3);
  @$pb.TagNumber(5)
  set framework(Repository v) {
    setField(5, v);
  }

  @$pb.TagNumber(5)
  $core.bool hasFramework() => $_has(3);
  @$pb.TagNumber(5)
  void clearFramework() => clearField(5);
  @$pb.TagNumber(5)
  Repository ensureFramework() => $_ensure(3);

  @$pb.TagNumber(6)
  $fixnum.Int64 get createdDate => $_getI64(4);
  @$pb.TagNumber(6)
  set createdDate($fixnum.Int64 v) {
    $_setInt64(4, v);
  }

  @$pb.TagNumber(6)
  $core.bool hasCreatedDate() => $_has(4);
  @$pb.TagNumber(6)
  void clearCreatedDate() => clearField(6);

  @$pb.TagNumber(7)
  $fixnum.Int64 get lastUpdatedDate => $_getI64(5);
  @$pb.TagNumber(7)
  set lastUpdatedDate($fixnum.Int64 v) {
    $_setInt64(5, v);
  }

  @$pb.TagNumber(7)
  $core.bool hasLastUpdatedDate() => $_has(5);
  @$pb.TagNumber(7)
  void clearLastUpdatedDate() => clearField(7);

  @$pb.TagNumber(8)
  $core.List<$core.String> get logs => $_getList(6);

  /// The current [ReleasePhase] that has yet to be completed.
  @$pb.TagNumber(9)
  ReleasePhase get currentPhase => $_getN(7);
  @$pb.TagNumber(9)
  set currentPhase(ReleasePhase v) {
    setField(9, v);
  }

  @$pb.TagNumber(9)
  $core.bool hasCurrentPhase() => $_has(7);
  @$pb.TagNumber(9)
  void clearCurrentPhase() => clearField(9);

  /// A string used to validate that the current conductor is the same version
  /// that created the [ConductorState] object.
  @$pb.TagNumber(10)
  $core.String get conductorVersion => $_getSZ(8);
  @$pb.TagNumber(10)
  set conductorVersion($core.String v) {
    $_setString(8, v);
  }

  @$pb.TagNumber(10)
  $core.bool hasConductorVersion() => $_has(8);
  @$pb.TagNumber(10)
  void clearConductorVersion() => clearField(10);

  @$pb.TagNumber(11)
  ReleaseType get releaseType => $_getN(9);
  @$pb.TagNumber(11)
  set releaseType(ReleaseType v) {
    setField(11, v);
  }

  @$pb.TagNumber(11)
  $core.bool hasReleaseType() => $_has(9);
  @$pb.TagNumber(11)
  void clearReleaseType() => clearField(11);
}

const _omitFieldNames = $core.bool.fromEnvironment('protobuf.omit_field_names');
const _omitMessageNames = $core.bool.fromEnvironment('protobuf.omit_message_names');
