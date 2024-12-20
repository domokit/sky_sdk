// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

//
//  Generated code. Do not modify.
//  source: conductor_state.proto
//
// @dart = 2.12

// ignore_for_file: annotate_overrides, camel_case_types
// ignore_for_file: constant_identifier_names, library_prefixes
// ignore_for_file: non_constant_identifier_names, prefer_final_fields
// ignore_for_file: unnecessary_import, unnecessary_this, unused_import

import 'dart:convert' as $convert;
import 'dart:core' as $core;
import 'dart:typed_data' as $typed_data;

@$core.Deprecated('Use releasePhaseDescriptor instead')
const ReleasePhase$json = {
  '1': 'ReleasePhase',
  '2': [
    {'1': 'APPLY_ENGINE_CHERRYPICKS', '2': 0},
    {'1': 'VERIFY_ENGINE_CI', '2': 1},
    {'1': 'APPLY_FRAMEWORK_CHERRYPICKS', '2': 2},
    {'1': 'PUBLISH_VERSION', '2': 3},
    {'1': 'VERIFY_RELEASE', '2': 5},
    {'1': 'RELEASE_COMPLETED', '2': 6},
  ],
  '4': [
    {'1': 4, '2': 4},
  ],
};

/// Descriptor for `ReleasePhase`. Decode as a `google.protobuf.EnumDescriptorProto`.
final $typed_data.Uint8List releasePhaseDescriptor = $convert
    .base64Decode('CgxSZWxlYXNlUGhhc2USHAoYQVBQTFlfRU5HSU5FX0NIRVJSWVBJQ0tTEAASFAoQVkVSSUZZX0'
        'VOR0lORV9DSRABEh8KG0FQUExZX0ZSQU1FV09SS19DSEVSUllQSUNLUxACEhMKD1BVQkxJU0hf'
        'VkVSU0lPThADEhIKDlZFUklGWV9SRUxFQVNFEAUSFQoRUkVMRUFTRV9DT01QTEVURUQQBiIECA'
        'QQBA==');

@$core.Deprecated('Use cherrypickStateDescriptor instead')
const CherrypickState$json = {
  '1': 'CherrypickState',
  '2': [
    {'1': 'PENDING', '2': 0},
    {'1': 'PENDING_WITH_CONFLICT', '2': 1},
    {'1': 'COMPLETED', '2': 2},
    {'1': 'ABANDONED', '2': 3},
  ],
};

/// Descriptor for `CherrypickState`. Decode as a `google.protobuf.EnumDescriptorProto`.
final $typed_data.Uint8List cherrypickStateDescriptor = $convert
    .base64Decode('Cg9DaGVycnlwaWNrU3RhdGUSCwoHUEVORElORxAAEhkKFVBFTkRJTkdfV0lUSF9DT05GTElDVB'
        'ABEg0KCUNPTVBMRVRFRBACEg0KCUFCQU5ET05FRBAD');

@$core.Deprecated('Use releaseTypeDescriptor instead')
const ReleaseType$json = {
  '1': 'ReleaseType',
  '2': [
    {'1': 'STABLE_INITIAL', '2': 0},
    {'1': 'STABLE_HOTFIX', '2': 1},
    {'1': 'BETA_INITIAL', '2': 2},
    {'1': 'BETA_HOTFIX', '2': 3},
  ],
};

/// Descriptor for `ReleaseType`. Decode as a `google.protobuf.EnumDescriptorProto`.
final $typed_data.Uint8List releaseTypeDescriptor = $convert
    .base64Decode('CgtSZWxlYXNlVHlwZRISCg5TVEFCTEVfSU5JVElBTBAAEhEKDVNUQUJMRV9IT1RGSVgQARIQCg'
        'xCRVRBX0lOSVRJQUwQAhIPCgtCRVRBX0hPVEZJWBAD');

@$core.Deprecated('Use remoteDescriptor instead')
const Remote$json = {
  '1': 'Remote',
  '2': [
    {'1': 'name', '3': 1, '4': 1, '5': 9, '10': 'name'},
    {'1': 'url', '3': 2, '4': 1, '5': 9, '10': 'url'},
  ],
};

/// Descriptor for `Remote`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List remoteDescriptor =
    $convert.base64Decode('CgZSZW1vdGUSEgoEbmFtZRgBIAEoCVIEbmFtZRIQCgN1cmwYAiABKAlSA3VybA==');

@$core.Deprecated('Use cherrypickDescriptor instead')
const Cherrypick$json = {
  '1': 'Cherrypick',
  '2': [
    {'1': 'trunkRevision', '3': 1, '4': 1, '5': 9, '10': 'trunkRevision'},
    {'1': 'appliedRevision', '3': 2, '4': 1, '5': 9, '10': 'appliedRevision'},
    {'1': 'state', '3': 3, '4': 1, '5': 14, '6': '.conductor_state.CherrypickState', '10': 'state'},
  ],
};

/// Descriptor for `Cherrypick`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List cherrypickDescriptor = $convert
    .base64Decode('CgpDaGVycnlwaWNrEiQKDXRydW5rUmV2aXNpb24YASABKAlSDXRydW5rUmV2aXNpb24SKAoPYX'
        'BwbGllZFJldmlzaW9uGAIgASgJUg9hcHBsaWVkUmV2aXNpb24SNgoFc3RhdGUYAyABKA4yIC5j'
        'b25kdWN0b3Jfc3RhdGUuQ2hlcnJ5cGlja1N0YXRlUgVzdGF0ZQ==');

@$core.Deprecated('Use repositoryDescriptor instead')
const Repository$json = {
  '1': 'Repository',
  '2': [
    {'1': 'candidateBranch', '3': 1, '4': 1, '5': 9, '10': 'candidateBranch'},
    {'1': 'startingGitHead', '3': 2, '4': 1, '5': 9, '10': 'startingGitHead'},
    {'1': 'currentGitHead', '3': 3, '4': 1, '5': 9, '10': 'currentGitHead'},
    {'1': 'checkoutPath', '3': 4, '4': 1, '5': 9, '10': 'checkoutPath'},
    {'1': 'upstream', '3': 5, '4': 1, '5': 11, '6': '.conductor_state.Remote', '10': 'upstream'},
    {'1': 'mirror', '3': 6, '4': 1, '5': 11, '6': '.conductor_state.Remote', '10': 'mirror'},
    {
      '1': 'cherrypicks',
      '3': 7,
      '4': 3,
      '5': 11,
      '6': '.conductor_state.Cherrypick',
      '10': 'cherrypicks'
    },
    {'1': 'dartRevision', '3': 8, '4': 1, '5': 9, '10': 'dartRevision'},
    {'1': 'workingBranch', '3': 9, '4': 1, '5': 9, '10': 'workingBranch'},
  ],
};

/// Descriptor for `Repository`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List repositoryDescriptor = $convert
    .base64Decode('CgpSZXBvc2l0b3J5EigKD2NhbmRpZGF0ZUJyYW5jaBgBIAEoCVIPY2FuZGlkYXRlQnJhbmNoEi'
        'gKD3N0YXJ0aW5nR2l0SGVhZBgCIAEoCVIPc3RhcnRpbmdHaXRIZWFkEiYKDmN1cnJlbnRHaXRI'
        'ZWFkGAMgASgJUg5jdXJyZW50R2l0SGVhZBIiCgxjaGVja291dFBhdGgYBCABKAlSDGNoZWNrb3'
        'V0UGF0aBIzCgh1cHN0cmVhbRgFIAEoCzIXLmNvbmR1Y3Rvcl9zdGF0ZS5SZW1vdGVSCHVwc3Ry'
        'ZWFtEi8KBm1pcnJvchgGIAEoCzIXLmNvbmR1Y3Rvcl9zdGF0ZS5SZW1vdGVSBm1pcnJvchI9Cg'
        'tjaGVycnlwaWNrcxgHIAMoCzIbLmNvbmR1Y3Rvcl9zdGF0ZS5DaGVycnlwaWNrUgtjaGVycnlw'
        'aWNrcxIiCgxkYXJ0UmV2aXNpb24YCCABKAlSDGRhcnRSZXZpc2lvbhIkCg13b3JraW5nQnJhbm'
        'NoGAkgASgJUg13b3JraW5nQnJhbmNo');

@$core.Deprecated('Use conductorStateDescriptor instead')
const ConductorState$json = {
  '1': 'ConductorState',
  '2': [
    {'1': 'releaseChannel', '3': 1, '4': 1, '5': 9, '10': 'releaseChannel'},
    {'1': 'releaseVersion', '3': 2, '4': 1, '5': 9, '10': 'releaseVersion'},
    {'1': 'engine', '3': 4, '4': 1, '5': 11, '6': '.conductor_state.Repository', '10': 'engine'},
    {
      '1': 'framework',
      '3': 5,
      '4': 1,
      '5': 11,
      '6': '.conductor_state.Repository',
      '10': 'framework'
    },
    {'1': 'createdDate', '3': 6, '4': 1, '5': 3, '10': 'createdDate'},
    {'1': 'lastUpdatedDate', '3': 7, '4': 1, '5': 3, '10': 'lastUpdatedDate'},
    {'1': 'logs', '3': 8, '4': 3, '5': 9, '10': 'logs'},
    {
      '1': 'currentPhase',
      '3': 9,
      '4': 1,
      '5': 14,
      '6': '.conductor_state.ReleasePhase',
      '10': 'currentPhase'
    },
    {'1': 'conductorVersion', '3': 10, '4': 1, '5': 9, '10': 'conductorVersion'},
    {
      '1': 'releaseType',
      '3': 11,
      '4': 1,
      '5': 14,
      '6': '.conductor_state.ReleaseType',
      '10': 'releaseType'
    },
  ],
};

/// Descriptor for `ConductorState`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List conductorStateDescriptor = $convert
    .base64Decode('Cg5Db25kdWN0b3JTdGF0ZRImCg5yZWxlYXNlQ2hhbm5lbBgBIAEoCVIOcmVsZWFzZUNoYW5uZW'
        'wSJgoOcmVsZWFzZVZlcnNpb24YAiABKAlSDnJlbGVhc2VWZXJzaW9uEjMKBmVuZ2luZRgEIAEo'
        'CzIbLmNvbmR1Y3Rvcl9zdGF0ZS5SZXBvc2l0b3J5UgZlbmdpbmUSOQoJZnJhbWV3b3JrGAUgAS'
        'gLMhsuY29uZHVjdG9yX3N0YXRlLlJlcG9zaXRvcnlSCWZyYW1ld29yaxIgCgtjcmVhdGVkRGF0'
        'ZRgGIAEoA1ILY3JlYXRlZERhdGUSKAoPbGFzdFVwZGF0ZWREYXRlGAcgASgDUg9sYXN0VXBkYX'
        'RlZERhdGUSEgoEbG9ncxgIIAMoCVIEbG9ncxJBCgxjdXJyZW50UGhhc2UYCSABKA4yHS5jb25k'
        'dWN0b3Jfc3RhdGUuUmVsZWFzZVBoYXNlUgxjdXJyZW50UGhhc2USKgoQY29uZHVjdG9yVmVyc2'
        'lvbhgKIAEoCVIQY29uZHVjdG9yVmVyc2lvbhI+CgtyZWxlYXNlVHlwZRgLIAEoDjIcLmNvbmR1'
        'Y3Rvcl9zdGF0ZS5SZWxlYXNlVHlwZVILcmVsZWFzZVR5cGU=');
