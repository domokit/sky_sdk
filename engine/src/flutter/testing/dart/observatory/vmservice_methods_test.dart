// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';
import 'dart:developer' as developer;
import 'dart:typed_data';
import 'dart:ui' as ui;

import 'package:test/test.dart';
import 'package:vm_service/vm_service.dart' as vms;
import 'package:vm_service/vm_service_io.dart';

import '../impeller_enabled.dart';

void main() {
  test('Setting invalid directory returns an error', () async {
    vms.VmService? vmService;
    try {
      final developer.ServiceProtocolInfo info = await developer.Service.getInfo();
      if (info.serverUri == null) {
        fail('This test must not be run with --disable-vm-service.');
      }

      vmService = await vmServiceConnectUri(
        'ws://localhost:${info.serverUri!.port}${info.serverUri!.path}ws',
      );
      final String viewId = await getViewId(vmService);

      dynamic error;
      try {
        await vmService.callMethod(
          '_flutter.setAssetBundlePath',
          args: <String, Object>{'viewId': viewId, 'assetDirectory': ''},
        );
      } catch (err) {
        error = err;
      }
      expect(error != null, true);
    } finally {
      await vmService?.dispose();
    }
  });

  test('Can return whether or not impeller is enabled', () async {
    vms.VmService? vmService;
    try {
      final developer.ServiceProtocolInfo info = await developer.Service.getInfo();
      if (info.serverUri == null) {
        fail('This test must not be run with --disable-vm-service.');
      }

      vmService = await vmServiceConnectUri(
        'ws://localhost:${info.serverUri!.port}${info.serverUri!.path}ws',
      );
      final String? isolateId = await getIsolateId(vmService);

      final vms.Response response = await vmService.callServiceExtension(
        'ext.ui.window.impellerEnabled',
        isolateId: isolateId,
      );
      expect(response.json!['enabled'], impellerEnabled);
    } finally {
      await vmService?.dispose();
    }
  });

  test('Reload fonts request sends font change notification', () async {
    vms.VmService? vmService;
    try {
      final developer.ServiceProtocolInfo info = await developer.Service.getInfo();
      if (info.serverUri == null) {
        fail('This test must not be run with --disable-vm-service.');
      }

      final Completer<String> completer = Completer<String>();
      ui.channelBuffers.setListener('flutter/system', (
        ByteData? data,
        ui.PlatformMessageResponseCallback callback,
      ) {
        final ByteBuffer buffer = data!.buffer;
        final Uint8List list = buffer.asUint8List(data.offsetInBytes, data.lengthInBytes);
        completer.complete(utf8.decode(list));
      });

      vmService = await vmServiceConnectUri(
        'ws://localhost:${info.serverUri!.port}${info.serverUri!.path}ws',
      );
      final String viewId = await getViewId(vmService);

      final vms.Response fontChangeResponse = await vmService.callMethod(
        '_flutter.reloadAssetFonts',
        args: <String, Object>{'viewId': viewId},
      );

      expect(fontChangeResponse.type, 'Success');
      expect(await completer.future, '{"type":"fontsChange"}');
    } finally {
      await vmService?.dispose();
      ui.channelBuffers.clearListener('flutter/system');
    }
  });
}

Future<String> getViewId(vms.VmService vmService) async {
  final vms.Response response = await vmService.callMethod('_flutter.listViews');
  final List<Object?>? rawViews = response.json!['views'] as List<Object?>?;
  return (rawViews![0]! as Map<String, Object?>?)!['id']! as String;
}

Future<String?> getIsolateId(vms.VmService vmService) async {
  final vms.VM vm = await vmService.getVM();
  for (final vms.IsolateRef isolate in vm.isolates!) {
    if (isolate.isSystemIsolate ?? false) {
      continue;
    }
    return isolate.id;
  }
  return null;
}
