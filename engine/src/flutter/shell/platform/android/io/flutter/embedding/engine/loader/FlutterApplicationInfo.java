// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.engine.loader;

/** Encapsulates all the information that Flutter needs from application manifest. */
public final class FlutterApplicationInfo {
  private static final String DEFAULT_AOT_SHARED_LIBRARY_NAME = "libapp.so";
  private static final String DEFAULT_VM_SNAPSHOT_DATA = "vm_snapshot_data";
  private static final String DEFAULT_ISOLATE_SNAPSHOT_DATA = "isolate_snapshot_data";
  private static final String DEFAULT_FLUTTER_ASSETS_DIR = "flutter_assets";

  public final String aotSharedLibraryName;
  public final String vmSnapshotData;
  public final String isolateSnapshotData;
  public final String flutterAssetsDir;
  public final String domainNetworkPolicy;
  public final String nativeLibraryDir;
  final boolean automaticallyRegisterPlugins;

  public FlutterApplicationInfo(
      String aotSharedLibraryName,
      String vmSnapshotData,
      String isolateSnapshotData,
      String flutterAssetsDir,
      String domainNetworkPolicy,
      String nativeLibraryDir,
      boolean automaticallyRegisterPlugins) {
    this.aotSharedLibraryName =
        aotSharedLibraryName == null ? DEFAULT_AOT_SHARED_LIBRARY_NAME : aotSharedLibraryName;
    this.vmSnapshotData = vmSnapshotData == null ? DEFAULT_VM_SNAPSHOT_DATA : vmSnapshotData;
    this.isolateSnapshotData =
        isolateSnapshotData == null ? DEFAULT_ISOLATE_SNAPSHOT_DATA : isolateSnapshotData;
    this.flutterAssetsDir =
        flutterAssetsDir == null ? DEFAULT_FLUTTER_ASSETS_DIR : flutterAssetsDir;
    this.nativeLibraryDir = nativeLibraryDir;
    this.domainNetworkPolicy = domainNetworkPolicy == null ? "" : domainNetworkPolicy;
    this.automaticallyRegisterPlugins = automaticallyRegisterPlugins;
  }
}
