// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.engine.deferredcomponents;

import android.annotation.SuppressLint;
import android.content.Context;
import android.content.pm.ApplicationInfo;
import android.content.pm.PackageManager;
import android.content.pm.PackageManager.NameNotFoundException;
import android.content.res.AssetManager;
import android.os.Build;
import android.os.Bundle;
import android.util.SparseArray;
import android.util.SparseIntArray;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import com.google.android.play.core.splitinstall.SplitInstallException;
import com.google.android.play.core.splitinstall.SplitInstallManager;
import com.google.android.play.core.splitinstall.SplitInstallManagerFactory;
import com.google.android.play.core.splitinstall.SplitInstallRequest;
import com.google.android.play.core.splitinstall.SplitInstallSessionState;
import com.google.android.play.core.splitinstall.SplitInstallStateUpdatedListener;
import com.google.android.play.core.splitinstall.model.SplitInstallErrorCode;
import com.google.android.play.core.splitinstall.model.SplitInstallSessionStatus;
import io.flutter.Log;
import io.flutter.embedding.engine.FlutterJNI;
import io.flutter.embedding.engine.loader.ApplicationInfoLoader;
import io.flutter.embedding.engine.loader.FlutterApplicationInfo;
import io.flutter.embedding.engine.systemchannels.DeferredComponentChannel;
import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Queue;

/**
 * Flutter default implementation of DeferredComponentManager that downloads deferred component from
 * the Google Play store as a dynamic feature module.
 */
public class PlayStoreDeferredComponentManager implements DeferredComponentManager {
  private static final String TAG = "PlayStoreDeferredComponentManager";

  public static final String MAPPING_KEY =
      DeferredComponentManager.class.getName() + ".loadingUnitMapping";

  private @NonNull SplitInstallManager splitInstallManager;
  private @Nullable FlutterJNI flutterJNI;
  private @Nullable DeferredComponentChannel channel;
  private @NonNull Context context;
  private @NonNull FlutterApplicationInfo flutterApplicationInfo;
  // Each request to install a feature module gets a session ID. These maps associate
  // the session ID with the loading unit and component name that was requested.
  private @NonNull SparseArray<String> sessionIdToName;
  private @NonNull SparseIntArray sessionIdToLoadingUnitId;
  private @NonNull SparseArray<String> sessionIdToState;
  private @NonNull Map<String, Integer> nameToSessionId;

  protected @NonNull SparseArray<String> loadingUnitIdToComponentNames;
  protected @NonNull SparseArray<String> loadingUnitIdToSharedLibraryNames;

  private FeatureInstallStateUpdatedListener listener;

  private class FeatureInstallStateUpdatedListener implements SplitInstallStateUpdatedListener {
    @SuppressLint("DefaultLocale")
    public void onStateUpdate(@NonNull SplitInstallSessionState state) {
      int sessionId = state.sessionId();
      if (sessionIdToName.get(sessionId) != null) {
        switch (state.status()) {
          case SplitInstallSessionStatus.FAILED:
            {
              Log.e(
                  TAG,
                  String.format(
                      "Module \"%s\" (sessionId %d) install failed with: %s",
                      sessionIdToName.get(sessionId), sessionId, state.errorCode()));
              flutterJNI.deferredComponentInstallFailure(
                  sessionIdToLoadingUnitId.get(sessionId),
                  "Module install failed with " + state.errorCode(),
                  true);
              if (channel != null) {
                channel.completeInstallError(
                    sessionIdToName.get(sessionId),
                    "Android Deferred Component failed to install.");
              }
              sessionIdToName.delete(sessionId);
              sessionIdToLoadingUnitId.delete(sessionId);
              sessionIdToState.put(sessionId, "failed");
              break;
            }
          case SplitInstallSessionStatus.INSTALLED:
            {
              Log.d(
                  TAG,
                  String.format(
                      "Module \"%s\" (sessionId %d) install successfully.",
                      sessionIdToName.get(sessionId), sessionId));
              loadAssets(sessionIdToLoadingUnitId.get(sessionId), sessionIdToName.get(sessionId));
              // We only load Dart shared lib for the loading unit id requested. Other loading units
              // (if present) in the deferred component are not loaded, but can be loaded by
              // calling again with their loading unit id. If no valid loadingUnitId was included in
              // the installation request such as for an asset only feature, then we can skip this.
              if (sessionIdToLoadingUnitId.get(sessionId) > 0) {
                loadDartLibrary(
                    sessionIdToLoadingUnitId.get(sessionId), sessionIdToName.get(sessionId));
              }
              if (channel != null) {
                channel.completeInstallSuccess(sessionIdToName.get(sessionId));
              }
              sessionIdToName.delete(sessionId);
              sessionIdToLoadingUnitId.delete(sessionId);
              sessionIdToState.put(sessionId, "installed");
              break;
            }
          case SplitInstallSessionStatus.CANCELED:
            {
              Log.d(
                  TAG,
                  String.format(
                      "Module \"%s\" (sessionId %d) install canceled.",
                      sessionIdToName.get(sessionId), sessionId));
              if (channel != null) {
                channel.completeInstallError(
                    sessionIdToName.get(sessionId),
                    "Android Deferred Component installation canceled.");
              }
              sessionIdToName.delete(sessionId);
              sessionIdToLoadingUnitId.delete(sessionId);
              sessionIdToState.put(sessionId, "cancelled");
              break;
            }
          case SplitInstallSessionStatus.CANCELING:
            {
              Log.d(
                  TAG,
                  String.format(
                      "Module \"%s\" (sessionId %d) install canceling.",
                      sessionIdToName.get(sessionId), sessionId));
              sessionIdToState.put(sessionId, "canceling");
              break;
            }
          case SplitInstallSessionStatus.PENDING:
            {
              Log.d(
                  TAG,
                  String.format(
                      "Module \"%s\" (sessionId %d) install pending.",
                      sessionIdToName.get(sessionId), sessionId));
              sessionIdToState.put(sessionId, "pending");
              break;
            }
          case SplitInstallSessionStatus.REQUIRES_USER_CONFIRMATION:
            {
              Log.d(
                  TAG,
                  String.format(
                      "Module \"%s\" (sessionId %d) install requires user confirmation.",
                      sessionIdToName.get(sessionId), sessionId));
              sessionIdToState.put(sessionId, "requiresUserConfirmation");
              break;
            }
          case SplitInstallSessionStatus.DOWNLOADING:
            {
              Log.d(
                  TAG,
                  String.format(
                      "Module \"%s\" (sessionId %d) downloading.",
                      sessionIdToName.get(sessionId), sessionId));
              sessionIdToState.put(sessionId, "downloading");
              break;
            }
          case SplitInstallSessionStatus.DOWNLOADED:
            {
              Log.d(
                  TAG,
                  String.format(
                      "Module \"%s\" (sessionId %d) downloaded.",
                      sessionIdToName.get(sessionId), sessionId));
              sessionIdToState.put(sessionId, "downloaded");
              break;
            }
          case SplitInstallSessionStatus.INSTALLING:
            {
              Log.d(
                  TAG,
                  String.format(
                      "Module \"%s\" (sessionId %d) installing.",
                      sessionIdToName.get(sessionId), sessionId));
              sessionIdToState.put(sessionId, "installing");
              break;
            }
          default:
            Log.d(TAG, "Unknown status: " + state.status());
        }
      }
    }
  }

  public PlayStoreDeferredComponentManager(
      @NonNull Context context, @Nullable FlutterJNI flutterJNI) {
    this.context = context;
    this.flutterJNI = flutterJNI;
    this.flutterApplicationInfo = ApplicationInfoLoader.load(context);
    splitInstallManager = SplitInstallManagerFactory.create(context);
    listener = new FeatureInstallStateUpdatedListener();
    splitInstallManager.registerListener(listener);
    sessionIdToName = new SparseArray<>();
    sessionIdToLoadingUnitId = new SparseIntArray();
    sessionIdToState = new SparseArray<>();
    nameToSessionId = new HashMap<>();

    loadingUnitIdToComponentNames = new SparseArray<>();
    loadingUnitIdToSharedLibraryNames = new SparseArray<>();
    initLoadingUnitMappingToComponentNames();
  }

  public void setJNI(@NonNull FlutterJNI flutterJNI) {
    this.flutterJNI = flutterJNI;
  }

  private boolean verifyJNI() {
    if (flutterJNI == null) {
      Log.e(
          TAG,
          "No FlutterJNI provided. `setJNI` must be called on the DeferredComponentManager before attempting to load dart libraries or invoking with platform channels.");
      return false;
    }
    return true;
  }

  public void setDeferredComponentChannel(@NonNull DeferredComponentChannel channel) {
    this.channel = channel;
  }

  @NonNull
  private ApplicationInfo getApplicationInfo() {
    try {
      return context
          .getPackageManager()
          .getApplicationInfo(context.getPackageName(), PackageManager.GET_META_DATA);
    } catch (NameNotFoundException e) {
      throw new RuntimeException(e);
    }
  }

  // Obtain and parses the metadata string. An example encoded string is:
  //
  //    "2:component2,3:component3,4:component1:libcomponent4.so,5:"
  //
  // Where loading unit 2 is included in component2, loading unit 3 is
  // included in component3, and loading unit 4 is included in component1.
  // An optional third parameter can be added to indicate the name of
  // the shared library of the loading unit. Loading unit 5 maps to an empty
  // string, indicating it is included in the base module and no dynamic
  // feature modules need to be downloaded.
  private void initLoadingUnitMappingToComponentNames() {
    String mappingKey = DeferredComponentManager.class.getName() + ".loadingUnitMapping";
    ApplicationInfo applicationInfo = getApplicationInfo();
    if (applicationInfo != null) {
      Bundle metaData = applicationInfo.metaData;
      if (metaData != null) {
        String rawMappingString = metaData.getString(MAPPING_KEY, null);
        if (rawMappingString == null) {
          Log.e(
              TAG,
              "No loading unit to dynamic feature module name found. Ensure '"
                  + MAPPING_KEY
                  + "' is defined in the base module's AndroidManifest.");
          return;
        }
        if (rawMappingString.equals("")) {
          // Asset-only components, so no loading units to map.
          return;
        }
        for (String entry : rawMappingString.split(",")) {
          // Split with -1 param to include empty string following trailing ":"
          String[] splitEntry = entry.split(":", -1);
          int loadingUnitId = Integer.parseInt(splitEntry[0]);
          loadingUnitIdToComponentNames.put(loadingUnitId, splitEntry[1]);
          if (splitEntry.length > 2) {
            loadingUnitIdToSharedLibraryNames.put(loadingUnitId, splitEntry[2]);
          }
        }
      }
    }
  }

  public void installDeferredComponent(int loadingUnitId, @Nullable String componentName) {
    String resolvedComponentName =
        componentName != null ? componentName : loadingUnitIdToComponentNames.get(loadingUnitId);
    if (resolvedComponentName == null) {
      Log.e(
          TAG, "Deferred component name was null and could not be resolved from loading unit id.");
      return;
    }

    // Handle a loading unit that is included in the base module that does not need download.
    if (resolvedComponentName.equals("") && loadingUnitId > 0) {
      // No need to load assets as base assets are already loaded.
      loadDartLibrary(loadingUnitId, resolvedComponentName);
      return;
    }

    SplitInstallRequest request =
        SplitInstallRequest.newBuilder().addModule(resolvedComponentName).build();

    splitInstallManager
        // Submits the request to install the module through the
        // asynchronous startInstall() task. Your app needs to be
        // in the foreground to submit the request.
        .startInstall(request)
        // Called when the install request is sent successfully. This is different than a successful
        // install which is handled in FeatureInstallStateUpdatedListener.
        .addOnSuccessListener(
            sessionId -> {
              sessionIdToName.put(sessionId, resolvedComponentName);
              sessionIdToLoadingUnitId.put(sessionId, loadingUnitId);
              if (nameToSessionId.containsKey(resolvedComponentName)) {
                sessionIdToState.remove(nameToSessionId.get(resolvedComponentName));
              }
              nameToSessionId.put(resolvedComponentName, sessionId);
              sessionIdToState.put(sessionId, "Requested");
            })
        .addOnFailureListener(
            exception -> {
              switch (((SplitInstallException) exception).getErrorCode()) {
                case SplitInstallErrorCode.NETWORK_ERROR:
                  flutterJNI.deferredComponentInstallFailure(
                      loadingUnitId,
                      "Install of deferred component module \""
                          + componentName
                          + "\" failed with a network error",
                      true);
                  break;
                case SplitInstallErrorCode.MODULE_UNAVAILABLE:
                  flutterJNI.deferredComponentInstallFailure(
                      loadingUnitId,
                      "Install of deferred component module \""
                          + componentName
                          + "\" failed as it is unavailable",
                      false);
                  break;
                default:
                  flutterJNI.deferredComponentInstallFailure(
                      loadingUnitId,
                      String.format(
                          "Install of deferred component module \"%s\" failed with error %d: %s",
                          componentName,
                          ((SplitInstallException) exception).getErrorCode(),
                          ((SplitInstallException) exception).getMessage()),
                      false);
                  break;
              }
            });
  }

  @NonNull
  public String getDeferredComponentInstallState(
      int loadingUnitId, @Nullable String componentName) {
    String resolvedComponentName =
        componentName != null ? componentName : loadingUnitIdToComponentNames.get(loadingUnitId);
    if (resolvedComponentName == null) {
      Log.e(
          TAG, "Deferred component name was null and could not be resolved from loading unit id.");
      return "unknown";
    }
    if (!nameToSessionId.containsKey(resolvedComponentName)) {
      if (splitInstallManager.getInstalledModules().contains(resolvedComponentName)) {
        return "installedPendingLoad";
      }
      return "unknown";
    }
    int sessionId = nameToSessionId.get(resolvedComponentName);
    return sessionIdToState.get(sessionId);
  }

  public void loadAssets(int loadingUnitId, @NonNull String componentName) {
    if (!verifyJNI()) {
      return;
    }
    // Since android deferred component asset manager is handled through
    // context, neither parameter is used here. Assets are stored in
    // the apk's `assets` directory allowing them to be accessed by
    // Android's AssetManager directly.
    try {
      context = context.createPackageContext(context.getPackageName(), 0);

      AssetManager assetManager = context.getAssets();
      flutterJNI.updateJavaAssetManager(assetManager, flutterApplicationInfo.flutterAssetsDir);
    } catch (NameNotFoundException e) {
      throw new RuntimeException(e);
    }
  }

  public void loadDartLibrary(int loadingUnitId, @NonNull String componentName) {
    if (!verifyJNI()) {
      return;
    }
    // Loading unit must be specified and valid to load a dart library.
    if (loadingUnitId < 0) {
      return;
    }

    String aotSharedLibraryName = loadingUnitIdToSharedLibraryNames.get(loadingUnitId);
    if (aotSharedLibraryName == null) {
      // If the filename is not specified, we use dart's loading unit naming convention.
      aotSharedLibraryName =
          flutterApplicationInfo.aotSharedLibraryName + "-" + loadingUnitId + ".part.so";
    }

    // Possible values: armeabi, armeabi-v7a, arm64-v8a, x86, x86_64, mips, mips64
    String abi = Build.SUPPORTED_ABIS[0];
    String pathAbi = abi.replace("-", "_"); // abis are represented with underscores in paths.

    // TODO(garyq): Optimize this apk/file discovery process to use less i/o and be more
    // performant and robust.

    // Search directly in APKs first
    List<String> apkPaths = new ArrayList<>();
    // If not found in APKs, we check in extracted native libs for the lib directly.
    List<String> soPaths = new ArrayList<>();

    Queue<File> searchFiles = new LinkedList<>();
    // Downloaded modules are stored here
    searchFiles.add(context.getFilesDir());
    // The initial installed apks are provided by `sourceDirs` in ApplicationInfo.
    // The jniLibs we want are in the splits not the baseDir. These
    // APKs are only searched as a fallback, as base libs generally do not need
    // to be fully path referenced.
    for (String path : context.getApplicationInfo().splitSourceDirs) {
      searchFiles.add(new File(path));
    }

    while (!searchFiles.isEmpty()) {
      File file = searchFiles.remove();
      if (file != null && file.isDirectory() && file.listFiles() != null) {
        for (File f : file.listFiles()) {
          searchFiles.add(f);
        }
        continue;
      }
      String name = file.getName();
      // Special case for "split_config" since android base module non-master apks are
      // initially installed with the "split_config" prefix/name.
      if (name.endsWith(".apk")
          && (name.startsWith(componentName) || name.startsWith("split_config"))
          && name.contains(pathAbi)) {
        apkPaths.add(file.getAbsolutePath());
        continue;
      }
      if (name.equals(aotSharedLibraryName)) {
        soPaths.add(file.getAbsolutePath());
      }
    }

    List<String> searchPaths = new ArrayList<>();

    // Add the bare filename as the first search path. In some devices, the so
    // file can be dlopen-ed with just the file name.
    searchPaths.add(aotSharedLibraryName);

    for (String path : apkPaths) {
      searchPaths.add(path + "!lib/" + abi + "/" + aotSharedLibraryName);
    }
    for (String path : soPaths) {
      searchPaths.add(path);
    }

    flutterJNI.loadDartDeferredLibrary(
        loadingUnitId, searchPaths.toArray(new String[searchPaths.size()]));
  }

  public boolean uninstallDeferredComponent(int loadingUnitId, @Nullable String componentName) {
    String resolvedComponentName =
        componentName != null ? componentName : loadingUnitIdToComponentNames.get(loadingUnitId);
    if (resolvedComponentName == null) {
      Log.e(
          TAG, "Deferred component name was null and could not be resolved from loading unit id.");
      return false;
    }
    List<String> modulesToUninstall = new ArrayList<>();
    modulesToUninstall.add(resolvedComponentName);
    splitInstallManager.deferredUninstall(modulesToUninstall);
    if (nameToSessionId.get(resolvedComponentName) != null) {
      sessionIdToState.delete(nameToSessionId.get(resolvedComponentName));
    }
    return true;
  }

  public void destroy() {
    splitInstallManager.unregisterListener(listener);
    channel = null;
    flutterJNI = null;
  }
}
