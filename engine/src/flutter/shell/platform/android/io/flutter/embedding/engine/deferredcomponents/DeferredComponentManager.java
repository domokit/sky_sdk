// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.engine.deferredcomponents;

import io.flutter.embedding.engine.FlutterJNI;
import io.flutter.embedding.engine.systemchannels.DeferredComponentChannel;

// TODO: add links to external documentation on how to use split aot features.
/**
 * Basic interface that handles downloading and loading of deferred components.
 *
 * <p>Flutter deferred component support is still in early developer preview and should not be used
 * in production apps yet.
 *
 * <p>The Flutter default implementation is PlayStoreDeferredComponentManager.
 *
 * <p>DeferredComponentManager handles the embedder/Android level tasks of downloading, installing,
 * and loading Dart deferred libraries. A typical code-flow begins with a Dart call to loadLibrary()
 * on deferred imported library. See https://dart.dev/guides/language/language-tour#deferred-loading
 * This call retrieves a unique identifier called the loading unit id, which is assigned by
 * gen_snapshot during compilation. The loading unit id is passed down through the engine and
 * invokes installDeferredComponent. Once the component is downloaded, loadAssets and
 * loadDartLibrary should be invoked. loadDartLibrary should find shared library .so files for the
 * engine to open and pass the .so path to FlutterJNI.loadDartDeferredLibrary. loadAssets should
 * typically ensure the new assets are available to the engine's asset manager by passing an updated
 * Android AssetManager to the engine via FlutterJNI.updateAssetManager.
 *
 * <p>The loadAssets and loadDartLibrary methods are separated out because they may also be called
 * manually via platform channel messages. A full installDeferredComponent implementation should
 * call these two methods as needed.
 *
 * <p>A deferred component is uniquely identified by a component name as defined in
 * bundle_config.yaml. Each component may contain one or more loading units, uniquely identified by
 * the loading unit ID and assets.
 */
public interface DeferredComponentManager {
  /**
   * Sets the FlutterJNI to be used to communication with the Flutter native engine.
   *
   * <p>A FlutterJNI is required in order to properly execute loadAssets and loadDartLibrary.
   *
   * <p>Since this class may be instantiated for injection before the FlutterEngine and FlutterJNI
   * is fully initialized, this method should be called to provide the FlutterJNI instance to use
   * for use in loadDartLibrary and loadAssets.
   */
  public abstract void setJNI(FlutterJNI flutterJNI);

  /**
   * Sets the DeferredComponentChannel system channel to handle the framework API to directly call
   * methods in DeferredComponentManager.
   *
   * <p>A DeferredComponentChannel is required to handle assets-only deferred components and
   * manually installed deferred components.
   *
   * <p>Since this class may be instantiated for injection before the FlutterEngine and System
   * Channels are initialized, this method should be called to provide the DeferredComponentChannel.
   * Similarly, the {@link DeferredComponentChannel.setDeferredComponentManager} method should also
   * be called with this DeferredComponentManager instance to properly forward method invocations.
   *
   * <p>The {@link DeferredComponentChannel} passes manual invocations of {@link
   * installDeferredComponent} and {@link getDeferredComponentInstallState} from the method channel
   * to this DeferredComponentManager. Upon completion of the install process, successful
   * installations should notify the DeferredComponentChannel by calling {@link
   * DeferredComponentChannel.completeInstallSuccess} while errors and failures should call {@link
   * DeferredComponentChannel.completeInstallError}.
   */
  public abstract void setDeferredComponentChannel(DeferredComponentChannel channel);

  /**
   * Request that the deferred component be downloaded and installed.
   *
   * <p>This method begins the download and installation of the specified deferred component. For
   * example, the Play Store dynamic delivery implementation uses SplitInstallManager to request the
   * download of the component. Download is not complete when this method returns. The download
   * process should be listened for and upon completion of download, listeners should invoke
   * loadAssets first and then loadDartLibrary to complete the deferred component load process.
   * Assets-only deferred components should also call {@link
   * DeferredComponentChannel.completeInstallSuccess} or {@link
   * DeferredComponentChannel.completeInstallError} to complete the method channel invocation's dart
   * Future.
   *
   * <p>Both parameters are not always necessary to identify which component to install. Asset-only
   * components do not have an associated loadingUnitId. Instead, an invalid ID like -1 may be
   * passed to download only with componentName. On the other hand, it can be possible to resolve
   * the componentName based on the loadingUnitId. This resolution is done if componentName is null.
   * At least one of loadingUnitId or componentName must be valid or non-null.
   *
   * <p>Flutter will typically call this method in two ways. When invoked as part of a dart
   * `loadLibrary()` call, a valid loadingUnitId is passed in while the componentName is null. In
   * this case, this method is responsible for figuring out what component the loadingUnitId
   * corresponds to.
   *
   * <p>When invoked manually as part of loading an assets-only component, loadingUnitId is -1
   * (invalid) and componentName is supplied. Without a loadingUnitId, this method just downloads
   * the component by name and attempts to load assets via loadAssets while loadDartLibrary is
   * skipped, even if the deferred component includes valid dart libs. To load dart libs, call
   * `loadLibrary()` using the first way described in the previous paragraph as the method channel
   * invocation will not load dart shared libraries.
   *
   * <p>While the Future retuned by either `loadLibary` or the method channel invocation will
   * indicate when the code and assets are ready to be used, informational querying of the install
   * process' state can be done with {@link getDeferredComponentInstallState}, though the results of
   * this query should not be used to decide if the deferred component is ready to use. Only the
   * Future completion should be used to do this.
   *
   * @param loadingUnitId The unique identifier associated with a Dart deferred library. This id is
   *     assigned by the compiler and can be seen for reference in bundle_config.yaml. This ID is
   *     primarily used in loadDartLibrary to indicate to Dart which Dart library is being loaded.
   *     Loading unit ids range from 0 to the number existing loading units. Passing a negative
   *     loading unit id indicates that no Dart deferred library should be loaded after download
   *     completes. This is the case when the deferred component is an assets-only component. If a
   *     negative loadingUnitId is passed, then componentName must not be null. Passing a
   *     loadingUnitId larger than the highest valid loading unit's id will cause the Dart
   *     loadLibrary() to complete with a failure.
   * @param componentName The deferred component name as defined in bundle_config.yaml. This may be
   *     null if the deferred component to be loaded is associated with a loading unit/deferred dart
   *     library. In this case, it is this method's responsibility to map the loadingUnitId to its
   *     corresponding componentName. When loading asset-only or other deferred components without
   *     an associated Dart deferred library, loading unit id should a negative value and
   *     componentName must be non-null.
   */
  public abstract void installDeferredComponent(int loadingUnitId, String componentName);

  /**
   * Gets the current state of the installation session corresponding to the specified loadingUnitId
   * and/or componentName.
   *
   * <p>Invocations of {@link installDeferredComponent} typically result in asynchronous downloading
   * and other tasks. This method enables querying of the state of the installation. Querying the
   * installation state is purely informational and does not impact the installation process. The
   * results of this query should not be used to decide if the deferred component is ready to use.
   * Upon completion of installation, the Future returned by the installation request will complete.
   * Only after dart Future completion is it safe to use code and assets from the deferred
   * component.
   *
   * <p>If no deferred component has been installed or requested to be installed by the provided
   * loadingUnitId or componentName, then this method will return null.
   *
   * <p>Depending on the implementation, the returned String may vary. The Play store default
   * implementation begins in the "requested" state before transitioning to the "downloading" and
   * "installed" states.
   *
   * <p>Only successfully requested components have state. Modules that are invalid or have not been
   * requested with {@link installDeferredComponent} will not have a state. Due to the asynchronous
   * nature of the download process, components may not immediately have a valid state upon return
   * of {@link installDeferredComponent}, though valid components will eventually obtain a state.
   *
   * <p>Both parameters are not always necessary to identify which component to install. Asset-only
   * components do not have an associated loadingUnitId. Instead, an invalid ID like -1 may be
   * passed to query only with componentName. On the other hand, it can be possible to resolve the
   * componentName based on the loadingUnitId. This resolution is done if componentName is null. At
   * least one of loadingUnitId or componentName must be valid or non-null.
   *
   * @param loadingUnitId The unique identifier associated with a Dart deferred library.
   * @param componentName The deferred component name as defined in bundle_config.yaml.
   */
  public abstract String getDeferredComponentInstallState(int loadingUnitId, String componentName);

  /**
   * Extract and load any assets and resources from the deferred component for use by Flutter.
   *
   * <p>This method should provide a refreshed AssetManager to FlutterJNI.updateAssetManager that
   * can access the new assets. If no assets are included as part of the deferred component, then
   * nothing needs to be done.
   *
   * <p>If using the Play Store deferred component delivery, refresh the context via: {@code
   * context.createPackageContext(context.getPackageName(), 0);} This returns a new context, from
   * which an updated asset manager may be obtained and passed to updateAssetManager in FlutterJNI.
   * This process does not require loadingUnitId or componentName, however, the two parameters are
   * still present for custom implementations that store assets outside of Android's native system.
   *
   * <p>Assets shoud be loaded before the Dart deferred library is loaded, as successful loading of
   * the Dart loading unit indicates the deferred component is fully loaded. Implementations of
   * installDeferredComponent should invoke this after successful download.
   *
   * @param loadingUnitId The unique identifier associated with a Dart deferred library.
   * @param componentName The deferred component name as defined in bundle_config.yaml.
   */
  public abstract void loadAssets(int loadingUnitId, String componentName);

  /**
   * Load the .so shared library file into the Dart VM.
   *
   * <p>When the download of a deferred component completes, this method should be called to find
   * the path .so library file. The path(s) should then be passed to
   * FlutterJNI.loadDartDeferredLibrary to be dlopen-ed and loaded into the Dart VM.
   *
   * <p>Specifically, APKs distributed by Android's app bundle format may vary by device and API
   * number, so FlutterJNI's loadDartDeferredLibrary accepts a list of search paths with can include
   * paths within APKs that have not been unpacked using the
   * `path/to/apk.apk!path/inside/apk/lib.so` format. Each search path will be attempted in order
   * until a shared library is found. This allows for the developer to avoid unpacking the apk zip.
   *
   * <p>Upon successful load of the Dart library, the Dart future from the originating loadLibary()
   * call completes and developers are able to use symbols and assets from the deferred component.
   *
   * @param loadingUnitId The unique identifier associated with a Dart deferred library. This id is
   *     assigned by the compiler and can be seen for reference in bundle_config.yaml. This ID is
   *     primarily used in loadDartLibrary to indicate to Dart which Dart library is being loaded.
   *     Loading unit ids range from 0 to the number existing loading units. Negative loading unit
   *     ids are considered invalid and this method will result in a no-op.
   * @param componentName The deferred component name as defined in bundle_config.yaml. If using
   *     Play Store deferred component delivery, this name corresponds to the root name on the
   *     installed APKs in which to search for the desired shared library .so file.
   */
  public abstract void loadDartLibrary(int loadingUnitId, String componentName);

  /**
   * Request that the specified component be uninstalled.
   *
   * <p>Since uninstallation requires significant disk i/o, this method only signals the intent to
   * uninstall. Actual uninstallation (eg, removal of assets and files) may occur at a later time.
   * However, once uninstallation is requested, the deferred component should not be used anymore
   * until {@link installDeferredComponent} is called again.
   *
   * <p>Uninstallation, once complete, removes downloaded files and will require redownloading to
   * install again.
   *
   * <p>Both parameters are not always necessary to identify which component to uninstall.
   * Asset-only components do not have an associated loadingUnitId. Instead, an invalid ID like -1
   * may be passed to download only with componentName. On the other hand, it can be possible to
   * resolve the componentName based on the loadingUnitId. This resolution is done if componentName
   * is null. At least one of loadingUnitId or componentName must be valid or non-null.
   *
   * @return false if no deferred component was found matching the input, true if an uninstall was
   *     successfully requested.
   * @param loadingUnitId The unique identifier associated with a Dart deferred library.
   * @param componentName The deferred component name as defined in bundle_config.yaml.
   */
  public abstract boolean uninstallDeferredComponent(int loadingUnitId, String componentName);

  /**
   * Cleans up and releases resources. This object is no longer usable after calling this method.
   */
  public abstract void destroy();
}
