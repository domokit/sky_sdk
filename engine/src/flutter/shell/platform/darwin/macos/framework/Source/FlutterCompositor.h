// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERCOMPOSITOR_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERCOMPOSITOR_H_

#include <functional>
#include <list>
#include <unordered_map>
#include <variant>

#include "flutter/fml/macros.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterMutatorView.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterPlatformViewController.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterTimeConverter.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterViewProvider.h"
#include "flutter/shell/platform/embedder/embedder.h"

@class FlutterMutatorView;
@class FlutterCursorCoordinator;

namespace flutter {

struct BackingStoreLayer {
  std::vector<FlutterRect> paint_region;
};

using LayerVariant = std::variant<PlatformViewLayer, BackingStoreLayer>;

// FlutterCompositor creates and manages the backing stores used for
// rendering Flutter content and presents Flutter content and Platform views.
// Platform views are not yet supported.
//
// TODO(cbracken): refactor for testability. https://github.com/flutter/flutter/issues/137648
class FlutterCompositor {
 public:
  // Create a FlutterCompositor with a view provider.
  //
  // The view_provider is used to query FlutterViews from view IDs,
  // which are used for presenting and creating backing stores.
  // It must not be null, and is typically FlutterViewEngineProvider.
  FlutterCompositor(id<FlutterViewProvider> view_provider,
                    FlutterTimeConverter* time_converter,
                    FlutterPlatformViewController* platform_views_controller);

  ~FlutterCompositor() = default;

  // Allocate the resources for displaying a view.
  //
  // This method must be called when a view is added to FlutterEngine, and must be
  // called on the main dispatch queue, or an assertion will be thrown.
  void AddView(FlutterViewId view_id);

  // Deallocate the resources for displaying a view.
  //
  // This method must be called when a view is removed from FlutterEngine, and
  // must be called on the main dispatch queue, or an assertion will be thrown.
  void RemoveView(FlutterViewId view_id);

  // Creates a backing store and saves updates the backing_store_out data with
  // the new FlutterBackingStore data.
  //
  // If the backing store is being requested for the first time for a given
  // frame, this compositor does not create a new backing store but rather
  // returns the backing store associated with the FlutterView's
  // FlutterSurfaceManager.
  //
  // Any additional state allocated for the backing store and saved as
  // user_data in the backing store must be collected in the backing_store's
  // destruction_callback field which will be called when the embedder collects
  // the backing store.
  bool CreateBackingStore(const FlutterBackingStoreConfig* config,
                          FlutterBackingStore* backing_store_out);

  // Presents the FlutterLayers by updating the FlutterView specified by
  // `view_id` using the layer content.
  bool Present(FlutterViewIdentifier view_id, const FlutterLayer** layers, size_t layers_count);

  // The number of views that the FlutterCompositor is keeping track of.
  //
  // This method must only be used in unit tests.
  size_t DebugNumViews();

 private:
  // A class that contains the information for a view to be presented.
  class ViewPresenter {
   public:
    ViewPresenter();

    void PresentPlatformViews(FlutterView* default_base_view,
                              const std::vector<LayerVariant>& layers,
                              const FlutterPlatformViewController* platform_views_controller);

   private:
    // Platform view to FlutterMutatorView that contains it.
    NSMapTable<NSView*, FlutterMutatorView*>* mutator_views_;

    // Coordinates mouse cursor changes between platform views and overlays.
    FlutterCursorCoordinator* cursor_coordinator_;

    // Presents the platform view layer represented by `layer`. `layer_index` is
    // used to position the layer in the z-axis. If the layer does not have a
    // superview, it will become subview of `default_base_view`.
    FlutterMutatorView* PresentPlatformView(
        FlutterView* default_base_view,
        const PlatformViewLayer& layer,
        size_t layer_position,
        const FlutterPlatformViewController* platform_views_controller);

    FML_DISALLOW_COPY_AND_ASSIGN(ViewPresenter);
  };

  // Where the compositor can query FlutterViews. Must not be null.
  id<FlutterViewProvider> const view_provider_;

  // Converts between engine time and core animation media time.
  FlutterTimeConverter* const time_converter_;

  // The controller used to manage creation and deletion of platform views.
  const FlutterPlatformViewController* platform_view_controller_;

  // The view presenters for views. Each key is a view ID.
  std::unordered_map<FlutterViewId, ViewPresenter> presenters_;

  FML_DISALLOW_COPY_AND_ASSIGN(FlutterCompositor);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERCOMPOSITOR_H_
