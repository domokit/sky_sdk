// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "fake_flatland.h"

#include <zircon/types.h>

#include <algorithm>  // For std::remove_if
#include <memory>

#include "flutter/fml/logging.h"

namespace flutter_runner::testing {

FakeFlatland::FakeFlatland()
    : allocator_binding_(this),
      flatland_binding_(this),
      present_handler_([](auto args) {}) {}

FakeFlatland::~FakeFlatland() = default;

fuchsia::ui::composition::AllocatorHandle FakeFlatland::ConnectAllocator(
    async_dispatcher_t* dispatcher) {
  FML_CHECK(!allocator_binding_.is_bound());

  fuchsia::ui::composition::AllocatorHandle allocator;
  allocator_binding_.Bind(allocator.NewRequest(), dispatcher);

  return allocator;
}

fuchsia::ui::composition::FlatlandHandle FakeFlatland::ConnectFlatland(
    async_dispatcher_t* dispatcher) {
  FML_CHECK(!flatland_binding_.is_bound());

  fuchsia::ui::composition::FlatlandHandle flatland;
  flatland_binding_.Bind(flatland.NewRequest(), dispatcher);

  return flatland;
}

void FakeFlatland::Disconnect(fuchsia::ui::composition::FlatlandError error) {
  flatland_binding_.events().OnError(error);
  flatland_binding_.Unbind();
  allocator_binding_
      .Unbind();  // TODO(fxb/85619): Does the real Scenic unbind this when
                  // Flatland has an error?  Or is it independent?
}

void FakeFlatland::SetPresentHandler(PresentHandler present_handler) {
  present_handler_ =
      present_handler ? std::move(present_handler) : [](auto args) {};
}

void FakeFlatland::FireOnNextFrameBeginEvent(
    fuchsia::ui::composition::OnNextFrameBeginValues
        on_next_frame_begin_values) {
  flatland_binding_.events().OnNextFrameBegin(
      std::move(on_next_frame_begin_values));
}

void FakeFlatland::FireOnFramePresentedEvent(
    fuchsia::scenic::scheduling::FramePresentedInfo frame_presented_info) {
  flatland_binding_.events().OnFramePresented(std::move(frame_presented_info));
}

void FakeFlatland::NotImplemented_(const std::string& name) {
  FML_LOG(FATAL) << "FakeFlatland does not implement " << name;
}

void FakeFlatland::RegisterBufferCollection(
    fuchsia::ui::composition::RegisterBufferCollectionArgs args,
    RegisterBufferCollectionCallback callback) {
  auto [export_token_koid, _] = GetKoids(args.export_token());
  auto [__, emplace_binding_success] =
      graph_bindings_.buffer_collections.emplace(
          export_token_koid,
          BufferCollectionBinding{
              .export_token = std::move(*args.mutable_export_token()),
              .sysmem_token =
                  std::move(*args.mutable_buffer_collection_token()),
              .usage = args.usage(),
          });
  // TODO(fxb/85619): Disconnect the Allocator here
  FML_CHECK(emplace_binding_success)
      << "FakeFlatland::RegisterBufferCollection: BufferCollection already "
         "exists with koid "
      << export_token_koid;
}

void FakeFlatland::Present(fuchsia::ui::composition::PresentArgs args) {
  // Each FIDL call between this `Present()` and the last one mutated the
  // `pending_graph_` independently of the `current_graph_`.  Only the
  // `current_graph_` is visible externally in the test.
  //
  // `Present()` updates the current graph with a deep clone of the pending one.
  current_graph_ = pending_graph_.Clone();

  present_handler_(std::move(args));
}

void FakeFlatland::CreateView(
    fuchsia::ui::views::ViewCreationToken token,
    fidl::InterfaceRequest<fuchsia::ui::composition::ParentViewportWatcher>
        parent_viewport_watcher) {
  CreateView2(std::move(token), fuchsia::ui::views::ViewIdentityOnCreation{},
              fuchsia::ui::composition::ViewBoundProtocols{},
              std::move(parent_viewport_watcher));
}

void FakeFlatland::CreateView2(
    fuchsia::ui::views::ViewCreationToken token,
    fuchsia::ui::views::ViewIdentityOnCreation view_identity,
    fuchsia::ui::composition::ViewBoundProtocols view_protocols,
    fidl::InterfaceRequest<fuchsia::ui::composition::ParentViewportWatcher>
        parent_viewport_watcher) {
  // TODO(fxb/85619): Handle a 2nd CreateView call
  FML_CHECK(!pending_graph_.view.has_value());
  FML_CHECK(!graph_bindings_.viewport_watcher.has_value());

  auto view_token_koids = GetKoids(token);
  auto view_ref_koids = GetKoids(view_identity.view_ref);
  auto view_ref_control_koids = GetKoids(view_identity.view_ref_control);
  FML_CHECK(view_ref_koids.first == view_ref_control_koids.second);
  FML_CHECK(view_ref_koids.second == view_ref_control_koids.first);

  pending_graph_.view.emplace(FakeView{
      .view_token = view_token_koids.first,
      .view_ref = view_ref_koids.first,
      .view_ref_control = view_ref_control_koids.first,
      .view_ref_focused =
          view_protocols.has_view_ref_focused()
              ? GetKoids(view_protocols.view_ref_focused()).first
              : zx_koid_t{},
      .focuser = view_protocols.has_view_focuser()
                     ? GetKoids(view_protocols.view_focuser()).first
                     : zx_koid_t{},
      .touch_source = view_protocols.has_touch_source()
                          ? GetKoids(view_protocols.touch_source()).first
                          : zx_koid_t{},
      .mouse_source = view_protocols.has_mouse_source()
                          ? GetKoids(view_protocols.mouse_source()).first
                          : zx_koid_t{},
      .parent_viewport_watcher = GetKoids(parent_viewport_watcher).first,
  });
  graph_bindings_.viewport_watcher.emplace(
      view_token_koids.first,
      ParentViewportWatcher(
          std::move(token), std::move(view_identity), std::move(view_protocols),
          std::move(parent_viewport_watcher), flatland_binding_.dispatcher()));
}

void FakeFlatland::CreateTransform(
    fuchsia::ui::composition::TransformId transform_id) {
  if (transform_id.value == 0) {
    // TODO(fxb/85619): Raise a FlatlandError here
    FML_CHECK(false)
        << "FakeFlatland::CreateTransform: TransformId 0 is invalid.";
    return;
  }
  if (pending_graph_.transform_map.count(transform_id.value) != 0) {
    // TODO(fxb/85619): Raise a FlatlandError here
    FML_CHECK(false) << "FakeFlatland::CreateTransform: TransformId "
                     << transform_id.value << " is already in use.";
    return;
  }

  auto [emplaced_transform, emplace_success] =
      pending_graph_.transform_map.emplace(
          transform_id.value, std::make_shared<FakeTransform>(FakeTransform{
                                  .id = transform_id,
                              }));
  // TODO(fxb/85619): Raise a FlatlandError here
  FML_CHECK(emplace_success)
      << "FakeFlatland::CreateTransform: Internal error (transform_map) adding "
         "transform with id: "
      << transform_id.value;

  auto [_, emplace_parent_success] = parents_map_.emplace(
      transform_id.value,
      std::make_pair(std::weak_ptr<FakeTransform>(),
                     std::weak_ptr(emplaced_transform->second)));
  // TODO(fxb/85619): Raise a FlatlandError here
  FML_CHECK(emplace_parent_success)
      << "FakeFlatland::CreateTransform: Internal error (parent_map) adding "
         "transform with id: "
      << transform_id.value;
}

void FakeFlatland::SetTranslation(
    fuchsia::ui::composition::TransformId transform_id,
    fuchsia::math::Vec translation) {
  if (transform_id.value == 0) {
    // TODO(fxb/85619): Raise a FlatlandError here
    FML_CHECK(false)
        << "FakeFlatland::SetTranslation: TransformId 0 is invalid.";
    return;
  }

  auto found_transform = pending_graph_.transform_map.find(transform_id.value);
  if (found_transform == pending_graph_.transform_map.end()) {
    // TODO(fxb/85619): Raise a FlatlandError here
    FML_CHECK(false) << "FakeFlatland::SetTranslation: TransformId "
                     << transform_id.value << " does not exist.";
    return;
  }

  auto& transform = found_transform->second;
  FML_CHECK(transform);
  transform->translation = translation;
}

void FakeFlatland::SetScale(fuchsia::ui::composition::TransformId transform_id,
                            fuchsia::math::VecF scale) {
  if (transform_id.value == 0) {
    // TODO(fxb/85619): Raise a FlatlandError here
    FML_CHECK(false) << "FakeFlatland::SetScale: TransformId 0 is invalid.";
    return;
  }

  auto found_transform = pending_graph_.transform_map.find(transform_id.value);
  if (found_transform == pending_graph_.transform_map.end()) {
    // TODO(fxb/85619): Raise a FlatlandError here
    FML_CHECK(false) << "FakeFlatland::SetScale: TransformId "
                     << transform_id.value << " does not exist.";
    return;
  }

  if (scale.x == 0.f || scale.y == 0.f) {
    FML_CHECK(false) << "SetScale failed, zero values not allowed (" << scale.x
                     << ", " << scale.y << " ).";
    return;
  }

  if (isinf(scale.x) || isinf(scale.y) || isnan(scale.x) || isnan(scale.y)) {
    FML_CHECK(false) << "SetScale failed, invalid scale values (" << scale.x
                     << ", " << scale.y << " ).";
    return;
  }

  auto& transform = found_transform->second;
  FML_CHECK(transform);
  transform->scale = scale;
}

void FakeFlatland::SetOrientation(
    fuchsia::ui::composition::TransformId transform_id,
    fuchsia::ui::composition::Orientation orientation) {
  if (transform_id.value == 0) {
    // TODO(fxb/85619): Raise a FlatlandError here
    FML_CHECK(false)
        << "FakeFlatland::SetOrientation: TransformId 0 is invalid.";
    return;
  }

  auto found_transform = pending_graph_.transform_map.find(transform_id.value);
  if (found_transform == pending_graph_.transform_map.end()) {
    // TODO(fxb/85619): Raise a FlatlandError here
    FML_CHECK(false) << "FakeFlatland::SetOrientation: TransformId "
                     << transform_id.value << " does not exist.";
    return;
  }

  auto& transform = found_transform->second;
  FML_CHECK(transform);
  transform->orientation = orientation;
}

void FakeFlatland::SetOpacity(
    fuchsia::ui::composition::TransformId transform_id,
    float value) {
  if (transform_id.value == 0) {
    // TODO(fxb/85619): Raise a FlatlandError here
    FML_CHECK(false) << "FakeFlatland::SetOpacity: TransformId 0 is invalid.";
    return;
  }

  auto found_transform = pending_graph_.transform_map.find(transform_id.value);
  if (found_transform == pending_graph_.transform_map.end()) {
    // TODO(fxb/85619): Raise a FlatlandError here
    FML_CHECK(false) << "FakeFlatland::SetOpacity: TransformId "
                     << transform_id.value << " does not exist.";
    return;
  }

  if (value < 0.f || value > 1.f) {
    FML_CHECK(false) << "FakeFlatland::SetOpacity: Invalid opacity value.";
  }

  auto& transform = found_transform->second;
  FML_CHECK(transform);
  transform->opacity = value;
}

void FakeFlatland::SetClipBoundary(
    fuchsia::ui::composition::TransformId transform_id,
    std::unique_ptr<fuchsia::math::Rect> bounds) {
  if (transform_id.value == 0) {
    // TODO(fxb/85619): Raise a FlatlandError here
    FML_CHECK(false)
        << "FakeFlatland::SetClipBoundary: TransformId 0 is invalid.";
    return;
  }

  auto found_transform = pending_graph_.transform_map.find(transform_id.value);
  if (found_transform == pending_graph_.transform_map.end()) {
    // TODO(fxb/85619): Raise a FlatlandError here
    FML_CHECK(false) << "FakeFlatland::SetClipBoundary: TransformId "
                     << transform_id.value << " does not exist.";
    return;
  }

  auto& transform = found_transform->second;
  FML_CHECK(transform);
  if (bounds == nullptr) {
    transform->clip_bounds = std::nullopt;
    return;
  }

  transform->clip_bounds = *bounds.get();
}

// TODO(fxbug.dev/89111): Re-enable once SDK rolls.
// void FakeFlatland::SetImageOpacity(
//     fuchsia::ui::composition::ContentId content_id,
//     float opacity) {
//   if (content_id.value == 0) {
//     // TODO(fxb/85619): Raise a FlatlandError here
//     FML_CHECK(false)
//         << "FakeFlatland::SetImageOpacity: ContentId 0 is invalid.";
//     return;
//   }

//   auto found_content = pending_graph_.content_map.find(image_id.value);
//   if (found_content == pending_graph_.content_map.end()) {
//     // TODO(fxb/85619): Raise a FlatlandError here
//     FML_CHECK(false) << "FakeFlatland::SetImageOpacity: ContentId "
//                      << image_id.value << " does not exist.";
//     return;
//   }

//   auto& content = found_content->second;
//   FML_CHECK(content);
//   FakeImage* image = std::get_if<FakeImage>(content.get());
//   if (image == nullptr) {
//     // TODO(fxb/85619): Raise a FlatlandError here
//     FML_CHECK(false) << "FakeFlatland::SetImageOpacity: ContentId "
//                      << image_id.value << " is not an Image.";
//     return;
//   }

//   image->opacity = opacity;
// }

void FakeFlatland::AddChild(
    fuchsia::ui::composition::TransformId parent_transform_id,
    fuchsia::ui::composition::TransformId child_transform_id) {
  if (parent_transform_id.value == 0) {
    // TODO(fxb/85619): Raise a FlatlandError here
    FML_CHECK(false)
        << "FakeFlatland::AddChild: Parent TransformId 0 is invalid.";
    return;
  }
  if (child_transform_id.value == 0) {
    // TODO(fxb/85619): Raise a FlatlandError here
    FML_CHECK(false)
        << "FakeFlatland::AddChild: Child TransformId 0 is invalid.";
    return;
  }

  auto found_parent =
      pending_graph_.transform_map.find(parent_transform_id.value);
  if (found_parent == pending_graph_.transform_map.end()) {
    // TODO(fxb/85619): Raise a FlatlandError here
    FML_CHECK(false) << "FakeFlatland::AddChild: Parent TransformId "
                     << parent_transform_id.value << " does not exist.";
    return;
  }
  auto found_child =
      pending_graph_.transform_map.find(child_transform_id.value);
  if (found_child == pending_graph_.transform_map.end()) {
    // TODO(fxb/85619): Raise a FlatlandError here
    FML_CHECK(false) << "FakeFlatland::AddChild: Child TransformId "
                     << child_transform_id.value << " does not exist.";
    return;
  }
  auto found_child_old_parent = parents_map_.find(child_transform_id.value);
  if (found_child_old_parent == parents_map_.end()) {
    // TODO(fxb/85619): Raise a FlatlandError here
    FML_CHECK(false)
        << "FakeFlatland::AddChild: Internal error - Child TransformId "
        << child_transform_id.value << " is not in parents_map.";
    return;
  }
  if (found_child_old_parent->second.second.expired()) {
    // TODO(fxb/85619): Raise a FlatlandError here
    FML_CHECK(false)
        << "FakeFlatland::AddChild: Internal error - Child TransformId "
        << child_transform_id.value << " is expired in parents_map.";
    return;
  }

  auto& child = found_child->second;
  auto& new_parent = found_parent->second;
  new_parent->children.push_back(child);
  if (auto old_parent = found_child_old_parent->second.first.lock()) {
    old_parent->children.erase(std::remove_if(
        old_parent->children.begin(), old_parent->children.end(),
        [&child](const auto& transform) { return transform == child; }));
  }
  found_child_old_parent->second.first = std::weak_ptr(new_parent);
}

void FakeFlatland::RemoveChild(
    fuchsia::ui::composition::TransformId parent_transform_id,
    fuchsia::ui::composition::TransformId child_transform_id) {
  if (parent_transform_id.value == 0) {
    // TODO(fxb/85619): Raise a FlatlandError here
    FML_CHECK(false)
        << "FakeFlatland::RemoveChild: Parent TransformId 0 is invalid.";
    return;
  }
  if (child_transform_id.value == 0) {
    // TODO(fxb/85619): Raise a FlatlandError here
    FML_CHECK(false)
        << "FakeFlatland::RemoveChild: Child TransformId 0 is invalid.";
    return;
  }

  auto found_child =
      pending_graph_.transform_map.find(child_transform_id.value);
  if (found_child == pending_graph_.transform_map.end()) {
    // TODO(fxb/85619): Raise a FlatlandError here
    FML_CHECK(false) << "FakeFlatland::RemoveChild: Child TransformId "
                     << child_transform_id.value << " does not exist.";
    return;
  }

  auto found_parent =
      pending_graph_.transform_map.find(parent_transform_id.value);
  if (found_parent == pending_graph_.transform_map.end()) {
    // TODO(fxb/85619): Raise a FlatlandError here
    FML_CHECK(false) << "FakeFlatland::RemoveChild: Parent TransformId "
                     << parent_transform_id.value << " does not exist.";
    return;
  }

  auto found_child_parent = parents_map_.find(child_transform_id.value);
  if (found_child_parent == parents_map_.end()) {
    // TODO(fxb/85619): Raise a FlatlandError here
    FML_CHECK(false)
        << "FakeFlatland::RemoveChild: Internal error - Child TransformId "
        << child_transform_id.value << " is not in parents_map.";
    return;
  }
  if (found_child_parent->second.second.expired()) {
    // TODO(fxb/85619): Raise a FlatlandError here
    FML_CHECK(false)
        << "FakeFlatland::RemoveChild: Internal error - Child TransformId "
        << child_transform_id.value << " is expired in parents_map.";
    return;
  }
  if (found_child_parent->second.first.lock() != found_parent->second) {
    // TODO(fxb/85619): Raise a FlatlandError here
    FML_CHECK(false)
        << "FakeFlatland::RemoveChild: Internal error - Child TransformId "
        << child_transform_id.value << " is not a child of Parent TransformId "
        << parent_transform_id.value << ".";
    return;
  }

  found_child_parent->second.first = std::weak_ptr<FakeTransform>();
  found_parent->second->children.erase(std::remove_if(
      found_parent->second->children.begin(),
      found_parent->second->children.end(),
      [child_to_remove = found_child->second](const auto& child) {
        return child == child_to_remove;
      }));
}

void FakeFlatland::SetContent(
    fuchsia::ui::composition::TransformId transform_id,
    fuchsia::ui::composition::ContentId content_id) {
  if (transform_id.value == 0) {
    // TODO(fxb/85619): Raise a FlatlandError here
    FML_CHECK(false) << "FakeFlatland::SetContent: TransformId 0 is invalid.";
    return;
  }

  auto found_transform = pending_graph_.transform_map.find(transform_id.value);
  if (found_transform == pending_graph_.transform_map.end()) {
    // TODO(fxb/85619): Raise a FlatlandError here
    FML_CHECK(false) << "FakeFlatland::SetContent: TransformId "
                     << transform_id.value << " does not exist.";
    return;
  }

  auto& transform = found_transform->second;
  FML_CHECK(transform);
  if (content_id.value == 0) {
    transform->content.reset();
    return;
  }

  auto found_content = pending_graph_.content_map.find(content_id.value);
  if (found_content == pending_graph_.content_map.end()) {
    // TODO(fxb/85619): Raise a FlatlandError here
    FML_CHECK(false) << "FakeFlatland::SetContent: ContentId "
                     << content_id.value << " does not exist.";
    return;
  }

  auto& content = found_content->second;
  FML_CHECK(content);
  transform->content = content;
}

void FakeFlatland::SetRootTransform(
    fuchsia::ui::composition::TransformId transform_id) {
  if (transform_id.value == 0) {
    // TODO(fxb/85619): Raise a FlatlandError here
    FML_CHECK(false)
        << "FakeFlatland::SetRootTransform: TransformId 0 is invalid.";
    return;
  }

  auto found_new_root = pending_graph_.transform_map.find(transform_id.value);
  if (found_new_root == pending_graph_.transform_map.end()) {
    // TODO(fxb/85619): Raise a FlatlandError here
    FML_CHECK(false) << "FakeFlatland::SetRootTransform: TransformId "
                     << transform_id.value << " does not exist.";
    return;
  }

  pending_graph_.root_transform = found_new_root->second;
}

void FakeFlatland::CreateViewport(
    fuchsia::ui::composition::ContentId viewport_id,
    fuchsia::ui::views::ViewportCreationToken token,
    fuchsia::ui::composition::ViewportProperties properties,
    fidl::InterfaceRequest<fuchsia::ui::composition::ChildViewWatcher>
        child_view_watcher) {
  if (viewport_id.value == 0) {
    // TODO(fxb/85619): Raise a FlatlandError here
    FML_CHECK(false) << "FakeFlatland::CreateViewport: ContentId 0 is invalid.";
    return;
  }
  if (pending_graph_.content_map.count(viewport_id.value) != 0) {
    // TODO(fxb/85619): Raise a FlatlandError here
    FML_CHECK(false) << "FakeFlatland::CreateViewport: ContentId "
                     << viewport_id.value << " is already in use.";
    return;
  }

  auto viewport_token_koids = GetKoids(token.value);
  auto [emplaced_viewport, emplace_success] =
      pending_graph_.content_map.emplace(
          viewport_id.value,
          std::make_shared<FakeContent>(FakeViewport{
              .id = viewport_id,
              .viewport_properties = std::move(properties),
              .viewport_token = viewport_token_koids.first,
              .child_view_watcher = GetKoids(child_view_watcher).first,
          }));
  // TODO(fxb/85619): Raise a FlatlandError here
  FML_CHECK(emplace_success)
      << "FakeFlatland::CreateViewport: Internal error (content_map) adding "
         "viewport with id: "
      << viewport_id.value;

  auto [_, emplace_binding_success] = graph_bindings_.view_watchers.emplace(
      viewport_token_koids.first,
      ChildViewWatcher(std::move(token), std::move(child_view_watcher),
                       flatland_binding_.dispatcher()));
  // TODO(fxb/85619): Raise a FlatlandError here
  FML_CHECK(emplace_binding_success)
      << "FakeFlatland::CreateViewport: Internal error (view_watcher) adding "
         "viewport with id: "
      << viewport_id.value;
}

void FakeFlatland::CreateImage(
    fuchsia::ui::composition::ContentId image_id,
    fuchsia::ui::composition::BufferCollectionImportToken import_token,
    uint32_t vmo_index,
    fuchsia::ui::composition::ImageProperties properties) {
  if (image_id.value == 0) {
    // TODO(fxb/85619): Raise a FlatlandError here
    FML_CHECK(false) << "FakeFlatland::CreateImage: ContentId 0 is invalid.";
    return;
  }
  if (pending_graph_.content_map.count(image_id.value) != 0) {
    // TODO(fxb/85619): Raise a FlatlandError here
    FML_CHECK(false) << "FakeFlatland::CreateImage: ContentId "
                     << image_id.value << " is already in use.";
    return;
  }

  auto import_token_koids = GetKoids(import_token);
  auto [emplaced_image, emplace_success] = pending_graph_.content_map.emplace(
      image_id.value, std::make_shared<FakeContent>(FakeImage{
                          .id = image_id,
                          .image_properties = std::move(properties),
                          .import_token = import_token_koids.first,
                          .vmo_index = vmo_index,
                      }));
  // TODO(fxb/85619): Raise a FlatlandError here
  FML_CHECK(emplace_success)
      << "FakeFlatland::CreateImage: Internal error (content_map) adding "
         "image with id: "
      << image_id.value;

  auto [_, emplace_binding_success] = graph_bindings_.images.emplace(
      import_token_koids.first, ImageBinding{
                                    .import_token = std::move(import_token),
                                });
  // TODO(fxb/85619): Raise a FlatlandError here
  FML_CHECK(emplace_binding_success)
      << "FakeFlatland::CreateImage: Internal error (images_binding) adding "
         "viewport with id: "
      << image_id.value;
}

void FakeFlatland::SetImageSampleRegion(
    fuchsia::ui::composition::ContentId image_id,
    fuchsia::math::RectF rect) {
  if (image_id.value == 0) {
    // TODO(fxb/85619): Raise a FlatlandError here
    FML_CHECK(false)
        << "FakeFlatland::SetImageSampleRegion: ContentId 0 is invalid.";
    return;
  }

  auto found_content = pending_graph_.content_map.find(image_id.value);
  if (found_content == pending_graph_.content_map.end()) {
    // TODO(fxb/85619): Raise a FlatlandError here
    FML_CHECK(false) << "FakeFlatland::SetImageSampleRegion: ContentId "
                     << image_id.value << " does not exist.";
    return;
  }

  auto& content = found_content->second;
  FML_CHECK(content);
  FakeImage* image = std::get_if<FakeImage>(content.get());
  if (image == nullptr) {
    // TODO(fxb/85619): Raise a FlatlandError here
    FML_CHECK(false) << "FakeFlatland::SetImageSampleRegion: ContentId "
                     << image_id.value << " is not an Image.";
    return;
  }

  image->sample_region = rect;
}

void FakeFlatland::SetImageDestinationSize(
    fuchsia::ui::composition::ContentId image_id,
    fuchsia::math::SizeU size) {
  if (image_id.value == 0) {
    // TODO(fxb/85619): Raise a FlatlandError here
    FML_CHECK(false)
        << "FakeFlatland::SetImageDestinationSize: ContentId 0 is invalid.";
    return;
  }

  auto found_content = pending_graph_.content_map.find(image_id.value);
  if (found_content == pending_graph_.content_map.end()) {
    // TODO(fxb/85619): Raise a FlatlandError here
    FML_CHECK(false) << "FakeFlatland::SetImageDestinationSize: ContentId "
                     << image_id.value << " does not exist.";
    return;
  }

  auto& content = found_content->second;
  FML_CHECK(content);
  FakeImage* image = std::get_if<FakeImage>(content.get());
  if (image == nullptr) {
    // TODO(fxb/85619): Raise a FlatlandError here
    FML_CHECK(false) << "FakeFlatland::SetImageDestinationSize: ContentId "
                     << image_id.value << " is not an Image.";
    return;
  }

  image->destination_size = size;
}

void FakeFlatland::SetImageBlendingFunction(
    fuchsia::ui::composition::ContentId image_id,
    fuchsia::ui::composition::BlendMode blend_mode) {
  if (image_id.value == 0) {
    // TODO(fxb/85619): Raise a FlatlandError here
    FML_CHECK(false)
        << "FakeFlatland::SetImageDestinationSize: ContentId 0 is invalid.";
    return;
  }

  auto found_content = pending_graph_.content_map.find(image_id.value);
  if (found_content == pending_graph_.content_map.end()) {
    // TODO(fxb/85619): Raise a FlatlandError here
    FML_CHECK(false) << "FakeFlatland::SetImageDestinationSize: ContentId "
                     << image_id.value << " does not exist.";
    return;
  }

  auto& content = found_content->second;
  FML_CHECK(content);
  FakeImage* image = std::get_if<FakeImage>(content.get());
  if (image == nullptr) {
    // TODO(fxb/85619): Raise a FlatlandError here
    FML_CHECK(false) << "FakeFlatland::SetImageDestinationSize: ContentId "
                     << image_id.value << " is not an Image.";
    return;
  }

  image->blend_mode = blend_mode;
}

void FakeFlatland::SetViewportProperties(
    fuchsia::ui::composition::ContentId viewport_id,
    fuchsia::ui::composition::ViewportProperties properties) {
  if (viewport_id.value == 0) {
    // TODO(fxb/85619): Raise a FlatlandError here
    FML_CHECK(false)
        << "FakeFlatland::SetViewportProperties: ContentId 0 is invalid.";
    return;
  }

  auto found_content = pending_graph_.content_map.find(viewport_id.value);
  if (found_content == pending_graph_.content_map.end()) {
    // TODO(fxb/85619): Raise a FlatlandError here
    FML_CHECK(false) << "FakeFlatland::SetViewportProperties: ContentId "
                     << viewport_id.value << " does not exist.";
    return;
  }

  auto& content = found_content->second;
  FML_CHECK(content);
  FakeViewport* viewport = std::get_if<FakeViewport>(content.get());
  if (viewport == nullptr) {
    // TODO(fxb/85619): Raise a FlatlandError here
    FML_CHECK(false) << "FakeFlatland::SetViewportProperties: ContentId "
                     << viewport_id.value << " is not a Viewport.";
    return;
  }

  viewport->viewport_properties = std::move(properties);
}

void FakeFlatland::ReleaseTransform(
    fuchsia::ui::composition::TransformId transform_id) {
  if (transform_id.value == 0) {
    // TODO(fxb/85619): Raise a FlatlandError here
    FML_CHECK(false)
        << "FakeFlatland::ReleaseTransform: TransformId 0 is invalid.";
    return;
  }

  size_t erased = pending_graph_.transform_map.erase(transform_id.value);
  if (erased == 0) {
    // TODO(fxb/85619): Raise a FlatlandError here
    FML_CHECK(false) << "FakeFlatland::ReleaseTransform: TransformId "
                     << transform_id.value << " does not exist.";
  }

  size_t parents_erased = parents_map_.erase(transform_id.value);
  if (parents_erased == 0) {
    // TODO(fxb/85619): Raise a FlatlandError here
    FML_CHECK(false) << "FakeFlatland::ReleaseTransform: TransformId "
                     << transform_id.value << " does not exist in parents_map.";
  }
}

void FakeFlatland::ReleaseViewport(
    fuchsia::ui::composition::ContentId viewport_id,
    ReleaseViewportCallback callback) {
  if (viewport_id.value == 0) {
    // TODO(fxb/85619): Raise a FlatlandError here
    FML_CHECK(false)
        << "FakeFlatland::ReleaseViewport: ContentId 0 is invalid.";
    return;
  }

  auto found_content = pending_graph_.content_map.find(viewport_id.value);
  if (found_content == pending_graph_.content_map.end()) {
    // TODO(fxb/85619): Raise a FlatlandError here
    FML_CHECK(false) << "FakeFlatland::ReleaseViewport: ContentId "
                     << viewport_id.value << " does not exist.";
    return;
  }

  auto& content = found_content->second;
  FML_CHECK(content);
  FakeViewport* viewport = std::get_if<FakeViewport>(content.get());
  if (viewport == nullptr) {
    // TODO(fxb/85619): Raise a FlatlandError here
    FML_CHECK(false) << "FakeFlatland::ReleaseViewport: ContentId "
                     << viewport_id.value << " is not a Viewport.";
    return;
  }

  pending_graph_.content_map.erase(found_content);
}

void FakeFlatland::ReleaseImage(fuchsia::ui::composition::ContentId image_id) {
  if (image_id.value == 0) {
    // TODO(fxb/85619): Raise a FlatlandError here
    FML_CHECK(false) << "FakeFlatland::ReleaseImage: ContentId 0 is invalid.";
    return;
  }

  auto found_content = pending_graph_.content_map.find(image_id.value);
  if (found_content == pending_graph_.content_map.end()) {
    // TODO(fxb/85619): Raise a FlatlandError here
    FML_CHECK(false) << "FakeFlatland::ReleaseImage: ContentId "
                     << image_id.value << " does not exist.";
    return;
  }

  auto& content = found_content->second;
  FML_CHECK(content);
  FakeImage* image = std::get_if<FakeImage>(content.get());
  if (image == nullptr) {
    // TODO(fxb/85619): Raise a FlatlandError here
    FML_CHECK(false) << "FakeFlatland::ReleaseImage: ContentId "
                     << image_id.value << " is not a Viewport.";
    return;
  }

  pending_graph_.content_map.erase(found_content);
}

void FakeFlatland::SetHitRegions(
    fuchsia::ui::composition::TransformId transform_id,
    std::vector<fuchsia::ui::composition::HitRegion> regions) {
  if (transform_id.value == 0) {
    // TODO(fxb/85619): Raise a FlatlandError here
    FML_CHECK(false)
        << "FakeFlatland::SetTranslation: TransformId 0 is invalid.";
    return;
  }

  auto found_transform = pending_graph_.transform_map.find(transform_id.value);
  if (found_transform == pending_graph_.transform_map.end()) {
    // TODO(fxb/85619): Raise a FlatlandError here
    FML_CHECK(false) << "FakeFlatland::SetTranslation: TransformId "
                     << transform_id.value << " does not exist.";
    return;
  }

  auto& transform = found_transform->second;
  FML_CHECK(transform);
  transform->hit_regions = std::move(regions);
}

void FakeFlatland::SetInfiniteHitRegion(
    fuchsia::ui::composition::TransformId transform_id,
    fuchsia::ui::composition::HitTestInteraction hit_test) {
  if (transform_id.value == 0) {
    // TODO(fxb/85619): Raise a FlatlandError here
    FML_CHECK(false)
        << "FakeFlatland::SetTranslation: TransformId 0 is invalid.";
    return;
  }

  auto found_transform = pending_graph_.transform_map.find(transform_id.value);
  if (found_transform == pending_graph_.transform_map.end()) {
    // TODO(fxb/85619): Raise a FlatlandError here
    FML_CHECK(false) << "FakeFlatland::SetTranslation: TransformId "
                     << transform_id.value << " does not exist.";
    return;
  }

  auto& transform = found_transform->second;
  ZX_ASSERT(transform);
  transform->hit_regions = {kInfiniteHitRegion};
}

void FakeFlatland::Clear() {
  parents_map_.clear();
  pending_graph_.Clear();
}

void FakeFlatland::SetDebugName(std::string debug_name) {
  debug_name_ = std::move(debug_name);
}

FakeFlatland::ParentViewportWatcher::ParentViewportWatcher(
    fuchsia::ui::views::ViewCreationToken view_token,
    fuchsia::ui::views::ViewIdentityOnCreation view_identity,
    fuchsia::ui::composition::ViewBoundProtocols view_protocols,
    fidl::InterfaceRequest<fuchsia::ui::composition::ParentViewportWatcher>
        parent_viewport_watcher,
    async_dispatcher_t* dispatcher)
    : view_token(std::move(view_token)),
      view_identity(std::move(view_identity)),
      view_protocols(std::move(view_protocols)),
      parent_viewport_watcher(this,
                              std::move(parent_viewport_watcher),
                              dispatcher) {}

FakeFlatland::ParentViewportWatcher::ParentViewportWatcher(
    ParentViewportWatcher&& other)
    : view_token(std::move(other.view_token)),
      view_identity(std::move(other.view_identity)),
      view_protocols(std::move(other.view_protocols)),
      parent_viewport_watcher(this,
                              other.parent_viewport_watcher.Unbind(),
                              other.parent_viewport_watcher.dispatcher()) {}

FakeFlatland::ParentViewportWatcher::~ParentViewportWatcher() = default;

void FakeFlatland::ParentViewportWatcher::NotImplemented_(
    const std::string& name) {
  FML_LOG(FATAL) << "FakeFlatland::ParentViewportWatcher does not implement "
                 << name;
}

void FakeFlatland::ParentViewportWatcher::GetLayout(
    GetLayoutCallback callback) {
  NotImplemented_("GetLayout");
}

void FakeFlatland::ParentViewportWatcher::GetStatus(
    GetStatusCallback callback) {
  NotImplemented_("GetStatus");
}

FakeFlatland::ChildViewWatcher::ChildViewWatcher(
    fuchsia::ui::views::ViewportCreationToken viewport_token,
    fidl::InterfaceRequest<fuchsia::ui::composition::ChildViewWatcher>
        child_view_watcher,
    async_dispatcher_t* dispatcher)
    : viewport_token(std::move(viewport_token)),
      child_view_watcher(this, std::move(child_view_watcher), dispatcher) {}

FakeFlatland::ChildViewWatcher::ChildViewWatcher(ChildViewWatcher&& other)
    : viewport_token(std::move(other.viewport_token)),
      child_view_watcher(this,
                         other.child_view_watcher.Unbind(),
                         other.child_view_watcher.dispatcher()) {}

FakeFlatland::ChildViewWatcher::~ChildViewWatcher() = default;

void FakeFlatland::ChildViewWatcher::NotImplemented_(const std::string& name) {
  FML_LOG(FATAL) << "FakeFlatland::ChildViewWatcher does not implement "
                 << name;
}

void FakeFlatland::ChildViewWatcher::GetStatus(GetStatusCallback callback) {
  NotImplemented_("GetStatus");
}

}  // namespace flutter_runner::testing
