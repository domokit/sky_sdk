// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "fake_flatland_types.h"

#include <lib/fidl/cpp/clone.h>
#include <lib/zx/channel.h>
#include <lib/zx/eventpair.h>

#include "flutter/fml/logging.h"

namespace flutter_runner::testing {
namespace {

using FakeTransformCache =
    std::unordered_map<const FakeTransform*, std::shared_ptr<FakeTransform>>;

std::vector<std::shared_ptr<FakeTransform>> CloneFakeTransformVector(
    const std::vector<std::shared_ptr<FakeTransform>>& transforms,
    FakeTransformCache& transform_cache);

std::shared_ptr<FakeContent> CloneFakeContent(
    const std::shared_ptr<FakeContent>& content) {
  if (content == nullptr) {
    return nullptr;
  }

  if (FakeViewport* viewport = std::get_if<FakeViewport>(content.get())) {
    return std::make_shared<FakeContent>(FakeViewport{
        .id = viewport->id,
        .viewport_properties = fidl::Clone(viewport->viewport_properties),
        .viewport_token = viewport->viewport_token,
        .child_view_watcher = viewport->child_view_watcher,
    });
  } else if (FakeImage* image = std::get_if<FakeImage>(content.get())) {
    return std::make_shared<FakeContent>(FakeImage{
        .id = image->id,
        .image_properties = fidl::Clone(image->image_properties),
        .sample_region = image->sample_region,
        .destination_size = image->destination_size,
        .opacity = image->opacity,
        .blend_mode = image->blend_mode,
        .import_token = image->import_token,
        .vmo_index = image->vmo_index,
    });
  } else {
    FML_UNREACHABLE();
  }
}

std::shared_ptr<FakeTransform> CloneFakeTransform(
    const std::shared_ptr<FakeTransform>& transform,
    FakeTransformCache& transform_cache) {
  if (transform == nullptr) {
    return nullptr;
  }

  auto found_transform = transform_cache.find(transform.get());
  if (found_transform != transform_cache.end()) {
    return found_transform->second;
  }

  auto [emplaced_transform, success] = transform_cache.emplace(
      transform.get(), std::make_shared<FakeTransform>(FakeTransform{
                           .id = transform->id,
                           .translation = transform->translation,
                           .scale = transform->scale,
                           .orientation = transform->orientation,
                           .clip_bounds = transform->clip_bounds,
                           .opacity = transform->opacity,
                           .children = CloneFakeTransformVector(
                               transform->children, transform_cache),
                           .content = CloneFakeContent(transform->content),
                           .hit_regions = transform->hit_regions,
                       }));
  FML_CHECK(success);

  return emplaced_transform->second;
}

std::vector<std::shared_ptr<FakeTransform>> CloneFakeTransformVector(
    const std::vector<std::shared_ptr<FakeTransform>>& transforms,
    FakeTransformCache& transform_cache) {
  std::vector<std::shared_ptr<FakeTransform>> clones;
  for (auto& transform : transforms) {
    clones.emplace_back(CloneFakeTransform(transform, transform_cache));
  }
  return clones;
}

}  // namespace

ViewTokenPair ViewTokenPair::New() {
  ViewTokenPair token_pair;
  auto status = zx::channel::create(0u, &token_pair.view_token.value,
                                    &token_pair.viewport_token.value);
  FML_CHECK(status == ZX_OK);

  return token_pair;
}

BufferCollectionTokenPair BufferCollectionTokenPair::New() {
  BufferCollectionTokenPair token_pair;
  auto status = zx::eventpair::create(0u, &token_pair.export_token.value,
                                      &token_pair.import_token.value);
  FML_CHECK(status == ZX_OK);

  return token_pair;
}

bool FakeView::operator==(const FakeView& other) const {
  return view_token == other.view_token && view_ref == other.view_ref &&
         view_ref_control == other.view_ref_control &&
         view_ref_focused == other.view_ref_focused &&
         focuser == other.focuser && touch_source == other.touch_source &&
         mouse_source == other.mouse_source &&
         parent_viewport_watcher == other.parent_viewport_watcher;
}

bool FakeViewport::operator==(const FakeViewport& other) const {
  return id == other.id && viewport_properties == other.viewport_properties &&
         viewport_token == other.viewport_token &&
         child_view_watcher == other.child_view_watcher;
}

bool FakeImage::operator==(const FakeImage& other) const {
  return id == other.id && image_properties == other.image_properties &&
         sample_region == other.sample_region &&
         destination_size == other.destination_size &&
         opacity == other.opacity && blend_mode == other.blend_mode &&
         import_token == other.import_token && vmo_index == other.vmo_index;
}

bool FakeTransform::operator==(const FakeTransform& other) const {
  return id == other.id && translation == other.translation &&
         *clip_bounds == *other.clip_bounds &&
         orientation == other.orientation && children == other.children &&
         content == other.content && hit_regions == other.hit_regions;
}

bool FakeGraph::operator==(const FakeGraph& other) const {
  return transform_map == other.transform_map &&
         content_map == other.content_map &&
         root_transform == other.root_transform && view == other.view;
}

void FakeGraph::Clear() {
  view.reset();
  root_transform.reset();
  transform_map.clear();
  content_map.clear();
}

FakeGraph FakeGraph::Clone() const {
  FakeGraph clone;
  FakeTransformCache transform_cache;

  for (const auto& [transform_id, transform] : transform_map) {
    FML_CHECK(transform);
    clone.transform_map.emplace(transform_id,
                                CloneFakeTransform(transform, transform_cache));
  }
  for (const auto& [content_id, content] : content_map) {
    FML_CHECK(content);
    clone.content_map.emplace(content_id, CloneFakeContent(content));
  }
  if (root_transform) {
    auto found_transform = transform_cache.find(root_transform.get());
    FML_CHECK(found_transform != transform_cache.end());
    clone.root_transform = found_transform->second;
  }
  if (view.has_value()) {
    clone.view.emplace(view.value());
  }

  return clone;
}

}  // namespace flutter_runner::testing
