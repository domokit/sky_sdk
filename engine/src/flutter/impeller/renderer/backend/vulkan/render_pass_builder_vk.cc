// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/backend/vulkan/render_pass_builder_vk.h"

#include "impeller/core/formats.h"
#include "impeller/renderer/backend/vulkan/formats_vk.h"

namespace impeller {

constexpr auto kSelfDependencySrcStageMask =
    vk::PipelineStageFlagBits::eColorAttachmentOutput;
constexpr auto kSelfDependencySrcAccessMask =
    vk::AccessFlagBits::eColorAttachmentWrite;

constexpr auto kSelfDependencyDstStageMask =
    vk::PipelineStageFlagBits::eFragmentShader;
constexpr auto kSelfDependencyDstAccessMask =
    vk::AccessFlagBits::eInputAttachmentRead;

constexpr auto kSelfDependencyFlags = vk::DependencyFlagBits::eByRegion;

RenderPassBuilderVK::RenderPassBuilderVK() = default;

RenderPassBuilderVK::~RenderPassBuilderVK() = default;

RenderPassBuilderVK& RenderPassBuilderVK::SetColorAttachment(
    size_t index,
    PixelFormat format,
    SampleCount sample_count,
    LoadAction load_action,
    StoreAction store_action,
    vk::ImageLayout current_layout) {
  vk::AttachmentDescription desc;
  desc.format = ToVKImageFormat(format);
  desc.samples = ToVKSampleCount(sample_count);
  desc.loadOp = ToVKAttachmentLoadOp(load_action);
  desc.storeOp = ToVKAttachmentStoreOp(store_action, false);
  desc.stencilLoadOp = vk::AttachmentLoadOp::eDontCare;
  desc.stencilStoreOp = vk::AttachmentStoreOp::eDontCare;
  if (load_action == LoadAction::kLoad) {
    desc.initialLayout = current_layout;
  } else {
    desc.initialLayout = vk::ImageLayout::eUndefined;
  }
  desc.finalLayout = vk::ImageLayout::eGeneral;

  const bool performs_resolves = StoreActionPerformsResolve(store_action);
  if (index == 0u) {
    color0_ = desc;

    if (performs_resolves) {
      desc.storeOp = ToVKAttachmentStoreOp(store_action, true);
      desc.samples = vk::SampleCountFlagBits::e1;
      color0_resolve_ = desc;
    } else {
      color0_resolve_ = std::nullopt;
    }
  } else {
    colors_[index] = desc;
    if (performs_resolves) {
      desc.storeOp = ToVKAttachmentStoreOp(store_action, true);
      desc.samples = vk::SampleCountFlagBits::e1;
      resolves_[index] = desc;
    } else {
      resolves_.erase(index);
    }
  }
  return *this;
}

RenderPassBuilderVK& RenderPassBuilderVK::SetDepthStencilAttachment(
    PixelFormat format,
    SampleCount sample_count,
    LoadAction load_action,
    StoreAction store_action) {
  vk::AttachmentDescription desc;
  desc.format = ToVKImageFormat(format);
  desc.samples = ToVKSampleCount(sample_count);
  desc.loadOp = ToVKAttachmentLoadOp(load_action);
  desc.storeOp = ToVKAttachmentStoreOp(store_action, false);
  desc.stencilLoadOp = desc.loadOp;    // Not separable in Impeller.
  desc.stencilStoreOp = desc.storeOp;  // Not separable in Impeller.
  desc.initialLayout = vk::ImageLayout::eUndefined;
  desc.finalLayout = vk::ImageLayout::eDepthStencilAttachmentOptimal;
  depth_stencil_ = desc;
  return *this;
}

RenderPassBuilderVK& RenderPassBuilderVK::SetStencilAttachment(
    PixelFormat format,
    SampleCount sample_count,
    LoadAction load_action,
    StoreAction store_action) {
  vk::AttachmentDescription desc;
  desc.format = ToVKImageFormat(format);
  desc.samples = ToVKSampleCount(sample_count);
  desc.loadOp = vk::AttachmentLoadOp::eDontCare;
  desc.storeOp = vk::AttachmentStoreOp::eDontCare;
  desc.stencilLoadOp = ToVKAttachmentLoadOp(load_action);
  desc.stencilStoreOp = ToVKAttachmentStoreOp(store_action, false);
  desc.initialLayout = vk::ImageLayout::eUndefined;
  desc.finalLayout = vk::ImageLayout::eDepthStencilAttachmentOptimal;
  depth_stencil_ = desc;
  return *this;
}

vk::UniqueRenderPass RenderPassBuilderVK::Build(
    const vk::Device& device) const {
  // This must be less than `VkPhysicalDeviceLimits::maxColorAttachments` but we
  // are not checking.
  auto color_attachments_count =
      colors_.empty() ? 0u : colors_.rbegin()->first + 1u;
  if (color0_.has_value()) {
    color_attachments_count++;
  }

  std::array<vk::AttachmentDescription, kMaxAttachments> attachments;
  std::array<vk::AttachmentReference, kMaxColorAttachments> color_refs;
  std::array<vk::AttachmentReference, kMaxColorAttachments> resolve_refs;
  vk::AttachmentReference depth_stencil_ref = kUnusedAttachmentReference;
  size_t attachments_index = 0;
  size_t color_index = 0;
  size_t resolve_index = 0;

  if (color0_.has_value()) {
    vk::AttachmentReference color_ref;
    color_ref.attachment = attachments_index;
    color_ref.layout = vk::ImageLayout::eGeneral;
    color_refs.at(color_index++) = color_ref;
    attachments.at(attachments_index++) = color0_.value();

    if (color0_resolve_.has_value()) {
      vk::AttachmentReference resolve_ref;
      resolve_ref.attachment = attachments_index;
      resolve_ref.layout = vk::ImageLayout::eGeneral;
      resolve_refs.at(resolve_index++) = resolve_ref;
      attachments.at(attachments_index++) = color0_resolve_.value();
    } else {
      resolve_refs.at(resolve_index++) = kUnusedAttachmentReference;
    }
  }

  for (const auto& color : colors_) {
    vk::AttachmentReference color_ref;
    color_ref.attachment = attachments_index;
    color_ref.layout = vk::ImageLayout::eGeneral;
    color_refs.at(color_index++) = color_ref;
    attachments.at(attachments_index++) = color.second;

    if (auto found = resolves_.find(color.first); found != resolves_.end()) {
      vk::AttachmentReference resolve_ref;
      resolve_ref.attachment = attachments_index;
      resolve_ref.layout = vk::ImageLayout::eGeneral;
      resolve_refs.at(resolve_index++) = resolve_ref;
      attachments.at(attachments_index++) = found->second;
    } else {
      resolve_refs.at(resolve_index++) = kUnusedAttachmentReference;
    }
  }

  if (depth_stencil_.has_value()) {
    depth_stencil_ref.attachment = attachments_index;
    depth_stencil_ref.layout = vk::ImageLayout::eGeneral;
    attachments.at(attachments_index++) = depth_stencil_.value();
  }

  vk::SubpassDescription subpass0;
  subpass0.pipelineBindPoint = vk::PipelineBindPoint::eGraphics;
  subpass0.setPInputAttachments(color_refs.data());
  subpass0.setInputAttachmentCount(color_index);
  subpass0.setPColorAttachments(color_refs.data());
  subpass0.setColorAttachmentCount(color_index);
  subpass0.setPResolveAttachments(resolve_refs.data());

  subpass0.setPDepthStencilAttachment(&depth_stencil_ref);

  vk::SubpassDependency self_dep;
  self_dep.srcSubpass = 0u;  // first subpass
  self_dep.dstSubpass = 0u;  // to itself
  self_dep.srcStageMask = kSelfDependencySrcStageMask;
  self_dep.srcAccessMask = kSelfDependencySrcAccessMask;
  self_dep.dstStageMask = kSelfDependencyDstStageMask;
  self_dep.dstAccessMask = kSelfDependencyDstAccessMask;
  self_dep.dependencyFlags = kSelfDependencyFlags;

  vk::RenderPassCreateInfo render_pass_desc;
  render_pass_desc.setPAttachments(attachments.data());
  render_pass_desc.setAttachmentCount(attachments_index);
  render_pass_desc.setSubpasses(subpass0);
  render_pass_desc.setDependencies(self_dep);

  auto [result, pass] = device.createRenderPassUnique(render_pass_desc);
  if (result != vk::Result::eSuccess) {
    VALIDATION_LOG << "Failed to create render pass: " << vk::to_string(result);
    return {};
  }
  return std::move(pass);
}

void InsertBarrierForInputAttachmentRead(const vk::CommandBuffer& buffer,
                                         const vk::Image& image) {
  // This barrier must be a subset of the masks specified in the subpass
  // dependency setup.
  vk::ImageMemoryBarrier barrier;
  barrier.srcAccessMask = kSelfDependencySrcAccessMask;
  barrier.dstAccessMask = kSelfDependencyDstAccessMask;
  barrier.oldLayout = vk::ImageLayout::eGeneral;
  barrier.newLayout = vk::ImageLayout::eGeneral;
  barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
  barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
  barrier.image = image;

  vk::ImageSubresourceRange image_levels;
  image_levels.aspectMask = vk::ImageAspectFlagBits::eColor;
  image_levels.baseArrayLayer = 0u;
  image_levels.baseMipLevel = 0u;
  image_levels.layerCount = VK_REMAINING_ARRAY_LAYERS;
  image_levels.levelCount = VK_REMAINING_MIP_LEVELS;
  barrier.subresourceRange = image_levels;

  buffer.pipelineBarrier(kSelfDependencySrcStageMask,  //
                         kSelfDependencyDstStageMask,  //
                         kSelfDependencyFlags,         //
                         {},                           //
                         {},                           //
                         barrier                       //
  );
}

const std::map<size_t, vk::AttachmentDescription>&
RenderPassBuilderVK::GetColorAttachments() const {
  return colors_;
}

const std::map<size_t, vk::AttachmentDescription>&
RenderPassBuilderVK::GetResolves() const {
  return resolves_;
}

const std::optional<vk::AttachmentDescription>&
RenderPassBuilderVK::GetDepthStencil() const {
  return depth_stencil_;
}

// Visible for testing.
std::optional<vk::AttachmentDescription> RenderPassBuilderVK::GetColor0()
    const {
  return color0_;
}

// Visible for testing.
std::optional<vk::AttachmentDescription> RenderPassBuilderVK::GetColor0Resolve()
    const {
  return color0_resolve_;
}

}  // namespace impeller
