// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/backend/vulkan/pipeline_vk.h"

#include "flutter/fml/make_copyable.h"
#include "flutter/fml/status_or.h"
#include "flutter/fml/trace_event.h"
#include "impeller/base/strings.h"
#include "impeller/base/timing.h"
#include "impeller/renderer/backend/vulkan/capabilities_vk.h"
#include "impeller/renderer/backend/vulkan/context_vk.h"
#include "impeller/renderer/backend/vulkan/formats_vk.h"
#include "impeller/renderer/backend/vulkan/render_pass_builder_vk.h"
#include "impeller/renderer/backend/vulkan/sampler_vk.h"
#include "impeller/renderer/backend/vulkan/shader_function_vk.h"
#include "impeller/renderer/backend/vulkan/vertex_descriptor_vk.h"

namespace impeller {

static vk::PipelineCreationFeedbackEXT EmptyFeedback() {
  vk::PipelineCreationFeedbackEXT feedback;
  // If the VK_PIPELINE_CREATION_FEEDBACK_VALID_BIT is not set in flags, an
  // implementation must not set any other bits in flags, and the values of all
  // other VkPipelineCreationFeedback data members are undefined.
  feedback.flags = vk::PipelineCreationFeedbackFlagBits::eValid;
  return feedback;
}

constexpr vk::FrontFace ToVKFrontFace(WindingOrder order) {
  switch (order) {
    case WindingOrder::kClockwise:
      return vk::FrontFace::eClockwise;
    case WindingOrder::kCounterClockwise:
      return vk::FrontFace::eCounterClockwise;
  }
  FML_UNREACHABLE();
}

static void ReportPipelineCreationFeedbackToLog(
    std::stringstream& stream,
    const vk::PipelineCreationFeedbackEXT& feedback) {
  const auto pipeline_cache_hit =
      feedback.flags &
      vk::PipelineCreationFeedbackFlagBits::eApplicationPipelineCacheHit;
  const auto base_pipeline_accl =
      feedback.flags &
      vk::PipelineCreationFeedbackFlagBits::eBasePipelineAcceleration;
  auto duration = std::chrono::duration_cast<MillisecondsF>(
      std::chrono::nanoseconds{feedback.duration});
  stream << "Time: " << duration.count() << "ms"
         << " Cache Hit: " << static_cast<bool>(pipeline_cache_hit)
         << " Base Accel: " << static_cast<bool>(base_pipeline_accl)
         << " Thread: " << std::this_thread::get_id();
}

static void ReportPipelineCreationFeedbackToLog(
    const PipelineDescriptor& desc,
    const vk::PipelineCreationFeedbackCreateInfoEXT& feedback) {
  std::stringstream stream;
  stream << std::fixed << std::showpoint << std::setprecision(2);
  stream << std::endl << ">>>>>>" << std::endl;
  stream << "Pipeline '" << desc.GetLabel() << "' ";
  ReportPipelineCreationFeedbackToLog(stream,
                                      *feedback.pPipelineCreationFeedback);
  if (feedback.pipelineStageCreationFeedbackCount != 0) {
    stream << std::endl;
  }
  for (size_t i = 0, count = feedback.pipelineStageCreationFeedbackCount;
       i < count; i++) {
    stream << "\tStage " << i + 1 << ": ";
    ReportPipelineCreationFeedbackToLog(
        stream, feedback.pPipelineStageCreationFeedbacks[i]);
    if (i != count - 1) {
      stream << std::endl;
    }
  }
  stream << std::endl << "<<<<<<" << std::endl;
  FML_LOG(ERROR) << stream.str();
}

static void ReportPipelineCreationFeedbackToTrace(
    const PipelineDescriptor& desc,
    const vk::PipelineCreationFeedbackCreateInfoEXT& feedback) {
  static int64_t gPipelineCacheHits = 0;
  static int64_t gPipelineCacheMisses = 0;
  static int64_t gPipelines = 0;
  if (feedback.pPipelineCreationFeedback->flags &
      vk::PipelineCreationFeedbackFlagBits::eApplicationPipelineCacheHit) {
    gPipelineCacheHits++;
  } else {
    gPipelineCacheMisses++;
  }
  gPipelines++;
  static constexpr int64_t kImpellerPipelineTraceID = 1988;
  FML_TRACE_COUNTER("impeller",                                   //
                    "PipelineCache",                              // series name
                    kImpellerPipelineTraceID,                     // series ID
                    "PipelineCacheHits", gPipelineCacheHits,      //
                    "PipelineCacheMisses", gPipelineCacheMisses,  //
                    "TotalPipelines", gPipelines                  //
  );
}

static void ReportPipelineCreationFeedback(
    const PipelineDescriptor& desc,
    const vk::PipelineCreationFeedbackCreateInfoEXT& feedback) {
  constexpr bool kReportPipelineCreationFeedbackToLogs = false;
  constexpr bool kReportPipelineCreationFeedbackToTraces = true;
  if (kReportPipelineCreationFeedbackToLogs) {
    ReportPipelineCreationFeedbackToLog(desc, feedback);
  }
  if (kReportPipelineCreationFeedbackToTraces) {
    ReportPipelineCreationFeedbackToTrace(desc, feedback);
  }
}

//----------------------------------------------------------------------------
/// Render Pass
/// We are NOT going to use the same render pass with the framebuffer (later)
/// and the graphics pipeline (here). Instead, we are going to ensure that the
/// sub-passes are compatible. To see the compatibility rules, see the Vulkan
/// spec:
/// https://www.khronos.org/registry/vulkan/specs/1.3-extensions/html/chap8.html#renderpass-compatibility
///
static vk::UniqueRenderPass CreateCompatRenderPassForPipeline(
    const vk::Device& device,
    const PipelineDescriptor& desc) {
  RenderPassBuilderVK builder;

  for (const auto& [bind_point, color] : desc.GetColorAttachmentDescriptors()) {
    builder.SetColorAttachment(bind_point,             //
                               color.format,           //
                               desc.GetSampleCount(),  //
                               LoadAction::kDontCare,  //
                               StoreAction::kDontCare  //
    );
  }

  if (auto depth = desc.GetDepthStencilAttachmentDescriptor();
      depth.has_value()) {
    builder.SetDepthStencilAttachment(desc.GetDepthPixelFormat(),  //
                                      desc.GetSampleCount(),       //
                                      LoadAction::kDontCare,       //
                                      StoreAction::kDontCare       //
    );
  } else if (desc.HasStencilAttachmentDescriptors()) {
    builder.SetStencilAttachment(desc.GetStencilPixelFormat(),  //
                                 desc.GetSampleCount(),         //
                                 LoadAction::kDontCare,         //
                                 StoreAction::kDontCare         //
    );
  }

  auto pass = builder.Build(device);
  if (!pass) {
    VALIDATION_LOG << "Failed to create render pass for pipeline: "
                   << desc.GetLabel();
    return {};
  }

#ifdef IMPELLER_DEBUG
  ContextVK::SetDebugName(
      device, pass.get(),
      SPrintF("Compat Render Pass: %s", desc.GetLabel().data()));
#endif  // IMPELLER_DEBUG

  return pass;
}

namespace {
fml::StatusOr<vk::UniqueDescriptorSetLayout> MakeDescriptorSetLayout(
    const PipelineDescriptor& desc,
    const std::shared_ptr<DeviceHolderVK>& device_holder,
    const std::shared_ptr<SamplerVK>& immutable_sampler) {
  std::vector<vk::DescriptorSetLayoutBinding> set_bindings;

  vk::Sampler vk_immutable_sampler =
      immutable_sampler ? immutable_sampler->GetSampler()
                        : static_cast<vk::Sampler>(VK_NULL_HANDLE);

  for (auto layout : desc.GetVertexDescriptor()->GetDescriptorSetLayouts()) {
    vk::DescriptorSetLayoutBinding set_binding;
    set_binding.binding = layout.binding;
    set_binding.descriptorCount = 1u;
    set_binding.descriptorType = ToVKDescriptorType(layout.descriptor_type);
    set_binding.stageFlags = ToVkShaderStage(layout.shader_stage);
    // TODO(143719): This specifies the immutable sampler for all sampled
    // images. This is incorrect. In cases where the shader samples from the
    // multiple images, there is currently no way to tell which sampler needs to
    // be immutable and which one needs a binding set in the render pass. Expect
    // errors if the shader has more than on sampled image. The sampling from
    // the one that is expected to be non-immutable will be incorrect.
    if (vk_immutable_sampler &&
        layout.descriptor_type == DescriptorType::kSampledImage) {
      set_binding.setImmutableSamplers(vk_immutable_sampler);
    }
    set_bindings.push_back(set_binding);
  }

  vk::DescriptorSetLayoutCreateInfo desc_set_layout_info;
  desc_set_layout_info.setBindings(set_bindings);

  auto [descs_result, descs_layout] =
      device_holder->GetDevice().createDescriptorSetLayoutUnique(
          desc_set_layout_info);
  if (descs_result != vk::Result::eSuccess) {
    VALIDATION_LOG << "unable to create uniform descriptors";
    return {fml::Status(fml::StatusCode::kUnknown,
                        "unable to create uniform descriptors")};
  }

#ifdef IMPELLER_DEBUG
  ContextVK::SetDebugName(
      device_holder->GetDevice(), descs_layout.get(),
      SPrintF("Descriptor Set Layout: %s", desc.GetLabel().data()));
#endif  // IMPELLER_DEBUG

  return fml::StatusOr<vk::UniqueDescriptorSetLayout>(std::move(descs_layout));
}

fml::StatusOr<vk::UniquePipelineLayout> MakePipelineLayout(
    const PipelineDescriptor& desc,
    const std::shared_ptr<DeviceHolderVK>& device_holder,
    const vk::DescriptorSetLayout& descs_layout) {
  vk::PipelineLayoutCreateInfo pipeline_layout_info;
  pipeline_layout_info.setSetLayouts(descs_layout);
  auto pipeline_layout = device_holder->GetDevice().createPipelineLayoutUnique(
      pipeline_layout_info);
  if (pipeline_layout.result != vk::Result::eSuccess) {
    VALIDATION_LOG << "Could not create pipeline layout for pipeline "
                   << desc.GetLabel() << ": "
                   << vk::to_string(pipeline_layout.result);
    return {fml::Status(fml::StatusCode::kUnknown,
                        "Could not create pipeline layout for pipeline.")};
  }

#ifdef IMPELLER_DEBUG
  ContextVK::SetDebugName(
      device_holder->GetDevice(), *pipeline_layout.value,
      SPrintF("Pipeline Layout %s", desc.GetLabel().data()));
#endif  // IMPELLER_DEBUG

  return std::move(pipeline_layout.value);
}

fml::StatusOr<vk::UniquePipeline> MakePipeline(
    const PipelineDescriptor& desc,
    const std::shared_ptr<DeviceHolderVK>& device_holder,
    const std::shared_ptr<PipelineCacheVK>& pso_cache,
    const vk::PipelineLayout& pipeline_layout,
    const vk::RenderPass& render_pass) {
  vk::StructureChain<vk::GraphicsPipelineCreateInfo,
                     vk::PipelineCreationFeedbackCreateInfoEXT>
      chain;

  const auto* caps = pso_cache->GetCapabilities();

  const auto supports_pipeline_creation_feedback = caps->HasExtension(
      OptionalDeviceExtensionVK::kEXTPipelineCreationFeedback);
  if (!supports_pipeline_creation_feedback) {
    chain.unlink<vk::PipelineCreationFeedbackCreateInfoEXT>();
  }

  auto& pipeline_info = chain.get<vk::GraphicsPipelineCreateInfo>();
  pipeline_info.setLayout(pipeline_layout);

  //----------------------------------------------------------------------------
  /// Dynamic States
  ///
  vk::PipelineDynamicStateCreateInfo dynamic_create_state_info;
  std::vector<vk::DynamicState> dynamic_states = {
      vk::DynamicState::eViewport,
      vk::DynamicState::eScissor,
      vk::DynamicState::eStencilReference,
  };
  dynamic_create_state_info.setDynamicStates(dynamic_states);
  pipeline_info.setPDynamicState(&dynamic_create_state_info);

  //----------------------------------------------------------------------------
  /// Viewport State
  ///
  vk::PipelineViewportStateCreateInfo viewport_state;
  viewport_state.setViewportCount(1u);
  viewport_state.setScissorCount(1u);
  // The actual viewport and scissor rects are not set here since they are
  // dynamic as mentioned above in the dynamic state info.
  pipeline_info.setPViewportState(&viewport_state);

  //----------------------------------------------------------------------------
  /// Shader Stages
  ///
  const auto& constants = desc.GetSpecializationConstants();

  std::vector<std::vector<vk::SpecializationMapEntry>> map_entries(
      desc.GetStageEntrypoints().size());
  std::vector<vk::SpecializationInfo> specialization_infos(
      desc.GetStageEntrypoints().size());
  std::vector<vk::PipelineShaderStageCreateInfo> shader_stages;

  size_t entrypoint_count = 0;
  for (const auto& entrypoint : desc.GetStageEntrypoints()) {
    auto stage = ToVKShaderStageFlagBits(entrypoint.first);
    if (!stage.has_value()) {
      VALIDATION_LOG << "Unsupported shader type in pipeline: "
                     << desc.GetLabel();
      return {fml::Status(fml::StatusCode::kUnknown,
                          "Unsupported shader type in pipeline.")};
    }

    std::vector<vk::SpecializationMapEntry>& entries =
        map_entries[entrypoint_count];
    for (auto i = 0u; i < constants.size(); i++) {
      vk::SpecializationMapEntry entry;
      entry.offset = (i * sizeof(Scalar));
      entry.size = sizeof(Scalar);
      entry.constantID = i;
      entries.emplace_back(entry);
    }

    vk::SpecializationInfo& specialization_info =
        specialization_infos[entrypoint_count];
    specialization_info.setMapEntries(map_entries[entrypoint_count]);
    specialization_info.setPData(constants.data());
    specialization_info.setDataSize(sizeof(Scalar) * constants.size());

    vk::PipelineShaderStageCreateInfo info;
    info.setStage(stage.value());
    info.setPName("main");
    info.setModule(
        ShaderFunctionVK::Cast(entrypoint.second.get())->GetModule());
    info.setPSpecializationInfo(&specialization_info);
    shader_stages.push_back(info);
    entrypoint_count++;
  }
  pipeline_info.setStages(shader_stages);

  //----------------------------------------------------------------------------
  /// Rasterization State
  ///
  vk::PipelineRasterizationStateCreateInfo rasterization_state;
  rasterization_state.setFrontFace(ToVKFrontFace(desc.GetWindingOrder()));
  rasterization_state.setCullMode(ToVKCullModeFlags(desc.GetCullMode()));
  rasterization_state.setPolygonMode(ToVKPolygonMode(desc.GetPolygonMode()));
  rasterization_state.setLineWidth(1.0f);
  rasterization_state.setDepthClampEnable(false);
  rasterization_state.setRasterizerDiscardEnable(false);
  pipeline_info.setPRasterizationState(&rasterization_state);

  //----------------------------------------------------------------------------
  /// Multi-sample State
  ///
  vk::PipelineMultisampleStateCreateInfo multisample_state;
  multisample_state.setRasterizationSamples(
      ToVKSampleCountFlagBits(desc.GetSampleCount()));
  pipeline_info.setPMultisampleState(&multisample_state);

  //----------------------------------------------------------------------------
  /// Primitive Input Assembly State
  vk::PipelineInputAssemblyStateCreateInfo input_assembly;
  const auto topology = ToVKPrimitiveTopology(desc.GetPrimitiveType());
  input_assembly.setTopology(topology);
  input_assembly.setPrimitiveRestartEnable(
      PrimitiveTopologySupportsPrimitiveRestart(desc.GetPrimitiveType()));
  pipeline_info.setPInputAssemblyState(&input_assembly);

  //----------------------------------------------------------------------------
  /// Color Blend State
  std::vector<vk::PipelineColorBlendAttachmentState> attachment_blend_state;
  for (const auto& color_desc : desc.GetColorAttachmentDescriptors()) {
    // TODO(csg): The blend states are per color attachment. But it isn't clear
    // how the color attachment indices are specified in the pipeline create
    // info. But, this should always work for one color attachment.
    attachment_blend_state.push_back(
        ToVKPipelineColorBlendAttachmentState(color_desc.second));
  }
  vk::PipelineColorBlendStateCreateInfo blend_state;
  blend_state.setAttachments(attachment_blend_state);
  pipeline_info.setPColorBlendState(&blend_state);

  // Convention wisdom says that the base acceleration pipelines are never used
  // by drivers for cache hits. Instead, the PSO cache is the preferred
  // mechanism.
  pipeline_info.setBasePipelineHandle(VK_NULL_HANDLE);
  pipeline_info.setSubpass(0u);
  pipeline_info.setRenderPass(render_pass);

  //----------------------------------------------------------------------------
  /// Vertex Input Setup
  ///
  std::vector<vk::VertexInputAttributeDescription> attr_descs;
  std::vector<vk::VertexInputBindingDescription> buffer_descs;

  const auto& stage_inputs = desc.GetVertexDescriptor()->GetStageInputs();
  const auto& stage_buffer_layouts =
      desc.GetVertexDescriptor()->GetStageLayouts();
  for (const ShaderStageIOSlot& stage_in : stage_inputs) {
    vk::VertexInputAttributeDescription attr_desc;
    attr_desc.setBinding(stage_in.binding);
    attr_desc.setLocation(stage_in.location);
    attr_desc.setFormat(ToVertexDescriptorFormat(stage_in));
    attr_desc.setOffset(stage_in.offset);
    attr_descs.push_back(attr_desc);
  }
  for (const ShaderStageBufferLayout& layout : stage_buffer_layouts) {
    vk::VertexInputBindingDescription binding_description;
    binding_description.setBinding(layout.binding);
    binding_description.setInputRate(vk::VertexInputRate::eVertex);
    binding_description.setStride(layout.stride);
    buffer_descs.push_back(binding_description);
  }

  vk::PipelineVertexInputStateCreateInfo vertex_input_state;
  vertex_input_state.setVertexAttributeDescriptions(attr_descs);
  vertex_input_state.setVertexBindingDescriptions(buffer_descs);

  pipeline_info.setPVertexInputState(&vertex_input_state);

  //----------------------------------------------------------------------------
  /// Create the depth stencil state.
  ///
  auto depth_stencil_state = ToVKPipelineDepthStencilStateCreateInfo(
      desc.GetDepthStencilAttachmentDescriptor(),
      desc.GetFrontStencilAttachmentDescriptor(),
      desc.GetBackStencilAttachmentDescriptor());
  pipeline_info.setPDepthStencilState(&depth_stencil_state);

  //----------------------------------------------------------------------------
  /// Setup the optional pipeline creation feedback struct so we can understand
  /// how Vulkan created the PSO.
  ///
  auto& feedback = chain.get<vk::PipelineCreationFeedbackCreateInfoEXT>();
  auto pipeline_feedback = EmptyFeedback();
  std::vector<vk::PipelineCreationFeedbackEXT> stage_feedbacks(
      pipeline_info.stageCount, EmptyFeedback());
  feedback.setPPipelineCreationFeedback(&pipeline_feedback);
  feedback.setPipelineStageCreationFeedbacks(stage_feedbacks);

  //----------------------------------------------------------------------------
  /// Finally, all done with the setup info. Create the pipeline itself.
  ///
  auto pipeline = pso_cache->CreatePipeline(pipeline_info);
  if (!pipeline) {
    VALIDATION_LOG << "Could not create graphics pipeline: " << desc.GetLabel();
    return {fml::Status(fml::StatusCode::kUnknown,
                        "Could not create graphics pipeline.")};
  }

  if (supports_pipeline_creation_feedback) {
    ReportPipelineCreationFeedback(desc, feedback);
  }

#ifdef IMPELLER_DEBUG
  ContextVK::SetDebugName(device_holder->GetDevice(), *pipeline,
                          SPrintF("Pipeline %s", desc.GetLabel().data()));
#endif  // IMPELLER_DEBUG

  return std::move(pipeline);
}
}  // namespace

std::unique_ptr<PipelineVK> PipelineVK::Create(
    const PipelineDescriptor& desc,
    const std::shared_ptr<DeviceHolderVK>& device_holder,
    const std::weak_ptr<PipelineLibrary>& weak_library,
    std::shared_ptr<SamplerVK> immutable_sampler) {
  TRACE_EVENT1("flutter", "PipelineVK::Create", "Name", desc.GetLabel().data());

  auto library = weak_library.lock();

  if (!device_holder || !library) {
    return nullptr;
  }

  const auto& pso_cache = PipelineLibraryVK::Cast(*library).GetPSOCache();

  fml::StatusOr<vk::UniqueDescriptorSetLayout> descs_layout =
      MakeDescriptorSetLayout(desc, device_holder, immutable_sampler);
  if (!descs_layout.ok()) {
    return nullptr;
  }

  fml::StatusOr<vk::UniquePipelineLayout> pipeline_layout =
      MakePipelineLayout(desc, device_holder, descs_layout.value().get());
  if (!pipeline_layout.ok()) {
    return nullptr;
  }

  vk::UniqueRenderPass render_pass =
      CreateCompatRenderPassForPipeline(device_holder->GetDevice(), desc);
  if (!render_pass) {
    VALIDATION_LOG << "Could not create render pass for pipeline.";
    return nullptr;
  }

  fml::StatusOr<vk::UniquePipeline> pipeline =
      MakePipeline(desc, device_holder, pso_cache,
                   pipeline_layout.value().get(), render_pass.get());
  if (!pipeline.ok()) {
    return nullptr;
  }

  auto pipeline_vk = std::unique_ptr<PipelineVK>(new PipelineVK(
      device_holder,                       //
      library,                             //
      desc,                                //
      std::move(pipeline.value()),         //
      std::move(render_pass),              //
      std::move(pipeline_layout.value()),  //
      std::move(descs_layout.value()),     //
      std::move(immutable_sampler)         //
      ));
  if (!pipeline_vk->IsValid()) {
    VALIDATION_LOG << "Could not create a valid pipeline.";
    return nullptr;
  }
  return pipeline_vk;
}

PipelineVK::PipelineVK(std::weak_ptr<DeviceHolderVK> device_holder,
                       std::weak_ptr<PipelineLibrary> library,
                       const PipelineDescriptor& desc,
                       vk::UniquePipeline pipeline,
                       vk::UniqueRenderPass render_pass,
                       vk::UniquePipelineLayout layout,
                       vk::UniqueDescriptorSetLayout descriptor_set_layout,
                       std::shared_ptr<SamplerVK> immutable_sampler)
    : Pipeline(std::move(library), desc),
      device_holder_(std::move(device_holder)),
      pipeline_(std::move(pipeline)),
      render_pass_(std::move(render_pass)),
      layout_(std::move(layout)),
      descriptor_set_layout_(std::move(descriptor_set_layout)),
      immutable_sampler_(std::move(immutable_sampler)) {
  is_valid_ = pipeline_ && render_pass_ && layout_ && descriptor_set_layout_;
}

PipelineVK::~PipelineVK() {
  if (auto device = device_holder_.lock(); !device) {
    descriptor_set_layout_.release();
    layout_.release();
    render_pass_.release();
    pipeline_.release();
  }
}

bool PipelineVK::IsValid() const {
  return is_valid_;
}

vk::Pipeline PipelineVK::GetPipeline() const {
  return *pipeline_;
}

const vk::PipelineLayout& PipelineVK::GetPipelineLayout() const {
  return *layout_;
}

const vk::DescriptorSetLayout& PipelineVK::GetDescriptorSetLayout() const {
  return *descriptor_set_layout_;
}

std::shared_ptr<PipelineVK> PipelineVK::CreateVariantForImmutableSamplers(
    const std::shared_ptr<SamplerVK>& immutable_sampler) const {
  if (!immutable_sampler) {
    return nullptr;
  }
  auto cache_key = ImmutableSamplerKeyVK{*immutable_sampler};
  Lock lock(immutable_sampler_variants_mutex_);
  auto found = immutable_sampler_variants_.find(cache_key);
  if (found != immutable_sampler_variants_.end()) {
    return found->second;
  }
  auto device_holder = device_holder_.lock();
  if (!device_holder) {
    return nullptr;
  }
  return (immutable_sampler_variants_[cache_key] =
              Create(desc_, device_holder, library_, immutable_sampler));
}

}  // namespace impeller
