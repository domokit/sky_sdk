// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <optional>
#include <tuple>
#include <vector>

// Use vulkan.hpp's convenient proc table and resolver.
#define VULKAN_HPP_NO_EXCEPTIONS 1
#define VULKAN_HPP_DISPATCH_LOADER_DYNAMIC 1
#include "vulkan/vulkan.hpp"
VULKAN_HPP_DEFAULT_DISPATCH_LOADER_DYNAMIC_STORAGE

#define S1(x) #x
#define S2(x) S1(x)
#define LOCATION __FILE__ " : " S2(__LINE__)

#define CHECK_VK_RESULT(x)                                 \
  do {                                                     \
    vk::resultCheck(static_cast<vk::Result>(x), LOCATION); \
  } while (0)

// Convenient reference to vulkan.hpp's global proc table.
auto& d = vk::defaultDispatchLoaderDynamic;

// GLFW needs to be included after Vulkan.
#include "GLFW/glfw3.h"

#include "embedder.h"  // Flutter's Embedder ABI.

struct {
  bool enable_validation_layers = true;
  bool utils_supported = false;
  bool report_supported = false;
  VkDebugReportCallbackEXT report_callback = VK_NULL_HANDLE;
  VkDebugUtilsMessengerEXT utils_messenger_callback = VK_NULL_HANDLE;
} g_debug;
// This value is calculated after the window is created.
static double g_pixelRatio = 1.0;
static const size_t kInitialWindowWidth = 800;
static const size_t kInitialWindowHeight = 600;
// Use `VK_PRESENT_MODE_FIFO_KHR` for full vsync (one swap per screen refresh),
// `VK_PRESENT_MODE_MAILBOX_KHR` for continual swap without horizontal tearing,
// or `VK_PRESENT_MODE_IMMEDIATE_KHR` for no vsync.
static const VkPresentModeKHR kPreferredPresentMode = VK_PRESENT_MODE_FIFO_KHR;
static constexpr FlutterViewId kImplicitViewId = 0;

static_assert(FLUTTER_ENGINE_VERSION == 1,
              "This Flutter Embedder was authored against the stable Flutter "
              "API at version 1. There has been a serious breakage in the "
              "API. Please read the ChangeLog and take appropriate action "
              "before updating this assertion");

/// Global struct for holding the Window+Vulkan state.
struct {
  GLFWwindow* window;

  std::vector<const char*> enabled_instance_extensions;
  std::vector<const char*> enabled_layer_names;
  VkInstance instance;
  VkSurfaceKHR surface;

  VkPhysicalDevice physical_device;
  std::vector<const char*> enabled_device_extensions;
  VkDevice device;
  uint32_t queue_family_index;
  VkQueue queue;

  VkCommandPool swapchain_command_pool;
  std::vector<VkCommandBuffer> present_transition_buffers;

  VkFence image_ready_fence;
  VkSemaphore present_transition_semaphore;

  VkSurfaceFormatKHR surface_format;
  VkSwapchainKHR swapchain;
  std::vector<VkImage> swapchain_images;
  uint32_t last_image_index;

  FlutterEngine engine;

  bool resize_pending = false;
} g_state;

void GLFW_ErrorCallback(int error, const char* description) {
  std::cerr << "GLFW Error: (" << error << ") " << description << std::endl;
}

void GLFWcursorPositionCallbackAtPhase(GLFWwindow* window,
                                       FlutterPointerPhase phase,
                                       double x,
                                       double y) {
  FlutterPointerEvent event = {};
  event.struct_size = sizeof(event);
  event.phase = phase;
  event.x = x * g_pixelRatio;
  event.y = y * g_pixelRatio;
  event.timestamp =
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::high_resolution_clock::now().time_since_epoch())
          .count();
  // This example only supports a single window, therefore we assume the event
  // occurred in the only view, the implicit view.
  event.view_id = kImplicitViewId;
  FlutterEngineSendPointerEvent(g_state.engine, &event, 1);
}

void GLFWcursorPositionCallback(GLFWwindow* window, double x, double y) {
  GLFWcursorPositionCallbackAtPhase(window, FlutterPointerPhase::kMove, x, y);
}

void GLFWmouseButtonCallback(GLFWwindow* window,
                             int key,
                             int action,
                             int mods) {
  if (key == GLFW_MOUSE_BUTTON_1 && action == GLFW_PRESS) {
    double x, y;
    glfwGetCursorPos(window, &x, &y);
    GLFWcursorPositionCallbackAtPhase(window, FlutterPointerPhase::kDown, x, y);
    glfwSetCursorPosCallback(window, GLFWcursorPositionCallback);
  }

  if (key == GLFW_MOUSE_BUTTON_1 && action == GLFW_RELEASE) {
    double x, y;
    glfwGetCursorPos(window, &x, &y);
    GLFWcursorPositionCallbackAtPhase(window, FlutterPointerPhase::kUp, x, y);
    glfwSetCursorPosCallback(window, nullptr);
  }
}

void GLFWKeyCallback(GLFWwindow* window,
                     int key,
                     int scancode,
                     int action,
                     int mods) {
  if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
    glfwSetWindowShouldClose(window, GLFW_TRUE);
  }
}

void GLFWframebufferSizeCallback(GLFWwindow* window, int width, int height) {
  g_state.resize_pending = true;

  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = width;
  event.height = height;
  event.pixel_ratio = g_pixelRatio;
  // This example only supports a single window, therefore we assume the event
  // occurred in the only view, the implicit view.
  event.view_id = kImplicitViewId;
  FlutterEngineSendWindowMetricsEvent(g_state.engine, &event);
}

void PrintUsage() {
  std::cerr
      << "usage: embedder_example_vulkan <path to project> <path to icudtl.dat>"
      << std::endl;
}

bool InitializeSwapchain() {
  if (g_state.resize_pending) {
    g_state.resize_pending = false;
    d.vkDestroySwapchainKHR(g_state.device, g_state.swapchain, nullptr);

    d.vkQueueWaitIdle(g_state.queue);
    d.vkResetCommandPool(g_state.device, g_state.swapchain_command_pool,
                         VK_COMMAND_POOL_RESET_RELEASE_RESOURCES_BIT);
  }

  /// --------------------------------------------------------------------------
  /// Choose an image format that can be presented to the surface, preferring
  /// the common BGRA+sRGB if available.
  /// --------------------------------------------------------------------------

  uint32_t format_count;
  d.vkGetPhysicalDeviceSurfaceFormatsKHR(
      g_state.physical_device, g_state.surface, &format_count, nullptr);
  std::vector<VkSurfaceFormatKHR> formats(format_count);
  d.vkGetPhysicalDeviceSurfaceFormatsKHR(
      g_state.physical_device, g_state.surface, &format_count, formats.data());
  assert(!formats.empty());  // Shouldn't be possible.

  g_state.surface_format = formats[0];
  for (const auto& format : formats) {
    if (format.format == VK_FORMAT_B8G8R8A8_UNORM &&
        format.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR) {
      g_state.surface_format = format;
    }
  }

  /// --------------------------------------------------------------------------
  /// Choose the presentable image size that's as close as possible to the
  /// window size.
  /// --------------------------------------------------------------------------

  VkExtent2D extent;

  VkSurfaceCapabilitiesKHR surface_capabilities;
  d.vkGetPhysicalDeviceSurfaceCapabilitiesKHR(
      g_state.physical_device, g_state.surface, &surface_capabilities);

  if (surface_capabilities.currentExtent.width != UINT32_MAX) {
    // If the surface reports a specific extent, we must use it.
    extent = surface_capabilities.currentExtent;
  } else {
    // `glfwGetWindowSize` returns the window size in screen coordinates, so we
    // instead use `glfwGetFramebufferSize` to get the size in pixels in order
    // to properly support high DPI displays.
    int width, height;
    glfwGetFramebufferSize(g_state.window, &width, &height);

    VkExtent2D actual_extent = {
        .width = static_cast<uint32_t>(width),
        .height = static_cast<uint32_t>(height),
    };
    actual_extent.width =
        std::max(surface_capabilities.minImageExtent.width,
                 std::min(surface_capabilities.maxImageExtent.width,
                          actual_extent.width));
    actual_extent.height =
        std::max(surface_capabilities.minImageExtent.height,
                 std::min(surface_capabilities.maxImageExtent.height,
                          actual_extent.height));
  }

  /// --------------------------------------------------------------------------
  /// Choose the present mode.
  /// --------------------------------------------------------------------------

  uint32_t mode_count;
  d.vkGetPhysicalDeviceSurfacePresentModesKHR(
      g_state.physical_device, g_state.surface, &mode_count, nullptr);
  std::vector<VkPresentModeKHR> modes(mode_count);
  d.vkGetPhysicalDeviceSurfacePresentModesKHR(
      g_state.physical_device, g_state.surface, &mode_count, modes.data());
  assert(!formats.empty());  // Shouldn't be possible.

  // If the preferred mode isn't available, just choose the first one.
  VkPresentModeKHR present_mode = modes[0];
  for (const auto& mode : modes) {
    if (mode == kPreferredPresentMode) {
      present_mode = mode;
      break;
    }
  }

  /// --------------------------------------------------------------------------
  /// Create the swapchain.
  /// --------------------------------------------------------------------------

  VkSwapchainCreateInfoKHR info = {
      .sType = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR,
      .surface = g_state.surface,
      .minImageCount = surface_capabilities.minImageCount + 1,
      .imageFormat = g_state.surface_format.format,
      .imageColorSpace = g_state.surface_format.colorSpace,
      .imageExtent = extent,
      .imageArrayLayers = 1,
      .imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT,
      .imageSharingMode = VK_SHARING_MODE_EXCLUSIVE,
      .queueFamilyIndexCount = 0,
      .pQueueFamilyIndices = nullptr,
      .preTransform = surface_capabilities.currentTransform,
      .compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR,
      .presentMode = present_mode,
      .clipped = true,
  };
  if (d.vkCreateSwapchainKHR(g_state.device, &info, nullptr,
                             &g_state.swapchain) != VK_SUCCESS) {
    return false;
  }

  /// --------------------------------------------------------------------------
  /// Fetch swapchain images.
  /// --------------------------------------------------------------------------

  uint32_t image_count;
  d.vkGetSwapchainImagesKHR(g_state.device, g_state.swapchain, &image_count,
                            nullptr);
  g_state.swapchain_images.resize(image_count);
  d.vkGetSwapchainImagesKHR(g_state.device, g_state.swapchain, &image_count,
                            g_state.swapchain_images.data());

  /// --------------------------------------------------------------------------
  /// Record a command buffer for each of the images to be executed prior to
  /// presenting.
  /// --------------------------------------------------------------------------

  g_state.present_transition_buffers.resize(g_state.swapchain_images.size());

  VkCommandBufferAllocateInfo buffers_info = {
      .sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO,
      .commandPool = g_state.swapchain_command_pool,
      .level = VK_COMMAND_BUFFER_LEVEL_PRIMARY,
      .commandBufferCount =
          static_cast<uint32_t>(g_state.present_transition_buffers.size()),
  };
  d.vkAllocateCommandBuffers(g_state.device, &buffers_info,
                             g_state.present_transition_buffers.data());

  for (size_t i = 0; i < g_state.swapchain_images.size(); i++) {
    auto image = g_state.swapchain_images[i];
    auto buffer = g_state.present_transition_buffers[i];

    VkCommandBufferBeginInfo begin_info = {
        .sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
    d.vkBeginCommandBuffer(buffer, &begin_info);

    // Flutter Engine hands back the image after writing to it
    VkImageMemoryBarrier barrier = {
        .sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER,
        .srcAccessMask = 0,
        .dstAccessMask = VK_ACCESS_MEMORY_READ_BIT,
        .oldLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
        .newLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR,
        .srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
        .dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
        .image = image,
        .subresourceRange = {
            .aspectMask = VK_IMAGE_ASPECT_COLOR_BIT,
            .baseMipLevel = 0,
            .levelCount = 1,
            .baseArrayLayer = 0,
            .layerCount = 1,
        }};
    d.vkCmdPipelineBarrier(
        buffer,                                         // commandBuffer
        VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,  // srcStageMask
        VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT,           // dstStageMask
        0,                                              // dependencyFlags
        0,                                              // memoryBarrierCount
        nullptr,                                        // pMemoryBarriers
        0,        // bufferMemoryBarrierCount
        nullptr,  // pBufferMemoryBarriers
        1,        // imageMemoryBarrierCount
        &barrier  // pImageMemoryBarriers
    );

    d.vkEndCommandBuffer(buffer);
  }

  return true;  // \o/
}

FlutterVulkanImage FlutterGetNextImageCallback(
    void* user_data,
    const FlutterFrameInfo* frame_info) {
  // If the GLFW framebuffer has been resized, discard the swapchain and create
  // a new one.
  if (g_state.resize_pending) {
    InitializeSwapchain();
  }

  d.vkAcquireNextImageKHR(g_state.device, g_state.swapchain, UINT64_MAX,
                          nullptr, g_state.image_ready_fence,
                          &g_state.last_image_index);

  // Flutter Engine expects the image to be available for transitioning and
  // attaching immediately, and so we need to force a host sync here before
  // returning.
  d.vkWaitForFences(g_state.device, 1, &g_state.image_ready_fence, true,
                    UINT64_MAX);
  d.vkResetFences(g_state.device, 1, &g_state.image_ready_fence);

  return {
      .struct_size = sizeof(FlutterVulkanImage),
      .image = reinterpret_cast<uint64_t>(
          g_state.swapchain_images[g_state.last_image_index]),
      .format = g_state.surface_format.format,
  };
}

bool FlutterPresentCallback(void* user_data, const FlutterVulkanImage* image) {
  VkPipelineStageFlags stage_flags =
      VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
  VkSubmitInfo submit_info = {
      .sType = VK_STRUCTURE_TYPE_SUBMIT_INFO,
      .waitSemaphoreCount = 0,
      .pWaitSemaphores = nullptr,
      .pWaitDstStageMask = &stage_flags,
      .commandBufferCount = 1,
      .pCommandBuffers =
          &g_state.present_transition_buffers[g_state.last_image_index],
      .signalSemaphoreCount = 1,
      .pSignalSemaphores = &g_state.present_transition_semaphore,
  };
  d.vkQueueSubmit(g_state.queue, 1, &submit_info, nullptr);

  VkPresentInfoKHR present_info = {
      .sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR,
      .waitSemaphoreCount = 1,
      .pWaitSemaphores = &g_state.present_transition_semaphore,
      .swapchainCount = 1,
      .pSwapchains = &g_state.swapchain,
      .pImageIndices = &g_state.last_image_index,
  };
  VkResult result = d.vkQueuePresentKHR(g_state.queue, &present_info);

  // If the swapchain is no longer compatible with the surface, discard the
  // swapchain and create a new one.
  if (result == VK_SUBOPTIMAL_KHR || result == VK_ERROR_OUT_OF_DATE_KHR) {
    InitializeSwapchain();
  }
  d.vkQueueWaitIdle(g_state.queue);

  return result == VK_SUCCESS;
}

void* FlutterGetInstanceProcAddressCallback(
    void* user_data,
    FlutterVulkanInstanceHandle instance,
    const char* procname) {
  auto* proc = glfwGetInstanceProcAddress(
      reinterpret_cast<VkInstance>(instance), procname);
  return reinterpret_cast<void*>(proc);
}

VkBool32 DebugReportCallback(VkDebugReportFlagsEXT flags,
                             VkDebugReportObjectTypeEXT /* objectType */,
                             uint64_t /* object */,
                             size_t /* location */,
                             int32_t /* messageCode */,
                             const char* pLayerPrefix,
                             const char* pMessage,
                             void* /* pUserData */) {
  if (flags & VK_DEBUG_REPORT_INFORMATION_BIT_EXT) {
    std::cout << "VULKAN: (" << pLayerPrefix << ") " << pMessage << std::endl;
  } else if (flags & VK_DEBUG_REPORT_WARNING_BIT_EXT) {
    std::cout << "VULKAN DEBUG: (" << pLayerPrefix << ") " << pMessage
              << std::endl;
  } else if (flags & VK_DEBUG_REPORT_PERFORMANCE_WARNING_BIT_EXT) {
    std::cout << "VULKAN PERF WARNING: (" << pLayerPrefix << ") " << pMessage
              << std::endl;
  } else if (flags & VK_DEBUG_REPORT_ERROR_BIT_EXT) {
    std::cerr << "VULKAN ERROR: (" << pLayerPrefix << ") " << pMessage
              << std::endl;
  } else if (flags & VK_DEBUG_REPORT_DEBUG_BIT_EXT) {
    std::cout << "VULKAN DEBUG: (" << pLayerPrefix << ") " << pMessage
              << std::endl;
  }
  return VK_FALSE;
}

VkBool32 DebugUtilsCallback(VkDebugUtilsMessageSeverityFlagBitsEXT severity,
                            VkDebugUtilsMessageTypeFlagsEXT /* types */,
                            const VkDebugUtilsMessengerCallbackDataEXT* cb_data,
                            void* /* pUserData */) {
  if (strstr(cb_data->pMessage, "ALL_GRAPHICS_BIT") ||
      strstr(cb_data->pMessage, "ALL_COMMANDS_BIT")) {
    return VK_FALSE;
  }
  if (severity & VK_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT) {
    std::cout << "VULKAN: (" << cb_data->pMessageIdName << ") "
              << cb_data->pMessage << std::endl;
  } else if (severity & VK_DEBUG_UTILS_MESSAGE_SEVERITY_INFO_BIT_EXT) {
    std::cout << "VULKAN INFO: (" << cb_data->pMessageIdName << ") "
              << cb_data->pMessage << std::endl;
  } else if (severity & VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT) {
    std::cout << "VULKAN WARN: (" << cb_data->pMessageIdName << ") "
              << cb_data->pMessage << std::endl;
  } else if (severity & VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT) {
    std::cerr << "VULKAN ERROR: (" << cb_data->pMessageIdName << ") "
              << cb_data->pMessage << std::endl;
  }
  return VK_TRUE;
}

int main(int argc, char** argv) {
  if (argc != 3) {
    PrintUsage();
    return 1;
  }

  std::string project_path = argv[1];
  std::string icudtl_path = argv[2];

  /// --------------------------------------------------------------------------
  /// Create a GLFW window.
  /// --------------------------------------------------------------------------

  {
    if (!glfwInit()) {
      std::cerr << "Failed to initialize GLFW." << std::endl;
      return EXIT_FAILURE;
    }

    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    g_state.window = glfwCreateWindow(kInitialWindowWidth, kInitialWindowHeight,
                                      "Flutter", nullptr, nullptr);
    if (!g_state.window) {
      std::cerr << "Failed to create GLFW window." << std::endl;
      return EXIT_FAILURE;
    }

    int framebuffer_width, framebuffer_height;
    glfwGetFramebufferSize(g_state.window, &framebuffer_width,
                           &framebuffer_height);
    g_pixelRatio = framebuffer_width / kInitialWindowWidth;

    glfwSetErrorCallback(GLFW_ErrorCallback);
  }

  /// --------------------------------------------------------------------------
  /// Dynamically load the Vulkan loader with GLFW and use it to populate GLAD's
  /// proc table.
  /// --------------------------------------------------------------------------

  if (!glfwVulkanSupported()) {
    std::cerr << "GLFW was unable to resolve either a Vulkan loader or a "
                 "compatible physical device!"
              << std::endl;
#if defined(__APPLE__)
    std::cerr
        << "NOTE: Apple platforms don't ship with a Vulkan loader or any "
           "Vulkan drivers. Follow this guide to set up a Vulkan loader on "
           "macOS and use the MoltenVK ICD: "
           "https://vulkan.lunarg.com/doc/sdk/latest/mac/getting_started.html"
        << std::endl;
#endif
    return EXIT_FAILURE;
  }

  VULKAN_HPP_DEFAULT_DISPATCHER.init(glfwGetInstanceProcAddress);

  /// --------------------------------------------------------------------------
  /// Create a Vulkan instance.
  /// --------------------------------------------------------------------------

  {
    uint32_t extension_count;
    const char** glfw_extensions =
        glfwGetRequiredInstanceExtensions(&extension_count);
    g_state.enabled_instance_extensions.resize(extension_count);
    memcpy(g_state.enabled_instance_extensions.data(), glfw_extensions,
           extension_count * sizeof(char*));

    if (g_debug.enable_validation_layers) {
      auto props = vk::enumerateInstanceExtensionProperties();
      for (const auto& prop : props.value) {
        if (strcmp(prop.extensionName,
                   VK_EXT_VALIDATION_FEATURES_EXTENSION_NAME) == 0) {
          g_state.enabled_instance_extensions.push_back(
              VK_EXT_VALIDATION_FEATURES_EXTENSION_NAME);
        }
        if (strcmp(prop.extensionName, VK_EXT_DEBUG_UTILS_EXTENSION_NAME) ==
            0) {
          g_debug.utils_supported = true;
        }
        if (strcmp(prop.extensionName, VK_EXT_DEBUG_REPORT_EXTENSION_NAME) ==
            0) {
          g_debug.report_supported = true;
        }
      }
      if (g_debug.utils_supported) {
        g_state.enabled_instance_extensions.push_back(
            VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
        g_debug.report_supported = false;
      } else if (g_debug.report_supported) {
        g_state.enabled_instance_extensions.push_back(
            VK_EXT_DEBUG_REPORT_EXTENSION_NAME);
      }

      auto available_layers = vk::enumerateInstanceLayerProperties();
      for (const auto& l : available_layers.value) {
        if (strcmp(l.layerName, "VK_LAYER_KHRONOS_validation") == 0) {
          g_state.enabled_layer_names.push_back("VK_LAYER_KHRONOS_validation");
        }
      }
    }

    std::cout << "Enabling " << g_state.enabled_instance_extensions.size()
              << " instance extensions:" << std::endl;
    for (const auto& extension : g_state.enabled_instance_extensions) {
      std::cout << "  - " << extension << std::endl;
    }
    std::cout << "Enabling " << g_state.enabled_layer_names.size()
              << " layers:" << std::endl;
    for (const auto& layer : g_state.enabled_layer_names) {
      std::cout << "  - " << layer << std::endl;
    }

    VkApplicationInfo app_info = {
        .sType = VK_STRUCTURE_TYPE_APPLICATION_INFO,
        .pNext = nullptr,
        .pApplicationName = "Flutter",
        .applicationVersion = VK_MAKE_VERSION(1, 0, 0),
        .pEngineName = "No Engine",
        .engineVersion = VK_MAKE_VERSION(1, 0, 0),
        .apiVersion = VK_MAKE_VERSION(1, 1, 0),
    };
    VkInstanceCreateInfo info = {};
    info.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
    info.flags = 0;
    info.pApplicationInfo = &app_info;
    info.enabledExtensionCount = g_state.enabled_instance_extensions.size();
    info.ppEnabledExtensionNames = g_state.enabled_instance_extensions.data();
    info.enabledLayerCount = g_state.enabled_layer_names.size();
    info.ppEnabledLayerNames = g_state.enabled_layer_names.data();

    if (d.vkCreateInstance(&info, nullptr, &g_state.instance) != VK_SUCCESS) {
      std::cerr << "Failed to create Vulkan instance." << std::endl;
      return EXIT_FAILURE;
    }
  }

  // Load instance procs.
  VULKAN_HPP_DEFAULT_DISPATCHER.init(vk::Instance(g_state.instance));

  /// --------------------------------------------------------------------------
  /// Create the window surface.
  /// --------------------------------------------------------------------------

  if (glfwCreateWindowSurface(g_state.instance, g_state.window, NULL,
                              &g_state.surface) != VK_SUCCESS) {
    std::cerr << "Failed to create window surface." << std::endl;
    return EXIT_FAILURE;
  }

  /// --------------------------------------------------------------------------
  /// Select a compatible physical device.
  /// --------------------------------------------------------------------------

  {
    uint32_t count;
    d.vkEnumeratePhysicalDevices(g_state.instance, &count, nullptr);
    std::vector<VkPhysicalDevice> physical_devices(count);
    d.vkEnumeratePhysicalDevices(g_state.instance, &count,
                                 physical_devices.data());

    std::cout << "Enumerating " << count << " physical device(s)." << std::endl;

    uint32_t selected_score = 0;
    for (const auto& pdevice : physical_devices) {
      VkPhysicalDeviceProperties properties;
      VkPhysicalDeviceFeatures features;
      d.vkGetPhysicalDeviceProperties(pdevice, &properties);
      d.vkGetPhysicalDeviceFeatures(pdevice, &features);

      std::cout << "Checking device: " << properties.deviceName << std::endl;

      uint32_t score = 0;
      std::vector<const char*> supported_extensions;

      uint32_t qfp_count;
      d.vkGetPhysicalDeviceQueueFamilyProperties(pdevice, &qfp_count, nullptr);
      std::vector<VkQueueFamilyProperties> qfp(qfp_count);
      d.vkGetPhysicalDeviceQueueFamilyProperties(pdevice, &qfp_count,
                                                 qfp.data());
      std::optional<uint32_t> graphics_queue_family;
      for (uint32_t i = 0; i < qfp.size(); i++) {
        // Only pick graphics queues that can also present to the surface.
        // Graphics queues that can't present are rare if not nonexistent, but
        // the spec allows for this, so check it anyways.
        VkBool32 surface_present_supported;
        d.vkGetPhysicalDeviceSurfaceSupportKHR(pdevice, i, g_state.surface,
                                               &surface_present_supported);

        if (!graphics_queue_family.has_value() &&
            qfp[i].queueFlags & VK_QUEUE_GRAPHICS_BIT &&
            surface_present_supported) {
          graphics_queue_family = i;
        }
      }

      // Skip physical devices that don't have a graphics queue.
      if (!graphics_queue_family.has_value()) {
        std::cout << "  - Skipping due to no suitable graphics queues."
                  << std::endl;
        continue;
      }

      // Prefer discrete GPUs.
      if (properties.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU) {
        score += 1 << 30;
      }

      uint32_t extension_count;
      d.vkEnumerateDeviceExtensionProperties(pdevice, nullptr, &extension_count,
                                             nullptr);
      std::vector<VkExtensionProperties> available_extensions(extension_count);
      d.vkEnumerateDeviceExtensionProperties(pdevice, nullptr, &extension_count,
                                             available_extensions.data());

      bool supports_swapchain = false;
      for (const auto& available_extension : available_extensions) {
        if (strcmp(VK_KHR_SWAPCHAIN_EXTENSION_NAME,
                   available_extension.extensionName) == 0) {
          supports_swapchain = true;
          supported_extensions.push_back(VK_KHR_SWAPCHAIN_EXTENSION_NAME);
        }
        // The spec requires VK_KHR_portability_subset be enabled whenever it's
        // available on a device. It's present on compatibility ICDs like
        // MoltenVK.
        else if (strcmp("VK_KHR_portability_subset",
                        available_extension.extensionName) == 0) {
          supported_extensions.push_back("VK_KHR_portability_subset");
        }

        // Prefer GPUs that support VK_KHR_get_memory_requirements2.
        else if (strcmp(VK_KHR_GET_MEMORY_REQUIREMENTS_2_EXTENSION_NAME,
                        available_extension.extensionName) == 0) {
          score += 1 << 29;
          supported_extensions.push_back(
              VK_KHR_GET_MEMORY_REQUIREMENTS_2_EXTENSION_NAME);
        }
      }

      // Skip physical devices that don't have swapchain support.
      if (!supports_swapchain) {
        std::cout << "  - Skipping due to lack of swapchain support."
                  << std::endl;
        continue;
      }

      // Prefer GPUs with larger max texture sizes.
      score += properties.limits.maxImageDimension2D;

      if (selected_score < score) {
        std::cout << "  - This is the best device so far. Score: 0x" << std::hex
                  << score << std::dec << std::endl;

        selected_score = score;
        g_state.physical_device = pdevice;
        g_state.enabled_device_extensions = supported_extensions;
        g_state.queue_family_index = graphics_queue_family.value_or(
            std::numeric_limits<uint32_t>::max());
      }
    }

    if (g_state.physical_device == nullptr) {
      std::cerr << "Failed to find a compatible Vulkan physical device."
                << std::endl;
      return EXIT_FAILURE;
    }
  }

  /// --------------------------------------------------------------------------
  /// Create a logical device and a graphics queue handle.
  /// --------------------------------------------------------------------------

  std::cout << "Enabling " << g_state.enabled_device_extensions.size()
            << " device extensions:" << std::endl;
  for (const char* extension : g_state.enabled_device_extensions) {
    std::cout << "  - " << extension << std::endl;
  }

  {
    VkPhysicalDeviceFeatures device_features = {};

    VkDeviceQueueCreateInfo graphics_queue = {};
    graphics_queue.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
    graphics_queue.queueFamilyIndex = g_state.queue_family_index;
    graphics_queue.queueCount = 1;
    float priority = 1.0f;
    graphics_queue.pQueuePriorities = &priority;

    VkDeviceCreateInfo device_info = {};
    device_info.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
    device_info.enabledExtensionCount =
        g_state.enabled_device_extensions.size();
    device_info.ppEnabledExtensionNames =
        g_state.enabled_device_extensions.data();
    device_info.pEnabledFeatures = &device_features;
    device_info.queueCreateInfoCount = 1;
    device_info.pQueueCreateInfos = &graphics_queue;

    if (d.vkCreateDevice(g_state.physical_device, &device_info, nullptr,
                         &g_state.device) != VK_SUCCESS) {
      std::cerr << "Failed to create Vulkan logical device." << std::endl;
      return EXIT_FAILURE;
    }
  }

  d.vkGetDeviceQueue(g_state.device, g_state.queue_family_index, 0,
                     &g_state.queue);

  /// --------------------------------------------------------------------------
  /// Create sync primitives and command pool to use in the render loop
  /// callbacks.
  /// --------------------------------------------------------------------------

  {
    VkFenceCreateInfo f_info = {.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
    d.vkCreateFence(g_state.device, &f_info, nullptr,
                    &g_state.image_ready_fence);

    VkSemaphoreCreateInfo s_info = {
        .sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO};
    d.vkCreateSemaphore(g_state.device, &s_info, nullptr,
                        &g_state.present_transition_semaphore);

    VkCommandPoolCreateInfo pool_info = {
        .sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO,
        .queueFamilyIndex = g_state.queue_family_index,
    };
    d.vkCreateCommandPool(g_state.device, &pool_info, nullptr,
                          &g_state.swapchain_command_pool);
  }

  /// --------------------------------------------------------------------------
  /// Create swapchain.
  /// --------------------------------------------------------------------------

  if (!InitializeSwapchain()) {
    std::cerr << "Failed to create swapchain." << std::endl;
    return EXIT_FAILURE;
  }

  /// --------------------------------------------------------------------------
  /// Start Flutter Engine.
  /// --------------------------------------------------------------------------

  {
    FlutterRendererConfig config = {};
    config.type = kVulkan;
    config.vulkan.struct_size = sizeof(config.vulkan);
    config.vulkan.version = VK_MAKE_VERSION(1, 1, 0);
    config.vulkan.instance = g_state.instance;
    config.vulkan.physical_device = g_state.physical_device;
    config.vulkan.device = g_state.device;
    config.vulkan.queue_family_index = g_state.queue_family_index;
    config.vulkan.queue = g_state.queue;
    config.vulkan.enabled_instance_extension_count =
        g_state.enabled_instance_extensions.size();
    config.vulkan.enabled_instance_extensions =
        g_state.enabled_instance_extensions.data();
    config.vulkan.enabled_device_extension_count =
        g_state.enabled_device_extensions.size();
    config.vulkan.enabled_device_extensions =
        g_state.enabled_device_extensions.data();
    config.vulkan.get_instance_proc_address_callback =
        FlutterGetInstanceProcAddressCallback;
    config.vulkan.get_next_image_callback = FlutterGetNextImageCallback;
    config.vulkan.present_image_callback = FlutterPresentCallback;

    // This directory is generated by `flutter build bundle`.
    std::string assets_path = project_path + "/build/flutter_assets";
    FlutterProjectArgs args = {
        .struct_size = sizeof(FlutterProjectArgs),
        .assets_path = assets_path.c_str(),
        .icu_data_path =
            icudtl_path.c_str(),  // Find this in your bin/cache directory.
    };
    FlutterEngineResult result =
        FlutterEngineRun(FLUTTER_ENGINE_VERSION, &config, &args, g_state.window,
                         &g_state.engine);
    if (result != kSuccess || g_state.engine == nullptr) {
      std::cerr << "Failed to start Flutter Engine." << std::endl;
      return EXIT_FAILURE;
    }

    // Trigger a FlutterEngineSendWindowMetricsEvent to communicate the initial
    // size of the window.
    int width, height;
    glfwGetFramebufferSize(g_state.window, &width, &height);
    GLFWframebufferSizeCallback(g_state.window, width, height);
    g_state.resize_pending = false;
  }

  /// --------------------------------------------------------------------------
  /// GLFW render loop.
  /// --------------------------------------------------------------------------

  glfwSetKeyCallback(g_state.window, GLFWKeyCallback);
  glfwSetFramebufferSizeCallback(g_state.window, GLFWframebufferSizeCallback);
  glfwSetMouseButtonCallback(g_state.window, GLFWmouseButtonCallback);

  while (!glfwWindowShouldClose(g_state.window)) {
    glfwWaitEvents();
  }

  /// --------------------------------------------------------------------------
  /// Cleanup.
  /// --------------------------------------------------------------------------

  if (FlutterEngineShutdown(g_state.engine) != kSuccess) {
    std::cerr << "Flutter Engine shutdown failed." << std::endl;
  }

  d.vkDestroyCommandPool(g_state.device, g_state.swapchain_command_pool,
                         nullptr);
  d.vkDestroySemaphore(g_state.device, g_state.present_transition_semaphore,
                       nullptr);
  d.vkDestroyFence(g_state.device, g_state.image_ready_fence, nullptr);

  if (g_state.swapchain) {
    d.vkDestroySwapchainKHR(g_state.device, g_state.swapchain, nullptr);
  }

  d.vkDestroyDevice(g_state.device, nullptr);
  d.vkDestroySurfaceKHR(g_state.instance, g_state.surface, nullptr);

  if (g_debug.enable_validation_layers) {
    if (g_debug.report_callback) {
      d.vkDestroyDebugReportCallbackEXT(g_state.instance,
                                        g_debug.report_callback, nullptr);
    }
    if (g_debug.utils_messenger_callback) {
      d.vkDestroyDebugUtilsMessengerEXT(
          g_state.instance, g_debug.utils_messenger_callback, nullptr);
    }
  }

  d.vkDestroyInstance(g_state.instance, nullptr);

  glfwDestroyWindow(g_state.window);
  glfwTerminate();

  return 0;
}
