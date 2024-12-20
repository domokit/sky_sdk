// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/backend/vulkan/driver_info_vk.h"

#include <iomanip>
#include <sstream>
#include <string_view>
#include <unordered_map>

#include "flutter/fml/build_config.h"

namespace impeller {

const std::unordered_map<std::string_view, AdrenoGPU> kAdrenoVersions = {
    // X
    // Note: I don't know if these strings actually match as there don't seem to
    // be any android devices that use these GPUs.
    {"X185", AdrenoGPU::kAdrenoX185},
    {"X145", AdrenoGPU::kAdrenoX145},
    // 700
    {"750", AdrenoGPU::kAdreno750},
    {"740", AdrenoGPU::kAdreno740},
    {"735", AdrenoGPU::kAdreno735},
    {"721", AdrenoGPU::kAdreno732},
    {"730", AdrenoGPU::kAdreno730},
    {"725", AdrenoGPU::kAdreno725},
    {"720", AdrenoGPU::kAdreno720},
    {"710", AdrenoGPU::kAdreno710},
    {"702", AdrenoGPU::kAdreno702},

    // 600
    {"695", AdrenoGPU::kAdreno695},
    {"690", AdrenoGPU::kAdreno690},
    {"685", AdrenoGPU::kAdreno685},
    {"680", AdrenoGPU::kAdreno680},
    {"675", AdrenoGPU::kAdreno675},
    {"663", AdrenoGPU::kAdreno663},
    {"660", AdrenoGPU::kAdreno660},
    {"650", AdrenoGPU::kAdreno650},
    {"644", AdrenoGPU::kAdreno644},
    {"643L", AdrenoGPU::kAdreno643L},
    {"642", AdrenoGPU::kAdreno642},
    {"642L", AdrenoGPU::kAdreno642L},
    {"640", AdrenoGPU::kAdreno640},
    {"630", AdrenoGPU::kAdreno630},
    {"620", AdrenoGPU::kAdreno620},
    {"619", AdrenoGPU::kAdreno619},
    {"619L", AdrenoGPU::kAdreno619L},
    {"618", AdrenoGPU::kAdreno618},
    {"616", AdrenoGPU::kAdreno616},
    {"615", AdrenoGPU::kAdreno615},
    {"613", AdrenoGPU::kAdreno613},
    {"612", AdrenoGPU::kAdreno612},
    {"610", AdrenoGPU::kAdreno610},
    {"608", AdrenoGPU::kAdreno608},
    {"605", AdrenoGPU::kAdreno605},
    // 500
    {"540", AdrenoGPU::kAdreno540},
    {"530", AdrenoGPU::kAdreno530},
    {"512", AdrenoGPU::kAdreno512},
    {"510", AdrenoGPU::kAdreno510},
    {"509", AdrenoGPU::kAdreno509},
    {"508", AdrenoGPU::kAdreno508},
    {"506", AdrenoGPU::kAdreno506},
    {"505", AdrenoGPU::kAdreno505},
    {"504", AdrenoGPU::kAdreno504},
};

const std::unordered_map<std::string_view, MaliGPU> kMaliVersions = {
    // 5th Gen.
    {"G925", MaliGPU::kG925},
    {"G725", MaliGPU::kG725},
    {"G625", MaliGPU::kG625},
    {"G720", MaliGPU::kG720},
    {"G620", MaliGPU::kG620},

    // Valhall
    // Note: there is an Immortalis-G715 a Mali-G715
    {"G715", MaliGPU::kG715},
    {"G615", MaliGPU::kG615},
    {"G710", MaliGPU::kG710},
    {"G610", MaliGPU::kG610},
    {"G510", MaliGPU::kG510},
    {"G310", MaliGPU::kG310},
    {"G78", MaliGPU::kG78},
    {"G68", MaliGPU::kG68},
    {"G77", MaliGPU::kG77},
    {"G57", MaliGPU::kG57},

    // Bifrost
    {"G76", MaliGPU::kG76},
    {"G72", MaliGPU::kG72},
    {"G52", MaliGPU::kG52},
    {"G71", MaliGPU::kG71},
    {"G51", MaliGPU::kG51},
    {"G31", MaliGPU::kG31},

    // These might be Vulkan 1.0 Only.
    {"T880", MaliGPU::kT880},
    {"T860", MaliGPU::kT860},
    {"T830", MaliGPU::kT830},
    {"T820", MaliGPU::kT820},
    {"T760", MaliGPU::kT760},
};

AdrenoGPU GetAdrenoVersion(std::string_view version) {
  /// The format that Adreno names follow is "Adreno (TM) VERSION".
  auto paren_pos = version.find("Adreno (TM) ");
  if (paren_pos == std::string::npos) {
    return AdrenoGPU::kUnknown;
  }
  auto version_string = version.substr(paren_pos + 12);
  const auto& result = kAdrenoVersions.find(version_string);
  if (result == kAdrenoVersions.end()) {
    return AdrenoGPU::kUnknown;
  }
  return result->second;
}

MaliGPU GetMaliVersion(std::string_view version) {
  // These names are usually Mali-VERSION or Mali-Version-EXTRA_CRAP.
  auto dash_pos = version.find("Mali-");
  if (dash_pos == std::string::npos) {
    return MaliGPU::kUnknown;
  }
  auto version_string_with_trailing = version.substr(dash_pos + 5);
  // Remove any trailing crap if present.
  auto more_dash_pos = version_string_with_trailing.find("-");
  if (more_dash_pos != std::string::npos) {
    version_string_with_trailing =
        version_string_with_trailing.substr(0, more_dash_pos);
  }

  const auto& result = kMaliVersions.find(version_string_with_trailing);
  if (result == kMaliVersions.end()) {
    return MaliGPU::kUnknown;
  }
  return result->second;
}

constexpr VendorVK IdentifyVendor(uint32_t vendor) {
  // Check if the vendor has a PCI ID:
  // https://pcisig.com/membership/member-companies
  switch (vendor) {
    case 0x1AE0:
      return VendorVK::kGoogle;
    case 0x168C:
    case 0x17CB:
    case 0x1969:
    case 0x5143:
      return VendorVK::kQualcomm;
    case 0x13B5:
      return VendorVK::kARM;
    case 0x1010:
      return VendorVK::kImgTec;
    case 0x1002:
    case 0x1022:
      return VendorVK::kAMD;
    case 0x10DE:
      return VendorVK::kNvidia;
    case 0x8086:  // :)
      return VendorVK::kIntel;
    case 0x106B:
      return VendorVK::kApple;
    case 0x19E5:
      return VendorVK::kHuawei;
  }
  // Check if the ID is a known Khronos vendor.
  switch (vendor) {
    case VK_VENDOR_ID_MESA:
      return VendorVK::kMesa;
      // There are others but have never been observed. These can be added as
      // needed.
  }
  return VendorVK::kUnknown;
}

constexpr const char* VendorToString(VendorVK vendor) {
  switch (vendor) {
    case VendorVK::kUnknown:
      return "Unknown";
    case VendorVK::kGoogle:
      return "Google";
    case VendorVK::kQualcomm:
      return "Qualcomm";
    case VendorVK::kARM:
      return "ARM";
    case VendorVK::kImgTec:
      return "ImgTec PowerVR";
    case VendorVK::kAMD:
      return "AMD";
    case VendorVK::kNvidia:
      return "Nvidia";
    case VendorVK::kIntel:
      return "Intel";
    case VendorVK::kMesa:
      return "Mesa";
    case VendorVK::kApple:
      return "Apple";
    case VendorVK::kHuawei:
      return "Huawei";
  }
  FML_UNREACHABLE();
}

constexpr const char* DeviceTypeToString(DeviceTypeVK type) {
  switch (type) {
    case DeviceTypeVK::kUnknown:
      return "Unknown";
    case DeviceTypeVK::kIntegratedGPU:
      return "Integrated GPU";
    case DeviceTypeVK::kDiscreteGPU:
      return "Discrete GPU";
    case DeviceTypeVK::kVirtualGPU:
      return "Virtual GPU";
    case DeviceTypeVK::kCPU:
      return "CPU";
  }
  FML_UNREACHABLE();
}

constexpr DeviceTypeVK ToDeviceType(const vk::PhysicalDeviceType& type) {
  switch (type) {
    case vk::PhysicalDeviceType::eOther:
      return DeviceTypeVK::kUnknown;
    case vk::PhysicalDeviceType::eIntegratedGpu:
      return DeviceTypeVK::kIntegratedGPU;
    case vk::PhysicalDeviceType::eDiscreteGpu:
      return DeviceTypeVK::kDiscreteGPU;
    case vk::PhysicalDeviceType::eVirtualGpu:
      return DeviceTypeVK::kVirtualGPU;
    case vk::PhysicalDeviceType::eCpu:
      return DeviceTypeVK::kCPU;
      break;
  }
  return DeviceTypeVK::kUnknown;
}

DriverInfoVK::DriverInfoVK(const vk::PhysicalDevice& device) {
  auto props = device.getProperties();
  api_version_ = Version{VK_API_VERSION_MAJOR(props.apiVersion),
                         VK_API_VERSION_MINOR(props.apiVersion),
                         VK_API_VERSION_PATCH(props.apiVersion)};
  vendor_ = IdentifyVendor(props.vendorID);
  if (vendor_ == VendorVK::kUnknown) {
    FML_LOG(WARNING) << "Unknown GPU Driver Vendor: " << props.vendorID
                     << ". This is not an error.";
  }
  type_ = ToDeviceType(props.deviceType);
  if (props.deviceName.data() != nullptr) {
    driver_name_ = props.deviceName.data();
  }

  switch (vendor_) {
    case VendorVK::kQualcomm:
      adreno_gpu_ = GetAdrenoVersion(driver_name_);
      break;
    case VendorVK::kARM:
      mali_gpu_ = GetMaliVersion(driver_name_);
      break;
    default:
      break;
  }
}

DriverInfoVK::~DriverInfoVK() = default;

const Version& DriverInfoVK::GetAPIVersion() const {
  return api_version_;
}

const VendorVK& DriverInfoVK::GetVendor() const {
  return vendor_;
}

const DeviceTypeVK& DriverInfoVK::GetDeviceType() const {
  return type_;
}

const std::string& DriverInfoVK::GetDriverName() const {
  return driver_name_;
}

void DriverInfoVK::DumpToLog() const {
  std::vector<std::pair<std::string, std::string>> items;
  items.emplace_back("Name", driver_name_);
  items.emplace_back("API Version", api_version_.ToString());
  items.emplace_back("Vendor", VendorToString(vendor_));
  items.emplace_back("Device Type", DeviceTypeToString(type_));
  items.emplace_back("Is Emulator", std::to_string(IsEmulator()));

  size_t padding = 0;

  for (const auto& item : items) {
    padding = std::max(padding, item.first.size());
  }

  padding += 1;

  std::stringstream stream;

  stream << std::endl;

  stream << "--- Driver Information ------------------------------------------";

  stream << std::endl;

  for (const auto& item : items) {
    stream << "| " << std::setw(static_cast<int>(padding)) << item.first
           << std::setw(0) << ": " << item.second << std::endl;
  }

  stream << "-----------------------------------------------------------------";

  FML_LOG(IMPORTANT) << stream.str();
}

bool DriverInfoVK::CanBatchSubmitCommandBuffers() const {
  return vendor_ == VendorVK::kARM ||
         (adreno_gpu_.has_value() &&
          adreno_gpu_.value() >= AdrenoGPU::kAdreno702);
}

bool DriverInfoVK::IsEmulator() const {
#if FML_OS_ANDROID
  // Google SwiftShader on Android.
  if (type_ == DeviceTypeVK::kCPU && vendor_ == VendorVK::kGoogle &&
      driver_name_.find("SwiftShader") != std::string::npos) {
    return true;
  }
#endif  // FML_OS_ANDROID
  return false;
}

bool DriverInfoVK::IsKnownBadDriver() const {
  if (adreno_gpu_.has_value()) {
    AdrenoGPU adreno = adreno_gpu_.value();
    // See:
    // https://github.com/flutter/flutter/issues/154103
    //
    // Reports "VK_INCOMPLETE" when compiling certain entity shader with
    // vkCreateGraphicsPipelines, which is not a valid return status.
    // See https://github.com/flutter/flutter/issues/155185 .
    //
    // https://github.com/flutter/flutter/issues/155185
    // Unknown crashes but device is not easily acquirable.
    if (adreno <= AdrenoGPU::kAdreno630) {
      return true;
    }
  }
  // Disable Maleoon series GPUs, see:
  // https://github.com/flutter/flutter/issues/156623
  if (vendor_ == VendorVK::kHuawei) {
    return true;
  }
  return false;
}

}  // namespace impeller
