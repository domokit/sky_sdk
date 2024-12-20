// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_RESOURCE_MANAGER_VK_H_
#define FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_RESOURCE_MANAGER_VK_H_

#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace impeller {

//------------------------------------------------------------------------------
/// @brief      A resource that may be reclaimed by a |ResourceManagerVK|.
///
/// To create a resource, use `UniqueResourceVKT` to create a unique handle:
///
///     auto resource = UniqueResourceVKT<SomeResource>(resource_manager);
///
/// @see        |ResourceManagerVK::Reclaim|.
class ResourceVK {
 public:
  virtual ~ResourceVK() = default;
};

//------------------------------------------------------------------------------
/// @brief      A resource manager controls how resources are allocated and
///             reclaimed.
///
///             Reclaimed resources are collected in a batch on a separate
///             thread. In the future, the resource manager may allow resource
///             pooling/reuse, delaying reclamation past frame workloads, etc...
///
class ResourceManagerVK final
    : public std::enable_shared_from_this<ResourceManagerVK> {
 public:
  //----------------------------------------------------------------------------
  /// @brief      Creates a shared resource manager (a dedicated thread).
  ///
  /// Upon creation, a thread is spawned which will collect resources as they
  /// are reclaimed (passed to `Reclaim`). The thread will exit when the
  /// resource manager is destroyed.
  ///
  /// @note       Only one |ResourceManagerVK| should be created per Vulkan
  ///             context, but that contract is not enforced by this method.
  ///
  /// @return     A resource manager if one could be created.
  ///
  static std::shared_ptr<ResourceManagerVK> Create();

  //----------------------------------------------------------------------------
  /// @brief      Mark a resource as being reclaimable.
  ///
  /// The resource will be reset at some point in the future.
  ///
  /// @param[in]  resource  The resource to reclaim.
  ///
  /// @note       Despite being a public API, this method cannot be invoked
  ///             directly. Instead, use `UniqueResourceVKT` to create a unique
  ///             handle to a resource, which will call this method.
  void Reclaim(std::unique_ptr<ResourceVK> resource);

  //----------------------------------------------------------------------------
  /// @brief      Destroys the resource manager.
  ///
  /// The resource manager will stop collecting resources and will be destroyed
  /// when all references to it are dropped.
  ~ResourceManagerVK();

 private:
  using Reclaimables = std::vector<std::unique_ptr<ResourceVK>>;

  ResourceManagerVK();
  std::mutex reclaimables_mutex_;
  std::condition_variable reclaimables_cv_;
  Reclaimables reclaimables_;
  bool should_exit_ = false;
  // This should be initialized last since it references the other instance
  // variables.
  std::thread waiter_;

  //----------------------------------------------------------------------------
  /// @brief      Starts the resource manager thread.
  ///
  /// This method is called implicitly by `Create`.
  void Start();

  //----------------------------------------------------------------------------
  /// @brief      Terminates the resource manager thread.
  ///
  /// Any resources given to the resource manager post termination will be
  /// collected when the resource manager is collected.
  void Terminate();

  ResourceManagerVK(const ResourceManagerVK&) = delete;

  ResourceManagerVK& operator=(const ResourceManagerVK&) = delete;
};

//------------------------------------------------------------------------------
/// @brief      An internal type that is used to move a resource reference.
///
///             Do not use directly, use `UniqueResourceVKT` instead.
///
/// @tparam     ResourceType_  The type of the resource.
///
/// @see        |UniqueResourceVKT|.
template <class ResourceType_>
class ResourceVKT : public ResourceVK {
 public:
  using ResourceType = ResourceType_;

  /// @brief      Construct a resource from a move-constructible resource.
  ///
  /// @param[in]  resource  The resource to move.
  explicit ResourceVKT(ResourceType&& resource)
      : resource_(std::move(resource)) {}

  /// @brief      Returns a pointer to the resource.
  const ResourceType* Get() const { return &resource_; }

 private:
  // Prevents subclassing, use `UniqueResourceVKT`.
  ResourceVKT() = default;

  ResourceType resource_;

  ResourceVKT(const ResourceVKT&) = delete;

  ResourceVKT& operator=(const ResourceVKT&) = delete;
};

//------------------------------------------------------------------------------
/// @brief      A unique handle to a resource which will be reclaimed by the
///             specified resource manager.
///
/// @tparam     ResourceType_  A move-constructible resource type.
///
template <class ResourceType_>
class UniqueResourceVKT final {
 public:
  using ResourceType = ResourceType_;

  /// @brief      Construct a unique resource handle belonging to a manager.
  ///
  /// Initially the handle is empty, and can be populated by calling `Swap`.
  ///
  /// @param[in]  resource_manager  The resource manager.
  explicit UniqueResourceVKT(std::weak_ptr<ResourceManagerVK> resource_manager)
      : resource_manager_(std::move(resource_manager)) {}

  /// @brief      Construct a unique resource handle belonging to a manager.
  ///
  /// Initially the handle is populated with the specified resource, but can
  /// be replaced (reclaiming the old resource) by calling `Swap`.
  ///
  /// @param[in]  resource_manager  The resource manager.
  /// @param[in]  resource          The resource to move.
  explicit UniqueResourceVKT(std::weak_ptr<ResourceManagerVK> resource_manager,
                             ResourceType&& resource)
      : resource_manager_(std::move(resource_manager)),
        resource_(
            std::make_unique<ResourceVKT<ResourceType>>(std::move(resource))) {}

  ~UniqueResourceVKT() { Reset(); }

  /// @brief      Returns a pointer to the resource.
  const ResourceType* operator->() const {
    // If this would segfault, fail with a nicer error message.
    FML_CHECK(resource_) << "UniqueResourceVKT was reclaimed.";

    return resource_.get()->Get();
  }

  /// @brief      Reclaims the existing resource, if any, and replaces it.
  ///
  /// @param[in]  other   The (new) resource to move.
  void Swap(ResourceType&& other) {
    Reset();
    resource_ = std::make_unique<ResourceVKT<ResourceType>>(std::move(other));
  }

  /// @brief      Reclaims the existing resource, if any.
  void Reset() {
    if (!resource_) {
      return;
    }
    // If there is a manager, ask it to reclaim the resource. If there isn't a
    // manager (because the manager has been destroyed), just drop it on the
    // floor here.
    if (auto manager = resource_manager_.lock()) {
      manager->Reclaim(std::move(resource_));
    }
    resource_.reset();
  }

 private:
  std::weak_ptr<ResourceManagerVK> resource_manager_;
  std::unique_ptr<ResourceVKT<ResourceType>> resource_;

  UniqueResourceVKT(const UniqueResourceVKT&) = delete;

  UniqueResourceVKT& operator=(const UniqueResourceVKT&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_RESOURCE_MANAGER_VK_H_
