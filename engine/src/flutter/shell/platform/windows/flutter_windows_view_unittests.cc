// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/windows/flutter_windows_view.h"

#include <UIAutomation.h>
#include <comdef.h>
#include <comutil.h>
#include <oleacc.h>

#include <future>
#include <vector>

#include "flutter/fml/synchronization/waitable_event.h"
#include "flutter/shell/platform/common/json_message_codec.h"
#include "flutter/shell/platform/embedder/test_utils/proc_table_replacement.h"
#include "flutter/shell/platform/windows/flutter_window.h"
#include "flutter/shell/platform/windows/flutter_windows_engine.h"
#include "flutter/shell/platform/windows/flutter_windows_texture_registrar.h"
#include "flutter/shell/platform/windows/flutter_windows_view_controller.h"
#include "flutter/shell/platform/windows/testing/egl/mock_context.h"
#include "flutter/shell/platform/windows/testing/egl/mock_manager.h"
#include "flutter/shell/platform/windows/testing/egl/mock_window_surface.h"
#include "flutter/shell/platform/windows/testing/engine_modifier.h"
#include "flutter/shell/platform/windows/testing/mock_window_binding_handler.h"
#include "flutter/shell/platform/windows/testing/mock_windows_proc_table.h"
#include "flutter/shell/platform/windows/testing/test_keyboard.h"
#include "flutter/shell/platform/windows/testing/view_modifier.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace flutter {
namespace testing {

using ::testing::_;
using ::testing::InSequence;
using ::testing::NiceMock;
using ::testing::Return;

constexpr uint64_t kScanCodeKeyA = 0x1e;
constexpr uint64_t kVirtualKeyA = 0x41;

namespace {

// A struct to use as a FlutterPlatformMessageResponseHandle so it can keep the
// callbacks and user data passed to the engine's
// PlatformMessageCreateResponseHandle for use in the SendPlatformMessage
// overridden function.
struct TestResponseHandle {
  FlutterDesktopBinaryReply callback;
  void* user_data;
};

static bool test_response = false;

constexpr uint64_t kKeyEventFromChannel = 0x11;
constexpr uint64_t kKeyEventFromEmbedder = 0x22;
static std::vector<int> key_event_logs;

std::unique_ptr<std::vector<uint8_t>> keyHandlingResponse(bool handled) {
  rapidjson::Document document;
  auto& allocator = document.GetAllocator();
  document.SetObject();
  document.AddMember("handled", test_response, allocator);
  return flutter::JsonMessageCodec::GetInstance().EncodeMessage(document);
}

// Returns a Flutter project with the required path values to create
// a test engine.
FlutterProjectBundle GetTestProject() {
  FlutterDesktopEngineProperties properties = {};
  properties.assets_path = L"C:\\foo\\flutter_assets";
  properties.icu_data_path = L"C:\\foo\\icudtl.dat";
  properties.aot_library_path = L"C:\\foo\\aot.so";

  return FlutterProjectBundle{properties};
}

// Returns an engine instance configured with test project path values, and
// overridden methods for sending platform messages, so that the engine can
// respond as if the framework were connected.
std::unique_ptr<FlutterWindowsEngine> GetTestEngine(
    std::shared_ptr<WindowsProcTable> windows_proc_table = nullptr) {
  auto engine = std::make_unique<FlutterWindowsEngine>(
      GetTestProject(), std::move(windows_proc_table));

  EngineModifier modifier(engine.get());
  modifier.SetEGLManager(nullptr);

  auto key_response_controller = std::make_shared<MockKeyResponseController>();
  key_response_controller->SetChannelResponse(
      [](MockKeyResponseController::ResponseCallback callback) {
        key_event_logs.push_back(kKeyEventFromChannel);
        callback(test_response);
      });
  key_response_controller->SetEmbedderResponse(
      [](const FlutterKeyEvent* event,
         MockKeyResponseController::ResponseCallback callback) {
        key_event_logs.push_back(kKeyEventFromEmbedder);
        callback(test_response);
      });
  modifier.embedder_api().NotifyDisplayUpdate =
      MOCK_ENGINE_PROC(NotifyDisplayUpdate,
                       ([engine_instance = engine.get()](
                            FLUTTER_API_SYMBOL(FlutterEngine) raw_engine,
                            const FlutterEngineDisplaysUpdateType update_type,
                            const FlutterEngineDisplay* embedder_displays,
                            size_t display_count) { return kSuccess; }));

  MockEmbedderApiForKeyboard(modifier, key_response_controller);

  engine->Run();
  return engine;
}

class MockFlutterWindowsEngine : public FlutterWindowsEngine {
 public:
  explicit MockFlutterWindowsEngine(
      std::shared_ptr<WindowsProcTable> windows_proc_table = nullptr)
      : FlutterWindowsEngine(GetTestProject(), std::move(windows_proc_table)) {}

  MOCK_METHOD(bool, running, (), (const));
  MOCK_METHOD(bool, Stop, (), ());
  MOCK_METHOD(void, RemoveView, (FlutterViewId view_id), ());
  MOCK_METHOD(bool, PostRasterThreadTask, (fml::closure), (const));

 private:
  FML_DISALLOW_COPY_AND_ASSIGN(MockFlutterWindowsEngine);
};

}  // namespace

// Ensure that submenu buttons have their expanded/collapsed status set
// apropriately.
TEST(FlutterWindowsViewTest, SubMenuExpandedState) {
  std::unique_ptr<FlutterWindowsEngine> engine = GetTestEngine();
  EngineModifier modifier(engine.get());
  modifier.embedder_api().UpdateSemanticsEnabled =
      [](FLUTTER_API_SYMBOL(FlutterEngine) engine, bool enabled) {
        return kSuccess;
      };

  auto window_binding_handler =
      std::make_unique<NiceMock<MockWindowBindingHandler>>();
  std::unique_ptr<FlutterWindowsView> view =
      engine->CreateView(std::move(window_binding_handler));

  // Enable semantics to instantiate accessibility bridge.
  view->OnUpdateSemanticsEnabled(true);

  auto bridge = view->accessibility_bridge().lock();
  ASSERT_TRUE(bridge);

  FlutterSemanticsNode2 root{sizeof(FlutterSemanticsNode2), 0};
  root.id = 0;
  root.label = "root";
  root.hint = "";
  root.value = "";
  root.increased_value = "";
  root.decreased_value = "";
  root.child_count = 0;
  root.custom_accessibility_actions_count = 0;
  root.flags = static_cast<FlutterSemanticsFlag>(
      FlutterSemanticsFlag::kFlutterSemanticsFlagHasExpandedState |
      FlutterSemanticsFlag::kFlutterSemanticsFlagIsExpanded);
  bridge->AddFlutterSemanticsNodeUpdate(root);

  bridge->CommitUpdates();

  {
    auto root_node = bridge->GetFlutterPlatformNodeDelegateFromID(0).lock();
    EXPECT_TRUE(root_node->GetData().HasState(ax::mojom::State::kExpanded));

    // Get the IAccessible for the root node.
    IAccessible* native_view = root_node->GetNativeViewAccessible();
    ASSERT_TRUE(native_view != nullptr);

    // Look up against the node itself (not one of its children).
    VARIANT varchild = {};
    varchild.vt = VT_I4;

    // Verify the submenu is expanded.
    varchild.lVal = CHILDID_SELF;
    VARIANT native_state = {};
    ASSERT_TRUE(SUCCEEDED(native_view->get_accState(varchild, &native_state)));
    EXPECT_TRUE(native_state.lVal & STATE_SYSTEM_EXPANDED);

    // Perform similar tests for UIA value;
    IRawElementProviderSimple* uia_node;
    native_view->QueryInterface(IID_PPV_ARGS(&uia_node));
    ASSERT_TRUE(SUCCEEDED(uia_node->GetPropertyValue(
        UIA_ExpandCollapseExpandCollapseStatePropertyId, &native_state)));
    EXPECT_EQ(native_state.lVal, ExpandCollapseState_Expanded);

    ASSERT_TRUE(SUCCEEDED(uia_node->GetPropertyValue(
        UIA_AriaPropertiesPropertyId, &native_state)));
    EXPECT_NE(std::wcsstr(native_state.bstrVal, L"expanded=true"), nullptr);
  }

  // Test collapsed too.
  root.flags = static_cast<FlutterSemanticsFlag>(
      FlutterSemanticsFlag::kFlutterSemanticsFlagHasExpandedState);
  bridge->AddFlutterSemanticsNodeUpdate(root);
  bridge->CommitUpdates();

  {
    auto root_node = bridge->GetFlutterPlatformNodeDelegateFromID(0).lock();
    EXPECT_TRUE(root_node->GetData().HasState(ax::mojom::State::kCollapsed));

    // Get the IAccessible for the root node.
    IAccessible* native_view = root_node->GetNativeViewAccessible();
    ASSERT_TRUE(native_view != nullptr);

    // Look up against the node itself (not one of its children).
    VARIANT varchild = {};
    varchild.vt = VT_I4;

    // Verify the submenu is collapsed.
    varchild.lVal = CHILDID_SELF;
    VARIANT native_state = {};
    ASSERT_TRUE(SUCCEEDED(native_view->get_accState(varchild, &native_state)));
    EXPECT_TRUE(native_state.lVal & STATE_SYSTEM_COLLAPSED);

    // Perform similar tests for UIA value;
    IRawElementProviderSimple* uia_node;
    native_view->QueryInterface(IID_PPV_ARGS(&uia_node));
    ASSERT_TRUE(SUCCEEDED(uia_node->GetPropertyValue(
        UIA_ExpandCollapseExpandCollapseStatePropertyId, &native_state)));
    EXPECT_EQ(native_state.lVal, ExpandCollapseState_Collapsed);

    ASSERT_TRUE(SUCCEEDED(uia_node->GetPropertyValue(
        UIA_AriaPropertiesPropertyId, &native_state)));
    EXPECT_NE(std::wcsstr(native_state.bstrVal, L"expanded=false"), nullptr);
  }
}

// The view's surface must be destroyed after the engine is shutdown.
// See: https://github.com/flutter/flutter/issues/124463
TEST(FlutterWindowsViewTest, Shutdown) {
  auto engine = std::make_unique<MockFlutterWindowsEngine>();
  auto window_binding_handler =
      std::make_unique<NiceMock<MockWindowBindingHandler>>();
  auto egl_manager = std::make_unique<egl::MockManager>();
  auto surface = std::make_unique<egl::MockWindowSurface>();
  egl::MockContext render_context;

  auto engine_ptr = engine.get();
  auto surface_ptr = surface.get();
  auto egl_manager_ptr = egl_manager.get();

  EngineModifier modifier{engine.get()};
  modifier.SetEGLManager(std::move(egl_manager));

  InSequence s;
  std::unique_ptr<FlutterWindowsView> view;

  // Mock render surface initialization.
  {
    EXPECT_CALL(*egl_manager_ptr, CreateWindowSurface)
        .WillOnce(Return(std::move(surface)));
    EXPECT_CALL(*engine_ptr, running).WillOnce(Return(false));
    EXPECT_CALL(*surface_ptr, IsValid).WillOnce(Return(true));
    EXPECT_CALL(*surface_ptr, MakeCurrent).WillOnce(Return(true));
    EXPECT_CALL(*surface_ptr, SetVSyncEnabled).WillOnce(Return(true));
    EXPECT_CALL(*egl_manager_ptr, render_context)
        .WillOnce(Return(&render_context));
    EXPECT_CALL(render_context, ClearCurrent).WillOnce(Return(true));

    view = engine->CreateView(std::move(window_binding_handler));
  }

  // The view must be removed before the surface can be destroyed.
  {
    auto view_id = view->view_id();
    FlutterWindowsViewController controller{std::move(engine), std::move(view)};

    EXPECT_CALL(*engine_ptr, running).WillOnce(Return(true));
    EXPECT_CALL(*engine_ptr, RemoveView(view_id)).Times(1);
    EXPECT_CALL(*engine_ptr, running).WillOnce(Return(true));
    EXPECT_CALL(*engine_ptr, PostRasterThreadTask)
        .WillOnce([](fml::closure callback) {
          callback();
          return true;
        });
    EXPECT_CALL(*surface_ptr, Destroy).Times(1);
  }
}

TEST(FlutterWindowsViewTest, KeySequence) {
  std::unique_ptr<FlutterWindowsEngine> engine = GetTestEngine();

  test_response = false;

  std::unique_ptr<FlutterWindowsView> view = engine->CreateView(
      std::make_unique<NiceMock<MockWindowBindingHandler>>());

  view->OnKey(kVirtualKeyA, kScanCodeKeyA, WM_KEYDOWN, 'a', false, false,
              [](bool handled) {});

  EXPECT_EQ(key_event_logs.size(), 2);
  EXPECT_EQ(key_event_logs[0], kKeyEventFromEmbedder);
  EXPECT_EQ(key_event_logs[1], kKeyEventFromChannel);

  key_event_logs.clear();
}

TEST(FlutterWindowsViewTest, EnableSemantics) {
  std::unique_ptr<FlutterWindowsEngine> engine = GetTestEngine();
  EngineModifier modifier(engine.get());

  bool semantics_enabled = false;
  modifier.embedder_api().UpdateSemanticsEnabled = MOCK_ENGINE_PROC(
      UpdateSemanticsEnabled,
      [&semantics_enabled](FLUTTER_API_SYMBOL(FlutterEngine) engine,
                           bool enabled) {
        semantics_enabled = enabled;
        return kSuccess;
      });

  auto window_binding_handler =
      std::make_unique<NiceMock<MockWindowBindingHandler>>();
  std::unique_ptr<FlutterWindowsView> view =
      engine->CreateView(std::move(window_binding_handler));

  view->OnUpdateSemanticsEnabled(true);
  EXPECT_TRUE(semantics_enabled);
}

TEST(FlutterWindowsViewTest, AddSemanticsNodeUpdate) {
  std::unique_ptr<FlutterWindowsEngine> engine = GetTestEngine();
  EngineModifier modifier(engine.get());
  modifier.embedder_api().UpdateSemanticsEnabled =
      [](FLUTTER_API_SYMBOL(FlutterEngine) engine, bool enabled) {
        return kSuccess;
      };

  auto window_binding_handler =
      std::make_unique<NiceMock<MockWindowBindingHandler>>();
  std::unique_ptr<FlutterWindowsView> view =
      engine->CreateView(std::move(window_binding_handler));

  // Enable semantics to instantiate accessibility bridge.
  view->OnUpdateSemanticsEnabled(true);

  auto bridge = view->accessibility_bridge().lock();
  ASSERT_TRUE(bridge);

  // Add root node.
  FlutterSemanticsNode2 node{sizeof(FlutterSemanticsNode2), 0};
  node.label = "name";
  node.value = "value";
  node.platform_view_id = -1;
  bridge->AddFlutterSemanticsNodeUpdate(node);
  bridge->CommitUpdates();

  // Look up the root windows node delegate.
  auto node_delegate = bridge->GetFlutterPlatformNodeDelegateFromID(0).lock();
  ASSERT_TRUE(node_delegate);
  EXPECT_EQ(node_delegate->GetChildCount(), 0);

  // Get the native IAccessible object.
  IAccessible* native_view = node_delegate->GetNativeViewAccessible();
  ASSERT_TRUE(native_view != nullptr);

  // Property lookups will be made against this node itself.
  VARIANT varchild{};
  varchild.vt = VT_I4;
  varchild.lVal = CHILDID_SELF;

  // Verify node name matches our label.
  BSTR bname = nullptr;
  ASSERT_EQ(native_view->get_accName(varchild, &bname), S_OK);
  std::string name(_com_util::ConvertBSTRToString(bname));
  EXPECT_EQ(name, "name");

  // Verify node value matches.
  BSTR bvalue = nullptr;
  ASSERT_EQ(native_view->get_accValue(varchild, &bvalue), S_OK);
  std::string value(_com_util::ConvertBSTRToString(bvalue));
  EXPECT_EQ(value, "value");

  // Verify node type is static text.
  VARIANT varrole{};
  varrole.vt = VT_I4;
  ASSERT_EQ(native_view->get_accRole(varchild, &varrole), S_OK);
  EXPECT_EQ(varrole.lVal, ROLE_SYSTEM_STATICTEXT);

  // Get the IRawElementProviderFragment object.
  IRawElementProviderSimple* uia_view;
  native_view->QueryInterface(IID_PPV_ARGS(&uia_view));
  ASSERT_TRUE(uia_view != nullptr);

  // Verify name property matches our label.
  VARIANT varname{};
  ASSERT_EQ(uia_view->GetPropertyValue(UIA_NamePropertyId, &varname), S_OK);
  EXPECT_EQ(varname.vt, VT_BSTR);
  name = _com_util::ConvertBSTRToString(varname.bstrVal);
  EXPECT_EQ(name, "name");

  // Verify value property matches our label.
  VARIANT varvalue{};
  ASSERT_EQ(uia_view->GetPropertyValue(UIA_ValueValuePropertyId, &varvalue),
            S_OK);
  EXPECT_EQ(varvalue.vt, VT_BSTR);
  value = _com_util::ConvertBSTRToString(varvalue.bstrVal);
  EXPECT_EQ(value, "value");

  // Verify node control type is text.
  varrole = {};
  ASSERT_EQ(uia_view->GetPropertyValue(UIA_ControlTypePropertyId, &varrole),
            S_OK);
  EXPECT_EQ(varrole.vt, VT_I4);
  EXPECT_EQ(varrole.lVal, UIA_TextControlTypeId);
}

// Verify the native IAccessible COM object tree is an accurate reflection of
// the platform-agnostic tree. Verify both a root node with children as well as
// a non-root node with children, since the AX tree includes special handling
// for the root.
//
//        node0
//        /   \
//    node1    node2
//               |
//             node3
//
// node0 and node2 are grouping nodes. node1 and node2 are static text nodes.
TEST(FlutterWindowsViewTest, AddSemanticsNodeUpdateWithChildren) {
  std::unique_ptr<FlutterWindowsEngine> engine = GetTestEngine();
  EngineModifier modifier(engine.get());
  modifier.embedder_api().UpdateSemanticsEnabled =
      [](FLUTTER_API_SYMBOL(FlutterEngine) engine, bool enabled) {
        return kSuccess;
      };

  std::unique_ptr<FlutterWindowsView> view = engine->CreateView(
      std::make_unique<NiceMock<MockWindowBindingHandler>>());

  // Enable semantics to instantiate accessibility bridge.
  view->OnUpdateSemanticsEnabled(true);

  auto bridge = view->accessibility_bridge().lock();
  ASSERT_TRUE(bridge);

  // Add root node.
  FlutterSemanticsNode2 node0{sizeof(FlutterSemanticsNode2), 0};
  std::vector<int32_t> node0_children{1, 2};
  node0.child_count = node0_children.size();
  node0.children_in_traversal_order = node0_children.data();
  node0.children_in_hit_test_order = node0_children.data();

  FlutterSemanticsNode2 node1{sizeof(FlutterSemanticsNode2), 1};
  node1.label = "prefecture";
  node1.value = "Kyoto";
  FlutterSemanticsNode2 node2{sizeof(FlutterSemanticsNode2), 2};
  std::vector<int32_t> node2_children{3};
  node2.child_count = node2_children.size();
  node2.children_in_traversal_order = node2_children.data();
  node2.children_in_hit_test_order = node2_children.data();
  FlutterSemanticsNode2 node3{sizeof(FlutterSemanticsNode2), 3};
  node3.label = "city";
  node3.value = "Uji";

  bridge->AddFlutterSemanticsNodeUpdate(node0);
  bridge->AddFlutterSemanticsNodeUpdate(node1);
  bridge->AddFlutterSemanticsNodeUpdate(node2);
  bridge->AddFlutterSemanticsNodeUpdate(node3);
  bridge->CommitUpdates();

  // Look up the root windows node delegate.
  auto node_delegate = bridge->GetFlutterPlatformNodeDelegateFromID(0).lock();
  ASSERT_TRUE(node_delegate);
  EXPECT_EQ(node_delegate->GetChildCount(), 2);

  // Get the native IAccessible object.
  IAccessible* node0_accessible = node_delegate->GetNativeViewAccessible();
  ASSERT_TRUE(node0_accessible != nullptr);

  // Property lookups will be made against this node itself.
  VARIANT varchild{};
  varchild.vt = VT_I4;
  varchild.lVal = CHILDID_SELF;

  // Verify node type is a group.
  VARIANT varrole{};
  varrole.vt = VT_I4;
  ASSERT_EQ(node0_accessible->get_accRole(varchild, &varrole), S_OK);
  EXPECT_EQ(varrole.lVal, ROLE_SYSTEM_GROUPING);

  // Verify child count.
  long node0_child_count = 0;
  ASSERT_EQ(node0_accessible->get_accChildCount(&node0_child_count), S_OK);
  EXPECT_EQ(node0_child_count, 2);

  {
    // Look up first child of node0 (node1), a static text node.
    varchild.lVal = 1;
    IDispatch* node1_dispatch = nullptr;
    ASSERT_EQ(node0_accessible->get_accChild(varchild, &node1_dispatch), S_OK);
    ASSERT_TRUE(node1_dispatch != nullptr);
    IAccessible* node1_accessible = nullptr;
    ASSERT_EQ(node1_dispatch->QueryInterface(
                  IID_IAccessible, reinterpret_cast<void**>(&node1_accessible)),
              S_OK);
    ASSERT_TRUE(node1_accessible != nullptr);

    // Verify node name matches our label.
    varchild.lVal = CHILDID_SELF;
    BSTR bname = nullptr;
    ASSERT_EQ(node1_accessible->get_accName(varchild, &bname), S_OK);
    std::string name(_com_util::ConvertBSTRToString(bname));
    EXPECT_EQ(name, "prefecture");

    // Verify node value matches.
    BSTR bvalue = nullptr;
    ASSERT_EQ(node1_accessible->get_accValue(varchild, &bvalue), S_OK);
    std::string value(_com_util::ConvertBSTRToString(bvalue));
    EXPECT_EQ(value, "Kyoto");

    // Verify node type is static text.
    VARIANT varrole{};
    varrole.vt = VT_I4;
    ASSERT_EQ(node1_accessible->get_accRole(varchild, &varrole), S_OK);
    EXPECT_EQ(varrole.lVal, ROLE_SYSTEM_STATICTEXT);

    // Verify the parent node is the root.
    IDispatch* parent_dispatch;
    node1_accessible->get_accParent(&parent_dispatch);
    IAccessible* parent_accessible;
    ASSERT_EQ(
        parent_dispatch->QueryInterface(
            IID_IAccessible, reinterpret_cast<void**>(&parent_accessible)),
        S_OK);
    EXPECT_EQ(parent_accessible, node0_accessible);
  }

  // Look up second child of node0 (node2), a parent group for node3.
  varchild.lVal = 2;
  IDispatch* node2_dispatch = nullptr;
  ASSERT_EQ(node0_accessible->get_accChild(varchild, &node2_dispatch), S_OK);
  ASSERT_TRUE(node2_dispatch != nullptr);
  IAccessible* node2_accessible = nullptr;
  ASSERT_EQ(node2_dispatch->QueryInterface(
                IID_IAccessible, reinterpret_cast<void**>(&node2_accessible)),
            S_OK);
  ASSERT_TRUE(node2_accessible != nullptr);

  {
    // Verify child count.
    long node2_child_count = 0;
    ASSERT_EQ(node2_accessible->get_accChildCount(&node2_child_count), S_OK);
    EXPECT_EQ(node2_child_count, 1);

    // Verify node type is static text.
    varchild.lVal = CHILDID_SELF;
    VARIANT varrole{};
    varrole.vt = VT_I4;
    ASSERT_EQ(node2_accessible->get_accRole(varchild, &varrole), S_OK);
    EXPECT_EQ(varrole.lVal, ROLE_SYSTEM_GROUPING);

    // Verify the parent node is the root.
    IDispatch* parent_dispatch;
    node2_accessible->get_accParent(&parent_dispatch);
    IAccessible* parent_accessible;
    ASSERT_EQ(
        parent_dispatch->QueryInterface(
            IID_IAccessible, reinterpret_cast<void**>(&parent_accessible)),
        S_OK);
    EXPECT_EQ(parent_accessible, node0_accessible);
  }

  {
    // Look up only child of node2 (node3), a static text node.
    varchild.lVal = 1;
    IDispatch* node3_dispatch = nullptr;
    ASSERT_EQ(node2_accessible->get_accChild(varchild, &node3_dispatch), S_OK);
    ASSERT_TRUE(node3_dispatch != nullptr);
    IAccessible* node3_accessible = nullptr;
    ASSERT_EQ(node3_dispatch->QueryInterface(
                  IID_IAccessible, reinterpret_cast<void**>(&node3_accessible)),
              S_OK);
    ASSERT_TRUE(node3_accessible != nullptr);

    // Verify node name matches our label.
    varchild.lVal = CHILDID_SELF;
    BSTR bname = nullptr;
    ASSERT_EQ(node3_accessible->get_accName(varchild, &bname), S_OK);
    std::string name(_com_util::ConvertBSTRToString(bname));
    EXPECT_EQ(name, "city");

    // Verify node value matches.
    BSTR bvalue = nullptr;
    ASSERT_EQ(node3_accessible->get_accValue(varchild, &bvalue), S_OK);
    std::string value(_com_util::ConvertBSTRToString(bvalue));
    EXPECT_EQ(value, "Uji");

    // Verify node type is static text.
    VARIANT varrole{};
    varrole.vt = VT_I4;
    ASSERT_EQ(node3_accessible->get_accRole(varchild, &varrole), S_OK);
    EXPECT_EQ(varrole.lVal, ROLE_SYSTEM_STATICTEXT);

    // Verify the parent node is node2.
    IDispatch* parent_dispatch;
    node3_accessible->get_accParent(&parent_dispatch);
    IAccessible* parent_accessible;
    ASSERT_EQ(
        parent_dispatch->QueryInterface(
            IID_IAccessible, reinterpret_cast<void**>(&parent_accessible)),
        S_OK);
    EXPECT_EQ(parent_accessible, node2_accessible);
  }
}

// Flutter used to assume that the accessibility root had ID 0.
// In a multi-view world, each view has its own accessibility root
// with a globally unique node ID.
//
//        node1
//          |
//        node2
//
// node1 is a grouping node, node0 is a static text node.
TEST(FlutterWindowsViewTest, NonZeroSemanticsRoot) {
  std::unique_ptr<FlutterWindowsEngine> engine = GetTestEngine();
  EngineModifier modifier(engine.get());
  modifier.embedder_api().UpdateSemanticsEnabled =
      [](FLUTTER_API_SYMBOL(FlutterEngine) engine, bool enabled) {
        return kSuccess;
      };

  std::unique_ptr<FlutterWindowsView> view = engine->CreateView(
      std::make_unique<NiceMock<MockWindowBindingHandler>>());

  // Enable semantics to instantiate accessibility bridge.
  view->OnUpdateSemanticsEnabled(true);

  auto bridge = view->accessibility_bridge().lock();
  ASSERT_TRUE(bridge);

  // Add root node.
  FlutterSemanticsNode2 node1{sizeof(FlutterSemanticsNode2), 1};
  std::vector<int32_t> node1_children{2};
  node1.child_count = node1_children.size();
  node1.children_in_traversal_order = node1_children.data();
  node1.children_in_hit_test_order = node1_children.data();

  FlutterSemanticsNode2 node2{sizeof(FlutterSemanticsNode2), 2};
  node2.label = "prefecture";
  node2.value = "Kyoto";

  bridge->AddFlutterSemanticsNodeUpdate(node1);
  bridge->AddFlutterSemanticsNodeUpdate(node2);
  bridge->CommitUpdates();

  // Look up the root windows node delegate.
  auto root_delegate = bridge->GetFlutterPlatformNodeDelegateFromID(1).lock();
  ASSERT_TRUE(root_delegate);
  EXPECT_EQ(root_delegate->GetChildCount(), 1);

  // Look up the child node delegate
  auto child_delegate = bridge->GetFlutterPlatformNodeDelegateFromID(2).lock();
  ASSERT_TRUE(child_delegate);
  EXPECT_EQ(child_delegate->GetChildCount(), 0);

  // Ensure a node with ID 0 does not exist.
  auto fake_delegate = bridge->GetFlutterPlatformNodeDelegateFromID(0).lock();
  ASSERT_FALSE(fake_delegate);

  // Get the root's native IAccessible object.
  IAccessible* node1_accessible = root_delegate->GetNativeViewAccessible();
  ASSERT_TRUE(node1_accessible != nullptr);

  // Property lookups will be made against this node itself.
  VARIANT varchild{};
  varchild.vt = VT_I4;
  varchild.lVal = CHILDID_SELF;

  // Verify node type is a group.
  VARIANT varrole{};
  varrole.vt = VT_I4;
  ASSERT_EQ(node1_accessible->get_accRole(varchild, &varrole), S_OK);
  EXPECT_EQ(varrole.lVal, ROLE_SYSTEM_GROUPING);

  // Verify child count.
  long node1_child_count = 0;
  ASSERT_EQ(node1_accessible->get_accChildCount(&node1_child_count), S_OK);
  EXPECT_EQ(node1_child_count, 1);

  {
    // Look up first child of node1 (node0), a static text node.
    varchild.lVal = 1;
    IDispatch* node2_dispatch = nullptr;
    ASSERT_EQ(node1_accessible->get_accChild(varchild, &node2_dispatch), S_OK);
    ASSERT_TRUE(node2_dispatch != nullptr);
    IAccessible* node2_accessible = nullptr;
    ASSERT_EQ(node2_dispatch->QueryInterface(
                  IID_IAccessible, reinterpret_cast<void**>(&node2_accessible)),
              S_OK);
    ASSERT_TRUE(node2_accessible != nullptr);

    // Verify node name matches our label.
    varchild.lVal = CHILDID_SELF;
    BSTR bname = nullptr;
    ASSERT_EQ(node2_accessible->get_accName(varchild, &bname), S_OK);
    std::string name(_com_util::ConvertBSTRToString(bname));
    EXPECT_EQ(name, "prefecture");

    // Verify node value matches.
    BSTR bvalue = nullptr;
    ASSERT_EQ(node2_accessible->get_accValue(varchild, &bvalue), S_OK);
    std::string value(_com_util::ConvertBSTRToString(bvalue));
    EXPECT_EQ(value, "Kyoto");

    // Verify node type is static text.
    VARIANT varrole{};
    varrole.vt = VT_I4;
    ASSERT_EQ(node2_accessible->get_accRole(varchild, &varrole), S_OK);
    EXPECT_EQ(varrole.lVal, ROLE_SYSTEM_STATICTEXT);

    // Verify the parent node is the root.
    IDispatch* parent_dispatch;
    node2_accessible->get_accParent(&parent_dispatch);
    IAccessible* parent_accessible;
    ASSERT_EQ(
        parent_dispatch->QueryInterface(
            IID_IAccessible, reinterpret_cast<void**>(&parent_accessible)),
        S_OK);
    EXPECT_EQ(parent_accessible, node1_accessible);
  }
}

// Verify the native IAccessible accHitTest method returns the correct
// IAccessible COM object for the given coordinates.
//
//                         +-----------+
//                         |     |     |
//        node0            |     |  B  |
//        /   \            |  A  |-----|
//    node1    node2       |     |  C  |
//               |         |     |     |
//             node3       +-----------+
//
// node0 and node2 are grouping nodes. node1 and node2 are static text nodes.
//
// node0 is located at 0,0 with size 500x500. It spans areas A, B, and C.
// node1 is located at 0,0 with size 250x500. It spans area A.
// node2 is located at 250,0 with size 250x500. It spans areas B and C.
// node3 is located at 250,250 with size 250x250. It spans area C.
TEST(FlutterWindowsViewTest, AccessibilityHitTesting) {
  constexpr FlutterTransformation kIdentityTransform = {1, 0, 0,  //
                                                        0, 1, 0,  //
                                                        0, 0, 1};

  std::unique_ptr<FlutterWindowsEngine> engine = GetTestEngine();
  EngineModifier modifier(engine.get());
  modifier.embedder_api().UpdateSemanticsEnabled =
      [](FLUTTER_API_SYMBOL(FlutterEngine) engine, bool enabled) {
        return kSuccess;
      };

  std::unique_ptr<FlutterWindowsView> view = engine->CreateView(
      std::make_unique<NiceMock<MockWindowBindingHandler>>());

  // Enable semantics to instantiate accessibility bridge.
  view->OnUpdateSemanticsEnabled(true);

  auto bridge = view->accessibility_bridge().lock();
  ASSERT_TRUE(bridge);

  // Add root node at origin. Size 500x500.
  FlutterSemanticsNode2 node0{sizeof(FlutterSemanticsNode2), 0};
  std::vector<int32_t> node0_children{1, 2};
  node0.rect = {0, 0, 500, 500};
  node0.transform = kIdentityTransform;
  node0.child_count = node0_children.size();
  node0.children_in_traversal_order = node0_children.data();
  node0.children_in_hit_test_order = node0_children.data();

  // Add node 1 located at 0,0 relative to node 0. Size 250x500.
  FlutterSemanticsNode2 node1{sizeof(FlutterSemanticsNode2), 1};
  node1.rect = {0, 0, 250, 500};
  node1.transform = kIdentityTransform;
  node1.label = "prefecture";
  node1.value = "Kyoto";

  // Add node 2 located at 250,0 relative to node 0. Size 250x500.
  FlutterSemanticsNode2 node2{sizeof(FlutterSemanticsNode2), 2};
  std::vector<int32_t> node2_children{3};
  node2.rect = {0, 0, 250, 500};
  node2.transform = {1, 0, 250, 0, 1, 0, 0, 0, 1};
  node2.child_count = node2_children.size();
  node2.children_in_traversal_order = node2_children.data();
  node2.children_in_hit_test_order = node2_children.data();

  // Add node 3 located at 0,250 relative to node 2. Size 250, 250.
  FlutterSemanticsNode2 node3{sizeof(FlutterSemanticsNode2), 3};
  node3.rect = {0, 0, 250, 250};
  node3.transform = {1, 0, 0, 0, 1, 250, 0, 0, 1};
  node3.label = "city";
  node3.value = "Uji";

  bridge->AddFlutterSemanticsNodeUpdate(node0);
  bridge->AddFlutterSemanticsNodeUpdate(node1);
  bridge->AddFlutterSemanticsNodeUpdate(node2);
  bridge->AddFlutterSemanticsNodeUpdate(node3);
  bridge->CommitUpdates();

  // Look up the root windows node delegate.
  auto node0_delegate = bridge->GetFlutterPlatformNodeDelegateFromID(0).lock();
  ASSERT_TRUE(node0_delegate);
  auto node1_delegate = bridge->GetFlutterPlatformNodeDelegateFromID(1).lock();
  ASSERT_TRUE(node1_delegate);
  auto node2_delegate = bridge->GetFlutterPlatformNodeDelegateFromID(2).lock();
  ASSERT_TRUE(node2_delegate);
  auto node3_delegate = bridge->GetFlutterPlatformNodeDelegateFromID(3).lock();
  ASSERT_TRUE(node3_delegate);

  // Get the native IAccessible root object.
  IAccessible* node0_accessible = node0_delegate->GetNativeViewAccessible();
  ASSERT_TRUE(node0_accessible != nullptr);

  // Perform a hit test that should hit node 1.
  VARIANT varchild{};
  ASSERT_TRUE(SUCCEEDED(node0_accessible->accHitTest(150, 150, &varchild)));
  EXPECT_EQ(varchild.vt, VT_DISPATCH);
  EXPECT_EQ(varchild.pdispVal, node1_delegate->GetNativeViewAccessible());

  // Perform a hit test that should hit node 2.
  varchild = {};
  ASSERT_TRUE(SUCCEEDED(node0_accessible->accHitTest(450, 150, &varchild)));
  EXPECT_EQ(varchild.vt, VT_DISPATCH);
  EXPECT_EQ(varchild.pdispVal, node2_delegate->GetNativeViewAccessible());

  // Perform a hit test that should hit node 3.
  varchild = {};
  ASSERT_TRUE(SUCCEEDED(node0_accessible->accHitTest(450, 450, &varchild)));
  EXPECT_EQ(varchild.vt, VT_DISPATCH);
  EXPECT_EQ(varchild.pdispVal, node3_delegate->GetNativeViewAccessible());
}

TEST(FlutterWindowsViewTest, WindowResizeTests) {
  auto windows_proc_table = std::make_shared<NiceMock<MockWindowsProcTable>>();
  std::unique_ptr<FlutterWindowsEngine> engine =
      GetTestEngine(windows_proc_table);

  EngineModifier engine_modifier{engine.get()};
  engine_modifier.embedder_api().PostRenderThreadTask = MOCK_ENGINE_PROC(
      PostRenderThreadTask,
      ([](auto engine, VoidCallback callback, void* user_data) {
        callback(user_data);
        return kSuccess;
      }));

  auto egl_manager = std::make_unique<egl::MockManager>();
  auto surface = std::make_unique<egl::MockWindowSurface>();
  auto resized_surface = std::make_unique<egl::MockWindowSurface>();
  egl::MockContext render_context;

  auto surface_ptr = surface.get();
  auto resized_surface_ptr = resized_surface.get();

  // Mock render surface creation
  EXPECT_CALL(*egl_manager, CreateWindowSurface)
      .WillOnce(Return(std::move(surface)));
  EXPECT_CALL(*surface_ptr, IsValid).WillRepeatedly(Return(true));
  EXPECT_CALL(*surface_ptr, MakeCurrent).WillOnce(Return(true));
  EXPECT_CALL(*surface_ptr, SetVSyncEnabled).WillOnce(Return(true));
  EXPECT_CALL(*egl_manager, render_context).WillOnce(Return(&render_context));
  EXPECT_CALL(render_context, ClearCurrent).WillOnce(Return(true));

  // Mock render surface resize
  EXPECT_CALL(*surface_ptr, Destroy).WillOnce(Return(true));
  EXPECT_CALL(*egl_manager.get(),
              CreateWindowSurface(_, /*width=*/500, /*height=*/500))
      .WillOnce(Return(std::move((resized_surface))));
  EXPECT_CALL(*resized_surface_ptr, MakeCurrent).WillOnce(Return(true));
  EXPECT_CALL(*resized_surface_ptr, SetVSyncEnabled).WillOnce(Return(true));
  EXPECT_CALL(*windows_proc_table.get(), DwmFlush).WillOnce(Return(S_OK));

  EXPECT_CALL(*resized_surface_ptr, Destroy).WillOnce(Return(true));

  engine_modifier.SetEGLManager(std::move(egl_manager));

  std::unique_ptr<FlutterWindowsView> view = engine->CreateView(
      std::make_unique<NiceMock<MockWindowBindingHandler>>());

  fml::AutoResetWaitableEvent metrics_sent_latch;
  engine_modifier.embedder_api().SendWindowMetricsEvent = MOCK_ENGINE_PROC(
      SendWindowMetricsEvent,
      ([&metrics_sent_latch](auto engine,
                             const FlutterWindowMetricsEvent* event) {
        metrics_sent_latch.Signal();
        return kSuccess;
      }));

  fml::AutoResetWaitableEvent resized_latch;
  std::thread([&resized_latch, &view]() {
    // Start the window resize. This sends the new window metrics
    // and then blocks until another thread completes the window resize.
    EXPECT_TRUE(view->OnWindowSizeChanged(500, 500));
    resized_latch.Signal();
  }).detach();

  // Wait until the platform thread has started the window resize.
  metrics_sent_latch.Wait();

  // Complete the window resize by reporting a frame with the new window size.
  ASSERT_TRUE(view->OnFrameGenerated(500, 500));
  view->OnFramePresented();
  resized_latch.Wait();
}

// Verify that an empty frame completes a view resize.
TEST(FlutterWindowsViewTest, TestEmptyFrameResizes) {
  auto windows_proc_table = std::make_shared<NiceMock<MockWindowsProcTable>>();
  std::unique_ptr<FlutterWindowsEngine> engine =
      GetTestEngine(windows_proc_table);

  EngineModifier engine_modifier{engine.get()};
  engine_modifier.embedder_api().PostRenderThreadTask = MOCK_ENGINE_PROC(
      PostRenderThreadTask,
      ([](auto engine, VoidCallback callback, void* user_data) {
        callback(user_data);
        return kSuccess;
      }));

  auto egl_manager = std::make_unique<egl::MockManager>();
  auto surface = std::make_unique<egl::MockWindowSurface>();
  auto resized_surface = std::make_unique<egl::MockWindowSurface>();
  auto resized_surface_ptr = resized_surface.get();

  EXPECT_CALL(*surface.get(), IsValid).WillRepeatedly(Return(true));
  EXPECT_CALL(*surface.get(), Destroy).WillOnce(Return(true));

  EXPECT_CALL(*egl_manager.get(),
              CreateWindowSurface(_, /*width=*/500, /*height=*/500))
      .WillOnce(Return(std::move((resized_surface))));
  EXPECT_CALL(*resized_surface_ptr, MakeCurrent).WillOnce(Return(true));
  EXPECT_CALL(*resized_surface_ptr, SetVSyncEnabled).WillOnce(Return(true));
  EXPECT_CALL(*windows_proc_table.get(), DwmFlush).WillOnce(Return(S_OK));

  EXPECT_CALL(*resized_surface_ptr, Destroy).WillOnce(Return(true));

  fml::AutoResetWaitableEvent metrics_sent_latch;
  engine_modifier.embedder_api().SendWindowMetricsEvent = MOCK_ENGINE_PROC(
      SendWindowMetricsEvent,
      ([&metrics_sent_latch](auto engine,
                             const FlutterWindowMetricsEvent* event) {
        metrics_sent_latch.Signal();
        return kSuccess;
      }));

  std::unique_ptr<FlutterWindowsView> view = engine->CreateView(
      std::make_unique<NiceMock<MockWindowBindingHandler>>());

  ViewModifier view_modifier{view.get()};
  engine_modifier.SetEGLManager(std::move(egl_manager));
  view_modifier.SetSurface(std::move(surface));

  fml::AutoResetWaitableEvent resized_latch;
  std::thread([&resized_latch, &view]() {
    // Start the window resize. This sends the new window metrics
    // and then blocks until another thread completes the window resize.
    EXPECT_TRUE(view->OnWindowSizeChanged(500, 500));
    resized_latch.Signal();
  }).detach();

  // Wait until the platform thread has started the window resize.
  metrics_sent_latch.Wait();

  // Complete the window resize by reporting an empty frame.
  view->OnEmptyFrameGenerated();
  view->OnFramePresented();
  resized_latch.Wait();
}

// A window resize can be interleaved between a frame generation and
// presentation. This should not crash the app. Regression test for:
// https://github.com/flutter/flutter/issues/141855
TEST(FlutterWindowsViewTest, WindowResizeRace) {
  std::unique_ptr<FlutterWindowsEngine> engine = GetTestEngine();

  EngineModifier engine_modifier(engine.get());
  engine_modifier.embedder_api().PostRenderThreadTask = MOCK_ENGINE_PROC(
      PostRenderThreadTask,
      ([](auto engine, VoidCallback callback, void* user_data) {
        callback(user_data);
        return kSuccess;
      }));

  auto egl_manager = std::make_unique<egl::MockManager>();
  auto surface = std::make_unique<egl::MockWindowSurface>();

  EXPECT_CALL(*surface.get(), IsValid).WillRepeatedly(Return(true));
  EXPECT_CALL(*surface.get(), Destroy).WillOnce(Return(true));

  std::unique_ptr<FlutterWindowsView> view = engine->CreateView(
      std::make_unique<NiceMock<MockWindowBindingHandler>>());

  ViewModifier view_modifier{view.get()};
  engine_modifier.SetEGLManager(std::move(egl_manager));
  view_modifier.SetSurface(std::move(surface));

  // Begin a frame.
  ASSERT_TRUE(view->OnFrameGenerated(100, 100));

  // Inject a window resize between the frame generation and
  // frame presentation. The new size invalidates the current frame.
  fml::AutoResetWaitableEvent resized_latch;
  std::thread([&resized_latch, &view]() {
    // The resize is never completed. The view times out and returns false.
    EXPECT_FALSE(view->OnWindowSizeChanged(500, 500));
    resized_latch.Signal();
  }).detach();

  // Wait until the platform thread has started the window resize.
  resized_latch.Wait();

  // Complete the invalidated frame while a resize is pending. Although this
  // might mean that we presented a frame with the wrong size, this should not
  // crash the app.
  view->OnFramePresented();
}

// Window resize should succeed even if the render surface could not be created
// even though EGL initialized successfully.
TEST(FlutterWindowsViewTest, WindowResizeInvalidSurface) {
  std::unique_ptr<FlutterWindowsEngine> engine = GetTestEngine();

  EngineModifier engine_modifier(engine.get());
  engine_modifier.embedder_api().PostRenderThreadTask = MOCK_ENGINE_PROC(
      PostRenderThreadTask,
      ([](auto engine, VoidCallback callback, void* user_data) {
        callback(user_data);
        return kSuccess;
      }));

  auto egl_manager = std::make_unique<egl::MockManager>();
  auto surface = std::make_unique<egl::MockWindowSurface>();

  EXPECT_CALL(*egl_manager.get(), CreateWindowSurface).Times(0);
  EXPECT_CALL(*surface.get(), IsValid).WillRepeatedly(Return(false));
  EXPECT_CALL(*surface.get(), Destroy).WillOnce(Return(false));

  std::unique_ptr<FlutterWindowsView> view = engine->CreateView(
      std::make_unique<NiceMock<MockWindowBindingHandler>>());

  ViewModifier view_modifier{view.get()};
  engine_modifier.SetEGLManager(std::move(egl_manager));
  view_modifier.SetSurface(std::move(surface));

  auto metrics_sent = false;
  engine_modifier.embedder_api().SendWindowMetricsEvent = MOCK_ENGINE_PROC(
      SendWindowMetricsEvent,
      ([&metrics_sent](auto engine, const FlutterWindowMetricsEvent* event) {
        metrics_sent = true;
        return kSuccess;
      }));

  view->OnWindowSizeChanged(500, 500);
}

// Window resize should succeed even if EGL initialized successfully
// but the EGL surface could not be created.
TEST(FlutterWindowsViewTest, WindowResizeWithoutSurface) {
  std::unique_ptr<FlutterWindowsEngine> engine = GetTestEngine();
  EngineModifier modifier(engine.get());

  auto egl_manager = std::make_unique<egl::MockManager>();

  EXPECT_CALL(*egl_manager.get(), CreateWindowSurface).Times(0);

  std::unique_ptr<FlutterWindowsView> view = engine->CreateView(
      std::make_unique<NiceMock<MockWindowBindingHandler>>());

  modifier.SetEGLManager(std::move(egl_manager));

  auto metrics_sent = false;
  modifier.embedder_api().SendWindowMetricsEvent = MOCK_ENGINE_PROC(
      SendWindowMetricsEvent,
      ([&metrics_sent](auto engine, const FlutterWindowMetricsEvent* event) {
        metrics_sent = true;
        return kSuccess;
      }));

  view->OnWindowSizeChanged(500, 500);
}

TEST(FlutterWindowsViewTest, WindowRepaintTests) {
  std::unique_ptr<FlutterWindowsEngine> engine = GetTestEngine();
  EngineModifier modifier(engine.get());

  FlutterWindowsView view{kImplicitViewId, engine.get(),
                          std::make_unique<flutter::FlutterWindow>(100, 100)};

  bool schedule_frame_called = false;
  modifier.embedder_api().ScheduleFrame =
      MOCK_ENGINE_PROC(ScheduleFrame, ([&schedule_frame_called](auto engine) {
                         schedule_frame_called = true;
                         return kSuccess;
                       }));

  view.OnWindowRepaint();
  EXPECT_TRUE(schedule_frame_called);
}

// Ensure that checkboxes have their checked status set apropriately
// Previously, only Radios could have this flag updated
// Resulted in the issue seen at
// https://github.com/flutter/flutter/issues/96218
// This test ensures that the native state of Checkboxes on Windows,
// specifically, is updated as desired.
TEST(FlutterWindowsViewTest, CheckboxNativeState) {
  std::unique_ptr<FlutterWindowsEngine> engine = GetTestEngine();
  EngineModifier modifier(engine.get());
  modifier.embedder_api().UpdateSemanticsEnabled =
      [](FLUTTER_API_SYMBOL(FlutterEngine) engine, bool enabled) {
        return kSuccess;
      };

  std::unique_ptr<FlutterWindowsView> view = engine->CreateView(
      std::make_unique<NiceMock<MockWindowBindingHandler>>());

  // Enable semantics to instantiate accessibility bridge.
  view->OnUpdateSemanticsEnabled(true);

  auto bridge = view->accessibility_bridge().lock();
  ASSERT_TRUE(bridge);

  FlutterSemanticsNode2 root{sizeof(FlutterSemanticsNode2), 0};
  root.id = 0;
  root.label = "root";
  root.hint = "";
  root.value = "";
  root.increased_value = "";
  root.decreased_value = "";
  root.child_count = 0;
  root.custom_accessibility_actions_count = 0;
  root.flags = static_cast<FlutterSemanticsFlag>(
      FlutterSemanticsFlag::kFlutterSemanticsFlagHasCheckedState |
      FlutterSemanticsFlag::kFlutterSemanticsFlagIsChecked);
  bridge->AddFlutterSemanticsNodeUpdate(root);

  bridge->CommitUpdates();

  {
    auto root_node = bridge->GetFlutterPlatformNodeDelegateFromID(0).lock();
    EXPECT_EQ(root_node->GetData().role, ax::mojom::Role::kCheckBox);
    EXPECT_EQ(root_node->GetData().GetCheckedState(),
              ax::mojom::CheckedState::kTrue);

    // Get the IAccessible for the root node.
    IAccessible* native_view = root_node->GetNativeViewAccessible();
    ASSERT_TRUE(native_view != nullptr);

    // Look up against the node itself (not one of its children).
    VARIANT varchild = {};
    varchild.vt = VT_I4;

    // Verify the checkbox is checked.
    varchild.lVal = CHILDID_SELF;
    VARIANT native_state = {};
    ASSERT_TRUE(SUCCEEDED(native_view->get_accState(varchild, &native_state)));
    EXPECT_TRUE(native_state.lVal & STATE_SYSTEM_CHECKED);

    // Perform similar tests for UIA value;
    IRawElementProviderSimple* uia_node;
    native_view->QueryInterface(IID_PPV_ARGS(&uia_node));
    ASSERT_TRUE(SUCCEEDED(uia_node->GetPropertyValue(
        UIA_ToggleToggleStatePropertyId, &native_state)));
    EXPECT_EQ(native_state.lVal, ToggleState_On);

    ASSERT_TRUE(SUCCEEDED(uia_node->GetPropertyValue(
        UIA_AriaPropertiesPropertyId, &native_state)));
    EXPECT_NE(std::wcsstr(native_state.bstrVal, L"checked=true"), nullptr);
  }

  // Test unchecked too.
  root.flags = static_cast<FlutterSemanticsFlag>(
      FlutterSemanticsFlag::kFlutterSemanticsFlagHasCheckedState);
  bridge->AddFlutterSemanticsNodeUpdate(root);
  bridge->CommitUpdates();

  {
    auto root_node = bridge->GetFlutterPlatformNodeDelegateFromID(0).lock();
    EXPECT_EQ(root_node->GetData().role, ax::mojom::Role::kCheckBox);
    EXPECT_EQ(root_node->GetData().GetCheckedState(),
              ax::mojom::CheckedState::kFalse);

    // Get the IAccessible for the root node.
    IAccessible* native_view = root_node->GetNativeViewAccessible();
    ASSERT_TRUE(native_view != nullptr);

    // Look up against the node itself (not one of its children).
    VARIANT varchild = {};
    varchild.vt = VT_I4;

    // Verify the checkbox is unchecked.
    varchild.lVal = CHILDID_SELF;
    VARIANT native_state = {};
    ASSERT_TRUE(SUCCEEDED(native_view->get_accState(varchild, &native_state)));
    EXPECT_FALSE(native_state.lVal & STATE_SYSTEM_CHECKED);

    // Perform similar tests for UIA value;
    IRawElementProviderSimple* uia_node;
    native_view->QueryInterface(IID_PPV_ARGS(&uia_node));
    ASSERT_TRUE(SUCCEEDED(uia_node->GetPropertyValue(
        UIA_ToggleToggleStatePropertyId, &native_state)));
    EXPECT_EQ(native_state.lVal, ToggleState_Off);

    ASSERT_TRUE(SUCCEEDED(uia_node->GetPropertyValue(
        UIA_AriaPropertiesPropertyId, &native_state)));
    EXPECT_NE(std::wcsstr(native_state.bstrVal, L"checked=false"), nullptr);
  }

  // Now check mixed state.
  root.flags = static_cast<FlutterSemanticsFlag>(
      FlutterSemanticsFlag::kFlutterSemanticsFlagHasCheckedState |
      FlutterSemanticsFlag::kFlutterSemanticsFlagIsCheckStateMixed);
  bridge->AddFlutterSemanticsNodeUpdate(root);
  bridge->CommitUpdates();

  {
    auto root_node = bridge->GetFlutterPlatformNodeDelegateFromID(0).lock();
    EXPECT_EQ(root_node->GetData().role, ax::mojom::Role::kCheckBox);
    EXPECT_EQ(root_node->GetData().GetCheckedState(),
              ax::mojom::CheckedState::kMixed);

    // Get the IAccessible for the root node.
    IAccessible* native_view = root_node->GetNativeViewAccessible();
    ASSERT_TRUE(native_view != nullptr);

    // Look up against the node itself (not one of its children).
    VARIANT varchild = {};
    varchild.vt = VT_I4;

    // Verify the checkbox is mixed.
    varchild.lVal = CHILDID_SELF;
    VARIANT native_state = {};
    ASSERT_TRUE(SUCCEEDED(native_view->get_accState(varchild, &native_state)));
    EXPECT_TRUE(native_state.lVal & STATE_SYSTEM_MIXED);

    // Perform similar tests for UIA value;
    IRawElementProviderSimple* uia_node;
    native_view->QueryInterface(IID_PPV_ARGS(&uia_node));
    ASSERT_TRUE(SUCCEEDED(uia_node->GetPropertyValue(
        UIA_ToggleToggleStatePropertyId, &native_state)));
    EXPECT_EQ(native_state.lVal, ToggleState_Indeterminate);

    ASSERT_TRUE(SUCCEEDED(uia_node->GetPropertyValue(
        UIA_AriaPropertiesPropertyId, &native_state)));
    EXPECT_NE(std::wcsstr(native_state.bstrVal, L"checked=mixed"), nullptr);
  }
}

// Ensure that switches have their toggle status set apropriately
TEST(FlutterWindowsViewTest, SwitchNativeState) {
  std::unique_ptr<FlutterWindowsEngine> engine = GetTestEngine();
  EngineModifier modifier(engine.get());
  modifier.embedder_api().UpdateSemanticsEnabled =
      [](FLUTTER_API_SYMBOL(FlutterEngine) engine, bool enabled) {
        return kSuccess;
      };

  std::unique_ptr<FlutterWindowsView> view = engine->CreateView(
      std::make_unique<NiceMock<MockWindowBindingHandler>>());

  // Enable semantics to instantiate accessibility bridge.
  view->OnUpdateSemanticsEnabled(true);

  auto bridge = view->accessibility_bridge().lock();
  ASSERT_TRUE(bridge);

  FlutterSemanticsNode2 root{sizeof(FlutterSemanticsNode2), 0};
  root.id = 0;
  root.label = "root";
  root.hint = "";
  root.value = "";
  root.increased_value = "";
  root.decreased_value = "";
  root.child_count = 0;
  root.custom_accessibility_actions_count = 0;
  root.flags = static_cast<FlutterSemanticsFlag>(
      FlutterSemanticsFlag::kFlutterSemanticsFlagHasToggledState |
      FlutterSemanticsFlag::kFlutterSemanticsFlagIsToggled);
  bridge->AddFlutterSemanticsNodeUpdate(root);

  bridge->CommitUpdates();

  {
    auto root_node = bridge->GetFlutterPlatformNodeDelegateFromID(0).lock();
    EXPECT_EQ(root_node->GetData().role, ax::mojom::Role::kSwitch);
    EXPECT_EQ(root_node->GetData().GetCheckedState(),
              ax::mojom::CheckedState::kTrue);

    // Get the IAccessible for the root node.
    IAccessible* native_view = root_node->GetNativeViewAccessible();
    ASSERT_TRUE(native_view != nullptr);

    // Look up against the node itself (not one of its children).
    VARIANT varchild = {};
    varchild.vt = VT_I4;

    varchild.lVal = CHILDID_SELF;
    VARIANT varrole = {};

    // Verify the role of the switch is CHECKBUTTON
    ASSERT_EQ(native_view->get_accRole(varchild, &varrole), S_OK);
    ASSERT_EQ(varrole.lVal, ROLE_SYSTEM_CHECKBUTTON);

    // Verify the switch is pressed.
    VARIANT native_state = {};
    ASSERT_TRUE(SUCCEEDED(native_view->get_accState(varchild, &native_state)));
    EXPECT_TRUE(native_state.lVal & STATE_SYSTEM_PRESSED);
    EXPECT_TRUE(native_state.lVal & STATE_SYSTEM_CHECKED);

    // Test similarly on UIA node.
    IRawElementProviderSimple* uia_node;
    native_view->QueryInterface(IID_PPV_ARGS(&uia_node));
    ASSERT_EQ(uia_node->GetPropertyValue(UIA_ControlTypePropertyId, &varrole),
              S_OK);
    EXPECT_EQ(varrole.lVal, UIA_ButtonControlTypeId);
    ASSERT_EQ(uia_node->GetPropertyValue(UIA_ToggleToggleStatePropertyId,
                                         &native_state),
              S_OK);
    EXPECT_EQ(native_state.lVal, ToggleState_On);
    ASSERT_EQ(
        uia_node->GetPropertyValue(UIA_AriaPropertiesPropertyId, &native_state),
        S_OK);
    EXPECT_NE(std::wcsstr(native_state.bstrVal, L"pressed=true"), nullptr);
  }

  // Test unpressed too.
  root.flags = static_cast<FlutterSemanticsFlag>(
      FlutterSemanticsFlag::kFlutterSemanticsFlagHasToggledState);
  bridge->AddFlutterSemanticsNodeUpdate(root);
  bridge->CommitUpdates();

  {
    auto root_node = bridge->GetFlutterPlatformNodeDelegateFromID(0).lock();
    EXPECT_EQ(root_node->GetData().role, ax::mojom::Role::kSwitch);
    EXPECT_EQ(root_node->GetData().GetCheckedState(),
              ax::mojom::CheckedState::kFalse);

    // Get the IAccessible for the root node.
    IAccessible* native_view = root_node->GetNativeViewAccessible();
    ASSERT_TRUE(native_view != nullptr);

    // Look up against the node itself (not one of its children).
    VARIANT varchild = {};
    varchild.vt = VT_I4;

    // Verify the switch is not pressed.
    varchild.lVal = CHILDID_SELF;
    VARIANT native_state = {};
    ASSERT_TRUE(SUCCEEDED(native_view->get_accState(varchild, &native_state)));
    EXPECT_FALSE(native_state.lVal & STATE_SYSTEM_PRESSED);
    EXPECT_FALSE(native_state.lVal & STATE_SYSTEM_CHECKED);

    // Test similarly on UIA node.
    IRawElementProviderSimple* uia_node;
    native_view->QueryInterface(IID_PPV_ARGS(&uia_node));
    ASSERT_EQ(uia_node->GetPropertyValue(UIA_ToggleToggleStatePropertyId,
                                         &native_state),
              S_OK);
    EXPECT_EQ(native_state.lVal, ToggleState_Off);
    ASSERT_EQ(
        uia_node->GetPropertyValue(UIA_AriaPropertiesPropertyId, &native_state),
        S_OK);
    EXPECT_NE(std::wcsstr(native_state.bstrVal, L"pressed=false"), nullptr);
  }
}

TEST(FlutterWindowsViewTest, TooltipNodeData) {
  std::unique_ptr<FlutterWindowsEngine> engine = GetTestEngine();
  EngineModifier modifier(engine.get());
  modifier.embedder_api().UpdateSemanticsEnabled =
      [](FLUTTER_API_SYMBOL(FlutterEngine) engine, bool enabled) {
        return kSuccess;
      };

  std::unique_ptr<FlutterWindowsView> view = engine->CreateView(
      std::make_unique<NiceMock<MockWindowBindingHandler>>());

  // Enable semantics to instantiate accessibility bridge.
  view->OnUpdateSemanticsEnabled(true);

  auto bridge = view->accessibility_bridge().lock();
  ASSERT_TRUE(bridge);

  FlutterSemanticsNode2 root{sizeof(FlutterSemanticsNode2), 0};
  root.id = 0;
  root.label = "root";
  root.hint = "";
  root.value = "";
  root.increased_value = "";
  root.decreased_value = "";
  root.tooltip = "tooltip";
  root.child_count = 0;
  root.custom_accessibility_actions_count = 0;
  root.flags = static_cast<FlutterSemanticsFlag>(
      FlutterSemanticsFlag::kFlutterSemanticsFlagIsTextField);
  bridge->AddFlutterSemanticsNodeUpdate(root);

  bridge->CommitUpdates();
  auto root_node = bridge->GetFlutterPlatformNodeDelegateFromID(0).lock();
  std::string tooltip = root_node->GetData().GetStringAttribute(
      ax::mojom::StringAttribute::kTooltip);
  EXPECT_EQ(tooltip, "tooltip");

  // Check that MSAA name contains the tooltip.
  IAccessible* native_view = bridge->GetFlutterPlatformNodeDelegateFromID(0)
                                 .lock()
                                 ->GetNativeViewAccessible();
  VARIANT varchild = {.vt = VT_I4, .lVal = CHILDID_SELF};
  BSTR bname;
  ASSERT_EQ(native_view->get_accName(varchild, &bname), S_OK);
  EXPECT_NE(std::wcsstr(bname, L"tooltip"), nullptr);

  // Check that UIA help text is equal to the tooltip.
  IRawElementProviderSimple* uia_node;
  native_view->QueryInterface(IID_PPV_ARGS(&uia_node));
  VARIANT varname{};
  ASSERT_EQ(uia_node->GetPropertyValue(UIA_HelpTextPropertyId, &varname), S_OK);
  std::string uia_tooltip = _com_util::ConvertBSTRToString(varname.bstrVal);
  EXPECT_EQ(uia_tooltip, "tooltip");
}

// Don't block until the v-blank if it is disabled by the window.
// The surface is updated on the platform thread at startup.
TEST(FlutterWindowsViewTest, DisablesVSyncAtStartup) {
  auto windows_proc_table = std::make_shared<MockWindowsProcTable>();
  auto engine = std::make_unique<MockFlutterWindowsEngine>(windows_proc_table);
  auto egl_manager = std::make_unique<egl::MockManager>();
  egl::MockContext render_context;
  auto surface = std::make_unique<egl::MockWindowSurface>();
  auto surface_ptr = surface.get();

  EXPECT_CALL(*engine.get(), running).WillRepeatedly(Return(false));
  EXPECT_CALL(*engine.get(), PostRasterThreadTask).Times(0);

  EXPECT_CALL(*windows_proc_table.get(), DwmIsCompositionEnabled)
      .WillOnce(Return(true));

  EXPECT_CALL(*egl_manager.get(), render_context)
      .WillOnce(Return(&render_context));
  EXPECT_CALL(*surface_ptr, IsValid).WillOnce(Return(true));

  InSequence s;
  EXPECT_CALL(*egl_manager.get(), CreateWindowSurface)
      .WillOnce(Return(std::move(surface)));
  EXPECT_CALL(*surface_ptr, MakeCurrent).WillOnce(Return(true));
  EXPECT_CALL(*surface_ptr, SetVSyncEnabled(false)).WillOnce(Return(true));
  EXPECT_CALL(render_context, ClearCurrent).WillOnce(Return(true));

  EXPECT_CALL(*surface_ptr, Destroy).Times(1);

  EngineModifier modifier{engine.get()};
  modifier.SetEGLManager(std::move(egl_manager));

  std::unique_ptr<FlutterWindowsView> view = engine->CreateView(
      std::make_unique<NiceMock<MockWindowBindingHandler>>());
}

// Blocks until the v-blank if it is enabled by the window.
// The surface is updated on the platform thread at startup.
TEST(FlutterWindowsViewTest, EnablesVSyncAtStartup) {
  auto windows_proc_table = std::make_shared<MockWindowsProcTable>();
  auto engine = std::make_unique<MockFlutterWindowsEngine>(windows_proc_table);
  auto egl_manager = std::make_unique<egl::MockManager>();
  egl::MockContext render_context;
  auto surface = std::make_unique<egl::MockWindowSurface>();
  auto surface_ptr = surface.get();

  EXPECT_CALL(*engine.get(), running).WillRepeatedly(Return(false));
  EXPECT_CALL(*engine.get(), PostRasterThreadTask).Times(0);
  EXPECT_CALL(*windows_proc_table.get(), DwmIsCompositionEnabled)
      .WillOnce(Return(false));

  EXPECT_CALL(*egl_manager.get(), render_context)
      .WillOnce(Return(&render_context));
  EXPECT_CALL(*surface_ptr, IsValid).WillOnce(Return(true));

  InSequence s;
  EXPECT_CALL(*egl_manager.get(), CreateWindowSurface)
      .WillOnce(Return(std::move(surface)));
  EXPECT_CALL(*surface_ptr, MakeCurrent).WillOnce(Return(true));
  EXPECT_CALL(*surface_ptr, SetVSyncEnabled(true)).WillOnce(Return(true));
  EXPECT_CALL(render_context, ClearCurrent).WillOnce(Return(true));

  EXPECT_CALL(*surface_ptr, Destroy).Times(1);

  EngineModifier modifier{engine.get()};
  modifier.SetEGLManager(std::move(egl_manager));

  std::unique_ptr<FlutterWindowsView> view = engine->CreateView(
      std::make_unique<NiceMock<MockWindowBindingHandler>>());
}

// Don't block until the v-blank if it is disabled by the window.
// The surface is updated on the raster thread if the engine is running.
TEST(FlutterWindowsViewTest, DisablesVSyncAfterStartup) {
  auto windows_proc_table = std::make_shared<MockWindowsProcTable>();
  auto engine = std::make_unique<MockFlutterWindowsEngine>(windows_proc_table);
  auto egl_manager = std::make_unique<egl::MockManager>();
  egl::MockContext render_context;
  auto surface = std::make_unique<egl::MockWindowSurface>();
  auto surface_ptr = surface.get();

  EXPECT_CALL(*engine.get(), running).WillRepeatedly(Return(true));
  EXPECT_CALL(*windows_proc_table.get(), DwmIsCompositionEnabled)
      .WillOnce(Return(true));

  EXPECT_CALL(*egl_manager.get(), render_context)
      .WillOnce(Return(&render_context));
  EXPECT_CALL(*surface_ptr, IsValid).WillOnce(Return(true));

  InSequence s;
  EXPECT_CALL(*egl_manager.get(), CreateWindowSurface)
      .WillOnce(Return(std::move(surface)));
  EXPECT_CALL(*engine.get(), PostRasterThreadTask)
      .WillOnce([](fml::closure callback) {
        callback();
        return true;
      });
  EXPECT_CALL(*surface_ptr, MakeCurrent).WillOnce(Return(true));
  EXPECT_CALL(*surface_ptr, SetVSyncEnabled(false)).WillOnce(Return(true));
  EXPECT_CALL(render_context, ClearCurrent).WillOnce(Return(true));
  EXPECT_CALL(*engine.get(), PostRasterThreadTask)
      .WillOnce([](fml::closure callback) {
        callback();
        return true;
      });
  EXPECT_CALL(*surface_ptr, Destroy).Times(1);

  EngineModifier modifier{engine.get()};
  modifier.SetEGLManager(std::move(egl_manager));

  std::unique_ptr<FlutterWindowsView> view = engine->CreateView(
      std::make_unique<NiceMock<MockWindowBindingHandler>>());
}

// Blocks until the v-blank if it is enabled by the window.
// The surface is updated on the raster thread if the engine is running.
TEST(FlutterWindowsViewTest, EnablesVSyncAfterStartup) {
  auto windows_proc_table = std::make_shared<MockWindowsProcTable>();
  auto engine = std::make_unique<MockFlutterWindowsEngine>(windows_proc_table);
  auto egl_manager = std::make_unique<egl::MockManager>();
  egl::MockContext render_context;
  auto surface = std::make_unique<egl::MockWindowSurface>();
  auto surface_ptr = surface.get();

  EXPECT_CALL(*engine.get(), running).WillRepeatedly(Return(true));

  EXPECT_CALL(*windows_proc_table.get(), DwmIsCompositionEnabled)
      .WillOnce(Return(false));

  EXPECT_CALL(*egl_manager.get(), render_context)
      .WillOnce(Return(&render_context));
  EXPECT_CALL(*surface_ptr, IsValid).WillOnce(Return(true));

  InSequence s;
  EXPECT_CALL(*egl_manager.get(), CreateWindowSurface)
      .WillOnce(Return(std::move(surface)));
  EXPECT_CALL(*engine.get(), PostRasterThreadTask)
      .WillOnce([](fml::closure callback) {
        callback();
        return true;
      });

  EXPECT_CALL(*surface_ptr, MakeCurrent).WillOnce(Return(true));
  EXPECT_CALL(*surface_ptr, SetVSyncEnabled(true)).WillOnce(Return(true));
  EXPECT_CALL(render_context, ClearCurrent).WillOnce(Return(true));

  EXPECT_CALL(*engine.get(), PostRasterThreadTask)
      .WillOnce([](fml::closure callback) {
        callback();
        return true;
      });
  EXPECT_CALL(*surface_ptr, Destroy).Times(1);

  EngineModifier modifier{engine.get()};
  modifier.SetEGLManager(std::move(egl_manager));

  std::unique_ptr<FlutterWindowsView> view = engine->CreateView(
      std::make_unique<NiceMock<MockWindowBindingHandler>>());
}

// Desktop Window Manager composition can be disabled on Windows 7.
// If this happens, the app must synchronize with the vsync to prevent
// screen tearing.
TEST(FlutterWindowsViewTest, UpdatesVSyncOnDwmUpdates) {
  auto windows_proc_table = std::make_shared<MockWindowsProcTable>();
  auto engine = std::make_unique<MockFlutterWindowsEngine>(windows_proc_table);
  auto egl_manager = std::make_unique<egl::MockManager>();
  egl::MockContext render_context;
  auto surface = std::make_unique<egl::MockWindowSurface>();
  auto surface_ptr = surface.get();

  EXPECT_CALL(*engine.get(), running).WillRepeatedly(Return(true));

  EXPECT_CALL(*engine.get(), PostRasterThreadTask)
      .WillRepeatedly([](fml::closure callback) {
        callback();
        return true;
      });

  EXPECT_CALL(*egl_manager.get(), render_context)
      .WillRepeatedly(Return(&render_context));

  EXPECT_CALL(*surface_ptr, IsValid).WillRepeatedly(Return(true));
  EXPECT_CALL(*surface_ptr, MakeCurrent).WillRepeatedly(Return(true));
  EXPECT_CALL(*surface_ptr, Destroy).Times(1);
  EXPECT_CALL(render_context, ClearCurrent).WillRepeatedly(Return(true));

  InSequence s;

  // Mock render surface initialization.
  std::unique_ptr<FlutterWindowsView> view;
  {
    EXPECT_CALL(*egl_manager, CreateWindowSurface)
        .WillOnce(Return(std::move(surface)));
    EXPECT_CALL(*windows_proc_table.get(), DwmIsCompositionEnabled)
        .WillOnce(Return(true));
    EXPECT_CALL(*surface_ptr, SetVSyncEnabled).WillOnce(Return(true));

    EngineModifier engine_modifier{engine.get()};
    engine_modifier.SetEGLManager(std::move(egl_manager));

    view = engine->CreateView(
        std::make_unique<NiceMock<MockWindowBindingHandler>>());
  }

  // Disabling DWM composition should enable vsync blocking on the surface.
  {
    EXPECT_CALL(*windows_proc_table.get(), DwmIsCompositionEnabled)
        .WillOnce(Return(false));
    EXPECT_CALL(*surface_ptr, SetVSyncEnabled(true)).WillOnce(Return(true));

    engine->OnDwmCompositionChanged();
  }

  // Enabling DWM composition should disable vsync blocking on the surface.
  {
    EXPECT_CALL(*windows_proc_table.get(), DwmIsCompositionEnabled)
        .WillOnce(Return(true));
    EXPECT_CALL(*surface_ptr, SetVSyncEnabled(false)).WillOnce(Return(true));

    engine->OnDwmCompositionChanged();
  }
}

}  // namespace testing
}  // namespace flutter
