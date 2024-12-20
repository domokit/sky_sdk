// Copyright 2021 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/testing/dart_isolate_runner.h"
#include "flutter/testing/fixture_test.h"

#include "tonic/dart_args.h"
#include "tonic/dart_wrappable.h"

namespace flutter {
namespace testing {

class MyNativeClass : public RefCountedDartWrappable<MyNativeClass> {
  DEFINE_WRAPPERTYPEINFO();
  FML_FRIEND_MAKE_REF_COUNTED(MyNativeClass);

  MyNativeClass(intptr_t value) : _value(value){};

 public:
  intptr_t _value = 0;

  static void Create(Dart_Handle path_handle, intptr_t value) {
    auto path = fml::MakeRefCounted<MyNativeClass>(value);
    path->AssociateWithDartWrapper(path_handle);
  }

  // Dummy test functions:

  static int32_t MyTestFunction(MyNativeClass* ptr,
                                int32_t x,
                                Dart_Handle handle) {
    return ptr->_value + x;
  }

  Dart_Handle MyTestMethod(int64_t x) { return Dart_NewInteger(_value + x); }
};

IMPLEMENT_WRAPPERTYPEINFO("Lib", MyNativeClass)

class FfiNativeTest : public FixtureTest {
 public:
  FfiNativeTest()
      : settings_(CreateSettingsForFixture()),
        vm_(DartVMRef::Create(settings_)),
        thread_(CreateNewThread()),
        task_runners_(GetCurrentTestName(),
                      thread_,
                      thread_,
                      thread_,
                      thread_) {}

  ~FfiNativeTest() = default;

  [[nodiscard]] bool RunWithEntrypoint(const std::string& entrypoint) {
    if (running_isolate_) {
      return false;
    }
    auto isolate =
        RunDartCodeInIsolate(vm_, settings_, task_runners_, entrypoint, {},
                             GetDefaultKernelFilePath());
    if (!isolate || isolate->get()->GetPhase() != DartIsolate::Phase::Running) {
      return false;
    }

    running_isolate_ = std::move(isolate);
    return true;
  }

  template <typename C, typename Signature, Signature function>
  void DoCallThroughTest(const char* testName, const char* testEntry) {
    auto weak_persistent_value = tonic::DartWeakPersistentValue();

    fml::AutoResetWaitableEvent event;

    AddFfiNativeCallback(
        testName, reinterpret_cast<void*>(
                      tonic::FfiDispatcher<C, Signature, function>::Call));

    AddFfiNativeCallback(
        "CreateNative",
        reinterpret_cast<void*>(
            tonic::FfiDispatcher<void, decltype(&MyNativeClass::Create),
                                 &MyNativeClass::Create>::Call));

    AddNativeCallback("SignalDone",
                      CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) {
                        // Clear on initiative of Dart.
                        weak_persistent_value.Clear();

                        event.Signal();
                      }));

    ASSERT_TRUE(RunWithEntrypoint(testEntry));
    event.Wait();

    running_isolate_->Shutdown();
  }

  template <typename C, typename Signature, Signature function>
  void DoSerialiseTest(bool leaf,
                       const char* returnFfi,
                       const char* returnDart,
                       const char* argsFfi,
                       const char* argsDart) {
    auto dispatcher = tonic::FfiDispatcher<C, Signature, function>();

    EXPECT_EQ(dispatcher.AllowedAsLeafCall(), leaf);
    EXPECT_STREQ(dispatcher.GetReturnFfiRepresentation(), returnFfi);
    EXPECT_STREQ(dispatcher.GetReturnDartRepresentation(), returnDart);

    {
      std::ostringstream stream_;
      dispatcher.WriteFfiArguments(&stream_);
      EXPECT_STREQ(stream_.str().c_str(), argsFfi);
    }

    {
      std::ostringstream stream_;
      dispatcher.WriteDartArguments(&stream_);
      EXPECT_STREQ(stream_.str().c_str(), argsDart);
    }
  }

 protected:
  Settings settings_;
  DartVMRef vm_;
  std::unique_ptr<AutoIsolateShutdown> running_isolate_;
  fml::RefPtr<fml::TaskRunner> thread_;
  TaskRunners task_runners_;
  FML_DISALLOW_COPY_AND_ASSIGN(FfiNativeTest);
};

//
// Call bindings.
//

// Call and serialise a simple function through the Tonic FFI bindings.

void Nop() {}

TEST_F(FfiNativeTest, FfiBindingCallNop) {
  DoCallThroughTest<void, decltype(&Nop), &Nop>("Nop", "callNop");
}

TEST_F(FfiNativeTest, SerialiseNop) {
  DoSerialiseTest<void, decltype(&Nop), &Nop>(
      /*leaf=*/true, "Void", "void", "", "");
}

// Call and serialise function with bool.

bool EchoBool(bool arg) {
  EXPECT_TRUE(arg);
  return arg;
}

TEST_F(FfiNativeTest, FfiBindingCallEchoBool) {
  DoCallThroughTest<void, decltype(&EchoBool), &EchoBool>("EchoBool",
                                                          "callEchoBool");
}

TEST_F(FfiNativeTest, SerialiseEchoBool) {
  DoSerialiseTest<void, decltype(&EchoBool), &EchoBool>(
      /*leaf=*/true, "Bool", "bool", "Bool", "bool");
}

// Call and serialise function with intptr_t.

intptr_t EchoIntPtr(intptr_t arg) {
  EXPECT_EQ(arg, 23);
  return arg;
}

TEST_F(FfiNativeTest, FfiBindingCallEchoIntPtr) {
  DoCallThroughTest<void, decltype(&EchoIntPtr), &EchoIntPtr>("EchoIntPtr",
                                                              "callEchoIntPtr");
}

TEST_F(FfiNativeTest, SerialiseEchoIntPtr) {
  if (sizeof(intptr_t) == 8) {
    DoSerialiseTest<void, decltype(&EchoIntPtr), &EchoIntPtr>(
        /*leaf=*/true, "Int64", "int", "Int64", "int");
  } else {
    EXPECT_EQ(sizeof(intptr_t), 4ul);
    DoSerialiseTest<void, decltype(&EchoIntPtr), &EchoIntPtr>(
        /*leaf=*/true, "Int32", "int", "Int32", "int");
  }
}

// Call and serialise function with double.

double EchoDouble(double arg) {
  EXPECT_EQ(arg, 23.0);
  return arg;
}

TEST_F(FfiNativeTest, FfiBindingCallEchoDouble) {
  DoCallThroughTest<void, decltype(&EchoDouble), &EchoDouble>("EchoDouble",
                                                              "callEchoDouble");
}

TEST_F(FfiNativeTest, SerialiseEchoDouble) {
  DoSerialiseTest<void, decltype(&EchoDouble), &EchoDouble>(
      /*leaf=*/true, "Double", "double", "Double", "double");
}

// Call and serialise function with Dart_Handle.

Dart_Handle EchoHandle(Dart_Handle str) {
  const char* c_str = nullptr;
  Dart_StringToCString(str, &c_str);
  EXPECT_STREQ(c_str, "Hello EchoHandle");
  return str;
}

TEST_F(FfiNativeTest, FfiBindingCallEchoHandle) {
  DoCallThroughTest<void, decltype(&EchoHandle), &EchoHandle>("EchoHandle",
                                                              "callEchoHandle");
}

TEST_F(FfiNativeTest, SerialiseEchoHandle) {
  DoSerialiseTest<void, decltype(&EchoHandle), &EchoHandle>(
      /*leaf=*/false, "Handle", "Object", "Handle", "Object");
}

//  Call and serialise function with std::string.

std::string EchoString(std::string arg) {
  EXPECT_STREQ(arg.c_str(), "Hello EchoString");
  return arg;
}

TEST_F(FfiNativeTest, FfiBindingCallEchoString) {
  DoCallThroughTest<void, decltype(&EchoString), &EchoString>("EchoString",
                                                              "callEchoString");
}

TEST_F(FfiNativeTest, SerialiseEchoString) {
  DoSerialiseTest<void, decltype(&EchoString), &EchoString>(
      /*leaf=*/false, "Handle", "String", "Handle", "String");
}

// Call and serialise function with std::u16string.

std::u16string EchoU16String(std::u16string arg) {
  EXPECT_EQ(arg, u"Hello EchoU16String");
  return arg;
}

TEST_F(FfiNativeTest, FfiBindingCallEchoU16String) {
  DoCallThroughTest<void, decltype(&EchoU16String), &EchoU16String>(
      "EchoU16String", "callEchoU16String");
}

TEST_F(FfiNativeTest, SerialiseEchoU16String) {
  DoSerialiseTest<void, decltype(&EchoU16String), &EchoU16String>(
      /*leaf=*/false, "Handle", "String", "Handle", "String");
}

//  Call and serialise function with std::vector.

std::vector<std::string> EchoVector(const std::vector<std::string>& arg) {
  EXPECT_STREQ(arg[0].c_str(), "Hello EchoVector");
  return arg;
}

TEST_F(FfiNativeTest, FfiBindingCallEchoVector) {
  DoCallThroughTest<void, decltype(&EchoVector), &EchoVector>("EchoVector",
                                                              "callEchoVector");
}

TEST_F(FfiNativeTest, SerialiseEchoVector) {
  DoSerialiseTest<void, decltype(&EchoVector), &EchoVector>(
      /*leaf=*/false, "Handle", "List", "Handle", "List");
}

//  Call and serialise function with  DartWrappable.

intptr_t EchoWrappable(MyNativeClass* arg) {
  EXPECT_EQ(arg->_value, 0x1234);
  return arg->_value;
}

TEST_F(FfiNativeTest, FfiBindingCallEchoWrappable) {
  DoCallThroughTest<void, decltype(&EchoWrappable), &EchoWrappable>(
      "EchoWrappable", "callEchoWrappable");
}

TEST_F(FfiNativeTest, SerialiseEchoWrappable) {
  if (sizeof(intptr_t) == 8) {
    DoSerialiseTest<void, decltype(&EchoWrappable), &EchoWrappable>(
        /*leaf=*/true, "Int64", "int", "Pointer", "Pointer");
  } else {
    EXPECT_EQ(sizeof(intptr_t), 4ul);
    DoSerialiseTest<void, decltype(&EchoWrappable), &EchoWrappable>(
        /*leaf=*/true, "Int32", "int", "Pointer", "Pointer");
  }
}

// Call and serialise function with TypedList<..>.

tonic::Float32List EchoTypedList(tonic::Float32List arg) {
  EXPECT_NEAR(arg[1], 3.14, 0.01);
  return arg;
}

TEST_F(FfiNativeTest, FfiBindingCallEchoTypedList) {
  DoCallThroughTest<void, decltype(&EchoTypedList), &EchoTypedList>(
      "EchoTypedList", "callEchoTypedList");
}

TEST_F(FfiNativeTest, SerialiseEchoTypedList) {
  DoSerialiseTest<void, decltype(&EchoTypedList), &EchoTypedList>(
      /*leaf=*/false, "Handle", "Object", "Handle", "Object");
}

// Call and serialise a static class member function.

TEST_F(FfiNativeTest, FfiBindingCallClassMemberFunction) {
  DoCallThroughTest<void, decltype(&MyNativeClass::MyTestFunction),
                    &MyNativeClass::MyTestFunction>(
      "MyNativeClass::MyTestFunction", "callMyTestFunction");
}

TEST_F(FfiNativeTest, SerialiseClassMemberFunction) {
  DoSerialiseTest<void, decltype(&MyNativeClass::MyTestFunction),
                  &MyNativeClass::MyTestFunction>(
      /*leaf=*/false, "Int32", "int", "Pointer, Int32, Handle",
      "Pointer, int, Object");
}

// Call and serialise an instance method.

TEST_F(FfiNativeTest, FfiBindingCallClassMemberMethod) {
  DoCallThroughTest<MyNativeClass, decltype(&MyNativeClass::MyTestMethod),
                    &MyNativeClass::MyTestMethod>("MyNativeClass::MyTestMethod",
                                                  "callMyTestMethod");
}

TEST_F(FfiNativeTest, SerialiseClassMemberMethod) {
  DoSerialiseTest<MyNativeClass, decltype(&MyNativeClass::MyTestMethod),
                  &MyNativeClass::MyTestMethod>(
      /*leaf=*/false, "Handle", "Object", "Pointer, Int64", "Pointer, int");
}

}  // namespace testing
}  // namespace flutter
