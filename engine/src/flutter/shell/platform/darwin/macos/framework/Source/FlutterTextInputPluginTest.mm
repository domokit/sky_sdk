// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/macos/framework/Headers/FlutterViewController.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterDartProject_Internal.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterEngineTestUtils.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterEngine_Internal.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterTextInputPlugin.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterTextInputSemanticsObject.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterViewController_Internal.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/NSView+ClipsToBounds.h"

#import <OCMock/OCMock.h>
#import "flutter/testing/testing.h"

@interface FlutterTextField (Testing)
- (void)setPlatformNode:(flutter::FlutterTextPlatformNode*)node;
@end

@interface FlutterTextFieldMock : FlutterTextField

@property(nonatomic, nullable, copy) NSString* lastUpdatedString;
@property(nonatomic) NSRange lastUpdatedSelection;

@end

@implementation FlutterTextFieldMock

- (void)updateString:(NSString*)string withSelection:(NSRange)selection {
  _lastUpdatedString = string;
  _lastUpdatedSelection = selection;
}

@end

@interface NSTextInputContext (Private)
// This is a private method.
- (BOOL)isActive;
@end

@interface TextInputTestViewController : FlutterViewController
@end

@implementation TextInputTestViewController
- (nonnull FlutterView*)createFlutterViewWithMTLDevice:(id<MTLDevice>)device
                                          commandQueue:(id<MTLCommandQueue>)commandQueue {
  return OCMClassMock([NSView class]);
}
@end

@interface FlutterInputPluginTestObjc : NSObject
- (bool)testEmptyCompositionRange;
- (bool)testClearClientDuringComposing;
@end

@implementation FlutterInputPluginTestObjc

- (bool)testEmptyCompositionRange {
  id engineMock = flutter::testing::CreateMockFlutterEngine(@"");
  id binaryMessengerMock = OCMProtocolMock(@protocol(FlutterBinaryMessenger));
  OCMStub(  // NOLINT(google-objc-avoid-throwing-exception)
      [engineMock binaryMessenger])
      .andReturn(binaryMessengerMock);

  FlutterViewController* viewController = [[FlutterViewController alloc] initWithEngine:engineMock
                                                                                nibName:@""
                                                                                 bundle:nil];

  FlutterTextInputPlugin* plugin =
      [[FlutterTextInputPlugin alloc] initWithViewController:viewController];

  NSDictionary* setClientConfig = @{
    @"inputAction" : @"action",
    @"inputType" : @{@"name" : @"inputName"},
  };
  [plugin handleMethodCall:[FlutterMethodCall methodCallWithMethodName:@"TextInput.setClient"
                                                             arguments:@[ @(1), setClientConfig ]]
                    result:^(id){
                    }];

  FlutterMethodCall* call = [FlutterMethodCall methodCallWithMethodName:@"TextInput.setEditingState"
                                                              arguments:@{
                                                                @"text" : @"Text",
                                                                @"selectionBase" : @(0),
                                                                @"selectionExtent" : @(0),
                                                                @"composingBase" : @(-1),
                                                                @"composingExtent" : @(-1),
                                                              }];

  [plugin handleMethodCall:call
                    result:^(id){
                    }];

  // Verify editing state was set.
  NSDictionary* editingState = [plugin editingState];
  EXPECT_STREQ([editingState[@"text"] UTF8String], "Text");
  EXPECT_STREQ([editingState[@"selectionAffinity"] UTF8String], "TextAffinity.upstream");
  EXPECT_FALSE([editingState[@"selectionIsDirectional"] boolValue]);
  EXPECT_EQ([editingState[@"selectionBase"] intValue], 0);
  EXPECT_EQ([editingState[@"selectionExtent"] intValue], 0);
  EXPECT_EQ([editingState[@"composingBase"] intValue], -1);
  EXPECT_EQ([editingState[@"composingExtent"] intValue], -1);
  return true;
}

- (bool)testSetMarkedTextWithSelectionChange {
  id engineMock = flutter::testing::CreateMockFlutterEngine(@"");
  id binaryMessengerMock = OCMProtocolMock(@protocol(FlutterBinaryMessenger));
  OCMStub(  // NOLINT(google-objc-avoid-throwing-exception)
      [engineMock binaryMessenger])
      .andReturn(binaryMessengerMock);

  FlutterViewController* viewController = [[FlutterViewController alloc] initWithEngine:engineMock
                                                                                nibName:@""
                                                                                 bundle:nil];

  FlutterTextInputPlugin* plugin =
      [[FlutterTextInputPlugin alloc] initWithViewController:viewController];

  NSDictionary* setClientConfig = @{
    @"inputAction" : @"action",
    @"inputType" : @{@"name" : @"inputName"},
  };
  [plugin handleMethodCall:[FlutterMethodCall methodCallWithMethodName:@"TextInput.setClient"
                                                             arguments:@[ @(1), setClientConfig ]]
                    result:^(id){
                    }];

  FlutterMethodCall* call = [FlutterMethodCall methodCallWithMethodName:@"TextInput.setEditingState"
                                                              arguments:@{
                                                                @"text" : @"Text",
                                                                @"selectionBase" : @(4),
                                                                @"selectionExtent" : @(4),
                                                                @"composingBase" : @(-1),
                                                                @"composingExtent" : @(-1),
                                                              }];
  [plugin handleMethodCall:call
                    result:^(id){
                    }];

  [plugin setMarkedText:@"marked"
          selectedRange:NSMakeRange(1, 0)
       replacementRange:NSMakeRange(NSNotFound, 0)];

  NSDictionary* expectedState = @{
    @"selectionBase" : @(5),
    @"selectionExtent" : @(5),
    @"selectionAffinity" : @"TextAffinity.upstream",
    @"selectionIsDirectional" : @(NO),
    @"composingBase" : @(4),
    @"composingExtent" : @(10),
    @"text" : @"Textmarked",
  };

  NSData* updateCall = [[FlutterJSONMethodCodec sharedInstance]
      encodeMethodCall:[FlutterMethodCall
                           methodCallWithMethodName:@"TextInputClient.updateEditingState"
                                          arguments:@[ @(1), expectedState ]]];

  OCMExpect(  // NOLINT(google-objc-avoid-throwing-exception)
      [binaryMessengerMock sendOnChannel:@"flutter/textinput" message:updateCall]);

  @try {
    OCMVerify(  // NOLINT(google-objc-avoid-throwing-exception)
        [binaryMessengerMock sendOnChannel:@"flutter/textinput" message:updateCall]);
  } @catch (...) {
    return false;
  }
  return true;
}

- (bool)testSetMarkedTextWithReplacementRange {
  id engineMock = flutter::testing::CreateMockFlutterEngine(@"");
  id binaryMessengerMock = OCMProtocolMock(@protocol(FlutterBinaryMessenger));
  OCMStub(  // NOLINT(google-objc-avoid-throwing-exception)
      [engineMock binaryMessenger])
      .andReturn(binaryMessengerMock);

  FlutterViewController* viewController = [[FlutterViewController alloc] initWithEngine:engineMock
                                                                                nibName:@""
                                                                                 bundle:nil];

  FlutterTextInputPlugin* plugin =
      [[FlutterTextInputPlugin alloc] initWithViewController:viewController];

  NSDictionary* setClientConfig = @{
    @"inputAction" : @"action",
    @"inputType" : @{@"name" : @"inputName"},
  };
  [plugin handleMethodCall:[FlutterMethodCall methodCallWithMethodName:@"TextInput.setClient"
                                                             arguments:@[ @(1), setClientConfig ]]
                    result:^(id){
                    }];

  FlutterMethodCall* call = [FlutterMethodCall methodCallWithMethodName:@"TextInput.setEditingState"
                                                              arguments:@{
                                                                @"text" : @"1234",
                                                                @"selectionBase" : @(3),
                                                                @"selectionExtent" : @(3),
                                                                @"composingBase" : @(-1),
                                                                @"composingExtent" : @(-1),
                                                              }];
  [plugin handleMethodCall:call
                    result:^(id){
                    }];

  [plugin setMarkedText:@"marked"
          selectedRange:NSMakeRange(1, 0)
       replacementRange:NSMakeRange(1, 2)];

  NSDictionary* expectedState = @{
    @"selectionBase" : @(2),
    @"selectionExtent" : @(2),
    @"selectionAffinity" : @"TextAffinity.upstream",
    @"selectionIsDirectional" : @(NO),
    @"composingBase" : @(1),
    @"composingExtent" : @(7),
    @"text" : @"1marked4",
  };

  NSData* updateCall = [[FlutterJSONMethodCodec sharedInstance]
      encodeMethodCall:[FlutterMethodCall
                           methodCallWithMethodName:@"TextInputClient.updateEditingState"
                                          arguments:@[ @(1), expectedState ]]];

  OCMExpect(  // NOLINT(google-objc-avoid-throwing-exception)
      [binaryMessengerMock sendOnChannel:@"flutter/textinput" message:updateCall]);

  @try {
    OCMVerify(  // NOLINT(google-objc-avoid-throwing-exception)
        [binaryMessengerMock sendOnChannel:@"flutter/textinput" message:updateCall]);
  } @catch (...) {
    return false;
  }
  return true;
}

- (bool)testComposingRegionRemovedByFramework {
  id engineMock = flutter::testing::CreateMockFlutterEngine(@"");
  id binaryMessengerMock = OCMProtocolMock(@protocol(FlutterBinaryMessenger));
  OCMStub(  // NOLINT(google-objc-avoid-throwing-exception)
      [engineMock binaryMessenger])
      .andReturn(binaryMessengerMock);

  FlutterViewController* viewController = [[FlutterViewController alloc] initWithEngine:engineMock
                                                                                nibName:@""
                                                                                 bundle:nil];

  FlutterTextInputPlugin* plugin =
      [[FlutterTextInputPlugin alloc] initWithViewController:viewController];

  NSDictionary* setClientConfig = @{
    @"inputAction" : @"action",
    @"inputType" : @{@"name" : @"inputName"},
  };
  [plugin handleMethodCall:[FlutterMethodCall methodCallWithMethodName:@"TextInput.setClient"
                                                             arguments:@[ @(1), setClientConfig ]]
                    result:^(id){
                    }];

  FlutterMethodCall* call = [FlutterMethodCall methodCallWithMethodName:@"TextInput.setEditingState"
                                                              arguments:@{
                                                                @"text" : @"Text",
                                                                @"selectionBase" : @(4),
                                                                @"selectionExtent" : @(4),
                                                                @"composingBase" : @(2),
                                                                @"composingExtent" : @(4),
                                                              }];
  [plugin handleMethodCall:call
                    result:^(id){
                    }];

  // Update with the composing region removed.
  call = [FlutterMethodCall methodCallWithMethodName:@"TextInput.setEditingState"
                                           arguments:@{
                                             @"text" : @"Te",
                                             @"selectionBase" : @(2),
                                             @"selectionExtent" : @(2),
                                             @"composingBase" : @(-1),
                                             @"composingExtent" : @(-1),
                                           }];
  [plugin handleMethodCall:call
                    result:^(id){
                    }];

  // Verify editing state was set.
  NSDictionary* editingState = [plugin editingState];
  EXPECT_STREQ([editingState[@"text"] UTF8String], "Te");
  EXPECT_STREQ([editingState[@"selectionAffinity"] UTF8String], "TextAffinity.upstream");
  EXPECT_FALSE([editingState[@"selectionIsDirectional"] boolValue]);
  EXPECT_EQ([editingState[@"selectionBase"] intValue], 2);
  EXPECT_EQ([editingState[@"selectionExtent"] intValue], 2);
  EXPECT_EQ([editingState[@"composingBase"] intValue], -1);
  EXPECT_EQ([editingState[@"composingExtent"] intValue], -1);
  return true;
}

- (bool)testClearClientDuringComposing {
  // Set up FlutterTextInputPlugin.
  id engineMock = flutter::testing::CreateMockFlutterEngine(@"");
  id binaryMessengerMock = OCMProtocolMock(@protocol(FlutterBinaryMessenger));
  OCMStub(  // NOLINT(google-objc-avoid-throwing-exception)
      [engineMock binaryMessenger])
      .andReturn(binaryMessengerMock);
  FlutterViewController* viewController = [[FlutterViewController alloc] initWithEngine:engineMock
                                                                                nibName:@""
                                                                                 bundle:nil];
  FlutterTextInputPlugin* plugin =
      [[FlutterTextInputPlugin alloc] initWithViewController:viewController];

  // Set input client 1.
  NSDictionary* setClientConfig = @{
    @"inputAction" : @"action",
    @"inputType" : @{@"name" : @"inputName"},
  };
  [plugin handleMethodCall:[FlutterMethodCall methodCallWithMethodName:@"TextInput.setClient"
                                                             arguments:@[ @(1), setClientConfig ]]
                    result:^(id){
                    }];

  // Set editing state with an active composing range.
  [plugin handleMethodCall:[FlutterMethodCall methodCallWithMethodName:@"TextInput.setEditingState"
                                                             arguments:@{
                                                               @"text" : @"Text",
                                                               @"selectionBase" : @(0),
                                                               @"selectionExtent" : @(0),
                                                               @"composingBase" : @(0),
                                                               @"composingExtent" : @(1),
                                                             }]
                    result:^(id){
                    }];

  // Verify composing range is (0, 1).
  NSDictionary* editingState = [plugin editingState];
  EXPECT_EQ([editingState[@"composingBase"] intValue], 0);
  EXPECT_EQ([editingState[@"composingExtent"] intValue], 1);

  // Clear input client.
  [plugin handleMethodCall:[FlutterMethodCall methodCallWithMethodName:@"TextInput.clearClient"
                                                             arguments:@[]]
                    result:^(id){
                    }];

  // Verify composing range is collapsed.
  editingState = [plugin editingState];
  EXPECT_EQ([editingState[@"composingBase"] intValue], [editingState[@"composingExtent"] intValue]);
  return true;
}

- (bool)testAutocompleteDisabledWhenAutofillNotSet {
  // Set up FlutterTextInputPlugin.
  id engineMock = flutter::testing::CreateMockFlutterEngine(@"");
  id binaryMessengerMock = OCMProtocolMock(@protocol(FlutterBinaryMessenger));
  OCMStub(  // NOLINT(google-objc-avoid-throwing-exception)
      [engineMock binaryMessenger])
      .andReturn(binaryMessengerMock);
  FlutterViewController* viewController = [[FlutterViewController alloc] initWithEngine:engineMock
                                                                                nibName:@""
                                                                                 bundle:nil];
  FlutterTextInputPlugin* plugin =
      [[FlutterTextInputPlugin alloc] initWithViewController:viewController];

  // Set input client 1.
  NSDictionary* setClientConfig = @{
    @"inputAction" : @"action",
    @"inputType" : @{@"name" : @"inputName"},
  };
  [plugin handleMethodCall:[FlutterMethodCall methodCallWithMethodName:@"TextInput.setClient"
                                                             arguments:@[ @(1), setClientConfig ]]
                    result:^(id){
                    }];

  // Verify autocomplete is disabled.
  EXPECT_FALSE([plugin isAutomaticTextCompletionEnabled]);
  return true;
}

- (bool)testAutocompleteEnabledWhenAutofillSet {
  // Set up FlutterTextInputPlugin.
  id engineMock = flutter::testing::CreateMockFlutterEngine(@"");
  id binaryMessengerMock = OCMProtocolMock(@protocol(FlutterBinaryMessenger));
  OCMStub(  // NOLINT(google-objc-avoid-throwing-exception)
      [engineMock binaryMessenger])
      .andReturn(binaryMessengerMock);
  FlutterViewController* viewController = [[FlutterViewController alloc] initWithEngine:engineMock
                                                                                nibName:@""
                                                                                 bundle:nil];
  FlutterTextInputPlugin* plugin =
      [[FlutterTextInputPlugin alloc] initWithViewController:viewController];

  // Set input client 1.
  NSDictionary* setClientConfig = @{
    @"inputAction" : @"action",
    @"inputType" : @{@"name" : @"inputName"},
    @"autofill" : @{
      @"uniqueIdentifier" : @"field1",
      @"hints" : @[ @"name" ],
      @"editingValue" : @{@"text" : @""},
    }
  };
  [plugin handleMethodCall:[FlutterMethodCall methodCallWithMethodName:@"TextInput.setClient"
                                                             arguments:@[ @(1), setClientConfig ]]
                    result:^(id){
                    }];

  // Verify autocomplete is enabled.
  EXPECT_TRUE([plugin isAutomaticTextCompletionEnabled]);

  // Verify content type is nil for unsupported content types.
  if (@available(macOS 11.0, *)) {
    EXPECT_EQ([plugin contentType], nil);
  }
  return true;
}

- (bool)testAutocompleteEnabledWhenAutofillSetNoHint {
  // Set up FlutterTextInputPlugin.
  id engineMock = flutter::testing::CreateMockFlutterEngine(@"");
  id binaryMessengerMock = OCMProtocolMock(@protocol(FlutterBinaryMessenger));
  OCMStub(  // NOLINT(google-objc-avoid-throwing-exception)
      [engineMock binaryMessenger])
      .andReturn(binaryMessengerMock);
  FlutterViewController* viewController = [[FlutterViewController alloc] initWithEngine:engineMock
                                                                                nibName:@""
                                                                                 bundle:nil];
  FlutterTextInputPlugin* plugin =
      [[FlutterTextInputPlugin alloc] initWithViewController:viewController];

  // Set input client 1.
  NSDictionary* setClientConfig = @{
    @"inputAction" : @"action",
    @"inputType" : @{@"name" : @"inputName"},
    @"autofill" : @{
      @"uniqueIdentifier" : @"field1",
      @"hints" : @[],
      @"editingValue" : @{@"text" : @""},
    }
  };
  [plugin handleMethodCall:[FlutterMethodCall methodCallWithMethodName:@"TextInput.setClient"
                                                             arguments:@[ @(1), setClientConfig ]]
                    result:^(id){
                    }];

  // Verify autocomplete is enabled.
  EXPECT_TRUE([plugin isAutomaticTextCompletionEnabled]);
  return true;
}

- (bool)testAutocompleteDisabledWhenObscureTextSet {
  // Set up FlutterTextInputPlugin.
  id engineMock = flutter::testing::CreateMockFlutterEngine(@"");
  id binaryMessengerMock = OCMProtocolMock(@protocol(FlutterBinaryMessenger));
  OCMStub(  // NOLINT(google-objc-avoid-throwing-exception)
      [engineMock binaryMessenger])
      .andReturn(binaryMessengerMock);
  FlutterViewController* viewController = [[FlutterViewController alloc] initWithEngine:engineMock
                                                                                nibName:@""
                                                                                 bundle:nil];
  FlutterTextInputPlugin* plugin =
      [[FlutterTextInputPlugin alloc] initWithViewController:viewController];

  // Set input client 1.
  NSDictionary* setClientConfig = @{
    @"inputAction" : @"action",
    @"inputType" : @{@"name" : @"inputName"},
    @"obscureText" : @YES,
    @"autofill" : @{
      @"uniqueIdentifier" : @"field1",
      @"editingValue" : @{@"text" : @""},
    }
  };
  [plugin handleMethodCall:[FlutterMethodCall methodCallWithMethodName:@"TextInput.setClient"
                                                             arguments:@[ @(1), setClientConfig ]]
                    result:^(id){
                    }];

  // Verify autocomplete is disabled.
  EXPECT_FALSE([plugin isAutomaticTextCompletionEnabled]);
  return true;
}

- (bool)testAutocompleteDisabledWhenPasswordAutofillSet {
  // Set up FlutterTextInputPlugin.
  id engineMock = flutter::testing::CreateMockFlutterEngine(@"");
  id binaryMessengerMock = OCMProtocolMock(@protocol(FlutterBinaryMessenger));
  OCMStub(  // NOLINT(google-objc-avoid-throwing-exception)
      [engineMock binaryMessenger])
      .andReturn(binaryMessengerMock);
  FlutterViewController* viewController = [[FlutterViewController alloc] initWithEngine:engineMock
                                                                                nibName:@""
                                                                                 bundle:nil];
  FlutterTextInputPlugin* plugin =
      [[FlutterTextInputPlugin alloc] initWithViewController:viewController];

  // Set input client 1.
  NSDictionary* setClientConfig = @{
    @"inputAction" : @"action",
    @"inputType" : @{@"name" : @"inputName"},
    @"autofill" : @{
      @"uniqueIdentifier" : @"field1",
      @"hints" : @[ @"password" ],
      @"editingValue" : @{@"text" : @""},
    }
  };
  [plugin handleMethodCall:[FlutterMethodCall methodCallWithMethodName:@"TextInput.setClient"
                                                             arguments:@[ @(1), setClientConfig ]]
                    result:^(id){
                    }];

  // Verify autocomplete is disabled.
  EXPECT_FALSE([plugin isAutomaticTextCompletionEnabled]);

  // Verify content type is password.
  if (@available(macOS 11.0, *)) {
    EXPECT_EQ([plugin contentType], NSTextContentTypePassword);
  }
  return true;
}

- (bool)testAutocompleteDisabledWhenAutofillGroupIncludesPassword {
  // Set up FlutterTextInputPlugin.
  id engineMock = flutter::testing::CreateMockFlutterEngine(@"");
  id binaryMessengerMock = OCMProtocolMock(@protocol(FlutterBinaryMessenger));
  OCMStub(  // NOLINT(google-objc-avoid-throwing-exception)
      [engineMock binaryMessenger])
      .andReturn(binaryMessengerMock);
  FlutterViewController* viewController = [[FlutterViewController alloc] initWithEngine:engineMock
                                                                                nibName:@""
                                                                                 bundle:nil];
  FlutterTextInputPlugin* plugin =
      [[FlutterTextInputPlugin alloc] initWithViewController:viewController];

  // Set input client 1.
  NSDictionary* setClientConfig = @{
    @"inputAction" : @"action",
    @"inputType" : @{@"name" : @"inputName"},
    @"fields" : @[
      @{
        @"inputAction" : @"action",
        @"inputType" : @{@"name" : @"inputName"},
        @"autofill" : @{
          @"uniqueIdentifier" : @"field1",
          @"hints" : @[ @"password" ],
          @"editingValue" : @{@"text" : @""},
        }
      },
      @{
        @"inputAction" : @"action",
        @"inputType" : @{@"name" : @"inputName"},
        @"autofill" : @{
          @"uniqueIdentifier" : @"field2",
          @"hints" : @[ @"name" ],
          @"editingValue" : @{@"text" : @""},
        }
      }
    ]
  };
  [plugin handleMethodCall:[FlutterMethodCall methodCallWithMethodName:@"TextInput.setClient"
                                                             arguments:@[ @(1), setClientConfig ]]
                    result:^(id){
                    }];

  // Verify autocomplete is disabled.
  EXPECT_FALSE([plugin isAutomaticTextCompletionEnabled]);
  return true;
}

- (bool)testContentTypeWhenAutofillTypeIsUsername {
  // Set up FlutterTextInputPlugin.
  id engineMock = flutter::testing::CreateMockFlutterEngine(@"");
  id binaryMessengerMock = OCMProtocolMock(@protocol(FlutterBinaryMessenger));
  OCMStub(  // NOLINT(google-objc-avoid-throwing-exception)
      [engineMock binaryMessenger])
      .andReturn(binaryMessengerMock);
  FlutterViewController* viewController = [[FlutterViewController alloc] initWithEngine:engineMock
                                                                                nibName:@""
                                                                                 bundle:nil];
  FlutterTextInputPlugin* plugin =
      [[FlutterTextInputPlugin alloc] initWithViewController:viewController];

  // Set input client 1.
  NSDictionary* setClientConfig = @{
    @"inputAction" : @"action",
    @"inputType" : @{@"name" : @"inputName"},
    @"autofill" : @{
      @"uniqueIdentifier" : @"field1",
      @"hints" : @[ @"name" ],
      @"editingValue" : @{@"text" : @""},
    }
  };
  [plugin handleMethodCall:[FlutterMethodCall methodCallWithMethodName:@"TextInput.setClient"
                                                             arguments:@[ @(1), setClientConfig ]]
                    result:^(id){
                    }];

  // Verify autocomplete is disabled.
  EXPECT_FALSE([plugin isAutomaticTextCompletionEnabled]);

  // Verify content type is username.
  if (@available(macOS 11.0, *)) {
    EXPECT_EQ([plugin contentType], NSTextContentTypeUsername);
  }
  return true;
}

- (bool)testContentTypeWhenAutofillTypeIsOneTimeCode {
  // Set up FlutterTextInputPlugin.
  id engineMock = flutter::testing::CreateMockFlutterEngine(@"");
  id binaryMessengerMock = OCMProtocolMock(@protocol(FlutterBinaryMessenger));
  OCMStub(  // NOLINT(google-objc-avoid-throwing-exception)
      [engineMock binaryMessenger])
      .andReturn(binaryMessengerMock);
  FlutterViewController* viewController = [[FlutterViewController alloc] initWithEngine:engineMock
                                                                                nibName:@""
                                                                                 bundle:nil];
  FlutterTextInputPlugin* plugin =
      [[FlutterTextInputPlugin alloc] initWithViewController:viewController];

  // Set input client 1.
  NSDictionary* setClientConfig = @{
    @"inputAction" : @"action",
    @"inputType" : @{@"name" : @"inputName"},
    @"autofill" : @{
      @"uniqueIdentifier" : @"field1",
      @"hints" : @[ @"oneTimeCode" ],
      @"editingValue" : @{@"text" : @""},
    }
  };
  [plugin handleMethodCall:[FlutterMethodCall methodCallWithMethodName:@"TextInput.setClient"
                                                             arguments:@[ @(1), setClientConfig ]]
                    result:^(id){
                    }];

  // Verify autocomplete is disabled.
  EXPECT_FALSE([plugin isAutomaticTextCompletionEnabled]);

  // Verify content type is username.
  if (@available(macOS 11.0, *)) {
    EXPECT_EQ([plugin contentType], NSTextContentTypeOneTimeCode);
  }
  return true;
}

- (bool)testFirstRectForCharacterRange {
  id engineMock = flutter::testing::CreateMockFlutterEngine(@"");
  id binaryMessengerMock = OCMProtocolMock(@protocol(FlutterBinaryMessenger));
  OCMStub(  // NOLINT(google-objc-avoid-throwing-exception)
      [engineMock binaryMessenger])
      .andReturn(binaryMessengerMock);
  FlutterViewController* controllerMock =
      [[TextInputTestViewController alloc] initWithEngine:engineMock nibName:nil bundle:nil];
  [controllerMock loadView];
  id viewMock = controllerMock.flutterView;
  OCMStub(  // NOLINT(google-objc-avoid-throwing-exception)
      [viewMock bounds])
      .andReturn(NSMakeRect(0, 0, 200, 200));

  id windowMock = OCMClassMock([NSWindow class]);
  OCMStub(  // NOLINT(google-objc-avoid-throwing-exception)
      [viewMock window])
      .andReturn(windowMock);

  OCMExpect(  // NOLINT(google-objc-avoid-throwing-exception)
      [viewMock convertRect:NSMakeRect(28, 10, 2, 19) toView:nil])
      .andReturn(NSMakeRect(28, 10, 2, 19));

  OCMExpect(  // NOLINT(google-objc-avoid-throwing-exception)
      [windowMock convertRectToScreen:NSMakeRect(28, 10, 2, 19)])
      .andReturn(NSMakeRect(38, 20, 2, 19));

  FlutterTextInputPlugin* plugin =
      [[FlutterTextInputPlugin alloc] initWithViewController:controllerMock];

  FlutterMethodCall* call = [FlutterMethodCall
      methodCallWithMethodName:@"TextInput.setEditableSizeAndTransform"
                     arguments:@{
                       @"height" : @(20.0),
                       @"transform" : @[
                         @(1.0), @(0.0), @(0.0), @(0.0), @(0.0), @(1.0), @(0.0), @(0.0), @(0.0),
                         @(0.0), @(1.0), @(0.0), @(20.0), @(10.0), @(0.0), @(1.0)
                       ],
                       @"width" : @(400.0),
                     }];

  [plugin handleMethodCall:call
                    result:^(id){
                    }];

  call = [FlutterMethodCall methodCallWithMethodName:@"TextInput.setCaretRect"
                                           arguments:@{
                                             @"height" : @(19.0),
                                             @"width" : @(2.0),
                                             @"x" : @(8.0),
                                             @"y" : @(0.0),
                                           }];

  [plugin handleMethodCall:call
                    result:^(id){
                    }];

  NSRect rect = [plugin firstRectForCharacterRange:NSMakeRange(0, 0) actualRange:nullptr];
  @try {
    OCMVerify(  // NOLINT(google-objc-avoid-throwing-exception)
        [windowMock convertRectToScreen:NSMakeRect(28, 10, 2, 19)]);
  } @catch (...) {
    return false;
  }

  return NSEqualRects(rect, NSMakeRect(38, 20, 2, 19));
}

- (bool)testFirstRectForCharacterRangeAtInfinity {
  id engineMock = flutter::testing::CreateMockFlutterEngine(@"");
  id binaryMessengerMock = OCMProtocolMock(@protocol(FlutterBinaryMessenger));
  OCMStub(  // NOLINT(google-objc-avoid-throwing-exception)
      [engineMock binaryMessenger])
      .andReturn(binaryMessengerMock);
  FlutterViewController* controllerMock =
      [[TextInputTestViewController alloc] initWithEngine:engineMock nibName:nil bundle:nil];
  [controllerMock loadView];
  id viewMock = controllerMock.flutterView;
  OCMStub(  // NOLINT(google-objc-avoid-throwing-exception)
      [viewMock bounds])
      .andReturn(NSMakeRect(0, 0, 200, 200));

  id windowMock = OCMClassMock([NSWindow class]);
  OCMStub(  // NOLINT(google-objc-avoid-throwing-exception)
      [viewMock window])
      .andReturn(windowMock);

  FlutterTextInputPlugin* plugin =
      [[FlutterTextInputPlugin alloc] initWithViewController:controllerMock];

  FlutterMethodCall* call = [FlutterMethodCall
      methodCallWithMethodName:@"TextInput.setEditableSizeAndTransform"
                     arguments:@{
                       @"height" : @(20.0),
                       // Projects all points to infinity.
                       @"transform" : @[
                         @(1.0), @(0.0), @(0.0), @(0.0), @(0.0), @(1.0), @(0.0), @(0.0), @(0.0),
                         @(0.0), @(1.0), @(0.0), @(20.0), @(10.0), @(0.0), @(0.0)
                       ],
                       @"width" : @(400.0),
                     }];

  [plugin handleMethodCall:call
                    result:^(id){
                    }];

  call = [FlutterMethodCall methodCallWithMethodName:@"TextInput.setCaretRect"
                                           arguments:@{
                                             @"height" : @(19.0),
                                             @"width" : @(2.0),
                                             @"x" : @(8.0),
                                             @"y" : @(0.0),
                                           }];

  [plugin handleMethodCall:call
                    result:^(id){
                    }];

  NSRect rect = [plugin firstRectForCharacterRange:NSMakeRange(0, 0) actualRange:nullptr];
  return NSEqualRects(rect, CGRectZero);
}

- (bool)testFirstRectForCharacterRangeWithEsotericAffineTransform {
  id engineMock = flutter::testing::CreateMockFlutterEngine(@"");
  id binaryMessengerMock = OCMProtocolMock(@protocol(FlutterBinaryMessenger));
  OCMStub(  // NOLINT(google-objc-avoid-throwing-exception)
      [engineMock binaryMessenger])
      .andReturn(binaryMessengerMock);
  FlutterViewController* controllerMock =
      [[TextInputTestViewController alloc] initWithEngine:engineMock nibName:nil bundle:nil];
  [controllerMock loadView];
  id viewMock = controllerMock.flutterView;
  OCMStub(  // NOLINT(google-objc-avoid-throwing-exception)
      [viewMock bounds])
      .andReturn(NSMakeRect(0, 0, 200, 200));

  id windowMock = OCMClassMock([NSWindow class]);
  OCMStub(  // NOLINT(google-objc-avoid-throwing-exception)
      [viewMock window])
      .andReturn(windowMock);

  OCMExpect(  // NOLINT(google-objc-avoid-throwing-exception)
      [viewMock convertRect:NSMakeRect(-18, 6, 3, 3) toView:nil])
      .andReturn(NSMakeRect(-18, 6, 3, 3));

  OCMExpect(  // NOLINT(google-objc-avoid-throwing-exception)
      [windowMock convertRectToScreen:NSMakeRect(-18, 6, 3, 3)])
      .andReturn(NSMakeRect(-18, 6, 3, 3));

  FlutterTextInputPlugin* plugin =
      [[FlutterTextInputPlugin alloc] initWithViewController:controllerMock];

  FlutterMethodCall* call = [FlutterMethodCall
      methodCallWithMethodName:@"TextInput.setEditableSizeAndTransform"
                     arguments:@{
                       @"height" : @(20.0),
                       // This matrix can be generated by running this dart code snippet:
                       // Matrix4.identity()..scale(3.0)..rotateZ(math.pi/2)..translate(1.0, 2.0,
                       // 3.0);
                       @"transform" : @[
                         @(0.0), @(3.0), @(0.0), @(0.0), @(-3.0), @(0.0), @(0.0), @(0.0), @(0.0),
                         @(0.0), @(3.0), @(0.0), @(-6.0), @(3.0), @(9.0), @(1.0)
                       ],
                       @"width" : @(400.0),
                     }];

  [plugin handleMethodCall:call
                    result:^(id){
                    }];

  call = [FlutterMethodCall methodCallWithMethodName:@"TextInput.setCaretRect"
                                           arguments:@{
                                             @"height" : @(1.0),
                                             @"width" : @(1.0),
                                             @"x" : @(1.0),
                                             @"y" : @(3.0),
                                           }];

  [plugin handleMethodCall:call
                    result:^(id){
                    }];

  NSRect rect = [plugin firstRectForCharacterRange:NSMakeRange(0, 0) actualRange:nullptr];

  @try {
    OCMVerify(  // NOLINT(google-objc-avoid-throwing-exception)
        [windowMock convertRectToScreen:NSMakeRect(-18, 6, 3, 3)]);
  } @catch (...) {
    return false;
  }

  return NSEqualRects(rect, NSMakeRect(-18, 6, 3, 3));
}

- (bool)testSetEditingStateWithTextEditingDelta {
  id engineMock = flutter::testing::CreateMockFlutterEngine(@"");
  id binaryMessengerMock = OCMProtocolMock(@protocol(FlutterBinaryMessenger));
  OCMStub(  // NOLINT(google-objc-avoid-throwing-exception)
      [engineMock binaryMessenger])
      .andReturn(binaryMessengerMock);

  FlutterViewController* viewController = [[FlutterViewController alloc] initWithEngine:engineMock
                                                                                nibName:@""
                                                                                 bundle:nil];

  FlutterTextInputPlugin* plugin =
      [[FlutterTextInputPlugin alloc] initWithViewController:viewController];

  NSDictionary* setClientConfig = @{
    @"inputAction" : @"action",
    @"enableDeltaModel" : @"true",
    @"inputType" : @{@"name" : @"inputName"},
  };
  [plugin handleMethodCall:[FlutterMethodCall methodCallWithMethodName:@"TextInput.setClient"
                                                             arguments:@[ @(1), setClientConfig ]]
                    result:^(id){
                    }];

  FlutterMethodCall* call = [FlutterMethodCall methodCallWithMethodName:@"TextInput.setEditingState"
                                                              arguments:@{
                                                                @"text" : @"Text",
                                                                @"selectionBase" : @(0),
                                                                @"selectionExtent" : @(0),
                                                                @"composingBase" : @(-1),
                                                                @"composingExtent" : @(-1),
                                                              }];

  [plugin handleMethodCall:call
                    result:^(id){
                    }];

  // Verify editing state was set.
  NSDictionary* editingState = [plugin editingState];
  EXPECT_STREQ([editingState[@"text"] UTF8String], "Text");
  EXPECT_STREQ([editingState[@"selectionAffinity"] UTF8String], "TextAffinity.upstream");
  EXPECT_FALSE([editingState[@"selectionIsDirectional"] boolValue]);
  EXPECT_EQ([editingState[@"selectionBase"] intValue], 0);
  EXPECT_EQ([editingState[@"selectionExtent"] intValue], 0);
  EXPECT_EQ([editingState[@"composingBase"] intValue], -1);
  EXPECT_EQ([editingState[@"composingExtent"] intValue], -1);
  return true;
}

- (bool)testOperationsThatTriggerDelta {
  id engineMock = flutter::testing::CreateMockFlutterEngine(@"");
  id binaryMessengerMock = OCMProtocolMock(@protocol(FlutterBinaryMessenger));
  OCMStub(  // NOLINT(google-objc-avoid-throwing-exception)
      [engineMock binaryMessenger])
      .andReturn(binaryMessengerMock);

  FlutterViewController* viewController = [[FlutterViewController alloc] initWithEngine:engineMock
                                                                                nibName:@""
                                                                                 bundle:nil];

  FlutterTextInputPlugin* plugin =
      [[FlutterTextInputPlugin alloc] initWithViewController:viewController];

  NSDictionary* setClientConfig = @{
    @"inputAction" : @"action",
    @"enableDeltaModel" : @"true",
    @"inputType" : @{@"name" : @"inputName"},
  };
  [plugin handleMethodCall:[FlutterMethodCall methodCallWithMethodName:@"TextInput.setClient"
                                                             arguments:@[ @(1), setClientConfig ]]
                    result:^(id){
                    }];
  [plugin insertText:@"text to insert"];

  NSDictionary* deltaToFramework = @{
    @"oldText" : @"",
    @"deltaText" : @"text to insert",
    @"deltaStart" : @(0),
    @"deltaEnd" : @(0),
    @"selectionBase" : @(14),
    @"selectionExtent" : @(14),
    @"selectionAffinity" : @"TextAffinity.upstream",
    @"selectionIsDirectional" : @(false),
    @"composingBase" : @(-1),
    @"composingExtent" : @(-1),
  };
  NSDictionary* expectedState = @{
    @"deltas" : @[ deltaToFramework ],
  };

  NSData* updateCall = [[FlutterJSONMethodCodec sharedInstance]
      encodeMethodCall:[FlutterMethodCall
                           methodCallWithMethodName:@"TextInputClient.updateEditingStateWithDeltas"
                                          arguments:@[ @(1), expectedState ]]];

  @try {
    OCMVerify(  // NOLINT(google-objc-avoid-throwing-exception)
        [binaryMessengerMock sendOnChannel:@"flutter/textinput" message:updateCall]);
  } @catch (...) {
    return false;
  }

  [plugin setMarkedText:@"marked text" selectedRange:NSMakeRange(0, 1)];

  deltaToFramework = @{
    @"oldText" : @"text to insert",
    @"deltaText" : @"marked text",
    @"deltaStart" : @(14),
    @"deltaEnd" : @(14),
    @"selectionBase" : @(14),
    @"selectionExtent" : @(15),
    @"selectionAffinity" : @"TextAffinity.upstream",
    @"selectionIsDirectional" : @(false),
    @"composingBase" : @(14),
    @"composingExtent" : @(25),
  };
  expectedState = @{
    @"deltas" : @[ deltaToFramework ],
  };

  updateCall = [[FlutterJSONMethodCodec sharedInstance]
      encodeMethodCall:[FlutterMethodCall
                           methodCallWithMethodName:@"TextInputClient.updateEditingStateWithDeltas"
                                          arguments:@[ @(1), expectedState ]]];

  @try {
    OCMVerify(  // NOLINT(google-objc-avoid-throwing-exception)
        [binaryMessengerMock sendOnChannel:@"flutter/textinput" message:updateCall]);
  } @catch (...) {
    return false;
  }

  [plugin unmarkText];

  deltaToFramework = @{
    @"oldText" : @"text to insertmarked text",
    @"deltaText" : @"",
    @"deltaStart" : @(-1),
    @"deltaEnd" : @(-1),
    @"selectionBase" : @(25),
    @"selectionExtent" : @(25),
    @"selectionAffinity" : @"TextAffinity.upstream",
    @"selectionIsDirectional" : @(false),
    @"composingBase" : @(-1),
    @"composingExtent" : @(-1),
  };
  expectedState = @{
    @"deltas" : @[ deltaToFramework ],
  };

  updateCall = [[FlutterJSONMethodCodec sharedInstance]
      encodeMethodCall:[FlutterMethodCall
                           methodCallWithMethodName:@"TextInputClient.updateEditingStateWithDeltas"
                                          arguments:@[ @(1), expectedState ]]];

  @try {
    OCMVerify(  // NOLINT(google-objc-avoid-throwing-exception)
        [binaryMessengerMock sendOnChannel:@"flutter/textinput" message:updateCall]);
  } @catch (...) {
    return false;
  }
  return true;
}

- (bool)testComposingWithDelta {
  id engineMock = flutter::testing::CreateMockFlutterEngine(@"");
  id binaryMessengerMock = OCMProtocolMock(@protocol(FlutterBinaryMessenger));
  OCMStub(  // NOLINT(google-objc-avoid-throwing-exception)
      [engineMock binaryMessenger])
      .andReturn(binaryMessengerMock);

  FlutterViewController* viewController = [[FlutterViewController alloc] initWithEngine:engineMock
                                                                                nibName:@""
                                                                                 bundle:nil];

  FlutterTextInputPlugin* plugin =
      [[FlutterTextInputPlugin alloc] initWithViewController:viewController];

  NSDictionary* setClientConfig = @{
    @"inputAction" : @"action",
    @"enableDeltaModel" : @"true",
    @"inputType" : @{@"name" : @"inputName"},
  };
  [plugin handleMethodCall:[FlutterMethodCall methodCallWithMethodName:@"TextInput.setClient"
                                                             arguments:@[ @(1), setClientConfig ]]
                    result:^(id){
                    }];
  [plugin setMarkedText:@"m" selectedRange:NSMakeRange(0, 1)];

  NSDictionary* deltaToFramework = @{
    @"oldText" : @"",
    @"deltaText" : @"m",
    @"deltaStart" : @(0),
    @"deltaEnd" : @(0),
    @"selectionBase" : @(0),
    @"selectionExtent" : @(1),
    @"selectionAffinity" : @"TextAffinity.upstream",
    @"selectionIsDirectional" : @(false),
    @"composingBase" : @(0),
    @"composingExtent" : @(1),
  };
  NSDictionary* expectedState = @{
    @"deltas" : @[ deltaToFramework ],
  };

  NSData* updateCall = [[FlutterJSONMethodCodec sharedInstance]
      encodeMethodCall:[FlutterMethodCall
                           methodCallWithMethodName:@"TextInputClient.updateEditingStateWithDeltas"
                                          arguments:@[ @(1), expectedState ]]];

  @try {
    OCMVerify(  // NOLINT(google-objc-avoid-throwing-exception)
        [binaryMessengerMock sendOnChannel:@"flutter/textinput" message:updateCall]);
  } @catch (...) {
    return false;
  }

  [plugin setMarkedText:@"ma" selectedRange:NSMakeRange(0, 1)];

  deltaToFramework = @{
    @"oldText" : @"m",
    @"deltaText" : @"ma",
    @"deltaStart" : @(0),
    @"deltaEnd" : @(1),
    @"selectionBase" : @(0),
    @"selectionExtent" : @(1),
    @"selectionAffinity" : @"TextAffinity.upstream",
    @"selectionIsDirectional" : @(false),
    @"composingBase" : @(0),
    @"composingExtent" : @(2),
  };
  expectedState = @{
    @"deltas" : @[ deltaToFramework ],
  };

  updateCall = [[FlutterJSONMethodCodec sharedInstance]
      encodeMethodCall:[FlutterMethodCall
                           methodCallWithMethodName:@"TextInputClient.updateEditingStateWithDeltas"
                                          arguments:@[ @(1), expectedState ]]];

  @try {
    OCMVerify(  // NOLINT(google-objc-avoid-throwing-exception)
        [binaryMessengerMock sendOnChannel:@"flutter/textinput" message:updateCall]);
  } @catch (...) {
    return false;
  }

  [plugin setMarkedText:@"mar" selectedRange:NSMakeRange(0, 1)];

  deltaToFramework = @{
    @"oldText" : @"ma",
    @"deltaText" : @"mar",
    @"deltaStart" : @(0),
    @"deltaEnd" : @(2),
    @"selectionBase" : @(0),
    @"selectionExtent" : @(1),
    @"selectionAffinity" : @"TextAffinity.upstream",
    @"selectionIsDirectional" : @(false),
    @"composingBase" : @(0),
    @"composingExtent" : @(3),
  };
  expectedState = @{
    @"deltas" : @[ deltaToFramework ],
  };

  updateCall = [[FlutterJSONMethodCodec sharedInstance]
      encodeMethodCall:[FlutterMethodCall
                           methodCallWithMethodName:@"TextInputClient.updateEditingStateWithDeltas"
                                          arguments:@[ @(1), expectedState ]]];

  @try {
    OCMVerify(  // NOLINT(google-objc-avoid-throwing-exception)
        [binaryMessengerMock sendOnChannel:@"flutter/textinput" message:updateCall]);
  } @catch (...) {
    return false;
  }

  [plugin setMarkedText:@"mark" selectedRange:NSMakeRange(0, 1)];

  deltaToFramework = @{
    @"oldText" : @"mar",
    @"deltaText" : @"mark",
    @"deltaStart" : @(0),
    @"deltaEnd" : @(3),
    @"selectionBase" : @(0),
    @"selectionExtent" : @(1),
    @"selectionAffinity" : @"TextAffinity.upstream",
    @"selectionIsDirectional" : @(false),
    @"composingBase" : @(0),
    @"composingExtent" : @(4),
  };
  expectedState = @{
    @"deltas" : @[ deltaToFramework ],
  };

  updateCall = [[FlutterJSONMethodCodec sharedInstance]
      encodeMethodCall:[FlutterMethodCall
                           methodCallWithMethodName:@"TextInputClient.updateEditingStateWithDeltas"
                                          arguments:@[ @(1), expectedState ]]];

  @try {
    OCMVerify(  // NOLINT(google-objc-avoid-throwing-exception)
        [binaryMessengerMock sendOnChannel:@"flutter/textinput" message:updateCall]);
  } @catch (...) {
    return false;
  }

  [plugin setMarkedText:@"marke" selectedRange:NSMakeRange(0, 1)];

  deltaToFramework = @{
    @"oldText" : @"mark",
    @"deltaText" : @"marke",
    @"deltaStart" : @(0),
    @"deltaEnd" : @(4),
    @"selectionBase" : @(0),
    @"selectionExtent" : @(1),
    @"selectionAffinity" : @"TextAffinity.upstream",
    @"selectionIsDirectional" : @(false),
    @"composingBase" : @(0),
    @"composingExtent" : @(5),
  };
  expectedState = @{
    @"deltas" : @[ deltaToFramework ],
  };

  updateCall = [[FlutterJSONMethodCodec sharedInstance]
      encodeMethodCall:[FlutterMethodCall
                           methodCallWithMethodName:@"TextInputClient.updateEditingStateWithDeltas"
                                          arguments:@[ @(1), expectedState ]]];

  @try {
    OCMVerify(  // NOLINT(google-objc-avoid-throwing-exception)
        [binaryMessengerMock sendOnChannel:@"flutter/textinput" message:updateCall]);
  } @catch (...) {
    return false;
  }

  [plugin setMarkedText:@"marked" selectedRange:NSMakeRange(0, 1)];

  deltaToFramework = @{
    @"oldText" : @"marke",
    @"deltaText" : @"marked",
    @"deltaStart" : @(0),
    @"deltaEnd" : @(5),
    @"selectionBase" : @(0),
    @"selectionExtent" : @(1),
    @"selectionAffinity" : @"TextAffinity.upstream",
    @"selectionIsDirectional" : @(false),
    @"composingBase" : @(0),
    @"composingExtent" : @(6),
  };
  expectedState = @{
    @"deltas" : @[ deltaToFramework ],
  };

  updateCall = [[FlutterJSONMethodCodec sharedInstance]
      encodeMethodCall:[FlutterMethodCall
                           methodCallWithMethodName:@"TextInputClient.updateEditingStateWithDeltas"
                                          arguments:@[ @(1), expectedState ]]];

  @try {
    OCMVerify(  // NOLINT(google-objc-avoid-throwing-exception)
        [binaryMessengerMock sendOnChannel:@"flutter/textinput" message:updateCall]);
  } @catch (...) {
    return false;
  }

  [plugin unmarkText];

  deltaToFramework = @{
    @"oldText" : @"marked",
    @"deltaText" : @"",
    @"deltaStart" : @(-1),
    @"deltaEnd" : @(-1),
    @"selectionBase" : @(6),
    @"selectionExtent" : @(6),
    @"selectionAffinity" : @"TextAffinity.upstream",
    @"selectionIsDirectional" : @(false),
    @"composingBase" : @(-1),
    @"composingExtent" : @(-1),
  };
  expectedState = @{
    @"deltas" : @[ deltaToFramework ],
  };

  updateCall = [[FlutterJSONMethodCodec sharedInstance]
      encodeMethodCall:[FlutterMethodCall
                           methodCallWithMethodName:@"TextInputClient.updateEditingStateWithDeltas"
                                          arguments:@[ @(1), expectedState ]]];

  @try {
    OCMVerify(  // NOLINT(google-objc-avoid-throwing-exception)
        [binaryMessengerMock sendOnChannel:@"flutter/textinput" message:updateCall]);
  } @catch (...) {
    return false;
  }
  return true;
}

- (bool)testComposingWithDeltasWhenSelectionIsActive {
  id engineMock = flutter::testing::CreateMockFlutterEngine(@"");
  id binaryMessengerMock = OCMProtocolMock(@protocol(FlutterBinaryMessenger));
  OCMStub(  // NOLINT(google-objc-avoid-throwing-exception)
      [engineMock binaryMessenger])
      .andReturn(binaryMessengerMock);

  FlutterViewController* viewController = [[FlutterViewController alloc] initWithEngine:engineMock
                                                                                nibName:@""
                                                                                 bundle:nil];

  FlutterTextInputPlugin* plugin =
      [[FlutterTextInputPlugin alloc] initWithViewController:viewController];

  NSDictionary* setClientConfig = @{
    @"inputAction" : @"action",
    @"enableDeltaModel" : @"true",
    @"inputType" : @{@"name" : @"inputName"},
  };
  [plugin handleMethodCall:[FlutterMethodCall methodCallWithMethodName:@"TextInput.setClient"
                                                             arguments:@[ @(1), setClientConfig ]]
                    result:^(id){
                    }];

  FlutterMethodCall* call = [FlutterMethodCall methodCallWithMethodName:@"TextInput.setEditingState"
                                                              arguments:@{
                                                                @"text" : @"Text",
                                                                @"selectionBase" : @(0),
                                                                @"selectionExtent" : @(4),
                                                                @"composingBase" : @(-1),
                                                                @"composingExtent" : @(-1),
                                                              }];
  [plugin handleMethodCall:call
                    result:^(id){
                    }];

  [plugin setMarkedText:@"~"
          selectedRange:NSMakeRange(1, 0)
       replacementRange:NSMakeRange(NSNotFound, 0)];

  NSDictionary* deltaToFramework = @{
    @"oldText" : @"Text",
    @"deltaText" : @"~",
    @"deltaStart" : @(0),
    @"deltaEnd" : @(4),
    @"selectionBase" : @(1),
    @"selectionExtent" : @(1),
    @"selectionAffinity" : @"TextAffinity.upstream",
    @"selectionIsDirectional" : @(false),
    @"composingBase" : @(0),
    @"composingExtent" : @(1),
  };
  NSDictionary* expectedState = @{
    @"deltas" : @[ deltaToFramework ],
  };

  NSData* updateCall = [[FlutterJSONMethodCodec sharedInstance]
      encodeMethodCall:[FlutterMethodCall
                           methodCallWithMethodName:@"TextInputClient.updateEditingStateWithDeltas"
                                          arguments:@[ @(1), expectedState ]]];

  @try {
    OCMVerify(  // NOLINT(google-objc-avoid-throwing-exception)
        [binaryMessengerMock sendOnChannel:@"flutter/textinput" message:updateCall]);
  } @catch (...) {
    return false;
  }
  return true;
}

- (bool)testPerformKeyEquivalent {
  __block NSEvent* eventBeingDispatchedByKeyboardManager = nil;
  FlutterViewController* viewControllerMock = OCMClassMock([FlutterViewController class]);
  OCMStub([viewControllerMock isDispatchingKeyEvent:[OCMArg any]])
      .andDo(^(NSInvocation* invocation) {
        NSEvent* event;
        [invocation getArgument:(void*)&event atIndex:2];
        BOOL result = event == eventBeingDispatchedByKeyboardManager;
        [invocation setReturnValue:&result];
      });

  NSEvent* event = [NSEvent keyEventWithType:NSEventTypeKeyDown
                                    location:NSZeroPoint
                               modifierFlags:0x100
                                   timestamp:0
                                windowNumber:0
                                     context:nil
                                  characters:@""
                 charactersIgnoringModifiers:@""
                                   isARepeat:NO
                                     keyCode:0x50];

  FlutterTextInputPlugin* plugin =
      [[FlutterTextInputPlugin alloc] initWithViewController:viewControllerMock];

  OCMExpect([viewControllerMock keyDown:event]);

  // Require that event is handled (returns YES)
  if (![plugin performKeyEquivalent:event]) {
    return false;
  };

  @try {
    OCMVerify(  // NOLINT(google-objc-avoid-throwing-exception)
        [viewControllerMock keyDown:event]);
  } @catch (...) {
    return false;
  }

  // performKeyEquivalent must not forward event if it is being
  // dispatched by keyboard manager
  eventBeingDispatchedByKeyboardManager = event;

  OCMReject([viewControllerMock keyDown:event]);
  @try {
    // Require that event is not handled (returns NO) and not
    // forwarded to controller
    if ([plugin performKeyEquivalent:event]) {
      return false;
    };
  } @catch (...) {
    return false;
  }

  return true;
}

- (bool)handleArrowKeyWhenImePopoverIsActive {
  id engineMock = flutter::testing::CreateMockFlutterEngine(@"");
  id binaryMessengerMock = OCMProtocolMock(@protocol(FlutterBinaryMessenger));
  OCMStub(  // NOLINT(google-objc-avoid-throwing-exception)
      [engineMock binaryMessenger])
      .andReturn(binaryMessengerMock);
  OCMStub([[engineMock ignoringNonObjectArgs] sendKeyEvent:FlutterKeyEvent {}
                                                  callback:nil
                                                  userData:nil]);

  NSTextInputContext* textInputContext = OCMClassMock([NSTextInputContext class]);
  OCMStub([textInputContext handleEvent:[OCMArg any]]).andReturn(YES);

  FlutterViewController* viewController = [[FlutterViewController alloc] initWithEngine:engineMock
                                                                                nibName:@""
                                                                                 bundle:nil];

  FlutterTextInputPlugin* plugin =
      [[FlutterTextInputPlugin alloc] initWithViewController:viewController];

  plugin.textInputContext = textInputContext;

  NSDictionary* setClientConfig = @{
    @"inputAction" : @"action",
    @"enableDeltaModel" : @"true",
    @"inputType" : @{@"name" : @"inputName"},
  };
  [plugin handleMethodCall:[FlutterMethodCall methodCallWithMethodName:@"TextInput.setClient"
                                                             arguments:@[ @(1), setClientConfig ]]
                    result:^(id){
                    }];

  [plugin handleMethodCall:[FlutterMethodCall methodCallWithMethodName:@"TextInput.show"
                                                             arguments:@[]]
                    result:^(id){
                    }];

  // Set marked text, simulate active IME popover.
  [plugin setMarkedText:@"m"
          selectedRange:NSMakeRange(0, 1)
       replacementRange:NSMakeRange(NSNotFound, 0)];

  // Right arrow key. This, unlike the key below should be handled by the plugin.
  NSEvent* event = [NSEvent keyEventWithType:NSEventTypeKeyDown
                                    location:NSZeroPoint
                               modifierFlags:0xa00100
                                   timestamp:0
                                windowNumber:0
                                     context:nil
                                  characters:@"\uF702"
                 charactersIgnoringModifiers:@"\uF702"
                                   isARepeat:NO
                                     keyCode:0x4];

  // Plugin should mark the event as key equivalent.
  [plugin performKeyEquivalent:event];

  if ([plugin handleKeyEvent:event] != true) {
    return false;
  }

  // CTRL+H (delete backwards)
  event = [NSEvent keyEventWithType:NSEventTypeKeyDown
                           location:NSZeroPoint
                      modifierFlags:0x40101
                          timestamp:0
                       windowNumber:0
                            context:nil
                         characters:@"\uF702"
        charactersIgnoringModifiers:@"\uF702"
                          isARepeat:NO
                            keyCode:0x4];

  // Plugin should mark the event as key equivalent.
  [plugin performKeyEquivalent:event];

  if ([plugin handleKeyEvent:event] != false) {
    return false;
  }

  return true;
}

- (bool)unhandledKeyEquivalent {
  id engineMock = flutter::testing::CreateMockFlutterEngine(@"");
  id binaryMessengerMock = OCMProtocolMock(@protocol(FlutterBinaryMessenger));
  OCMStub(  // NOLINT(google-objc-avoid-throwing-exception)
      [engineMock binaryMessenger])
      .andReturn(binaryMessengerMock);

  FlutterViewController* viewController = [[FlutterViewController alloc] initWithEngine:engineMock
                                                                                nibName:@""
                                                                                 bundle:nil];

  FlutterTextInputPlugin* plugin =
      [[FlutterTextInputPlugin alloc] initWithViewController:viewController];

  NSDictionary* setClientConfig = @{
    @"inputAction" : @"action",
    @"enableDeltaModel" : @"true",
    @"inputType" : @{@"name" : @"inputName"},
  };
  [plugin handleMethodCall:[FlutterMethodCall methodCallWithMethodName:@"TextInput.setClient"
                                                             arguments:@[ @(1), setClientConfig ]]
                    result:^(id){
                    }];

  [plugin handleMethodCall:[FlutterMethodCall methodCallWithMethodName:@"TextInput.show"
                                                             arguments:@[]]
                    result:^(id){
                    }];

  // CTRL+H (delete backwards)
  NSEvent* event = [NSEvent keyEventWithType:NSEventTypeKeyDown
                                    location:NSZeroPoint
                               modifierFlags:0x40101
                                   timestamp:0
                                windowNumber:0
                                     context:nil
                                  characters:@""
                 charactersIgnoringModifiers:@"h"
                                   isARepeat:NO
                                     keyCode:0x4];

  // Plugin should mark the event as key equivalent.
  [plugin performKeyEquivalent:event];

  // Simulate KeyboardManager sending unhandled event to plugin. This must return
  // true because it is a known editing command.
  if ([plugin handleKeyEvent:event] != true) {
    return false;
  }

  // CMD+W
  event = [NSEvent keyEventWithType:NSEventTypeKeyDown
                           location:NSZeroPoint
                      modifierFlags:0x100108
                          timestamp:0
                       windowNumber:0
                            context:nil
                         characters:@"w"
        charactersIgnoringModifiers:@"w"
                          isARepeat:NO
                            keyCode:0x13];

  // Plugin should mark the event as key equivalent.
  [plugin performKeyEquivalent:event];

  // This is not a valid editing command, plugin must return false so that
  // KeyboardManager sends the event to next responder.
  if ([plugin handleKeyEvent:event] != false) {
    return false;
  }

  return true;
}

- (bool)testInsertNewLine {
  id engineMock = flutter::testing::CreateMockFlutterEngine(@"");
  id binaryMessengerMock = OCMProtocolMock(@protocol(FlutterBinaryMessenger));
  OCMStub(  // NOLINT(google-objc-avoid-throwing-exception)
      [engineMock binaryMessenger])
      .andReturn(binaryMessengerMock);
  OCMStub([[engineMock ignoringNonObjectArgs] sendKeyEvent:FlutterKeyEvent {}
                                                  callback:nil
                                                  userData:nil]);

  FlutterViewController* viewController = [[FlutterViewController alloc] initWithEngine:engineMock
                                                                                nibName:@""
                                                                                 bundle:nil];

  FlutterTextInputPlugin* plugin =
      [[FlutterTextInputPlugin alloc] initWithViewController:viewController];

  NSDictionary* setClientConfig = @{
    @"inputType" : @{@"name" : @"TextInputType.multiline"},
    @"inputAction" : @"TextInputAction.newline",
  };
  [plugin handleMethodCall:[FlutterMethodCall methodCallWithMethodName:@"TextInput.setClient"
                                                             arguments:@[ @(1), setClientConfig ]]
                    result:^(id){
                    }];

  FlutterMethodCall* call = [FlutterMethodCall methodCallWithMethodName:@"TextInput.setEditingState"
                                                              arguments:@{
                                                                @"text" : @"Text",
                                                                @"selectionBase" : @(4),
                                                                @"selectionExtent" : @(4),
                                                                @"composingBase" : @(-1),
                                                                @"composingExtent" : @(-1),
                                                              }];

  [plugin handleMethodCall:call
                    result:^(id){
                    }];

  // Verify editing state was set.
  NSDictionary* editingState = [plugin editingState];
  EXPECT_STREQ([editingState[@"text"] UTF8String], "Text");
  EXPECT_STREQ([editingState[@"selectionAffinity"] UTF8String], "TextAffinity.upstream");
  EXPECT_FALSE([editingState[@"selectionIsDirectional"] boolValue]);
  EXPECT_EQ([editingState[@"selectionBase"] intValue], 4);
  EXPECT_EQ([editingState[@"selectionExtent"] intValue], 4);
  EXPECT_EQ([editingState[@"composingBase"] intValue], -1);
  EXPECT_EQ([editingState[@"composingExtent"] intValue], -1);

  [plugin doCommandBySelector:@selector(insertNewline:)];

  // Verify editing state was set.
  editingState = [plugin editingState];
  EXPECT_STREQ([editingState[@"text"] UTF8String], "Text\n");
  EXPECT_STREQ([editingState[@"selectionAffinity"] UTF8String], "TextAffinity.upstream");
  EXPECT_FALSE([editingState[@"selectionIsDirectional"] boolValue]);
  EXPECT_EQ([editingState[@"selectionBase"] intValue], 5);
  EXPECT_EQ([editingState[@"selectionExtent"] intValue], 5);
  EXPECT_EQ([editingState[@"composingBase"] intValue], -1);
  EXPECT_EQ([editingState[@"composingExtent"] intValue], -1);

  return true;
}

- (bool)testSendActionDoNotInsertNewLine {
  id engineMock = flutter::testing::CreateMockFlutterEngine(@"");
  id binaryMessengerMock = OCMProtocolMock(@protocol(FlutterBinaryMessenger));
  OCMStub(  // NOLINT(google-objc-avoid-throwing-exception)
      [engineMock binaryMessenger])
      .andReturn(binaryMessengerMock);
  OCMStub([[engineMock ignoringNonObjectArgs] sendKeyEvent:FlutterKeyEvent {}
                                                  callback:nil
                                                  userData:nil]);

  FlutterViewController* viewController = [[FlutterViewController alloc] initWithEngine:engineMock
                                                                                nibName:@""
                                                                                 bundle:nil];

  FlutterTextInputPlugin* plugin =
      [[FlutterTextInputPlugin alloc] initWithViewController:viewController];

  NSDictionary* setClientConfig = @{
    @"inputType" : @{@"name" : @"TextInputType.multiline"},
    @"inputAction" : @"TextInputAction.send",
  };
  [plugin handleMethodCall:[FlutterMethodCall methodCallWithMethodName:@"TextInput.setClient"
                                                             arguments:@[ @(1), setClientConfig ]]
                    result:^(id){
                    }];

  FlutterMethodCall* call = [FlutterMethodCall methodCallWithMethodName:@"TextInput.setEditingState"
                                                              arguments:@{
                                                                @"text" : @"Text",
                                                                @"selectionBase" : @(4),
                                                                @"selectionExtent" : @(4),
                                                                @"composingBase" : @(-1),
                                                                @"composingExtent" : @(-1),
                                                              }];

  NSDictionary* expectedState = @{
    @"selectionBase" : @(4),
    @"selectionExtent" : @(4),
    @"selectionAffinity" : @"TextAffinity.upstream",
    @"selectionIsDirectional" : @(NO),
    @"composingBase" : @(-1),
    @"composingExtent" : @(-1),
    @"text" : @"Text",
  };

  NSData* updateCall = [[FlutterJSONMethodCodec sharedInstance]
      encodeMethodCall:[FlutterMethodCall
                           methodCallWithMethodName:@"TextInputClient.updateEditingState"
                                          arguments:@[ @(1), expectedState ]]];

  OCMExpect(  // NOLINT(google-objc-avoid-throwing-exception)
      [binaryMessengerMock sendOnChannel:@"flutter/textinput" message:updateCall]);

  [plugin handleMethodCall:call
                    result:^(id){
                    }];

  [plugin doCommandBySelector:@selector(insertNewline:)];

  NSData* performActionCall = [[FlutterJSONMethodCodec sharedInstance]
      encodeMethodCall:[FlutterMethodCall
                           methodCallWithMethodName:@"TextInputClient.performAction"
                                          arguments:@[ @(1), @"TextInputAction.send" ]]];

  // Input action should be notified.
  @try {
    OCMVerify(  // NOLINT(google-objc-avoid-throwing-exception)
        [binaryMessengerMock sendOnChannel:@"flutter/textinput" message:performActionCall]);
  } @catch (...) {
    return false;
  }

  NSDictionary* updatedState = @{
    @"selectionBase" : @(5),
    @"selectionExtent" : @(5),
    @"selectionAffinity" : @"TextAffinity.upstream",
    @"selectionIsDirectional" : @(NO),
    @"composingBase" : @(-1),
    @"composingExtent" : @(-1),
    @"text" : @"Text\n",
  };

  updateCall = [[FlutterJSONMethodCodec sharedInstance]
      encodeMethodCall:[FlutterMethodCall
                           methodCallWithMethodName:@"TextInputClient.updateEditingState"
                                          arguments:@[ @(1), updatedState ]]];

  // Verify that editing state was not be updated.
  @try {
    OCMVerify(  // NOLINT(google-objc-avoid-throwing-exception)
        [binaryMessengerMock sendOnChannel:@"flutter/textinput" message:updateCall]);
    return false;
  } @catch (...) {
    // Expected.
  }

  return true;
}

- (bool)testLocalTextAndSelectionUpdateAfterDelta {
  id engineMock = flutter::testing::CreateMockFlutterEngine(@"");
  id binaryMessengerMock = OCMProtocolMock(@protocol(FlutterBinaryMessenger));
  OCMStub(  // NOLINT(google-objc-avoid-throwing-exception)
      [engineMock binaryMessenger])
      .andReturn(binaryMessengerMock);

  FlutterViewController* viewController = [[FlutterViewController alloc] initWithEngine:engineMock
                                                                                nibName:@""
                                                                                 bundle:nil];

  FlutterTextInputPlugin* plugin =
      [[FlutterTextInputPlugin alloc] initWithViewController:viewController];

  NSDictionary* setClientConfig = @{
    @"inputAction" : @"action",
    @"enableDeltaModel" : @"true",
    @"inputType" : @{@"name" : @"inputName"},
  };
  [plugin handleMethodCall:[FlutterMethodCall methodCallWithMethodName:@"TextInput.setClient"
                                                             arguments:@[ @(1), setClientConfig ]]
                    result:^(id){
                    }];
  [plugin insertText:@"text to insert"];

  NSDictionary* deltaToFramework = @{
    @"oldText" : @"",
    @"deltaText" : @"text to insert",
    @"deltaStart" : @(0),
    @"deltaEnd" : @(0),
    @"selectionBase" : @(14),
    @"selectionExtent" : @(14),
    @"selectionAffinity" : @"TextAffinity.upstream",
    @"selectionIsDirectional" : @(false),
    @"composingBase" : @(-1),
    @"composingExtent" : @(-1),
  };
  NSDictionary* expectedState = @{
    @"deltas" : @[ deltaToFramework ],
  };

  NSData* updateCall = [[FlutterJSONMethodCodec sharedInstance]
      encodeMethodCall:[FlutterMethodCall
                           methodCallWithMethodName:@"TextInputClient.updateEditingStateWithDeltas"
                                          arguments:@[ @(1), expectedState ]]];

  @try {
    OCMVerify(  // NOLINT(google-objc-avoid-throwing-exception)
        [binaryMessengerMock sendOnChannel:@"flutter/textinput" message:updateCall]);
  } @catch (...) {
    return false;
  }

  bool localTextAndSelectionUpdated = [plugin.string isEqualToString:@"text to insert"] &&
                                      NSEqualRanges(plugin.selectedRange, NSMakeRange(14, 0));

  return localTextAndSelectionUpdated;
}

- (bool)testSelectorsAreForwardedToFramework {
  id engineMock = flutter::testing::CreateMockFlutterEngine(@"");
  id binaryMessengerMock = OCMProtocolMock(@protocol(FlutterBinaryMessenger));
  OCMStub(  // NOLINT(google-objc-avoid-throwing-exception)
      [engineMock binaryMessenger])
      .andReturn(binaryMessengerMock);

  FlutterViewController* viewController = [[FlutterViewController alloc] initWithEngine:engineMock
                                                                                nibName:@""
                                                                                 bundle:nil];

  FlutterTextInputPlugin* plugin =
      [[FlutterTextInputPlugin alloc] initWithViewController:viewController];

  NSDictionary* setClientConfig = @{
    @"inputAction" : @"action",
    @"enableDeltaModel" : @"true",
    @"inputType" : @{@"name" : @"inputName"},
  };
  [plugin handleMethodCall:[FlutterMethodCall methodCallWithMethodName:@"TextInput.setClient"
                                                             arguments:@[ @(1), setClientConfig ]]
                    result:^(id){
                    }];

  // Can't run CFRunLoop in default mode because it causes crashes from scheduled
  // sources from other tests.
  NSString* runLoopMode = @"FlutterTestRunLoopMode";
  plugin.customRunLoopMode = runLoopMode;

  // Ensure both selectors are grouped in one platform channel call.
  [plugin doCommandBySelector:@selector(moveUp:)];
  [plugin doCommandBySelector:@selector(moveRightAndModifySelection:)];

  __block bool done = false;
  CFRunLoopPerformBlock(CFRunLoopGetMain(), (__bridge CFStringRef)runLoopMode, ^{
    done = true;
  });

  while (!done) {
    // Each invocation will handle one source.
    CFRunLoopRunInMode((__bridge CFStringRef)runLoopMode, 0, true);
  }

  NSData* performSelectorCall = [[FlutterJSONMethodCodec sharedInstance]
      encodeMethodCall:[FlutterMethodCall
                           methodCallWithMethodName:@"TextInputClient.performSelectors"
                                          arguments:@[
                                            @(1), @[ @"moveUp:", @"moveRightAndModifySelection:" ]
                                          ]]];

  @try {
    OCMVerify(  // NOLINT(google-objc-avoid-throwing-exception)
        [binaryMessengerMock sendOnChannel:@"flutter/textinput" message:performSelectorCall]);
  } @catch (...) {
    return false;
  }

  return true;
}

- (bool)testSelectorsNotForwardedToFrameworkIfNoClient {
  id engineMock = flutter::testing::CreateMockFlutterEngine(@"");
  id binaryMessengerMock = OCMProtocolMock(@protocol(FlutterBinaryMessenger));
  OCMStub(  // NOLINT(google-objc-avoid-throwing-exception)
      [engineMock binaryMessenger])
      .andReturn(binaryMessengerMock);
  // Make sure the selectors are not forwarded to the framework.
  OCMReject([binaryMessengerMock sendOnChannel:@"flutter/textinput" message:[OCMArg any]]);
  FlutterViewController* viewController = [[FlutterViewController alloc] initWithEngine:engineMock
                                                                                nibName:@""
                                                                                 bundle:nil];

  FlutterTextInputPlugin* plugin =
      [[FlutterTextInputPlugin alloc] initWithViewController:viewController];

  // Can't run CFRunLoop in default mode because it causes crashes from scheduled
  // sources from other tests.
  NSString* runLoopMode = @"FlutterTestRunLoopMode";
  plugin.customRunLoopMode = runLoopMode;

  // Call selectors without setting a client.
  [plugin doCommandBySelector:@selector(moveUp:)];
  [plugin doCommandBySelector:@selector(moveRightAndModifySelection:)];

  __block bool done = false;
  CFRunLoopPerformBlock(CFRunLoopGetMain(), (__bridge CFStringRef)runLoopMode, ^{
    done = true;
  });

  while (!done) {
    CFRunLoopRunInMode((__bridge CFStringRef)runLoopMode, 0, true);
  }
  // At this point the selectors should be dropped; otherwise, OCMReject will throw.
  return true;
}

@end

namespace flutter::testing {

namespace {
// Allocates and returns an engine configured for the text fixture resource configuration.
FlutterEngine* CreateTestEngine() {
  NSString* fixtures = @(testing::GetFixturesPath());
  FlutterDartProject* project = [[FlutterDartProject alloc]
      initWithAssetsPath:fixtures
             ICUDataPath:[fixtures stringByAppendingString:@"/icudtl.dat"]];
  return [[FlutterEngine alloc] initWithName:@"test" project:project allowHeadlessExecution:true];
}
}  // namespace

TEST(FlutterTextInputPluginTest, TestEmptyCompositionRange) {
  ASSERT_TRUE([[FlutterInputPluginTestObjc alloc] testEmptyCompositionRange]);
}

TEST(FlutterTextInputPluginTest, TestSetMarkedTextWithSelectionChange) {
  ASSERT_TRUE([[FlutterInputPluginTestObjc alloc] testSetMarkedTextWithSelectionChange]);
}

TEST(FlutterTextInputPluginTest, TestSetMarkedTextWithReplacementRange) {
  ASSERT_TRUE([[FlutterInputPluginTestObjc alloc] testSetMarkedTextWithReplacementRange]);
}

TEST(FlutterTextInputPluginTest, TestComposingRegionRemovedByFramework) {
  ASSERT_TRUE([[FlutterInputPluginTestObjc alloc] testComposingRegionRemovedByFramework]);
}

TEST(FlutterTextInputPluginTest, TestClearClientDuringComposing) {
  ASSERT_TRUE([[FlutterInputPluginTestObjc alloc] testClearClientDuringComposing]);
}

TEST(FlutterTextInputPluginTest, TestAutocompleteDisabledWhenAutofillNotSet) {
  ASSERT_TRUE([[FlutterInputPluginTestObjc alloc] testAutocompleteDisabledWhenAutofillNotSet]);
}

TEST(FlutterTextInputPluginTest, TestAutocompleteEnabledWhenAutofillSet) {
  ASSERT_TRUE([[FlutterInputPluginTestObjc alloc] testAutocompleteEnabledWhenAutofillSet]);
}

TEST(FlutterTextInputPluginTest, TestAutocompleteEnabledWhenAutofillSetNoHint) {
  ASSERT_TRUE([[FlutterInputPluginTestObjc alloc] testAutocompleteEnabledWhenAutofillSetNoHint]);
}

TEST(FlutterTextInputPluginTest, TestAutocompleteDisabledWhenObscureTextSet) {
  ASSERT_TRUE([[FlutterInputPluginTestObjc alloc] testAutocompleteDisabledWhenObscureTextSet]);
}

TEST(FlutterTextInputPluginTest, TestAutocompleteDisabledWhenPasswordAutofillSet) {
  ASSERT_TRUE([[FlutterInputPluginTestObjc alloc] testAutocompleteDisabledWhenPasswordAutofillSet]);
}

TEST(FlutterTextInputPluginTest, TestAutocompleteDisabledWhenAutofillGroupIncludesPassword) {
  ASSERT_TRUE([[FlutterInputPluginTestObjc alloc]
      testAutocompleteDisabledWhenAutofillGroupIncludesPassword]);
}

TEST(FlutterTextInputPluginTest, TestFirstRectForCharacterRange) {
  ASSERT_TRUE([[FlutterInputPluginTestObjc alloc] testFirstRectForCharacterRange]);
}

TEST(FlutterTextInputPluginTest, TestFirstRectForCharacterRangeAtInfinity) {
  ASSERT_TRUE([[FlutterInputPluginTestObjc alloc] testFirstRectForCharacterRangeAtInfinity]);
}

TEST(FlutterTextInputPluginTest, TestFirstRectForCharacterRangeWithEsotericAffineTransform) {
  ASSERT_TRUE([[FlutterInputPluginTestObjc alloc]
      testFirstRectForCharacterRangeWithEsotericAffineTransform]);
}

TEST(FlutterTextInputPluginTest, TestSetEditingStateWithTextEditingDelta) {
  ASSERT_TRUE([[FlutterInputPluginTestObjc alloc] testSetEditingStateWithTextEditingDelta]);
}

TEST(FlutterTextInputPluginTest, TestOperationsThatTriggerDelta) {
  ASSERT_TRUE([[FlutterInputPluginTestObjc alloc] testOperationsThatTriggerDelta]);
}

TEST(FlutterTextInputPluginTest, TestComposingWithDelta) {
  ASSERT_TRUE([[FlutterInputPluginTestObjc alloc] testComposingWithDelta]);
}

TEST(FlutterTextInputPluginTest, TestComposingWithDeltasWhenSelectionIsActive) {
  ASSERT_TRUE([[FlutterInputPluginTestObjc alloc] testComposingWithDeltasWhenSelectionIsActive]);
}

TEST(FlutterTextInputPluginTest, TestLocalTextAndSelectionUpdateAfterDelta) {
  ASSERT_TRUE([[FlutterInputPluginTestObjc alloc] testLocalTextAndSelectionUpdateAfterDelta]);
}

TEST(FlutterTextInputPluginTest, TestPerformKeyEquivalent) {
  ASSERT_TRUE([[FlutterInputPluginTestObjc alloc] testPerformKeyEquivalent]);
}

TEST(FlutterTextInputPluginTest, HandleArrowKeyWhenImePopoverIsActive) {
  ASSERT_TRUE([[FlutterInputPluginTestObjc alloc] handleArrowKeyWhenImePopoverIsActive]);
}

TEST(FlutterTextInputPluginTest, UnhandledKeyEquivalent) {
  ASSERT_TRUE([[FlutterInputPluginTestObjc alloc] unhandledKeyEquivalent]);
}

TEST(FlutterTextInputPluginTest, TestSelectorsAreForwardedToFramework) {
  ASSERT_TRUE([[FlutterInputPluginTestObjc alloc] testSelectorsAreForwardedToFramework]);
}

TEST(FlutterTextInputPluginTest, TestSelectorsNotForwardedToFrameworkIfNoClient) {
  ASSERT_TRUE([[FlutterInputPluginTestObjc alloc] testSelectorsNotForwardedToFrameworkIfNoClient]);
}

TEST(FlutterTextInputPluginTest, TestInsertNewLine) {
  ASSERT_TRUE([[FlutterInputPluginTestObjc alloc] testInsertNewLine]);
}

TEST(FlutterTextInputPluginTest, TestSendActionDoNotInsertNewLine) {
  ASSERT_TRUE([[FlutterInputPluginTestObjc alloc] testSendActionDoNotInsertNewLine]);
}

TEST(FlutterTextInputPluginTest, TestAttributedSubstringOutOfRange) {
  id engineMock = flutter::testing::CreateMockFlutterEngine(@"");
  id binaryMessengerMock = OCMProtocolMock(@protocol(FlutterBinaryMessenger));
  OCMStub(  // NOLINT(google-objc-avoid-throwing-exception)
      [engineMock binaryMessenger])
      .andReturn(binaryMessengerMock);

  FlutterViewController* viewController = [[FlutterViewController alloc] initWithEngine:engineMock
                                                                                nibName:@""
                                                                                 bundle:nil];

  FlutterTextInputPlugin* plugin =
      [[FlutterTextInputPlugin alloc] initWithViewController:viewController];

  NSDictionary* setClientConfig = @{
    @"inputAction" : @"action",
    @"enableDeltaModel" : @"true",
    @"inputType" : @{@"name" : @"inputName"},
  };
  [plugin handleMethodCall:[FlutterMethodCall methodCallWithMethodName:@"TextInput.setClient"
                                                             arguments:@[ @(1), setClientConfig ]]
                    result:^(id){
                    }];

  FlutterMethodCall* call = [FlutterMethodCall methodCallWithMethodName:@"TextInput.setEditingState"
                                                              arguments:@{
                                                                @"text" : @"Text",
                                                                @"selectionBase" : @(0),
                                                                @"selectionExtent" : @(0),
                                                                @"composingBase" : @(-1),
                                                                @"composingExtent" : @(-1),
                                                              }];

  [plugin handleMethodCall:call
                    result:^(id){
                    }];

  NSRange out;
  NSAttributedString* text = [plugin attributedSubstringForProposedRange:NSMakeRange(1, 10)
                                                             actualRange:&out];
  EXPECT_TRUE([text.string isEqualToString:@"ext"]);
  EXPECT_EQ(out.location, 1u);
  EXPECT_EQ(out.length, 3u);

  text = [plugin attributedSubstringForProposedRange:NSMakeRange(4, 10) actualRange:&out];
  EXPECT_EQ(text, nil);
}

TEST(FlutterTextInputPluginTest, CanWorkWithFlutterTextField) {
  FlutterEngine* engine = CreateTestEngine();
  FlutterViewController* viewController = [[FlutterViewController alloc] initWithEngine:engine
                                                                                nibName:nil
                                                                                 bundle:nil];
  [viewController loadView];
  // Create a NSWindow so that the native text field can become first responder.
  NSWindow* window = [[NSWindow alloc] initWithContentRect:NSMakeRect(0, 0, 800, 600)
                                                 styleMask:NSBorderlessWindowMask
                                                   backing:NSBackingStoreBuffered
                                                     defer:NO];
  window.contentView = viewController.view;

  engine.semanticsEnabled = YES;

  auto bridge = viewController.accessibilityBridge.lock();
  FlutterPlatformNodeDelegateMac delegate(bridge, viewController);
  ui::AXTree tree;
  ui::AXNode ax_node(&tree, nullptr, 0, 0);
  ui::AXNodeData node_data;
  node_data.SetValue("initial text");
  ax_node.SetData(node_data);
  delegate.Init(viewController.accessibilityBridge, &ax_node);
  {
    FlutterTextPlatformNode text_platform_node(&delegate, viewController);

    FlutterTextFieldMock* mockTextField =
        [[FlutterTextFieldMock alloc] initWithPlatformNode:&text_platform_node
                                               fieldEditor:viewController.textInputPlugin];
    [viewController.view addSubview:mockTextField];
    [mockTextField startEditing];

    NSDictionary* setClientConfig = @{
      @"inputAction" : @"action",
      @"inputType" : @{@"name" : @"inputName"},
    };
    FlutterMethodCall* methodCall =
        [FlutterMethodCall methodCallWithMethodName:@"TextInput.setClient"
                                          arguments:@[ @(1), setClientConfig ]];
    FlutterResult result = ^(id result) {
    };
    [viewController.textInputPlugin handleMethodCall:methodCall result:result];

    NSDictionary* arguments = @{
      @"text" : @"new text",
      @"selectionBase" : @(1),
      @"selectionExtent" : @(2),
      @"composingBase" : @(-1),
      @"composingExtent" : @(-1),
    };
    methodCall = [FlutterMethodCall methodCallWithMethodName:@"TextInput.setEditingState"
                                                   arguments:arguments];
    [viewController.textInputPlugin handleMethodCall:methodCall result:result];
    EXPECT_EQ([mockTextField.lastUpdatedString isEqualToString:@"new text"], YES);
    EXPECT_EQ(NSEqualRanges(mockTextField.lastUpdatedSelection, NSMakeRange(1, 1)), YES);

    // This blocks the FlutterTextFieldMock, which is held onto by the main event
    // loop, from crashing.
    [mockTextField setPlatformNode:nil];
  }

  // This verifies that clearing the platform node works.
  [[NSRunLoop currentRunLoop] runUntilDate:[NSDate dateWithTimeIntervalSinceNow:0.1]];
}

TEST(FlutterTextInputPluginTest, CanNotBecomeResponderIfNoViewController) {
  FlutterEngine* engine = CreateTestEngine();
  FlutterViewController* viewController = [[FlutterViewController alloc] initWithEngine:engine
                                                                                nibName:nil
                                                                                 bundle:nil];
  [viewController loadView];
  // Creates a NSWindow so that the native text field can become first responder.
  NSWindow* window = [[NSWindow alloc] initWithContentRect:NSMakeRect(0, 0, 800, 600)
                                                 styleMask:NSBorderlessWindowMask
                                                   backing:NSBackingStoreBuffered
                                                     defer:NO];
  window.contentView = viewController.view;

  engine.semanticsEnabled = YES;

  auto bridge = viewController.accessibilityBridge.lock();
  FlutterPlatformNodeDelegateMac delegate(bridge, viewController);
  ui::AXTree tree;
  ui::AXNode ax_node(&tree, nullptr, 0, 0);
  ui::AXNodeData node_data;
  node_data.SetValue("initial text");
  ax_node.SetData(node_data);
  delegate.Init(viewController.accessibilityBridge, &ax_node);
  FlutterTextPlatformNode text_platform_node(&delegate, viewController);

  FlutterTextField* textField = text_platform_node.GetNativeViewAccessible();
  EXPECT_EQ([textField becomeFirstResponder], YES);
  // Removes view controller.
  [engine setViewController:nil];
  FlutterTextPlatformNode text_platform_node_no_controller(&delegate, nil);
  textField = text_platform_node_no_controller.GetNativeViewAccessible();
  EXPECT_EQ([textField becomeFirstResponder], NO);
}

TEST(FlutterTextInputPluginTest, IsAddedAndRemovedFromViewHierarchy) {
  FlutterEngine* engine = CreateTestEngine();
  FlutterViewController* viewController = [[FlutterViewController alloc] initWithEngine:engine
                                                                                nibName:nil
                                                                                 bundle:nil];
  [viewController loadView];

  NSWindow* window = [[NSWindow alloc] initWithContentRect:NSMakeRect(0, 0, 800, 600)
                                                 styleMask:NSBorderlessWindowMask
                                                   backing:NSBackingStoreBuffered
                                                     defer:NO];
  window.contentView = viewController.view;

  ASSERT_EQ(viewController.textInputPlugin.superview, nil);
  ASSERT_FALSE(window.firstResponder == viewController.textInputPlugin);

  [viewController.textInputPlugin
      handleMethodCall:[FlutterMethodCall methodCallWithMethodName:@"TextInput.show" arguments:@[]]
                result:^(id){
                }];

  ASSERT_EQ(viewController.textInputPlugin.superview, viewController.view);
  ASSERT_TRUE(window.firstResponder == viewController.textInputPlugin);

  [viewController.textInputPlugin
      handleMethodCall:[FlutterMethodCall methodCallWithMethodName:@"TextInput.hide" arguments:@[]]
                result:^(id){
                }];

  ASSERT_EQ(viewController.textInputPlugin.superview, nil);
  ASSERT_FALSE(window.firstResponder == viewController.textInputPlugin);
}

TEST(FlutterTextInputPluginTest, FirstResponderIsCorrect) {
  FlutterEngine* engine = CreateTestEngine();
  FlutterViewController* viewController = [[FlutterViewController alloc] initWithEngine:engine
                                                                                nibName:nil
                                                                                 bundle:nil];
  [viewController loadView];

  NSWindow* window = [[NSWindow alloc] initWithContentRect:NSMakeRect(0, 0, 800, 600)
                                                 styleMask:NSBorderlessWindowMask
                                                   backing:NSBackingStoreBuffered
                                                     defer:NO];
  window.contentView = viewController.view;

  ASSERT_TRUE(viewController.flutterView.acceptsFirstResponder);

  [window makeFirstResponder:viewController.flutterView];

  [viewController.textInputPlugin
      handleMethodCall:[FlutterMethodCall methodCallWithMethodName:@"TextInput.show" arguments:@[]]
                result:^(id){
                }];

  ASSERT_TRUE(window.firstResponder == viewController.textInputPlugin);

  ASSERT_FALSE(viewController.flutterView.acceptsFirstResponder);

  [viewController.textInputPlugin
      handleMethodCall:[FlutterMethodCall methodCallWithMethodName:@"TextInput.hide" arguments:@[]]
                result:^(id){
                }];

  ASSERT_TRUE(viewController.flutterView.acceptsFirstResponder);
  ASSERT_TRUE(window.firstResponder == viewController.flutterView);
}

TEST(FlutterTextInputPluginTest, HasZeroSizeAndClipsToBounds) {
  id engineMock = flutter::testing::CreateMockFlutterEngine(@"");
  id binaryMessengerMock = OCMProtocolMock(@protocol(FlutterBinaryMessenger));
  OCMStub(  // NOLINT(google-objc-avoid-throwing-exception)
      [engineMock binaryMessenger])
      .andReturn(binaryMessengerMock);

  FlutterViewController* viewController = [[FlutterViewController alloc] initWithEngine:engineMock
                                                                                nibName:@""
                                                                                 bundle:nil];

  FlutterTextInputPlugin* plugin =
      [[FlutterTextInputPlugin alloc] initWithViewController:viewController];

  ASSERT_TRUE(NSIsEmptyRect(plugin.frame));
  ASSERT_TRUE(plugin.clipsToBounds);
}

}  // namespace flutter::testing
