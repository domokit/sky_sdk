// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterSpellCheckPlugin.h"

#import <OCMock/OCMock.h>
#import <XCTest/XCTest.h>

#pragma mark -

// A mock class representing the UITextChecker used in the tests.
//
// Because OCMock doesn't support mocking NSRange as method arguments,
// this is necessary.
@interface MockTextChecker : UITextChecker

// The range of misspelled word based on the startingIndex.
//
// Key is the starting index, value is the range
@property(strong, nonatomic) NSMutableDictionary<NSNumber*, NSValue*>* startingIndexToRange;

// The suggestions of misspelled word based on the starting index of the misspelled word.
//
// Key is a string representing the range of the misspelled word, value is the suggestions.
@property(strong, nonatomic)
    NSMutableDictionary<NSString*, NSArray<NSString*>*>* rangeToSuggestions;

// Mock the spell checking results.
//
// When no misspelled word should be detected, pass (NSNotFound, 0) for the `range` parameter, and
// an empty array for `suggestions`.
//
// Call `reset` to remove all the mocks.
- (void)mockResultRange:(NSRange)range
            suggestions:(nonnull NSArray<NSString*>*)suggestions
      withStartingIndex:(NSInteger)startingIndex;

// Remove all mocks.
- (void)reset;

@end

@implementation MockTextChecker

- (instancetype)init {
  self = [super init];
  if (self) {
    _startingIndexToRange = [[NSMutableDictionary alloc] init];
    _rangeToSuggestions = [[NSMutableDictionary alloc] init];
  }
  return self;
}

- (void)mockResultRange:(NSRange)range
            suggestions:(NSArray<NSString*>*)suggestions
      withStartingIndex:(NSInteger)startingIndex {
  NSValue* valueForRange = [NSValue valueWithRange:range];
  self.startingIndexToRange[@(startingIndex)] = valueForRange;
  NSString* rangeString = NSStringFromRange(valueForRange.rangeValue);
  self.rangeToSuggestions[rangeString] = suggestions;
}

- (void)reset {
  [self.startingIndexToRange removeAllObjects];
  [self.rangeToSuggestions removeAllObjects];
}

#pragma mark UITextChecker Overrides

- (NSRange)rangeOfMisspelledWordInString:(NSString*)stringToCheck
                                   range:(NSRange)range
                              startingAt:(NSInteger)startingOffset
                                    wrap:(BOOL)wrapFlag
                                language:(NSString*)language {
  return self.startingIndexToRange[@(startingOffset)].rangeValue;
}

- (NSArray<NSString*>*)guessesForWordRange:(NSRange)range
                                  inString:(NSString*)string
                                  language:(NSString*)language {
  return self.rangeToSuggestions[NSStringFromRange(range)];
}

@end

@interface FlutterSpellCheckPlugin ()
- (void)handleMethodCall:(FlutterMethodCall*)call result:(FlutterResult)result;
- (UITextChecker*)textChecker;
@end

@interface FlutterSpellCheckPluginTest : XCTestCase

@property(strong, nonatomic) id mockMethodChannel;
@property(strong, nonatomic) FlutterSpellCheckPlugin* plugin;
@property(strong, nonatomic) id mockTextChecker;
@property(strong, nonatomic) id partialMockPlugin;

@end

#pragma mark -

@implementation FlutterSpellCheckPluginTest

- (void)setUp {
  [super setUp];
  self.mockMethodChannel = OCMClassMock([FlutterMethodChannel class]);
  self.plugin = [[FlutterSpellCheckPlugin alloc] init];
  __weak FlutterSpellCheckPlugin* weakPlugin = self.plugin;
  OCMStub([self.mockMethodChannel invokeMethod:[OCMArg any]
                                     arguments:[OCMArg any]
                                        result:[OCMArg any]])
      .andDo(^(NSInvocation* invocation) {
        NSString* name;
        id args;
        FlutterResult result;
        [invocation getArgument:&name atIndex:2];
        [invocation getArgument:&args atIndex:3];
        [invocation getArgument:&result atIndex:4];
        FlutterMethodCall* methodCall = [FlutterMethodCall methodCallWithMethodName:name
                                                                          arguments:args];
        [weakPlugin handleMethodCall:methodCall result:result];
      });
  self.mockTextChecker = [[MockTextChecker alloc] init];
}

- (void)tearDown {
  self.plugin = nil;
  [super tearDown];
}

#pragma mark - Tests

// Test to make sure the while loop that checks all the misspelled word stops when the a
// `NSNotFound` is found.
- (void)testFindAllSpellCheckSuggestionsForText {
  self.partialMockPlugin = OCMPartialMock(self.plugin);
  OCMStub([self.partialMockPlugin textChecker]).andReturn(self.mockTextChecker);
  id textCheckerClassMock = OCMClassMock([UITextChecker class]);
  [[[textCheckerClassMock stub] andReturn:@[ @"en" ]] availableLanguages];
  NSArray* suggestions1 = @[ @"suggestion 1", @"suggestion 2" ];
  NSArray* suggestions2 = @[ @"suggestion 3", @"suggestion 4" ];
  // 0-4 is a misspelled word.
  [self mockUITextCheckerWithExpectedMisspelledWordRange:NSMakeRange(0, 5)
                                           startingIndex:0
                                             suggestions:suggestions1];
  // 5-9 is a misspelled word.
  [self mockUITextCheckerWithExpectedMisspelledWordRange:NSMakeRange(5, 5)
                                           startingIndex:5
                                             suggestions:suggestions2];
  // No misspelled word after index 10.
  [self mockUITextCheckerWithExpectedMisspelledWordRange:NSMakeRange(NSNotFound, 0)
                                           startingIndex:10
                                             suggestions:@[]];
  __block NSArray* capturedResult;
  [self.mockMethodChannel invokeMethod:@"SpellCheck.initiateSpellCheck"
                             arguments:@[ @"en", @"ksajlkdf aslkdfl kasdf asdfjk" ]
                                result:^(id _Nullable result) {
                                  capturedResult = result;
                                }];
  XCTAssertTrue(capturedResult.count == 2);
  NSDictionary* suggestionsJSON1 = capturedResult.firstObject;
  XCTAssertEqualObjects(suggestionsJSON1[@"startIndex"], @0);
  XCTAssertEqualObjects(suggestionsJSON1[@"endIndex"], @5);
  XCTAssertEqualObjects(suggestionsJSON1[@"suggestions"], suggestions1);
  NSDictionary* suggestionsJSON2 = capturedResult[1];
  XCTAssertEqualObjects(suggestionsJSON2[@"startIndex"], @5);
  XCTAssertEqualObjects(suggestionsJSON2[@"endIndex"], @10);
  XCTAssertEqualObjects(suggestionsJSON2[@"suggestions"], suggestions2);
  [self.mockTextChecker reset];
  [textCheckerClassMock stopMocking];
}

// Test to make sure while loop that checks all the misspelled word stops when the last word is
// misspelled (aka nextIndex is out of bounds)
- (void)testStopFindingMoreWhenTheLastWordIsMisspelled {
  self.partialMockPlugin = OCMPartialMock(self.plugin);
  OCMStub([self.partialMockPlugin textChecker]).andReturn(self.mockTextChecker);
  id textCheckerClassMock = OCMClassMock([UITextChecker class]);
  [[[textCheckerClassMock stub] andReturn:@[ @"en" ]] availableLanguages];
  NSArray* suggestions1 = @[ @"suggestion 1", @"suggestion 2" ];
  NSArray* suggestions2 = @[ @"suggestion 3", @"suggestion 4" ];
  // 0-4 is a misspelled word.
  [self mockUITextCheckerWithExpectedMisspelledWordRange:NSMakeRange(0, 5)
                                           startingIndex:0
                                             suggestions:suggestions1];
  // 5-9 is a misspelled word.
  [self mockUITextCheckerWithExpectedMisspelledWordRange:NSMakeRange(6, 4)
                                           startingIndex:5
                                             suggestions:suggestions2];

  __block NSArray* capturedResult;
  [self.mockMethodChannel invokeMethod:@"SpellCheck.initiateSpellCheck"
                             arguments:@[ @"en", @"hejjo abcd" ]
                                result:^(id _Nullable result) {
                                  capturedResult = result;
                                }];
  XCTAssertTrue(capturedResult.count == 2);
  NSDictionary* suggestionsJSON1 = capturedResult.firstObject;
  XCTAssertEqualObjects(suggestionsJSON1[@"startIndex"], @0);
  XCTAssertEqualObjects(suggestionsJSON1[@"endIndex"], @5);
  XCTAssertEqualObjects(suggestionsJSON1[@"suggestions"], suggestions1);
  NSDictionary* suggestionsJSON2 = capturedResult[1];
  XCTAssertEqualObjects(suggestionsJSON2[@"startIndex"], @6);
  XCTAssertEqualObjects(suggestionsJSON2[@"endIndex"], @10);
  XCTAssertEqualObjects(suggestionsJSON2[@"suggestions"], suggestions2);
  [self.mockTextChecker reset];
  [textCheckerClassMock stopMocking];
}

- (void)testStopFindingMoreWhenTheWholeStringIsAMisspelledWord {
  self.partialMockPlugin = OCMPartialMock(self.plugin);
  OCMStub([self.partialMockPlugin textChecker]).andReturn(self.mockTextChecker);
  id textCheckerClassMock = OCMClassMock([UITextChecker class]);
  [[[textCheckerClassMock stub] andReturn:@[ @"en" ]] availableLanguages];
  NSArray* suggestions1 = @[ @"suggestion 1", @"suggestion 2" ];
  // 0-4 is a misspelled word.
  [self mockUITextCheckerWithExpectedMisspelledWordRange:NSMakeRange(0, 5)
                                           startingIndex:0
                                             suggestions:suggestions1];

  __block NSArray* capturedResult;
  [self.mockMethodChannel invokeMethod:@"SpellCheck.initiateSpellCheck"
                             arguments:@[ @"en", @"hejjo" ]
                                result:^(id _Nullable result) {
                                  capturedResult = result;
                                }];
  XCTAssertTrue(capturedResult.count == 1);
  NSDictionary* suggestionsJSON1 = capturedResult.firstObject;
  XCTAssertEqualObjects(suggestionsJSON1[@"startIndex"], @0);
  XCTAssertEqualObjects(suggestionsJSON1[@"endIndex"], @5);
  XCTAssertEqualObjects(suggestionsJSON1[@"suggestions"], suggestions1);
  [self.mockTextChecker reset];
  [textCheckerClassMock stopMocking];
}

- (void)testInitiateSpellCheckWithNoMisspelledWord {
  self.partialMockPlugin = OCMPartialMock(self.plugin);
  OCMStub([self.partialMockPlugin textChecker]).andReturn(self.mockTextChecker);
  id textCheckerClassMock = OCMClassMock([UITextChecker class]);
  [[[textCheckerClassMock stub] andReturn:@[ @"en" ]] availableLanguages];
  [self mockUITextCheckerWithExpectedMisspelledWordRange:NSMakeRange(NSNotFound, 0)
                                           startingIndex:0
                                             suggestions:@[]];
  __block id capturedResult;
  [self.mockMethodChannel invokeMethod:@"SpellCheck.initiateSpellCheck"
                             arguments:@[ @"en", @"helloo" ]
                                result:^(id _Nullable result) {
                                  capturedResult = result;
                                }];
  XCTAssertEqualObjects(capturedResult, @[]);
  [textCheckerClassMock stopMocking];
}

- (void)testUnsupportedLanguageShouldReturnNil {
  self.partialMockPlugin = OCMPartialMock(self.plugin);
  OCMStub([self.partialMockPlugin textChecker]).andReturn(self.mockTextChecker);
  id textCheckerClassMock = OCMClassMock([UITextChecker class]);
  [[[textCheckerClassMock stub] andReturn:@[ @"en" ]] availableLanguages];
  [self mockUITextCheckerWithExpectedMisspelledWordRange:NSMakeRange(0, 5)
                                           startingIndex:0
                                             suggestions:@[]];
  __block id capturedResult;
  [self.mockMethodChannel invokeMethod:@"SpellCheck.initiateSpellCheck"
                             arguments:@[ @"xx", @"helloo" ]
                                result:^(id _Nullable result) {
                                  capturedResult = result;
                                }];
  XCTAssertNil(capturedResult);
  [textCheckerClassMock stopMocking];
}

- (void)testSupportSubLanguage {
  self.partialMockPlugin = OCMPartialMock(self.plugin);
  OCMStub([self.partialMockPlugin textChecker]).andReturn(self.mockTextChecker);
  id textCheckerClassMock = OCMClassMock([UITextChecker class]);
  [[[textCheckerClassMock stub] andReturn:@[ @"en_US" ]] availableLanguages];
  NSArray* suggestions1 = @[ @"suggestion 1", @"suggestion 2" ];

  [self mockUITextCheckerWithExpectedMisspelledWordRange:NSMakeRange(0, 5)
                                           startingIndex:0
                                             suggestions:suggestions1];
  __block NSArray* capturedResult;
  [self.mockMethodChannel invokeMethod:@"SpellCheck.initiateSpellCheck"
                             arguments:@[ @"en-us", @"hejjo" ]
                                result:^(id _Nullable result) {
                                  capturedResult = result;
                                }];
  NSDictionary* suggestionsJSON1 = capturedResult.firstObject;
  XCTAssertEqualObjects(suggestionsJSON1[@"startIndex"], @0);
  XCTAssertEqualObjects(suggestionsJSON1[@"endIndex"], @5);
  XCTAssertEqualObjects(suggestionsJSON1[@"suggestions"], suggestions1);
  [textCheckerClassMock stopMocking];
}

- (void)testEmptyStringShouldReturnEmptyResults {
  self.partialMockPlugin = OCMPartialMock(self.plugin);
  OCMStub([self.partialMockPlugin textChecker]).andReturn(self.mockTextChecker);
  // Use real UITextChecker for this as we want to rely on the actual behavior of UITextChecker
  // to ensure that spell checks on an empty result always return empty.
  [self.partialMockPlugin stopMocking];

  id textCheckerClassMock = OCMClassMock([UITextChecker class]);
  [[[textCheckerClassMock stub] andReturn:@[ @"en" ]] availableLanguages];
  __block id capturedResult;
  [self.mockMethodChannel invokeMethod:@"SpellCheck.initiateSpellCheck"
                             arguments:@[ @"en", @"" ]
                                result:^(id _Nullable result) {
                                  capturedResult = result;
                                }];
  XCTAssertEqualObjects(capturedResult, @[]);
  [textCheckerClassMock stopMocking];
}

- (void)testNullStringArgumentShouldReturnNilResults {
  self.partialMockPlugin = OCMPartialMock(self.plugin);
  OCMStub([self.partialMockPlugin textChecker]).andReturn(self.mockTextChecker);
  id textCheckerClassMock = OCMClassMock([UITextChecker class]);
  [[[textCheckerClassMock stub] andReturn:@[ @"en" ]] availableLanguages];
  __block id capturedResult;
  [self.mockMethodChannel invokeMethod:@"SpellCheck.initiateSpellCheck"
                             arguments:@[ @"en", [NSNull null] ]
                                result:^(id _Nullable result) {
                                  capturedResult = result;
                                }];
  XCTAssertNil(capturedResult);
  [textCheckerClassMock stopMocking];
}

- (void)testNullLanguageArgumentShouldReturnNilResults {
  self.partialMockPlugin = OCMPartialMock(self.plugin);
  OCMStub([self.partialMockPlugin textChecker]).andReturn(self.mockTextChecker);
  id textCheckerClassMock = OCMClassMock([UITextChecker class]);
  [[[textCheckerClassMock stub] andReturn:@[ @"en" ]] availableLanguages];
  __block id capturedResult;
  [self.mockMethodChannel invokeMethod:@"SpellCheck.initiateSpellCheck"
                             arguments:@[ [NSNull null], @"some string" ]
                                result:^(id _Nullable result) {
                                  capturedResult = result;
                                }];
  XCTAssertNil(capturedResult);
  [textCheckerClassMock stopMocking];
}

- (void)testUITextCheckerIsInitializedAfterMethodChannelCall {
  XCTAssertNil([self.plugin textChecker]);
  __block id capturedResult;
  [self.mockMethodChannel invokeMethod:@"SpellCheck.initiateSpellCheck"
                             arguments:@[ [NSNull null], @"some string" ]
                                result:^(id _Nullable result) {
                                  capturedResult = result;
                                }];
  XCTAssertNotNil([self.plugin textChecker]);
}

- (void)mockUITextCheckerWithExpectedMisspelledWordRange:(NSRange)expectedRange
                                           startingIndex:(NSInteger)startingIndex
                                             suggestions:(NSArray*)suggestions {
  [self.mockTextChecker mockResultRange:expectedRange
                            suggestions:suggestions
                      withStartingIndex:startingIndex];
}

@end
