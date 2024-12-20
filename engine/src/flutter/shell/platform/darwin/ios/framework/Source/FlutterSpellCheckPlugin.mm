// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterSpellCheckPlugin.h"

#import <Foundation/Foundation.h>
#import <UIKit/UIKit.h>

#import "flutter/fml/logging.h"

FLUTTER_ASSERT_ARC

// Method Channel name to start spell check.
static NSString* const kInitiateSpellCheck = @"SpellCheck.initiateSpellCheck";

@interface FlutterSpellCheckResult : NSObject

@property(nonatomic, copy, readonly) NSArray<NSString*>* suggestions;
@property(nonatomic, assign, readonly) NSRange misspelledRange;

- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
- (instancetype)initWithMisspelledRange:(NSRange)range
                            suggestions:(NSArray<NSString*>*)suggestions NS_DESIGNATED_INITIALIZER;
- (NSDictionary<NSString*, NSObject*>*)toDictionary;

@end

@interface FlutterSpellCheckPlugin ()

@property(nonatomic) UITextChecker* textChecker;

@end

@implementation FlutterSpellCheckPlugin

- (void)handleMethodCall:(FlutterMethodCall*)call result:(FlutterResult)result {
  if (!self.textChecker) {
    // UITextChecker is an expensive object to initiate, see:
    // https://github.com/flutter/flutter/issues/104454. Lazily initialate the UITextChecker object
    // until at first method channel call. We avoid using lazy getter for testing.
    self.textChecker = [[UITextChecker alloc] init];
  }
  NSString* method = call.method;
  NSArray* args = call.arguments;
  if ([method isEqualToString:kInitiateSpellCheck]) {
    FML_DCHECK(args.count == 2);
    id language = args[0];
    id text = args[1];
    if (language == [NSNull null] || text == [NSNull null]) {
      // Bail if null arguments are passed from dart.
      result(nil);
      return;
    }

    NSArray<NSDictionary<NSString*, id>*>* spellCheckResult =
        [self findAllSpellCheckSuggestionsForText:text inLanguage:language];
    result(spellCheckResult);
  }
}

// Get all the misspelled words and suggestions in the entire String.
//
// The result will be formatted as an NSArray.
// Each item of the array is a dictionary representing a misspelled word and suggestions.
// The format looks like:
// {
//  startIndex: 0,
//  endIndex: 5,
//  suggestions: [hello, ...]
// }
//
// Returns nil if the language is invalid.
// Returns an empty array if no spell check suggestions.
- (NSArray<NSDictionary<NSString*, id>*>*)findAllSpellCheckSuggestionsForText:(NSString*)text
                                                                   inLanguage:(NSString*)language {
  // Transform Dart Locale format to iOS language format if necessary.
  if ([language containsString:@"-"]) {
    NSArray<NSString*>* languageCodes = [language componentsSeparatedByString:@"-"];
    FML_DCHECK(languageCodes.count == 2);
    NSString* lastCode = [[languageCodes lastObject] uppercaseString];
    language = [NSString stringWithFormat:@"%@_%@", [languageCodes firstObject], lastCode];
  }

  if (![UITextChecker.availableLanguages containsObject:language]) {
    return nil;
  }

  NSMutableArray<FlutterSpellCheckResult*>* allSpellSuggestions = [[NSMutableArray alloc] init];

  FlutterSpellCheckResult* nextSpellSuggestion;
  NSUInteger nextOffset = 0;
  do {
    nextSpellSuggestion = [self findSpellCheckSuggestionsForText:text
                                                      inLanguage:language
                                                  startingOffset:nextOffset];
    if (nextSpellSuggestion != nil) {
      [allSpellSuggestions addObject:nextSpellSuggestion];
      nextOffset =
          nextSpellSuggestion.misspelledRange.location + nextSpellSuggestion.misspelledRange.length;
    }
  } while (nextSpellSuggestion != nil && nextOffset < text.length);

  NSMutableArray* methodChannelResult =
      [[NSMutableArray alloc] initWithCapacity:allSpellSuggestions.count];

  for (FlutterSpellCheckResult* result in allSpellSuggestions) {
    [methodChannelResult addObject:[result toDictionary]];
  }

  return methodChannelResult;
}

// Get the misspelled word and suggestions.
//
// Returns nil if no spell check suggestions.
- (FlutterSpellCheckResult*)findSpellCheckSuggestionsForText:(NSString*)text
                                                  inLanguage:(NSString*)language
                                              startingOffset:(NSInteger)startingOffset {
  FML_DCHECK([UITextChecker.availableLanguages containsObject:language]);
  NSRange misspelledRange =
      [self.textChecker rangeOfMisspelledWordInString:text
                                                range:NSMakeRange(0, text.length)
                                           startingAt:startingOffset
                                                 wrap:NO
                                             language:language];
  if (misspelledRange.location == NSNotFound) {
    // No misspelled word found
    return nil;
  }

  // If no possible guesses, the API returns an empty array:
  // https://developer.apple.com/documentation/uikit/uitextchecker/1621037-guessesforwordrange?language=objc
  NSArray<NSString*>* suggestions = [self.textChecker guessesForWordRange:misspelledRange
                                                                 inString:text
                                                                 language:language];
  return [[FlutterSpellCheckResult alloc] initWithMisspelledRange:misspelledRange
                                                      suggestions:suggestions];
}

@end

@implementation FlutterSpellCheckResult

- (instancetype)initWithMisspelledRange:(NSRange)range
                            suggestions:(NSArray<NSString*>*)suggestions {
  self = [super init];
  if (self) {
    _suggestions = [suggestions copy];
    _misspelledRange = range;
  }
  return self;
}

- (NSDictionary<NSString*, NSObject*>*)toDictionary {
  return @{
    @"startIndex" : @(_misspelledRange.location),
    // The end index represents the next index after the last character of a misspelled word to
    // match the behavior of Dart's TextRange:
    // https://api.flutter.dev/flutter/dart-ui/TextRange/end.html
    @"endIndex" : @(_misspelledRange.location + _misspelledRange.length),
    @"suggestions" : _suggestions,
  };
}

@end
