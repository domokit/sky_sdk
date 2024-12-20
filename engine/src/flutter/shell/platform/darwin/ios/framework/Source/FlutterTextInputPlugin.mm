// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterTextInputPlugin.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/UIViewController+FlutterScreenAndSceneIfLoaded.h"

#import <Foundation/Foundation.h>
#import <UIKit/UIKit.h>

#include "unicode/uchar.h"

#include "flutter/fml/logging.h"
#include "flutter/fml/platform/darwin/string_range_sanitization.h"

FLUTTER_ASSERT_ARC

static const char kTextAffinityDownstream[] = "TextAffinity.downstream";
static const char kTextAffinityUpstream[] = "TextAffinity.upstream";
// A delay before enabling the accessibility of FlutterTextInputView after
// it is activated.
static constexpr double kUITextInputAccessibilityEnablingDelaySeconds = 0.5;

// A delay before reenabling the UIView areAnimationsEnabled to YES
// in order for becomeFirstResponder to receive the proper value.
static const NSTimeInterval kKeyboardAnimationDelaySeconds = 0.1;

// A time set for the screenshot to animate back to the assigned position.
static const NSTimeInterval kKeyboardAnimationTimeToCompleteion = 0.3;

// The "canonical" invalid CGRect, similar to CGRectNull, used to
// indicate a CGRect involved in firstRectForRange calculation is
// invalid. The specific value is chosen so that if firstRectForRange
// returns kInvalidFirstRect, iOS will not show the IME candidates view.
const CGRect kInvalidFirstRect = {{-1, -1}, {9999, 9999}};

#pragma mark - TextInput channel method names.
// See https://api.flutter.dev/flutter/services/SystemChannels/textInput-constant.html
static NSString* const kShowMethod = @"TextInput.show";
static NSString* const kHideMethod = @"TextInput.hide";
static NSString* const kSetClientMethod = @"TextInput.setClient";
static NSString* const kSetPlatformViewClientMethod = @"TextInput.setPlatformViewClient";
static NSString* const kSetEditingStateMethod = @"TextInput.setEditingState";
static NSString* const kClearClientMethod = @"TextInput.clearClient";
static NSString* const kSetEditableSizeAndTransformMethod =
    @"TextInput.setEditableSizeAndTransform";
static NSString* const kSetMarkedTextRectMethod = @"TextInput.setMarkedTextRect";
static NSString* const kFinishAutofillContextMethod = @"TextInput.finishAutofillContext";
// TODO(justinmc): Remove the TextInput method constant when the framework has
// finished transitioning to using the Scribble channel.
// https://github.com/flutter/flutter/pull/104128
static NSString* const kDeprecatedSetSelectionRectsMethod = @"TextInput.setSelectionRects";
static NSString* const kSetSelectionRectsMethod = @"Scribble.setSelectionRects";
static NSString* const kStartLiveTextInputMethod = @"TextInput.startLiveTextInput";
static NSString* const kUpdateConfigMethod = @"TextInput.updateConfig";
static NSString* const kOnInteractiveKeyboardPointerMoveMethod =
    @"TextInput.onPointerMoveForInteractiveKeyboard";
static NSString* const kOnInteractiveKeyboardPointerUpMethod =
    @"TextInput.onPointerUpForInteractiveKeyboard";

#pragma mark - TextInputConfiguration Field Names
static NSString* const kSecureTextEntry = @"obscureText";
static NSString* const kKeyboardType = @"inputType";
static NSString* const kKeyboardAppearance = @"keyboardAppearance";
static NSString* const kInputAction = @"inputAction";
static NSString* const kEnableDeltaModel = @"enableDeltaModel";
static NSString* const kEnableInteractiveSelection = @"enableInteractiveSelection";

static NSString* const kSmartDashesType = @"smartDashesType";
static NSString* const kSmartQuotesType = @"smartQuotesType";

static NSString* const kAssociatedAutofillFields = @"fields";

// TextInputConfiguration.autofill and sub-field names
static NSString* const kAutofillProperties = @"autofill";
static NSString* const kAutofillId = @"uniqueIdentifier";
static NSString* const kAutofillEditingValue = @"editingValue";
static NSString* const kAutofillHints = @"hints";

static NSString* const kAutocorrectionType = @"autocorrect";

#pragma mark - Static Functions

// Determine if the character at `range` of `text` is an emoji.
static BOOL IsEmoji(NSString* text, NSRange charRange) {
  UChar32 codePoint;
  BOOL gotCodePoint = [text getBytes:&codePoint
                           maxLength:sizeof(codePoint)
                          usedLength:NULL
                            encoding:NSUTF32StringEncoding
                             options:kNilOptions
                               range:charRange
                      remainingRange:NULL];
  return gotCodePoint && u_hasBinaryProperty(codePoint, UCHAR_EMOJI);
}

// "TextInputType.none" is a made-up input type that's typically
// used when there's an in-app virtual keyboard. If
// "TextInputType.none" is specified, disable the system
// keyboard.
static BOOL ShouldShowSystemKeyboard(NSDictionary* type) {
  NSString* inputType = type[@"name"];
  return ![inputType isEqualToString:@"TextInputType.none"];
}
static UIKeyboardType ToUIKeyboardType(NSDictionary* type) {
  NSString* inputType = type[@"name"];
  if ([inputType isEqualToString:@"TextInputType.address"]) {
    return UIKeyboardTypeDefault;
  }
  if ([inputType isEqualToString:@"TextInputType.datetime"]) {
    return UIKeyboardTypeNumbersAndPunctuation;
  }
  if ([inputType isEqualToString:@"TextInputType.emailAddress"]) {
    return UIKeyboardTypeEmailAddress;
  }
  if ([inputType isEqualToString:@"TextInputType.multiline"]) {
    return UIKeyboardTypeDefault;
  }
  if ([inputType isEqualToString:@"TextInputType.name"]) {
    return UIKeyboardTypeNamePhonePad;
  }
  if ([inputType isEqualToString:@"TextInputType.number"]) {
    if ([type[@"signed"] boolValue]) {
      return UIKeyboardTypeNumbersAndPunctuation;
    }
    if ([type[@"decimal"] boolValue]) {
      return UIKeyboardTypeDecimalPad;
    }
    return UIKeyboardTypeNumberPad;
  }
  if ([inputType isEqualToString:@"TextInputType.phone"]) {
    return UIKeyboardTypePhonePad;
  }
  if ([inputType isEqualToString:@"TextInputType.text"]) {
    return UIKeyboardTypeDefault;
  }
  if ([inputType isEqualToString:@"TextInputType.url"]) {
    return UIKeyboardTypeURL;
  }
  if ([inputType isEqualToString:@"TextInputType.visiblePassword"]) {
    return UIKeyboardTypeASCIICapable;
  }
  if ([inputType isEqualToString:@"TextInputType.webSearch"]) {
    return UIKeyboardTypeWebSearch;
  }
  return UIKeyboardTypeDefault;
}

static UITextAutocapitalizationType ToUITextAutoCapitalizationType(NSDictionary* type) {
  NSString* textCapitalization = type[@"textCapitalization"];
  if ([textCapitalization isEqualToString:@"TextCapitalization.characters"]) {
    return UITextAutocapitalizationTypeAllCharacters;
  } else if ([textCapitalization isEqualToString:@"TextCapitalization.sentences"]) {
    return UITextAutocapitalizationTypeSentences;
  } else if ([textCapitalization isEqualToString:@"TextCapitalization.words"]) {
    return UITextAutocapitalizationTypeWords;
  }
  return UITextAutocapitalizationTypeNone;
}

static UIReturnKeyType ToUIReturnKeyType(NSString* inputType) {
  // Where did the term "unspecified" come from? iOS has a "default" and Android
  // has "unspecified." These 2 terms seem to mean the same thing but we need
  // to pick just one. "unspecified" was chosen because "default" is often a
  // reserved word in languages with switch statements (dart, java, etc).
  if ([inputType isEqualToString:@"TextInputAction.unspecified"]) {
    return UIReturnKeyDefault;
  }

  if ([inputType isEqualToString:@"TextInputAction.done"]) {
    return UIReturnKeyDone;
  }

  if ([inputType isEqualToString:@"TextInputAction.go"]) {
    return UIReturnKeyGo;
  }

  if ([inputType isEqualToString:@"TextInputAction.send"]) {
    return UIReturnKeySend;
  }

  if ([inputType isEqualToString:@"TextInputAction.search"]) {
    return UIReturnKeySearch;
  }

  if ([inputType isEqualToString:@"TextInputAction.next"]) {
    return UIReturnKeyNext;
  }

  if ([inputType isEqualToString:@"TextInputAction.continueAction"]) {
    return UIReturnKeyContinue;
  }

  if ([inputType isEqualToString:@"TextInputAction.join"]) {
    return UIReturnKeyJoin;
  }

  if ([inputType isEqualToString:@"TextInputAction.route"]) {
    return UIReturnKeyRoute;
  }

  if ([inputType isEqualToString:@"TextInputAction.emergencyCall"]) {
    return UIReturnKeyEmergencyCall;
  }

  if ([inputType isEqualToString:@"TextInputAction.newline"]) {
    return UIReturnKeyDefault;
  }

  // Present default key if bad input type is given.
  return UIReturnKeyDefault;
}

static UITextContentType ToUITextContentType(NSArray<NSString*>* hints) {
  if (!hints || hints.count == 0) {
    // If no hints are specified, use the default content type nil.
    return nil;
  }

  NSString* hint = hints[0];
  if ([hint isEqualToString:@"addressCityAndState"]) {
    return UITextContentTypeAddressCityAndState;
  }

  if ([hint isEqualToString:@"addressState"]) {
    return UITextContentTypeAddressState;
  }

  if ([hint isEqualToString:@"addressCity"]) {
    return UITextContentTypeAddressCity;
  }

  if ([hint isEqualToString:@"sublocality"]) {
    return UITextContentTypeSublocality;
  }

  if ([hint isEqualToString:@"streetAddressLine1"]) {
    return UITextContentTypeStreetAddressLine1;
  }

  if ([hint isEqualToString:@"streetAddressLine2"]) {
    return UITextContentTypeStreetAddressLine2;
  }

  if ([hint isEqualToString:@"countryName"]) {
    return UITextContentTypeCountryName;
  }

  if ([hint isEqualToString:@"fullStreetAddress"]) {
    return UITextContentTypeFullStreetAddress;
  }

  if ([hint isEqualToString:@"postalCode"]) {
    return UITextContentTypePostalCode;
  }

  if ([hint isEqualToString:@"location"]) {
    return UITextContentTypeLocation;
  }

  if ([hint isEqualToString:@"creditCardNumber"]) {
    return UITextContentTypeCreditCardNumber;
  }

  if ([hint isEqualToString:@"email"]) {
    return UITextContentTypeEmailAddress;
  }

  if ([hint isEqualToString:@"jobTitle"]) {
    return UITextContentTypeJobTitle;
  }

  if ([hint isEqualToString:@"givenName"]) {
    return UITextContentTypeGivenName;
  }

  if ([hint isEqualToString:@"middleName"]) {
    return UITextContentTypeMiddleName;
  }

  if ([hint isEqualToString:@"familyName"]) {
    return UITextContentTypeFamilyName;
  }

  if ([hint isEqualToString:@"name"]) {
    return UITextContentTypeName;
  }

  if ([hint isEqualToString:@"namePrefix"]) {
    return UITextContentTypeNamePrefix;
  }

  if ([hint isEqualToString:@"nameSuffix"]) {
    return UITextContentTypeNameSuffix;
  }

  if ([hint isEqualToString:@"nickname"]) {
    return UITextContentTypeNickname;
  }

  if ([hint isEqualToString:@"organizationName"]) {
    return UITextContentTypeOrganizationName;
  }

  if ([hint isEqualToString:@"telephoneNumber"]) {
    return UITextContentTypeTelephoneNumber;
  }

  if ([hint isEqualToString:@"password"]) {
    return UITextContentTypePassword;
  }

  if ([hint isEqualToString:@"oneTimeCode"]) {
    return UITextContentTypeOneTimeCode;
  }

  if ([hint isEqualToString:@"newPassword"]) {
    return UITextContentTypeNewPassword;
  }

  return hints[0];
}

// Retrieves the autofillId from an input field's configuration. Returns
// nil if the field is nil and the input field is not a password field.
static NSString* AutofillIdFromDictionary(NSDictionary* dictionary) {
  NSDictionary* autofill = dictionary[kAutofillProperties];
  if (autofill) {
    return autofill[kAutofillId];
  }

  // When autofill is nil, the field may still need an autofill id
  // if the field is for password.
  return [dictionary[kSecureTextEntry] boolValue] ? @"password" : nil;
}

// # Autofill Implementation Notes:
//
// Currently there're 2 types of autofills on iOS:
// - Regular autofill, including contact information and one-time-code,
//   takes place in the form of predictive text in the quick type bar.
//   This type of autofill does not save user input, and the keyboard
//   currently only populates the focused field when a predictive text entry
//   is selected by the user.
//
// - Password autofill, includes automatic strong password and regular
//   password autofill. The former happens automatically when a
//   "new password" field is detected and focused, and only that password
//   field will be populated. The latter appears in the quick type bar when
//   an eligible input field (which either has a UITextContentTypePassword
//   contentType, or is a secure text entry) becomes the first responder, and may
//   fill both the username and the password fields. iOS will attempt
//   to save user input for both kinds of password fields. It's relatively
//   tricky to deal with password autofill since it can autofill more than one
//   field at a time and may employ heuristics based on what other text fields
//   are in the same view controller.
//
// When a flutter text field is focused, and autofill is not explicitly disabled
// for it ("autofillable"), the framework collects its attributes and checks if
// it's in an AutofillGroup, and collects the attributes of other autofillable
// text fields in the same AutofillGroup if so. The attributes are sent to the
// text input plugin via a "TextInput.setClient" platform channel message. If
// autofill is disabled for a text field, its "autofill" field will be nil in
// the configuration json.
//
// The text input plugin then tries to determine which kind of autofill the text
// field needs. If the AutofillGroup the text field belongs to contains an
// autofillable text field that's password related, this text 's autofill type
// will be kFlutterAutofillTypePassword. If autofill is disabled for a text field,
// then its type will be kFlutterAutofillTypeNone. Otherwise the text field will
// have an autofill type of kFlutterAutofillTypeRegular.
//
// The text input plugin creates a new UIView for every kFlutterAutofillTypeNone
// text field. The UIView instance is never reused for other flutter text fields
// since the software keyboard often uses the identity of a UIView to distinguish
// different views and provides the same predictive text suggestions or restore
// the composing region if a UIView is reused for a different flutter text field.
//
// The text input plugin creates a new "autofill context" if the text field has
// the type of kFlutterAutofillTypePassword, to represent the AutofillGroup of
// the text field, and creates one FlutterTextInputView for every text field in
// the AutofillGroup.
//
// The text input plugin will try to reuse a UIView if a flutter text field's
// type is kFlutterAutofillTypeRegular, and has the same autofill id.
typedef NS_ENUM(NSInteger, FlutterAutofillType) {
  // The field does not have autofillable content. Additionally if
  // the field is currently in the autofill context, it will be
  // removed from the context without triggering autofill save.
  kFlutterAutofillTypeNone,
  kFlutterAutofillTypeRegular,
  kFlutterAutofillTypePassword,
};

static BOOL IsFieldPasswordRelated(NSDictionary* configuration) {
  // Autofill is explicitly disabled if the id isn't present.
  if (!AutofillIdFromDictionary(configuration)) {
    return NO;
  }

  BOOL isSecureTextEntry = [configuration[kSecureTextEntry] boolValue];
  if (isSecureTextEntry) {
    return YES;
  }

  NSDictionary* autofill = configuration[kAutofillProperties];
  UITextContentType contentType = ToUITextContentType(autofill[kAutofillHints]);

  if ([contentType isEqualToString:UITextContentTypePassword] ||
      [contentType isEqualToString:UITextContentTypeUsername]) {
    return YES;
  }

  if ([contentType isEqualToString:UITextContentTypeNewPassword]) {
    return YES;
  }

  return NO;
}

static FlutterAutofillType AutofillTypeOf(NSDictionary* configuration) {
  for (NSDictionary* field in configuration[kAssociatedAutofillFields]) {
    if (IsFieldPasswordRelated(field)) {
      return kFlutterAutofillTypePassword;
    }
  }

  if (IsFieldPasswordRelated(configuration)) {
    return kFlutterAutofillTypePassword;
  }

  NSDictionary* autofill = configuration[kAutofillProperties];
  UITextContentType contentType = ToUITextContentType(autofill[kAutofillHints]);
  return !autofill || [contentType isEqualToString:@""] ? kFlutterAutofillTypeNone
                                                        : kFlutterAutofillTypeRegular;
}

static BOOL IsApproximatelyEqual(float x, float y, float delta) {
  return fabsf(x - y) <= delta;
}

// This is a helper function for floating cursor selection logic to determine which text
// position is closer to a point.
// Checks whether point should be considered closer to selectionRect compared to
// otherSelectionRect.
//
// If `useTrailingBoundaryOfSelectionRect` is not set, it uses the leading-center point
// on selectionRect and otherSelectionRect to compare.
// For left-to-right text, this means the left-center point, and for right-to-left text,
// this means the right-center point.
//
// If useTrailingBoundaryOfSelectionRect is set, the trailing-center point on selectionRect
// will be used instead of the leading-center point, while leading-center point is still used
// for otherSelectionRect.
//
// This uses special (empirically determined using a 1st gen iPad pro, 9.7" model running
// iOS 14.7.1) logic for determining the closer rect, rather than a simple distance calculation.
// - First, the rect with closer y distance wins.
// - Otherwise (same y distance):
//   - If the point is above bottom of the rect, the rect boundary with closer x distance wins.
//   - Otherwise (point is below bottom of the rect), the rect boundary with farthest x wins.
//     This is because when the point is below the bottom line of text, we want to select the
//     whole line of text, so we mark the farthest rect as closest.
static BOOL IsSelectionRectBoundaryCloserToPoint(CGPoint point,
                                                 CGRect selectionRect,
                                                 BOOL selectionRectIsRTL,
                                                 BOOL useTrailingBoundaryOfSelectionRect,
                                                 CGRect otherSelectionRect,
                                                 BOOL otherSelectionRectIsRTL,
                                                 CGFloat verticalPrecision) {
  // The point is inside the selectionRect's corresponding half-rect area.
  if (CGRectContainsPoint(
          CGRectMake(
              selectionRect.origin.x + ((useTrailingBoundaryOfSelectionRect ^ selectionRectIsRTL)
                                            ? 0.5 * selectionRect.size.width
                                            : 0),
              selectionRect.origin.y, 0.5 * selectionRect.size.width, selectionRect.size.height),
          point)) {
    return YES;
  }
  // pointForSelectionRect is either leading-center or trailing-center point of selectionRect.
  CGPoint pointForSelectionRect = CGPointMake(
      selectionRect.origin.x +
          (selectionRectIsRTL ^ useTrailingBoundaryOfSelectionRect ? selectionRect.size.width : 0),
      selectionRect.origin.y + selectionRect.size.height * 0.5);
  float yDist = fabs(pointForSelectionRect.y - point.y);
  float xDist = fabs(pointForSelectionRect.x - point.x);

  // pointForOtherSelectionRect is the leading-center point of otherSelectionRect.
  CGPoint pointForOtherSelectionRect = CGPointMake(
      otherSelectionRect.origin.x + (otherSelectionRectIsRTL ? otherSelectionRect.size.width : 0),
      otherSelectionRect.origin.y + otherSelectionRect.size.height * 0.5);
  float yDistOther = fabs(pointForOtherSelectionRect.y - point.y);
  float xDistOther = fabs(pointForOtherSelectionRect.x - point.x);

  // This serves a similar purpose to IsApproximatelyEqual, allowing a little buffer before
  // declaring something closer vertically to account for the small variations in size and position
  // of SelectionRects, especially when dealing with emoji.
  BOOL isCloserVertically = yDist < yDistOther - verticalPrecision;
  BOOL isEqualVertically = IsApproximatelyEqual(yDist, yDistOther, verticalPrecision);
  BOOL isAboveBottomOfLine = point.y <= selectionRect.origin.y + selectionRect.size.height;
  BOOL isCloserHorizontally = xDist < xDistOther;
  BOOL isBelowBottomOfLine = point.y > selectionRect.origin.y + selectionRect.size.height;
  // Is "farther away", or is closer to the end of the text line.
  BOOL isFarther;
  if (selectionRectIsRTL) {
    isFarther = selectionRect.origin.x < otherSelectionRect.origin.x;
  } else {
    isFarther = selectionRect.origin.x +
                    (useTrailingBoundaryOfSelectionRect ? selectionRect.size.width : 0) >
                otherSelectionRect.origin.x;
  }
  return (isCloserVertically ||
          (isEqualVertically &&
           ((isAboveBottomOfLine && isCloserHorizontally) || (isBelowBottomOfLine && isFarther))));
}

#pragma mark - FlutterTextPosition

@implementation FlutterTextPosition

+ (instancetype)positionWithIndex:(NSUInteger)index {
  return [[FlutterTextPosition alloc] initWithIndex:index affinity:UITextStorageDirectionForward];
}

+ (instancetype)positionWithIndex:(NSUInteger)index affinity:(UITextStorageDirection)affinity {
  return [[FlutterTextPosition alloc] initWithIndex:index affinity:affinity];
}

- (instancetype)initWithIndex:(NSUInteger)index affinity:(UITextStorageDirection)affinity {
  self = [super init];
  if (self) {
    _index = index;
    _affinity = affinity;
  }
  return self;
}

@end

#pragma mark - FlutterTextRange

@implementation FlutterTextRange

+ (instancetype)rangeWithNSRange:(NSRange)range {
  return [[FlutterTextRange alloc] initWithNSRange:range];
}

- (instancetype)initWithNSRange:(NSRange)range {
  self = [super init];
  if (self) {
    _range = range;
  }
  return self;
}

- (UITextPosition*)start {
  return [FlutterTextPosition positionWithIndex:self.range.location
                                       affinity:UITextStorageDirectionForward];
}

- (UITextPosition*)end {
  return [FlutterTextPosition positionWithIndex:self.range.location + self.range.length
                                       affinity:UITextStorageDirectionBackward];
}

- (BOOL)isEmpty {
  return self.range.length == 0;
}

- (id)copyWithZone:(NSZone*)zone {
  return [[FlutterTextRange allocWithZone:zone] initWithNSRange:self.range];
}

- (BOOL)isEqualTo:(FlutterTextRange*)other {
  return NSEqualRanges(self.range, other.range);
}
@end

#pragma mark - FlutterTokenizer

@interface FlutterTokenizer ()

@property(nonatomic, weak) FlutterTextInputView* textInputView;

@end

@implementation FlutterTokenizer

- (instancetype)initWithTextInput:(UIResponder<UITextInput>*)textInput {
  NSAssert([textInput isKindOfClass:[FlutterTextInputView class]],
           @"The FlutterTokenizer can only be used in a FlutterTextInputView");
  self = [super initWithTextInput:textInput];
  if (self) {
    _textInputView = (FlutterTextInputView*)textInput;
  }
  return self;
}

- (UITextRange*)rangeEnclosingPosition:(UITextPosition*)position
                       withGranularity:(UITextGranularity)granularity
                           inDirection:(UITextDirection)direction {
  UITextRange* result;
  switch (granularity) {
    case UITextGranularityLine:
      // The default UITextInputStringTokenizer does not handle line granularity
      // correctly. We need to implement our own line tokenizer.
      result = [self lineEnclosingPosition:position inDirection:direction];
      break;
    case UITextGranularityCharacter:
    case UITextGranularityWord:
    case UITextGranularitySentence:
    case UITextGranularityParagraph:
    case UITextGranularityDocument:
      // The UITextInputStringTokenizer can handle all these cases correctly.
      result = [super rangeEnclosingPosition:position
                             withGranularity:granularity
                                 inDirection:direction];
      break;
  }
  return result;
}

- (UITextRange*)lineEnclosingPosition:(UITextPosition*)position
                          inDirection:(UITextDirection)direction {
  // TODO(hellohuanlin): remove iOS 17 check. The same logic should apply to older iOS version.
  if (@available(iOS 17.0, *)) {
    // According to the API doc if the text position is at a text-unit boundary, it is considered
    // enclosed only if the next position in the given direction is entirely enclosed. Link:
    // https://developer.apple.com/documentation/uikit/uitextinputtokenizer/1614464-rangeenclosingposition?language=objc
    FlutterTextPosition* flutterPosition = (FlutterTextPosition*)position;
    if (flutterPosition.index > _textInputView.text.length ||
        (flutterPosition.index == _textInputView.text.length &&
         direction == UITextStorageDirectionForward)) {
      return nil;
    }
  }

  // Gets the first line break position after the input position.
  NSString* textAfter = [_textInputView
      textInRange:[_textInputView textRangeFromPosition:position
                                             toPosition:[_textInputView endOfDocument]]];
  NSArray<NSString*>* linesAfter = [textAfter componentsSeparatedByString:@"\n"];
  NSInteger offSetToLineBreak = [linesAfter firstObject].length;
  UITextPosition* lineBreakAfter = [_textInputView positionFromPosition:position
                                                                 offset:offSetToLineBreak];
  // Gets the first line break position before the input position.
  NSString* textBefore = [_textInputView
      textInRange:[_textInputView textRangeFromPosition:[_textInputView beginningOfDocument]
                                             toPosition:position]];
  NSArray<NSString*>* linesBefore = [textBefore componentsSeparatedByString:@"\n"];
  NSInteger offSetFromLineBreak = [linesBefore lastObject].length;
  UITextPosition* lineBreakBefore = [_textInputView positionFromPosition:position
                                                                  offset:-offSetFromLineBreak];

  return [_textInputView textRangeFromPosition:lineBreakBefore toPosition:lineBreakAfter];
}

@end

#pragma mark - FlutterTextSelectionRect

@implementation FlutterTextSelectionRect

// Synthesize properties declared readonly in UITextSelectionRect.
@synthesize rect = _rect;
@synthesize writingDirection = _writingDirection;
@synthesize containsStart = _containsStart;
@synthesize containsEnd = _containsEnd;
@synthesize isVertical = _isVertical;

+ (instancetype)selectionRectWithRectAndInfo:(CGRect)rect
                                    position:(NSUInteger)position
                            writingDirection:(NSWritingDirection)writingDirection
                               containsStart:(BOOL)containsStart
                                 containsEnd:(BOOL)containsEnd
                                  isVertical:(BOOL)isVertical {
  return [[FlutterTextSelectionRect alloc] initWithRectAndInfo:rect
                                                      position:position
                                              writingDirection:writingDirection
                                                 containsStart:containsStart
                                                   containsEnd:containsEnd
                                                    isVertical:isVertical];
}

+ (instancetype)selectionRectWithRect:(CGRect)rect position:(NSUInteger)position {
  return [[FlutterTextSelectionRect alloc] initWithRectAndInfo:rect
                                                      position:position
                                              writingDirection:NSWritingDirectionNatural
                                                 containsStart:NO
                                                   containsEnd:NO
                                                    isVertical:NO];
}

+ (instancetype)selectionRectWithRect:(CGRect)rect
                             position:(NSUInteger)position
                     writingDirection:(NSWritingDirection)writingDirection {
  return [[FlutterTextSelectionRect alloc] initWithRectAndInfo:rect
                                                      position:position
                                              writingDirection:writingDirection
                                                 containsStart:NO
                                                   containsEnd:NO
                                                    isVertical:NO];
}

- (instancetype)initWithRectAndInfo:(CGRect)rect
                           position:(NSUInteger)position
                   writingDirection:(NSWritingDirection)writingDirection
                      containsStart:(BOOL)containsStart
                        containsEnd:(BOOL)containsEnd
                         isVertical:(BOOL)isVertical {
  self = [super init];
  if (self) {
    self.rect = rect;
    self.position = position;
    self.writingDirection = writingDirection;
    self.containsStart = containsStart;
    self.containsEnd = containsEnd;
    self.isVertical = isVertical;
  }
  return self;
}

- (BOOL)isRTL {
  return _writingDirection == NSWritingDirectionRightToLeft;
}

@end

#pragma mark - FlutterTextPlaceholder

@implementation FlutterTextPlaceholder

- (NSArray<UITextSelectionRect*>*)rects {
  // Returning anything other than an empty array here seems to cause PencilKit to enter an
  // infinite loop of allocating placeholders until the app crashes
  return @[];
}

@end

// A FlutterTextInputView that masquerades as a UITextField, and forwards
// selectors it can't respond to a shared UITextField instance.
//
// Relevant API docs claim that password autofill supports any custom view
// that adopts the UITextInput protocol, automatic strong password seems to
// currently only support UITextFields, and password saving only supports
// UITextFields and UITextViews, as of iOS 13.5.
@interface FlutterSecureTextInputView : FlutterTextInputView
@property(nonatomic, retain, readonly) UITextField* textField;
@end

@implementation FlutterSecureTextInputView {
  UITextField* _textField;
}

- (UITextField*)textField {
  if (!_textField) {
    _textField = [[UITextField alloc] init];
  }
  return _textField;
}

- (BOOL)isKindOfClass:(Class)aClass {
  return [super isKindOfClass:aClass] || (aClass == [UITextField class]);
}

- (NSMethodSignature*)methodSignatureForSelector:(SEL)aSelector {
  NSMethodSignature* signature = [super methodSignatureForSelector:aSelector];
  if (!signature) {
    signature = [self.textField methodSignatureForSelector:aSelector];
  }
  return signature;
}

- (void)forwardInvocation:(NSInvocation*)anInvocation {
  [anInvocation invokeWithTarget:self.textField];
}

@end

@interface FlutterTextInputPlugin ()
@property(nonatomic, readonly, weak) id<FlutterTextInputDelegate> textInputDelegate;
@property(nonatomic, readonly) UIView* hostView;
@end

@interface FlutterTextInputView ()
@property(nonatomic, readonly, weak) FlutterTextInputPlugin* textInputPlugin;
@property(nonatomic, copy) NSString* autofillId;
@property(nonatomic, readonly) CATransform3D editableTransform;
@property(nonatomic, assign) CGRect markedRect;
// Disables the cursor from dismissing when firstResponder is resigned
@property(nonatomic, assign) BOOL preventCursorDismissWhenResignFirstResponder;
@property(nonatomic) BOOL isVisibleToAutofill;
@property(nonatomic, assign) BOOL accessibilityEnabled;
@property(nonatomic, assign) int textInputClient;
// The composed character that is temporarily removed by the keyboard API.
// This is cleared at the start of each keyboard interaction. (Enter a character, delete a character
// etc)
@property(nonatomic, copy) NSString* temporarilyDeletedComposedCharacter;
@property(nonatomic, assign) CGRect editMenuTargetRect;

- (void)setEditableTransform:(NSArray*)matrix;
@end

@implementation FlutterTextInputView {
  int _textInputClient;
  const char* _selectionAffinity;
  FlutterTextRange* _selectedTextRange;
  UIInputViewController* _inputViewController;
  CGRect _cachedFirstRect;
  FlutterScribbleInteractionStatus _scribbleInteractionStatus;
  BOOL _hasPlaceholder;
  // Whether to show the system keyboard when this view
  // becomes the first responder. Typically set to false
  // when the app shows its own in-flutter keyboard.
  bool _isSystemKeyboardEnabled;
  bool _isFloatingCursorActive;
  CGPoint _floatingCursorOffset;
  bool _enableInteractiveSelection;
  UITextInteraction* _textInteraction API_AVAILABLE(ios(13.0));
}

@synthesize tokenizer = _tokenizer;

- (instancetype)initWithOwner:(FlutterTextInputPlugin*)textInputPlugin {
  self = [super initWithFrame:CGRectZero];
  if (self) {
    _textInputPlugin = textInputPlugin;
    _textInputClient = 0;
    _selectionAffinity = kTextAffinityUpstream;
    _preventCursorDismissWhenResignFirstResponder = NO;

    // UITextInput
    _text = [[NSMutableString alloc] init];
    _selectedTextRange = [[FlutterTextRange alloc] initWithNSRange:NSMakeRange(0, 0)];
    _markedRect = kInvalidFirstRect;
    _cachedFirstRect = kInvalidFirstRect;
    _scribbleInteractionStatus = FlutterScribbleInteractionStatusNone;
    _pendingDeltas = [[NSMutableArray alloc] init];
    // Initialize with the zero matrix which is not
    // an affine transform.
    _editableTransform = CATransform3D();

    // UITextInputTraits
    _autocapitalizationType = UITextAutocapitalizationTypeSentences;
    _autocorrectionType = UITextAutocorrectionTypeDefault;
    _spellCheckingType = UITextSpellCheckingTypeDefault;
    _enablesReturnKeyAutomatically = NO;
    _keyboardAppearance = UIKeyboardAppearanceDefault;
    _keyboardType = UIKeyboardTypeDefault;
    _returnKeyType = UIReturnKeyDone;
    _secureTextEntry = NO;
    _enableDeltaModel = NO;
    _enableInteractiveSelection = YES;
    _accessibilityEnabled = NO;
    _smartQuotesType = UITextSmartQuotesTypeYes;
    _smartDashesType = UITextSmartDashesTypeYes;
    _selectionRects = [[NSArray alloc] init];

    if (@available(iOS 14.0, *)) {
      UIScribbleInteraction* interaction = [[UIScribbleInteraction alloc] initWithDelegate:self];
      [self addInteraction:interaction];
    }
  }

  if (@available(iOS 16.0, *)) {
    _editMenuInteraction = [[UIEditMenuInteraction alloc] initWithDelegate:self];
    [self addInteraction:_editMenuInteraction];
  }

  return self;
}

- (UIMenu*)editMenuInteraction:(UIEditMenuInteraction*)interaction
          menuForConfiguration:(UIEditMenuConfiguration*)configuration
              suggestedActions:(NSArray<UIMenuElement*>*)suggestedActions API_AVAILABLE(ios(16.0)) {
  return [UIMenu menuWithChildren:suggestedActions];
}

- (void)editMenuInteraction:(UIEditMenuInteraction*)interaction
    willDismissMenuForConfiguration:(UIEditMenuConfiguration*)configuration
                           animator:(id<UIEditMenuInteractionAnimating>)animator
    API_AVAILABLE(ios(16.0)) {
  [self.textInputDelegate flutterTextInputView:self
        willDismissEditMenuWithTextInputClient:_textInputClient];
}

- (CGRect)editMenuInteraction:(UIEditMenuInteraction*)interaction
    targetRectForConfiguration:(UIEditMenuConfiguration*)configuration API_AVAILABLE(ios(16.0)) {
  return _editMenuTargetRect;
}

- (void)showEditMenuWithTargetRect:(CGRect)targetRect API_AVAILABLE(ios(16.0)) {
  _editMenuTargetRect = targetRect;
  UIEditMenuConfiguration* config =
      [UIEditMenuConfiguration configurationWithIdentifier:nil sourcePoint:CGPointZero];
  [self.editMenuInteraction presentEditMenuWithConfiguration:config];
}

- (void)hideEditMenu API_AVAILABLE(ios(16.0)) {
  [self.editMenuInteraction dismissMenu];
}

- (void)configureWithDictionary:(NSDictionary*)configuration {
  NSDictionary* inputType = configuration[kKeyboardType];
  NSString* keyboardAppearance = configuration[kKeyboardAppearance];
  NSDictionary* autofill = configuration[kAutofillProperties];

  self.secureTextEntry = [configuration[kSecureTextEntry] boolValue];
  self.enableDeltaModel = [configuration[kEnableDeltaModel] boolValue];

  _isSystemKeyboardEnabled = ShouldShowSystemKeyboard(inputType);
  self.keyboardType = ToUIKeyboardType(inputType);
  self.returnKeyType = ToUIReturnKeyType(configuration[kInputAction]);
  self.autocapitalizationType = ToUITextAutoCapitalizationType(configuration);
  _enableInteractiveSelection = [configuration[kEnableInteractiveSelection] boolValue];
  NSString* smartDashesType = configuration[kSmartDashesType];
  // This index comes from the SmartDashesType enum in the framework.
  bool smartDashesIsDisabled = smartDashesType && [smartDashesType isEqualToString:@"0"];
  self.smartDashesType = smartDashesIsDisabled ? UITextSmartDashesTypeNo : UITextSmartDashesTypeYes;
  NSString* smartQuotesType = configuration[kSmartQuotesType];
  // This index comes from the SmartQuotesType enum in the framework.
  bool smartQuotesIsDisabled = smartQuotesType && [smartQuotesType isEqualToString:@"0"];
  self.smartQuotesType = smartQuotesIsDisabled ? UITextSmartQuotesTypeNo : UITextSmartQuotesTypeYes;
  if ([keyboardAppearance isEqualToString:@"Brightness.dark"]) {
    self.keyboardAppearance = UIKeyboardAppearanceDark;
  } else if ([keyboardAppearance isEqualToString:@"Brightness.light"]) {
    self.keyboardAppearance = UIKeyboardAppearanceLight;
  } else {
    self.keyboardAppearance = UIKeyboardAppearanceDefault;
  }
  NSString* autocorrect = configuration[kAutocorrectionType];
  bool autocorrectIsDisabled = autocorrect && ![autocorrect boolValue];
  self.autocorrectionType =
      autocorrectIsDisabled ? UITextAutocorrectionTypeNo : UITextAutocorrectionTypeDefault;
  self.spellCheckingType =
      autocorrectIsDisabled ? UITextSpellCheckingTypeNo : UITextSpellCheckingTypeDefault;
  self.autofillId = AutofillIdFromDictionary(configuration);
  if (autofill == nil) {
    self.textContentType = @"";
  } else {
    self.textContentType = ToUITextContentType(autofill[kAutofillHints]);
    [self setTextInputState:autofill[kAutofillEditingValue]];
    NSAssert(_autofillId, @"The autofill configuration must contain an autofill id");
  }
  // The input field needs to be visible for the system autofill
  // to find it.
  self.isVisibleToAutofill = autofill || _secureTextEntry;
}

- (UITextContentType)textContentType {
  return _textContentType;
}

// Prevent UIKit from showing selection handles or highlights. This is needed
// because Scribble interactions require the view to have it's actual frame on
// the screen. They're not needed on iOS 17 with the new
// UITextSelectionDisplayInteraction API.
//
// These are undocumented methods. On iOS 17, the insertion point color is also
// used as the highlighted background of the selected IME candidate:
// https://github.com/flutter/flutter/issues/132548
// So the respondsToSelector method is overridden to return NO for this method
// on iOS 17+.
- (UIColor*)insertionPointColor {
  return [UIColor clearColor];
}

- (UIColor*)selectionBarColor {
  return [UIColor clearColor];
}

- (UIColor*)selectionHighlightColor {
  return [UIColor clearColor];
}

- (UIInputViewController*)inputViewController {
  if (_isSystemKeyboardEnabled) {
    return nil;
  }

  if (!_inputViewController) {
    _inputViewController = [[UIInputViewController alloc] init];
  }
  return _inputViewController;
}

- (id<FlutterTextInputDelegate>)textInputDelegate {
  return _textInputPlugin.textInputDelegate;
}

- (BOOL)respondsToSelector:(SEL)selector {
  if (@available(iOS 17.0, *)) {
    // See the comment on this method.
    if (selector == @selector(insertionPointColor)) {
      return NO;
    }
  }
  return [super respondsToSelector:selector];
}

- (void)setTextInputClient:(int)client {
  _textInputClient = client;
  _hasPlaceholder = NO;
}

- (UITextInteraction*)textInteraction API_AVAILABLE(ios(13.0)) {
  if (!_textInteraction) {
    _textInteraction = [UITextInteraction textInteractionForMode:UITextInteractionModeEditable];
    _textInteraction.textInput = self;
  }
  return _textInteraction;
}

- (void)setTextInputState:(NSDictionary*)state {
  if (@available(iOS 13.0, *)) {
    // [UITextInteraction willMoveToView:] sometimes sets the textInput's inputDelegate
    // to nil. This is likely a bug in UIKit. In order to inform the keyboard of text
    // and selection changes when that happens, add a dummy UITextInteraction to this
    // view so it sets a valid inputDelegate that we can call textWillChange et al. on.
    // See https://github.com/flutter/engine/pull/32881.
    if (!self.inputDelegate && self.isFirstResponder) {
      [self addInteraction:self.textInteraction];
    }
  }

  NSString* newText = state[@"text"];
  BOOL textChanged = ![self.text isEqualToString:newText];
  if (textChanged) {
    [self.inputDelegate textWillChange:self];
    [self.text setString:newText];
  }
  NSInteger composingBase = [state[@"composingBase"] intValue];
  NSInteger composingExtent = [state[@"composingExtent"] intValue];
  NSRange composingRange = [self clampSelection:NSMakeRange(MIN(composingBase, composingExtent),
                                                            ABS(composingBase - composingExtent))
                                        forText:self.text];

  self.markedTextRange =
      composingRange.length > 0 ? [FlutterTextRange rangeWithNSRange:composingRange] : nil;

  NSRange selectedRange = [self clampSelectionFromBase:[state[@"selectionBase"] intValue]
                                                extent:[state[@"selectionExtent"] intValue]
                                               forText:self.text];

  NSRange oldSelectedRange = [(FlutterTextRange*)self.selectedTextRange range];
  if (!NSEqualRanges(selectedRange, oldSelectedRange)) {
    [self.inputDelegate selectionWillChange:self];

    [self setSelectedTextRangeLocal:[FlutterTextRange rangeWithNSRange:selectedRange]];

    _selectionAffinity = kTextAffinityDownstream;
    if ([state[@"selectionAffinity"] isEqualToString:@(kTextAffinityUpstream)]) {
      _selectionAffinity = kTextAffinityUpstream;
    }
    [self.inputDelegate selectionDidChange:self];
  }

  if (textChanged) {
    [self.inputDelegate textDidChange:self];
  }

  if (@available(iOS 13.0, *)) {
    if (_textInteraction) {
      [self removeInteraction:_textInteraction];
    }
  }
}

// Forward touches to the viewResponder to allow tapping inside the UITextField as normal.
- (void)touchesBegan:(NSSet*)touches withEvent:(UIEvent*)event {
  _scribbleFocusStatus = FlutterScribbleFocusStatusUnfocused;
  [self resetScribbleInteractionStatusIfEnding];
  [self.viewResponder touchesBegan:touches withEvent:event];
}

- (void)touchesMoved:(NSSet*)touches withEvent:(UIEvent*)event {
  [self.viewResponder touchesMoved:touches withEvent:event];
}

- (void)touchesEnded:(NSSet*)touches withEvent:(UIEvent*)event {
  [self.viewResponder touchesEnded:touches withEvent:event];
}

- (void)touchesCancelled:(NSSet*)touches withEvent:(UIEvent*)event {
  [self.viewResponder touchesCancelled:touches withEvent:event];
}

- (void)touchesEstimatedPropertiesUpdated:(NSSet*)touches {
  [self.viewResponder touchesEstimatedPropertiesUpdated:touches];
}

// Extracts the selection information from the editing state dictionary.
//
// The state may contain an invalid selection, such as when no selection was
// explicitly set in the framework. This is handled here by setting the
// selection to (0,0). In contrast, Android handles this situation by
// clearing the selection, but the result in both cases is that the cursor
// is placed at the beginning of the field.
- (NSRange)clampSelectionFromBase:(int)selectionBase
                           extent:(int)selectionExtent
                          forText:(NSString*)text {
  int loc = MIN(selectionBase, selectionExtent);
  int len = ABS(selectionExtent - selectionBase);
  return loc < 0 ? NSMakeRange(0, 0)
                 : [self clampSelection:NSMakeRange(loc, len) forText:self.text];
}

- (NSRange)clampSelection:(NSRange)range forText:(NSString*)text {
  NSUInteger start = MIN(MAX(range.location, 0), text.length);
  NSUInteger length = MIN(range.length, text.length - start);
  return NSMakeRange(start, length);
}

- (BOOL)isVisibleToAutofill {
  return self.frame.size.width > 0 && self.frame.size.height > 0;
}

// An input view is generally ignored by password autofill attempts, if it's
// not the first responder and is zero-sized. For input fields that are in the
// autofill context but do not belong to the current autofill group, setting
// their frames to CGRectZero prevents ios autofill from taking them into
// account.
- (void)setIsVisibleToAutofill:(BOOL)isVisibleToAutofill {
  // This probably needs to change (think it is getting overwritten by the updateSizeAndTransform
  // stuff for now).
  self.frame = isVisibleToAutofill ? CGRectMake(0, 0, 1, 1) : CGRectZero;
}

#pragma mark UIScribbleInteractionDelegate

// Checks whether Scribble features are possibly available – meaning this is an iPad running iOS
// 14 or higher.
- (BOOL)isScribbleAvailable {
  if (@available(iOS 14.0, *)) {
    if (UI_USER_INTERFACE_IDIOM() == UIUserInterfaceIdiomPad) {
      return YES;
    }
  }
  return NO;
}

- (void)scribbleInteractionWillBeginWriting:(UIScribbleInteraction*)interaction
    API_AVAILABLE(ios(14.0)) {
  _scribbleInteractionStatus = FlutterScribbleInteractionStatusStarted;
  [self.textInputDelegate flutterTextInputViewScribbleInteractionBegan:self];
}

- (void)scribbleInteractionDidFinishWriting:(UIScribbleInteraction*)interaction
    API_AVAILABLE(ios(14.0)) {
  _scribbleInteractionStatus = FlutterScribbleInteractionStatusEnding;
  [self.textInputDelegate flutterTextInputViewScribbleInteractionFinished:self];
}

- (BOOL)scribbleInteraction:(UIScribbleInteraction*)interaction
      shouldBeginAtLocation:(CGPoint)location API_AVAILABLE(ios(14.0)) {
  return YES;
}

- (BOOL)scribbleInteractionShouldDelayFocus:(UIScribbleInteraction*)interaction
    API_AVAILABLE(ios(14.0)) {
  return NO;
}

#pragma mark - UIResponder Overrides

- (BOOL)canBecomeFirstResponder {
  // Only the currently focused input field can
  // become the first responder. This prevents iOS
  // from changing focus by itself (the framework
  // focus will be out of sync if that happens).
  return _textInputClient != 0;
}

- (BOOL)resignFirstResponder {
  BOOL success = [super resignFirstResponder];
  if (success) {
    if (!_preventCursorDismissWhenResignFirstResponder) {
      [self.textInputDelegate flutterTextInputView:self
          didResignFirstResponderWithTextInputClient:_textInputClient];
    }
  }
  return success;
}

- (BOOL)canPerformAction:(SEL)action withSender:(id)sender {
  if (action == @selector(paste:)) {
    // Forbid pasting images, memojis, or other non-string content.
    return [UIPasteboard generalPasteboard].hasStrings;
  } else if (action == @selector(copy:) || action == @selector(cut:) ||
             action == @selector(delete:)) {
    return [self textInRange:_selectedTextRange].length > 0;
  } else if (action == @selector(selectAll:)) {
    return self.hasText;
  }
  return [super canPerformAction:action withSender:sender];
}

#pragma mark - UIResponderStandardEditActions Overrides

- (void)cut:(id)sender {
  [UIPasteboard generalPasteboard].string = [self textInRange:_selectedTextRange];
  [self replaceRange:_selectedTextRange withText:@""];
}

- (void)copy:(id)sender {
  [UIPasteboard generalPasteboard].string = [self textInRange:_selectedTextRange];
}

- (void)paste:(id)sender {
  NSString* pasteboardString = [UIPasteboard generalPasteboard].string;
  if (pasteboardString != nil) {
    [self insertText:pasteboardString];
  }
}

- (void)delete:(id)sender {
  [self replaceRange:_selectedTextRange withText:@""];
}

- (void)selectAll:(id)sender {
  [self setSelectedTextRange:[self textRangeFromPosition:[self beginningOfDocument]
                                              toPosition:[self endOfDocument]]];
}

#pragma mark - UITextInput Overrides

- (id<UITextInputTokenizer>)tokenizer {
  if (_tokenizer == nil) {
    _tokenizer = [[FlutterTokenizer alloc] initWithTextInput:self];
  }
  return _tokenizer;
}

- (UITextRange*)selectedTextRange {
  return [_selectedTextRange copy];
}

// Change the range of selected text, without notifying the framework.
- (void)setSelectedTextRangeLocal:(UITextRange*)selectedTextRange {
  if (_selectedTextRange != selectedTextRange) {
    if (self.hasText) {
      FlutterTextRange* flutterTextRange = (FlutterTextRange*)selectedTextRange;
      _selectedTextRange = [[FlutterTextRange
          rangeWithNSRange:fml::RangeForCharactersInRange(self.text, flutterTextRange.range)] copy];
    } else {
      _selectedTextRange = [selectedTextRange copy];
    }
  }
}

- (void)setSelectedTextRange:(UITextRange*)selectedTextRange {
  if (!_enableInteractiveSelection) {
    return;
  }

  [self setSelectedTextRangeLocal:selectedTextRange];

  if (_enableDeltaModel) {
    [self updateEditingStateWithDelta:flutter::TextEditingDelta([self.text UTF8String])];
  } else {
    [self updateEditingState];
  }

  if (_scribbleInteractionStatus != FlutterScribbleInteractionStatusNone ||
      _scribbleFocusStatus == FlutterScribbleFocusStatusFocused) {
    NSAssert([selectedTextRange isKindOfClass:[FlutterTextRange class]],
             @"Expected a FlutterTextRange for range (got %@).", [selectedTextRange class]);
    FlutterTextRange* flutterTextRange = (FlutterTextRange*)selectedTextRange;
    if (flutterTextRange.range.length > 0) {
      [self.textInputDelegate flutterTextInputView:self showToolbar:_textInputClient];
    }
  }

  [self resetScribbleInteractionStatusIfEnding];
}

- (id)insertDictationResultPlaceholder {
  return @"";
}

- (void)removeDictationResultPlaceholder:(id)placeholder willInsertResult:(BOOL)willInsertResult {
}

- (NSString*)textInRange:(UITextRange*)range {
  if (!range) {
    return nil;
  }
  NSAssert([range isKindOfClass:[FlutterTextRange class]],
           @"Expected a FlutterTextRange for range (got %@).", [range class]);
  NSRange textRange = ((FlutterTextRange*)range).range;
  NSAssert(textRange.location != NSNotFound, @"Expected a valid text range.");
  // Sanitize the range to prevent going out of bounds.
  NSUInteger location = MIN(textRange.location, self.text.length);
  NSUInteger length = MIN(self.text.length - location, textRange.length);
  NSRange safeRange = NSMakeRange(location, length);
  return [self.text substringWithRange:safeRange];
}

// Replace the text within the specified range with the given text,
// without notifying the framework.
- (void)replaceRangeLocal:(NSRange)range withText:(NSString*)text {
  [self.text replaceCharactersInRange:[self clampSelection:range forText:self.text]
                           withString:text];

  // Adjust the selected range and the marked text range. There's no
  // documentation but UITextField always sets markedTextRange to nil,
  // and collapses the selection to the end of the new replacement text.
  const NSRange newSelectionRange =
      [self clampSelection:NSMakeRange(range.location + text.length, 0) forText:self.text];

  [self setSelectedTextRangeLocal:[FlutterTextRange rangeWithNSRange:newSelectionRange]];
  self.markedTextRange = nil;
}

- (void)replaceRange:(UITextRange*)range withText:(NSString*)text {
  NSString* textBeforeChange = [self.text copy];
  NSRange replaceRange = ((FlutterTextRange*)range).range;
  [self replaceRangeLocal:replaceRange withText:text];
  if (_enableDeltaModel) {
    NSRange nextReplaceRange = [self clampSelection:replaceRange forText:textBeforeChange];
    [self updateEditingStateWithDelta:flutter::TextEditingDelta(
                                          [textBeforeChange UTF8String],
                                          flutter::TextRange(
                                              nextReplaceRange.location,
                                              nextReplaceRange.location + nextReplaceRange.length),
                                          [text UTF8String])];
  } else {
    [self updateEditingState];
  }
}

- (BOOL)shouldChangeTextInRange:(UITextRange*)range replacementText:(NSString*)text {
  // `temporarilyDeletedComposedCharacter` should only be used during a single text change session.
  // So it needs to be cleared at the start of each text editing session.
  self.temporarilyDeletedComposedCharacter = nil;

  if (self.returnKeyType == UIReturnKeyDefault && [text isEqualToString:@"\n"]) {
    [self.textInputDelegate flutterTextInputView:self
                                   performAction:FlutterTextInputActionNewline
                                      withClient:_textInputClient];
    return YES;
  }

  if ([text isEqualToString:@"\n"]) {
    FlutterTextInputAction action;
    switch (self.returnKeyType) {
      case UIReturnKeyDefault:
        action = FlutterTextInputActionUnspecified;
        break;
      case UIReturnKeyDone:
        action = FlutterTextInputActionDone;
        break;
      case UIReturnKeyGo:
        action = FlutterTextInputActionGo;
        break;
      case UIReturnKeySend:
        action = FlutterTextInputActionSend;
        break;
      case UIReturnKeySearch:
      case UIReturnKeyGoogle:
      case UIReturnKeyYahoo:
        action = FlutterTextInputActionSearch;
        break;
      case UIReturnKeyNext:
        action = FlutterTextInputActionNext;
        break;
      case UIReturnKeyContinue:
        action = FlutterTextInputActionContinue;
        break;
      case UIReturnKeyJoin:
        action = FlutterTextInputActionJoin;
        break;
      case UIReturnKeyRoute:
        action = FlutterTextInputActionRoute;
        break;
      case UIReturnKeyEmergencyCall:
        action = FlutterTextInputActionEmergencyCall;
        break;
    }

    [self.textInputDelegate flutterTextInputView:self
                                   performAction:action
                                      withClient:_textInputClient];
    return NO;
  }

  return YES;
}

// Either replaces the existing marked text or, if none is present, inserts it in
// place of the current selection.
- (void)setMarkedText:(NSString*)markedText selectedRange:(NSRange)markedSelectedRange {
  NSString* textBeforeChange = [self.text copy];

  if (_scribbleInteractionStatus != FlutterScribbleInteractionStatusNone ||
      _scribbleFocusStatus != FlutterScribbleFocusStatusUnfocused) {
    return;
  }

  if (markedText == nil) {
    markedText = @"";
  }

  const FlutterTextRange* currentMarkedTextRange = (FlutterTextRange*)self.markedTextRange;
  const NSRange& actualReplacedRange = currentMarkedTextRange && !currentMarkedTextRange.isEmpty
                                           ? currentMarkedTextRange.range
                                           : _selectedTextRange.range;
  // No need to call replaceRangeLocal as this method always adjusts the
  // selected/marked text ranges anyways.
  [self.text replaceCharactersInRange:actualReplacedRange withString:markedText];

  const NSRange newMarkedRange = NSMakeRange(actualReplacedRange.location, markedText.length);
  self.markedTextRange =
      newMarkedRange.length > 0 ? [FlutterTextRange rangeWithNSRange:newMarkedRange] : nil;

  [self setSelectedTextRangeLocal:
            [FlutterTextRange
                rangeWithNSRange:[self clampSelection:NSMakeRange(markedSelectedRange.location +
                                                                      newMarkedRange.location,
                                                                  markedSelectedRange.length)
                                              forText:self.text]]];
  if (_enableDeltaModel) {
    NSRange nextReplaceRange = [self clampSelection:actualReplacedRange forText:textBeforeChange];
    [self updateEditingStateWithDelta:flutter::TextEditingDelta(
                                          [textBeforeChange UTF8String],
                                          flutter::TextRange(
                                              nextReplaceRange.location,
                                              nextReplaceRange.location + nextReplaceRange.length),
                                          [markedText UTF8String])];
  } else {
    [self updateEditingState];
  }
}

- (void)unmarkText {
  if (!self.markedTextRange) {
    return;
  }
  self.markedTextRange = nil;
  if (_enableDeltaModel) {
    [self updateEditingStateWithDelta:flutter::TextEditingDelta([self.text UTF8String])];
  } else {
    [self updateEditingState];
  }
}

- (UITextRange*)textRangeFromPosition:(UITextPosition*)fromPosition
                           toPosition:(UITextPosition*)toPosition {
  NSUInteger fromIndex = ((FlutterTextPosition*)fromPosition).index;
  NSUInteger toIndex = ((FlutterTextPosition*)toPosition).index;
  if (toIndex >= fromIndex) {
    return [FlutterTextRange rangeWithNSRange:NSMakeRange(fromIndex, toIndex - fromIndex)];
  } else {
    // toIndex can be smaller than fromIndex, because
    // UITextInputStringTokenizer does not handle CJK characters
    // well in some cases. See:
    // https://github.com/flutter/flutter/issues/58750#issuecomment-644469521
    // Swap fromPosition and toPosition to match the behavior of native
    // UITextViews.
    return [FlutterTextRange rangeWithNSRange:NSMakeRange(toIndex, fromIndex - toIndex)];
  }
}

- (NSUInteger)decrementOffsetPosition:(NSUInteger)position {
  return fml::RangeForCharacterAtIndex(self.text, MAX(0, position - 1)).location;
}

- (NSUInteger)incrementOffsetPosition:(NSUInteger)position {
  NSRange charRange = fml::RangeForCharacterAtIndex(self.text, position);
  return MIN(position + charRange.length, self.text.length);
}

- (UITextPosition*)positionFromPosition:(UITextPosition*)position offset:(NSInteger)offset {
  NSUInteger offsetPosition = ((FlutterTextPosition*)position).index;

  NSInteger newLocation = (NSInteger)offsetPosition + offset;
  if (newLocation < 0 || newLocation > (NSInteger)self.text.length) {
    return nil;
  }

  if (_scribbleInteractionStatus != FlutterScribbleInteractionStatusNone) {
    return [FlutterTextPosition positionWithIndex:newLocation];
  }

  if (offset >= 0) {
    for (NSInteger i = 0; i < offset && offsetPosition < self.text.length; ++i) {
      offsetPosition = [self incrementOffsetPosition:offsetPosition];
    }
  } else {
    for (NSInteger i = 0; i < ABS(offset) && offsetPosition > 0; ++i) {
      offsetPosition = [self decrementOffsetPosition:offsetPosition];
    }
  }
  return [FlutterTextPosition positionWithIndex:offsetPosition];
}

- (UITextPosition*)positionFromPosition:(UITextPosition*)position
                            inDirection:(UITextLayoutDirection)direction
                                 offset:(NSInteger)offset {
  // TODO(cbracken) Add RTL handling.
  switch (direction) {
    case UITextLayoutDirectionLeft:
    case UITextLayoutDirectionUp:
      return [self positionFromPosition:position offset:offset * -1];
    case UITextLayoutDirectionRight:
    case UITextLayoutDirectionDown:
      return [self positionFromPosition:position offset:1];
  }
}

- (UITextPosition*)beginningOfDocument {
  return [FlutterTextPosition positionWithIndex:0 affinity:UITextStorageDirectionForward];
}

- (UITextPosition*)endOfDocument {
  return [FlutterTextPosition positionWithIndex:self.text.length
                                       affinity:UITextStorageDirectionBackward];
}

- (NSComparisonResult)comparePosition:(UITextPosition*)position toPosition:(UITextPosition*)other {
  NSUInteger positionIndex = ((FlutterTextPosition*)position).index;
  NSUInteger otherIndex = ((FlutterTextPosition*)other).index;
  if (positionIndex < otherIndex) {
    return NSOrderedAscending;
  }
  if (positionIndex > otherIndex) {
    return NSOrderedDescending;
  }
  UITextStorageDirection positionAffinity = ((FlutterTextPosition*)position).affinity;
  UITextStorageDirection otherAffinity = ((FlutterTextPosition*)other).affinity;
  if (positionAffinity == otherAffinity) {
    return NSOrderedSame;
  }
  if (positionAffinity == UITextStorageDirectionBackward) {
    // positionAffinity points backwards, otherAffinity points forwards
    return NSOrderedAscending;
  }
  // positionAffinity points forwards, otherAffinity points backwards
  return NSOrderedDescending;
}

- (NSInteger)offsetFromPosition:(UITextPosition*)from toPosition:(UITextPosition*)toPosition {
  return ((FlutterTextPosition*)toPosition).index - ((FlutterTextPosition*)from).index;
}

- (UITextPosition*)positionWithinRange:(UITextRange*)range
                   farthestInDirection:(UITextLayoutDirection)direction {
  NSUInteger index;
  UITextStorageDirection affinity;
  switch (direction) {
    case UITextLayoutDirectionLeft:
    case UITextLayoutDirectionUp:
      index = ((FlutterTextPosition*)range.start).index;
      affinity = UITextStorageDirectionForward;
      break;
    case UITextLayoutDirectionRight:
    case UITextLayoutDirectionDown:
      index = ((FlutterTextPosition*)range.end).index;
      affinity = UITextStorageDirectionBackward;
      break;
  }
  return [FlutterTextPosition positionWithIndex:index affinity:affinity];
}

- (UITextRange*)characterRangeByExtendingPosition:(UITextPosition*)position
                                      inDirection:(UITextLayoutDirection)direction {
  NSUInteger positionIndex = ((FlutterTextPosition*)position).index;
  NSUInteger startIndex;
  NSUInteger endIndex;
  switch (direction) {
    case UITextLayoutDirectionLeft:
    case UITextLayoutDirectionUp:
      startIndex = [self decrementOffsetPosition:positionIndex];
      endIndex = positionIndex;
      break;
    case UITextLayoutDirectionRight:
    case UITextLayoutDirectionDown:
      startIndex = positionIndex;
      endIndex = [self incrementOffsetPosition:positionIndex];
      break;
  }
  return [FlutterTextRange rangeWithNSRange:NSMakeRange(startIndex, endIndex - startIndex)];
}

#pragma mark - UITextInput text direction handling

- (UITextWritingDirection)baseWritingDirectionForPosition:(UITextPosition*)position
                                              inDirection:(UITextStorageDirection)direction {
  // TODO(cbracken) Add RTL handling.
  return UITextWritingDirectionNatural;
}

- (void)setBaseWritingDirection:(UITextWritingDirection)writingDirection
                       forRange:(UITextRange*)range {
  // TODO(cbracken) Add RTL handling.
}

#pragma mark - UITextInput cursor, selection rect handling

- (void)setMarkedRect:(CGRect)markedRect {
  _markedRect = markedRect;
  // Invalidate the cache.
  _cachedFirstRect = kInvalidFirstRect;
}

// This method expects a 4x4 perspective matrix
// stored in a NSArray in column-major order.
- (void)setEditableTransform:(NSArray*)matrix {
  CATransform3D* transform = &_editableTransform;

  transform->m11 = [matrix[0] doubleValue];
  transform->m12 = [matrix[1] doubleValue];
  transform->m13 = [matrix[2] doubleValue];
  transform->m14 = [matrix[3] doubleValue];

  transform->m21 = [matrix[4] doubleValue];
  transform->m22 = [matrix[5] doubleValue];
  transform->m23 = [matrix[6] doubleValue];
  transform->m24 = [matrix[7] doubleValue];

  transform->m31 = [matrix[8] doubleValue];
  transform->m32 = [matrix[9] doubleValue];
  transform->m33 = [matrix[10] doubleValue];
  transform->m34 = [matrix[11] doubleValue];

  transform->m41 = [matrix[12] doubleValue];
  transform->m42 = [matrix[13] doubleValue];
  transform->m43 = [matrix[14] doubleValue];
  transform->m44 = [matrix[15] doubleValue];

  // Invalidate the cache.
  _cachedFirstRect = kInvalidFirstRect;
}

// Returns the bounding CGRect of the transformed incomingRect, in the view's
// coordinates.
- (CGRect)localRectFromFrameworkTransform:(CGRect)incomingRect {
  CGPoint points[] = {
      incomingRect.origin,
      CGPointMake(incomingRect.origin.x, incomingRect.origin.y + incomingRect.size.height),
      CGPointMake(incomingRect.origin.x + incomingRect.size.width, incomingRect.origin.y),
      CGPointMake(incomingRect.origin.x + incomingRect.size.width,
                  incomingRect.origin.y + incomingRect.size.height)};

  CGPoint origin = CGPointMake(CGFLOAT_MAX, CGFLOAT_MAX);
  CGPoint farthest = CGPointMake(-CGFLOAT_MAX, -CGFLOAT_MAX);

  for (int i = 0; i < 4; i++) {
    const CGPoint point = points[i];

    CGFloat x = _editableTransform.m11 * point.x + _editableTransform.m21 * point.y +
                _editableTransform.m41;
    CGFloat y = _editableTransform.m12 * point.x + _editableTransform.m22 * point.y +
                _editableTransform.m42;

    const CGFloat w = _editableTransform.m14 * point.x + _editableTransform.m24 * point.y +
                      _editableTransform.m44;

    if (w == 0.0) {
      return kInvalidFirstRect;
    } else if (w != 1.0) {
      x /= w;
      y /= w;
    }

    origin.x = MIN(origin.x, x);
    origin.y = MIN(origin.y, y);
    farthest.x = MAX(farthest.x, x);
    farthest.y = MAX(farthest.y, y);
  }
  return CGRectMake(origin.x, origin.y, farthest.x - origin.x, farthest.y - origin.y);
}

// The following methods are required to support force-touch cursor positioning
// and to position the
// candidates view for multi-stage input methods (e.g., Japanese) when using a
// physical keyboard.
// Returns the rect for the queried range, or a subrange through the end of line, if
// the range encompasses multiple lines.
- (CGRect)firstRectForRange:(UITextRange*)range {
  NSAssert([range.start isKindOfClass:[FlutterTextPosition class]],
           @"Expected a FlutterTextPosition for range.start (got %@).", [range.start class]);
  NSAssert([range.end isKindOfClass:[FlutterTextPosition class]],
           @"Expected a FlutterTextPosition for range.end (got %@).", [range.end class]);
  NSUInteger start = ((FlutterTextPosition*)range.start).index;
  NSUInteger end = ((FlutterTextPosition*)range.end).index;
  if (_markedTextRange != nil) {
    // The candidates view can't be shown if the framework has not sent the
    // first caret rect.
    if (CGRectEqualToRect(kInvalidFirstRect, _markedRect)) {
      return kInvalidFirstRect;
    }

    if (CGRectEqualToRect(_cachedFirstRect, kInvalidFirstRect)) {
      // If the width returned is too small, that means the framework sent us
      // the caret rect instead of the marked text rect. Expand it to 0.2 so
      // the IME candidates view would show up.
      CGRect rect = _markedRect;
      if (CGRectIsEmpty(rect)) {
        rect = CGRectInset(rect, -0.1, 0);
      }
      _cachedFirstRect = [self localRectFromFrameworkTransform:rect];
    }

    UIView* hostView = _textInputPlugin.hostView;
    NSAssert(hostView == nil || [self isDescendantOfView:hostView], @"%@ is not a descendant of %@",
             self, hostView);
    return hostView ? [hostView convertRect:_cachedFirstRect toView:self] : _cachedFirstRect;
  }

  if (_scribbleInteractionStatus == FlutterScribbleInteractionStatusNone &&
      _scribbleFocusStatus == FlutterScribbleFocusStatusUnfocused) {
    if (@available(iOS 17.0, *)) {
      // Disable auto-correction highlight feature for iOS 17+.
      // In iOS 17+, whenever a character is inserted or deleted, the system will always query
      // the rect for every single character of the current word.
      // GitHub Issue: https://github.com/flutter/flutter/issues/128406
    } else {
      // This tells the framework to show the highlight for incorrectly spelled word that is
      // about to be auto-corrected.
      // There is no other UITextInput API that informs about the auto-correction highlight.
      // So we simply add the call here as a workaround.
      [self.textInputDelegate flutterTextInputView:self
              showAutocorrectionPromptRectForStart:start
                                               end:end
                                        withClient:_textInputClient];
    }
  }

  // The iOS 16 system highlight does not repect the height returned by `firstRectForRange`
  // API (unlike iOS 17). So we return CGRectZero to hide it (unless if scribble is enabled).
  // To support scribble's advanced gestures (e.g. insert a space with a vertical bar),
  // at least 1 character's width is required.
  if (@available(iOS 17, *)) {
    // No-op
  } else if (![self isScribbleAvailable]) {
    return CGRectZero;
  }

  NSUInteger first = start;
  if (end < start) {
    first = end;
  }

  CGRect startSelectionRect = CGRectNull;
  CGRect endSelectionRect = CGRectNull;
  // Selection rects from different langauges may have different minY/maxY.
  // So we need to iterate through each rects to update minY/maxY.
  CGFloat minY = CGFLOAT_MAX;
  CGFloat maxY = CGFLOAT_MIN;

  FlutterTextRange* textRange = [FlutterTextRange
      rangeWithNSRange:fml::RangeForCharactersInRange(self.text, NSMakeRange(0, self.text.length))];
  for (NSUInteger i = 0; i < [_selectionRects count]; i++) {
    BOOL startsOnOrBeforeStartOfRange = _selectionRects[i].position <= first;
    BOOL isLastSelectionRect = i + 1 == [_selectionRects count];
    BOOL endOfTextIsAfterStartOfRange = isLastSelectionRect && textRange.range.length > first;
    BOOL nextSelectionRectIsAfterStartOfRange =
        !isLastSelectionRect && _selectionRects[i + 1].position > first;
    if (startsOnOrBeforeStartOfRange &&
        (endOfTextIsAfterStartOfRange || nextSelectionRectIsAfterStartOfRange)) {
      // TODO(hellohaunlin): Remove iOS 17 check. The logic should also work for older versions.
      if (@available(iOS 17, *)) {
        startSelectionRect = _selectionRects[i].rect;
      } else {
        return _selectionRects[i].rect;
      }
    }
    if (!CGRectIsNull(startSelectionRect)) {
      minY = fmin(minY, CGRectGetMinY(_selectionRects[i].rect));
      maxY = fmax(maxY, CGRectGetMaxY(_selectionRects[i].rect));
      BOOL endsOnOrAfterEndOfRange = _selectionRects[i].position >= end - 1;  // end is exclusive
      BOOL nextSelectionRectIsOnNextLine =
          !isLastSelectionRect &&
          // Selection rects from different langauges in 2 lines may overlap with each other.
          // A good approximation is to check if the center of next rect is below the bottom of
          // current rect.
          // TODO(hellohuanlin): Consider passing the line break info from framework.
          CGRectGetMidY(_selectionRects[i + 1].rect) > CGRectGetMaxY(_selectionRects[i].rect);
      if (endsOnOrAfterEndOfRange || isLastSelectionRect || nextSelectionRectIsOnNextLine) {
        endSelectionRect = _selectionRects[i].rect;
        break;
      }
    }
  }
  if (CGRectIsNull(startSelectionRect) || CGRectIsNull(endSelectionRect)) {
    return CGRectZero;
  } else {
    // fmin/fmax to support both LTR and RTL languages.
    CGFloat minX = fmin(CGRectGetMinX(startSelectionRect), CGRectGetMinX(endSelectionRect));
    CGFloat maxX = fmax(CGRectGetMaxX(startSelectionRect), CGRectGetMaxX(endSelectionRect));
    return CGRectMake(minX, minY, maxX - minX, maxY - minY);
  }
}

- (CGRect)caretRectForPosition:(UITextPosition*)position {
  NSInteger index = ((FlutterTextPosition*)position).index;
  UITextStorageDirection affinity = ((FlutterTextPosition*)position).affinity;
  // Get the selectionRect of the characters before and after the requested caret position.
  NSArray<UITextSelectionRect*>* rects = [self
      selectionRectsForRange:[FlutterTextRange
                                 rangeWithNSRange:fml::RangeForCharactersInRange(
                                                      self.text,
                                                      NSMakeRange(
                                                          MAX(0, index - 1),
                                                          (index >= (NSInteger)self.text.length)
                                                              ? 1
                                                              : 2))]];
  if (rects.count == 0) {
    return CGRectZero;
  }
  if (index == 0) {
    // There is no character before the caret, so this will be the bounds of the character after the
    // caret position.
    CGRect characterAfterCaret = rects[0].rect;
    // Return a zero-width rectangle along the upstream edge of the character after the caret
    // position.
    if ([rects[0] isKindOfClass:[FlutterTextSelectionRect class]] &&
        ((FlutterTextSelectionRect*)rects[0]).isRTL) {
      return CGRectMake(characterAfterCaret.origin.x + characterAfterCaret.size.width,
                        characterAfterCaret.origin.y, 0, characterAfterCaret.size.height);
    } else {
      return CGRectMake(characterAfterCaret.origin.x, characterAfterCaret.origin.y, 0,
                        characterAfterCaret.size.height);
    }
  } else if (rects.count == 2 && affinity == UITextStorageDirectionForward) {
    // There are characters before and after the caret, with forward direction affinity.
    // It's better to use the character after the caret.
    CGRect characterAfterCaret = rects[1].rect;
    // Return a zero-width rectangle along the upstream edge of the character after the caret
    // position.
    if ([rects[1] isKindOfClass:[FlutterTextSelectionRect class]] &&
        ((FlutterTextSelectionRect*)rects[1]).isRTL) {
      return CGRectMake(characterAfterCaret.origin.x + characterAfterCaret.size.width,
                        characterAfterCaret.origin.y, 0, characterAfterCaret.size.height);
    } else {
      return CGRectMake(characterAfterCaret.origin.x, characterAfterCaret.origin.y, 0,
                        characterAfterCaret.size.height);
    }
  }

  // Covers 2 remaining cases:
  // 1. there are characters before and after the caret, with backward direction affinity.
  // 2. there is only 1 character before the caret (caret is at the end of text).
  // For both cases, return a zero-width rectangle along the downstream edge of the character
  // before the caret position.
  CGRect characterBeforeCaret = rects[0].rect;
  if ([rects[0] isKindOfClass:[FlutterTextSelectionRect class]] &&
      ((FlutterTextSelectionRect*)rects[0]).isRTL) {
    return CGRectMake(characterBeforeCaret.origin.x, characterBeforeCaret.origin.y, 0,
                      characterBeforeCaret.size.height);
  } else {
    return CGRectMake(characterBeforeCaret.origin.x + characterBeforeCaret.size.width,
                      characterBeforeCaret.origin.y, 0, characterBeforeCaret.size.height);
  }
}

- (UITextPosition*)closestPositionToPoint:(CGPoint)point {
  if ([_selectionRects count] == 0) {
    NSAssert([_selectedTextRange.start isKindOfClass:[FlutterTextPosition class]],
             @"Expected a FlutterTextPosition for position (got %@).",
             [_selectedTextRange.start class]);
    NSUInteger currentIndex = ((FlutterTextPosition*)_selectedTextRange.start).index;
    UITextStorageDirection currentAffinity =
        ((FlutterTextPosition*)_selectedTextRange.start).affinity;
    return [FlutterTextPosition positionWithIndex:currentIndex affinity:currentAffinity];
  }

  FlutterTextRange* range = [FlutterTextRange
      rangeWithNSRange:fml::RangeForCharactersInRange(self.text, NSMakeRange(0, self.text.length))];
  return [self closestPositionToPoint:point withinRange:range];
}

- (NSArray*)selectionRectsForRange:(UITextRange*)range {
  // At least in the simulator, swapping to the Japanese keyboard crashes the app as this method
  // is called immediately with a UITextRange with a UITextPosition rather than FlutterTextPosition
  // for the start and end.
  if (![range.start isKindOfClass:[FlutterTextPosition class]]) {
    return @[];
  }
  NSAssert([range.start isKindOfClass:[FlutterTextPosition class]],
           @"Expected a FlutterTextPosition for range.start (got %@).", [range.start class]);
  NSAssert([range.end isKindOfClass:[FlutterTextPosition class]],
           @"Expected a FlutterTextPosition for range.end (got %@).", [range.end class]);
  NSUInteger start = ((FlutterTextPosition*)range.start).index;
  NSUInteger end = ((FlutterTextPosition*)range.end).index;
  NSMutableArray* rects = [[NSMutableArray alloc] init];
  for (NSUInteger i = 0; i < [_selectionRects count]; i++) {
    if (_selectionRects[i].position >= start &&
        (_selectionRects[i].position < end ||
         (start == end && _selectionRects[i].position <= end))) {
      float width = _selectionRects[i].rect.size.width;
      if (start == end) {
        width = 0;
      }
      CGRect rect = CGRectMake(_selectionRects[i].rect.origin.x, _selectionRects[i].rect.origin.y,
                               width, _selectionRects[i].rect.size.height);
      FlutterTextSelectionRect* selectionRect = [FlutterTextSelectionRect
          selectionRectWithRectAndInfo:rect
                              position:_selectionRects[i].position
                      writingDirection:NSWritingDirectionNatural
                         containsStart:(i == 0)
                           containsEnd:(i == fml::RangeForCharactersInRange(
                                                 self.text, NSMakeRange(0, self.text.length))
                                                 .length)
                            isVertical:NO];
      [rects addObject:selectionRect];
    }
  }
  return rects;
}

- (UITextPosition*)closestPositionToPoint:(CGPoint)point withinRange:(UITextRange*)range {
  NSAssert([range.start isKindOfClass:[FlutterTextPosition class]],
           @"Expected a FlutterTextPosition for range.start (got %@).", [range.start class]);
  NSAssert([range.end isKindOfClass:[FlutterTextPosition class]],
           @"Expected a FlutterTextPosition for range.end (got %@).", [range.end class]);
  NSUInteger start = ((FlutterTextPosition*)range.start).index;
  NSUInteger end = ((FlutterTextPosition*)range.end).index;

  // Selecting text using the floating cursor is not as precise as the pencil.
  // Allow further vertical deviation and base more of the decision on horizontal comparison.
  CGFloat verticalPrecision = _isFloatingCursorActive ? 10 : 1;

  // Find the selectionRect with a leading-center point that is closest to a given point.
  BOOL isFirst = YES;
  NSUInteger _closestRectIndex = 0;
  for (NSUInteger i = 0; i < [_selectionRects count]; i++) {
    NSUInteger position = _selectionRects[i].position;
    if (position >= start && position <= end) {
      if (isFirst ||
          IsSelectionRectBoundaryCloserToPoint(
              point, _selectionRects[i].rect, _selectionRects[i].isRTL,
              /*useTrailingBoundaryOfSelectionRect=*/NO, _selectionRects[_closestRectIndex].rect,
              _selectionRects[_closestRectIndex].isRTL, verticalPrecision)) {
        isFirst = NO;
        _closestRectIndex = i;
      }
    }
  }

  FlutterTextPosition* closestPosition =
      [FlutterTextPosition positionWithIndex:_selectionRects[_closestRectIndex].position
                                    affinity:UITextStorageDirectionForward];

  // Check if the far side of the closest rect is a better fit (e.g. tapping end of line)
  // Cannot simply check the _closestRectIndex result from the previous for loop due to RTL
  // writing direction and the gaps between selectionRects. So we also need to consider
  // the adjacent selectionRects to refine _closestRectIndex.
  for (NSUInteger i = MAX(0, _closestRectIndex - 1);
       i < MIN(_closestRectIndex + 2, [_selectionRects count]); i++) {
    NSUInteger position = _selectionRects[i].position + 1;
    if (position >= start && position <= end) {
      if (IsSelectionRectBoundaryCloserToPoint(
              point, _selectionRects[i].rect, _selectionRects[i].isRTL,
              /*useTrailingBoundaryOfSelectionRect=*/YES, _selectionRects[_closestRectIndex].rect,
              _selectionRects[_closestRectIndex].isRTL, verticalPrecision)) {
        // This is an upstream position
        closestPosition = [FlutterTextPosition positionWithIndex:position
                                                        affinity:UITextStorageDirectionBackward];
      }
    }
  }

  return closestPosition;
}

- (UITextRange*)characterRangeAtPoint:(CGPoint)point {
  // TODO(cbracken) Implement.
  NSUInteger currentIndex = ((FlutterTextPosition*)_selectedTextRange.start).index;
  return [FlutterTextRange rangeWithNSRange:fml::RangeForCharacterAtIndex(self.text, currentIndex)];
}

// Overall logic for floating cursor's "move" gesture and "selection" gesture:
//
// Floating cursor's "move" gesture takes 1 finger to force press the space bar, and then move the
// cursor. The process starts with `beginFloatingCursorAtPoint`. When the finger is moved,
// `updateFloatingCursorAtPoint` will be called. When the finger is released, `endFloatingCursor`
// will be called. In all cases, we send the point (relative to the initial point registered in
// beginFloatingCursorAtPoint) to the framework, so that framework can animate the floating cursor.
//
// During the move gesture, the framework only animate the cursor visually. It's only
// after the gesture is complete, will the framework update the selection to the cursor's
// new position (with zero selection length). This means during the animation, the visual effect
// of the cursor is temporarily out of sync with the selection state in both framework and engine.
// But it will be in sync again after the animation is complete.
//
// Floating cursor's "selection" gesture also starts with 1 finger to force press the space bar,
// so exactly the same functions as the "move gesture" discussed above will be called. When the
// second finger is pressed, `setSelectedText` will be called. This mechanism requires
// `closestPositionFromPoint` to be implemented, to allow UIKit to translate the finger touch
// location displacement to the text range to select. When the selection is completed
// (i.e. when both of the 2 fingers are released), similar to "move" gesture,
// the `endFloatingCursor` will be called.
//
// When the 2nd finger is pressed, it does not trigger another startFloatingCursor call. So
// floating cursor move/selection logic has to be implemented in iOS embedder rather than
// just the framework side.
//
// Whenever a selection is updated, the engine sends the new selection to the framework. So unlike
// the move gesture, the selections in the framework and the engine are always kept in sync.
- (void)beginFloatingCursorAtPoint:(CGPoint)point {
  // For "beginFloatingCursorAtPoint" and "updateFloatingCursorAtPoint", "point" is roughly:
  //
  // CGPoint(
  //   width >= 0 ? point.x.clamp(boundingBox.left, boundingBox.right) : point.x,
  //   height >= 0 ? point.y.clamp(boundingBox.top, boundingBox.bottom) : point.y,
  // )
  // where
  //   point = keyboardPanGestureRecognizer.translationInView(textInputView) + caretRectForPosition
  //   boundingBox = self.convertRect(bounds, fromView:textInputView)
  //   bounds = self._selectionClipRect ?? self.bounds
  //
  // It seems impossible to use a negative "width" or "height", as the "convertRect"
  // call always turns a CGRect's negative dimensions into non-negative values, e.g.,
  // (1, 2, -3, -4) would become (-2, -2, 3, 4).
  _isFloatingCursorActive = YES;
  _floatingCursorOffset = point;
  [self.textInputDelegate flutterTextInputView:self
                          updateFloatingCursor:FlutterFloatingCursorDragStateStart
                                    withClient:_textInputClient
                                  withPosition:@{@"X" : @0, @"Y" : @0}];
}

- (void)updateFloatingCursorAtPoint:(CGPoint)point {
  [self.textInputDelegate flutterTextInputView:self
                          updateFloatingCursor:FlutterFloatingCursorDragStateUpdate
                                    withClient:_textInputClient
                                  withPosition:@{
                                    @"X" : @(point.x - _floatingCursorOffset.x),
                                    @"Y" : @(point.y - _floatingCursorOffset.y)
                                  }];
}

- (void)endFloatingCursor {
  _isFloatingCursorActive = NO;
  [self.textInputDelegate flutterTextInputView:self
                          updateFloatingCursor:FlutterFloatingCursorDragStateEnd
                                    withClient:_textInputClient
                                  withPosition:@{@"X" : @0, @"Y" : @0}];
}

#pragma mark - UIKeyInput Overrides

- (void)updateEditingState {
  NSUInteger selectionBase = ((FlutterTextPosition*)_selectedTextRange.start).index;
  NSUInteger selectionExtent = ((FlutterTextPosition*)_selectedTextRange.end).index;

  // Empty compositing range is represented by the framework's TextRange.empty.
  NSInteger composingBase = -1;
  NSInteger composingExtent = -1;
  if (self.markedTextRange != nil) {
    composingBase = ((FlutterTextPosition*)self.markedTextRange.start).index;
    composingExtent = ((FlutterTextPosition*)self.markedTextRange.end).index;
  }
  NSDictionary* state = @{
    @"selectionBase" : @(selectionBase),
    @"selectionExtent" : @(selectionExtent),
    @"selectionAffinity" : @(_selectionAffinity),
    @"selectionIsDirectional" : @(false),
    @"composingBase" : @(composingBase),
    @"composingExtent" : @(composingExtent),
    @"text" : [NSString stringWithString:self.text],
  };

  if (_textInputClient == 0 && _autofillId != nil) {
    [self.textInputDelegate flutterTextInputView:self
                             updateEditingClient:_textInputClient
                                       withState:state
                                         withTag:_autofillId];
  } else {
    [self.textInputDelegate flutterTextInputView:self
                             updateEditingClient:_textInputClient
                                       withState:state];
  }
}

- (void)updateEditingStateWithDelta:(flutter::TextEditingDelta)delta {
  NSUInteger selectionBase = ((FlutterTextPosition*)_selectedTextRange.start).index;
  NSUInteger selectionExtent = ((FlutterTextPosition*)_selectedTextRange.end).index;

  // Empty compositing range is represented by the framework's TextRange.empty.
  NSInteger composingBase = -1;
  NSInteger composingExtent = -1;
  if (self.markedTextRange != nil) {
    composingBase = ((FlutterTextPosition*)self.markedTextRange.start).index;
    composingExtent = ((FlutterTextPosition*)self.markedTextRange.end).index;
  }

  NSDictionary* deltaToFramework = @{
    @"oldText" : @(delta.old_text().c_str()),
    @"deltaText" : @(delta.delta_text().c_str()),
    @"deltaStart" : @(delta.delta_start()),
    @"deltaEnd" : @(delta.delta_end()),
    @"selectionBase" : @(selectionBase),
    @"selectionExtent" : @(selectionExtent),
    @"selectionAffinity" : @(_selectionAffinity),
    @"selectionIsDirectional" : @(false),
    @"composingBase" : @(composingBase),
    @"composingExtent" : @(composingExtent),
  };

  [_pendingDeltas addObject:deltaToFramework];

  if (_pendingDeltas.count == 1) {
    __weak FlutterTextInputView* weakSelf = self;
    dispatch_async(dispatch_get_main_queue(), ^{
      __strong FlutterTextInputView* strongSelf = weakSelf;
      if (strongSelf && strongSelf.pendingDeltas.count > 0) {
        NSDictionary* deltas = @{
          @"deltas" : strongSelf.pendingDeltas,
        };

        [strongSelf.textInputDelegate flutterTextInputView:strongSelf
                                       updateEditingClient:strongSelf->_textInputClient
                                                 withDelta:deltas];
        [strongSelf.pendingDeltas removeAllObjects];
      }
    });
  }
}

- (BOOL)hasText {
  return self.text.length > 0;
}

- (void)insertText:(NSString*)text {
  if (self.temporarilyDeletedComposedCharacter.length > 0 && text.length == 1 && !text.UTF8String &&
      [text characterAtIndex:0] == [self.temporarilyDeletedComposedCharacter characterAtIndex:0]) {
    // Workaround for https://github.com/flutter/flutter/issues/111494
    // TODO(cyanglaz): revert this workaround if when flutter supports a minimum iOS version which
    // this bug is fixed by Apple.
    text = self.temporarilyDeletedComposedCharacter;
    self.temporarilyDeletedComposedCharacter = nil;
  }

  NSMutableArray<FlutterTextSelectionRect*>* copiedRects =
      [[NSMutableArray alloc] initWithCapacity:[_selectionRects count]];
  NSAssert([_selectedTextRange.start isKindOfClass:[FlutterTextPosition class]],
           @"Expected a FlutterTextPosition for position (got %@).",
           [_selectedTextRange.start class]);
  NSUInteger insertPosition = ((FlutterTextPosition*)_selectedTextRange.start).index;
  for (NSUInteger i = 0; i < [_selectionRects count]; i++) {
    NSUInteger rectPosition = _selectionRects[i].position;
    if (rectPosition == insertPosition) {
      for (NSUInteger j = 0; j <= text.length; j++) {
        [copiedRects addObject:[FlutterTextSelectionRect
                                   selectionRectWithRect:_selectionRects[i].rect
                                                position:rectPosition + j
                                        writingDirection:_selectionRects[i].writingDirection]];
      }
    } else {
      if (rectPosition > insertPosition) {
        rectPosition = rectPosition + text.length;
      }
      [copiedRects addObject:[FlutterTextSelectionRect
                                 selectionRectWithRect:_selectionRects[i].rect
                                              position:rectPosition
                                      writingDirection:_selectionRects[i].writingDirection]];
    }
  }

  _scribbleFocusStatus = FlutterScribbleFocusStatusUnfocused;
  [self resetScribbleInteractionStatusIfEnding];
  self.selectionRects = copiedRects;
  _selectionAffinity = kTextAffinityDownstream;
  [self replaceRange:_selectedTextRange withText:text];
}

- (UITextPlaceholder*)insertTextPlaceholderWithSize:(CGSize)size API_AVAILABLE(ios(13.0)) {
  [self.textInputDelegate flutterTextInputView:self
                 insertTextPlaceholderWithSize:size
                                    withClient:_textInputClient];
  _hasPlaceholder = YES;
  return [[FlutterTextPlaceholder alloc] init];
}

- (void)removeTextPlaceholder:(UITextPlaceholder*)textPlaceholder API_AVAILABLE(ios(13.0)) {
  _hasPlaceholder = NO;
  [self.textInputDelegate flutterTextInputView:self removeTextPlaceholder:_textInputClient];
}

- (void)deleteBackward {
  _selectionAffinity = kTextAffinityDownstream;
  _scribbleFocusStatus = FlutterScribbleFocusStatusUnfocused;
  [self resetScribbleInteractionStatusIfEnding];

  // When deleting Thai vowel, _selectedTextRange has location
  // but does not have length, so we have to manually set it.
  // In addition, we needed to delete only a part of grapheme cluster
  // because it is the expected behavior of Thai input.
  // https://github.com/flutter/flutter/issues/24203
  // https://github.com/flutter/flutter/issues/21745
  // https://github.com/flutter/flutter/issues/39399
  //
  // This is needed for correct handling of the deletion of Thai vowel input.
  // TODO(cbracken): Get a good understanding of expected behavior of Thai
  // input and ensure that this is the correct solution.
  // https://github.com/flutter/flutter/issues/28962
  if (_selectedTextRange.isEmpty && [self hasText]) {
    UITextRange* oldSelectedRange = _selectedTextRange;
    NSRange oldRange = ((FlutterTextRange*)oldSelectedRange).range;
    if (oldRange.location > 0) {
      NSRange newRange = NSMakeRange(oldRange.location - 1, 1);

      // We should check if the last character is a part of emoji.
      // If so, we must delete the entire emoji to prevent the text from being malformed.
      NSRange charRange = fml::RangeForCharacterAtIndex(self.text, oldRange.location - 1);
      if (IsEmoji(self.text, charRange)) {
        newRange = NSMakeRange(charRange.location, oldRange.location - charRange.location);
      }

      _selectedTextRange = [[FlutterTextRange rangeWithNSRange:newRange] copy];
    }
  }

  if (!_selectedTextRange.isEmpty) {
    // Cache the last deleted emoji to use for an iOS bug where the next
    // insertion corrupts the emoji characters.
    // See: https://github.com/flutter/flutter/issues/111494#issuecomment-1248441346
    if (IsEmoji(self.text, _selectedTextRange.range)) {
      NSString* deletedText = [self.text substringWithRange:_selectedTextRange.range];
      NSRange deleteFirstCharacterRange = fml::RangeForCharacterAtIndex(deletedText, 0);
      self.temporarilyDeletedComposedCharacter =
          [deletedText substringWithRange:deleteFirstCharacterRange];
    }
    [self replaceRange:_selectedTextRange withText:@""];
  }
}

- (void)postAccessibilityNotification:(UIAccessibilityNotifications)notification target:(id)target {
  UIAccessibilityPostNotification(notification, target);
}

- (void)accessibilityElementDidBecomeFocused {
  if ([self accessibilityElementIsFocused]) {
    // For most of the cases, this flutter text input view should never
    // receive the focus. If we do receive the focus, we make the best effort
    // to send the focus back to the real text field.
    FML_DCHECK(_backingTextInputAccessibilityObject);
    [self postAccessibilityNotification:UIAccessibilityScreenChangedNotification
                                 target:_backingTextInputAccessibilityObject];
  }
}

- (BOOL)accessibilityElementsHidden {
  return !_accessibilityEnabled;
}

- (void)resetScribbleInteractionStatusIfEnding {
  if (_scribbleInteractionStatus == FlutterScribbleInteractionStatusEnding) {
    _scribbleInteractionStatus = FlutterScribbleInteractionStatusNone;
  }
}

#pragma mark - Key Events Handling
- (void)pressesBegan:(NSSet<UIPress*>*)presses
           withEvent:(UIPressesEvent*)event API_AVAILABLE(ios(9.0)) {
  [_textInputPlugin.viewController pressesBegan:presses withEvent:event];
}

- (void)pressesChanged:(NSSet<UIPress*>*)presses
             withEvent:(UIPressesEvent*)event API_AVAILABLE(ios(9.0)) {
  [_textInputPlugin.viewController pressesChanged:presses withEvent:event];
}

- (void)pressesEnded:(NSSet<UIPress*>*)presses
           withEvent:(UIPressesEvent*)event API_AVAILABLE(ios(9.0)) {
  [_textInputPlugin.viewController pressesEnded:presses withEvent:event];
}

- (void)pressesCancelled:(NSSet<UIPress*>*)presses
               withEvent:(UIPressesEvent*)event API_AVAILABLE(ios(9.0)) {
  [_textInputPlugin.viewController pressesCancelled:presses withEvent:event];
}

@end

/**
 * Hides `FlutterTextInputView` from iOS accessibility system so it
 * does not show up twice, once where it is in the `UIView` hierarchy,
 * and a second time as part of the `SemanticsObject` hierarchy.
 *
 * This prevents the `FlutterTextInputView` from receiving the focus
 * due to swiping gesture.
 *
 * There are other cases the `FlutterTextInputView` may receive
 * focus. One example is during screen changes, the accessibility
 * tree will undergo a dramatic structural update. The Voiceover may
 * decide to focus the `FlutterTextInputView` that is not involved
 * in the structural update instead. If that happens, the
 * `FlutterTextInputView` will make a best effort to direct the
 * focus back to the `SemanticsObject`.
 */
@interface FlutterTextInputViewAccessibilityHider : UIView {
}

@end

@implementation FlutterTextInputViewAccessibilityHider {
}

- (BOOL)accessibilityElementsHidden {
  return YES;
}

@end

@interface FlutterTextInputPlugin ()
- (void)enableActiveViewAccessibility;
@end

@interface FlutterTimerProxy : NSObject
@property(nonatomic, weak) FlutterTextInputPlugin* target;
@end

@implementation FlutterTimerProxy

+ (instancetype)proxyWithTarget:(FlutterTextInputPlugin*)target {
  FlutterTimerProxy* proxy = [[self alloc] init];
  if (proxy) {
    proxy.target = target;
  }
  return proxy;
}

- (void)enableActiveViewAccessibility {
  [self.target enableActiveViewAccessibility];
}

@end

@interface FlutterTextInputPlugin ()
// The current password-autofillable input fields that have yet to be saved.
@property(nonatomic, readonly)
    NSMutableDictionary<NSString*, FlutterTextInputView*>* autofillContext;
@property(nonatomic, retain) FlutterTextInputView* activeView;
@property(nonatomic, retain) FlutterTextInputViewAccessibilityHider* inputHider;
@property(nonatomic, readonly, weak) id<FlutterViewResponder> viewResponder;

@property(nonatomic, strong) UIView* keyboardViewContainer;
@property(nonatomic, strong) UIView* keyboardView;
@property(nonatomic, strong) UIView* cachedFirstResponder;
@property(nonatomic, assign) CGRect keyboardRect;
@property(nonatomic, assign) CGFloat previousPointerYPosition;
@property(nonatomic, assign) CGFloat pointerYVelocity;
@end

@implementation FlutterTextInputPlugin {
  NSTimer* _enableFlutterTextInputViewAccessibilityTimer;
}

- (instancetype)initWithDelegate:(id<FlutterTextInputDelegate>)textInputDelegate {
  self = [super init];
  if (self) {
    // `_textInputDelegate` is a weak reference because it should retain FlutterTextInputPlugin.
    _textInputDelegate = textInputDelegate;
    _autofillContext = [[NSMutableDictionary alloc] init];
    _inputHider = [[FlutterTextInputViewAccessibilityHider alloc] init];
    _scribbleElements = [[NSMutableDictionary alloc] init];
    _keyboardViewContainer = [[UIView alloc] init];

    [[NSNotificationCenter defaultCenter] addObserver:self
                                             selector:@selector(handleKeyboardWillShow:)
                                                 name:UIKeyboardWillShowNotification
                                               object:nil];
  }

  return self;
}

- (void)handleKeyboardWillShow:(NSNotification*)notification {
  NSDictionary* keyboardInfo = [notification userInfo];
  NSValue* keyboardFrameEnd = [keyboardInfo valueForKey:UIKeyboardFrameEndUserInfoKey];
  _keyboardRect = [keyboardFrameEnd CGRectValue];
}

- (void)dealloc {
  [self hideTextInput];
}

- (void)removeEnableFlutterTextInputViewAccessibilityTimer {
  if (_enableFlutterTextInputViewAccessibilityTimer) {
    [_enableFlutterTextInputViewAccessibilityTimer invalidate];
    _enableFlutterTextInputViewAccessibilityTimer = nil;
  }
}

- (UIView<UITextInput>*)textInputView {
  return _activeView;
}

- (void)handleMethodCall:(FlutterMethodCall*)call result:(FlutterResult)result {
  NSString* method = call.method;
  id args = call.arguments;
  if ([method isEqualToString:kShowMethod]) {
    [self showTextInput];
    result(nil);
  } else if ([method isEqualToString:kHideMethod]) {
    [self hideTextInput];
    result(nil);
  } else if ([method isEqualToString:kSetClientMethod]) {
    [self setTextInputClient:[args[0] intValue] withConfiguration:args[1]];
    result(nil);
  } else if ([method isEqualToString:kSetPlatformViewClientMethod]) {
    // This method call has a `platformViewId` argument, but we do not need it for iOS for now.
    [self setPlatformViewTextInputClient];
    result(nil);
  } else if ([method isEqualToString:kSetEditingStateMethod]) {
    [self setTextInputEditingState:args];
    result(nil);
  } else if ([method isEqualToString:kClearClientMethod]) {
    [self clearTextInputClient];
    result(nil);
  } else if ([method isEqualToString:kSetEditableSizeAndTransformMethod]) {
    [self setEditableSizeAndTransform:args];
    result(nil);
  } else if ([method isEqualToString:kSetMarkedTextRectMethod]) {
    [self updateMarkedRect:args];
    result(nil);
  } else if ([method isEqualToString:kFinishAutofillContextMethod]) {
    [self triggerAutofillSave:[args boolValue]];
    result(nil);
    // TODO(justinmc): Remove the TextInput method constant when the framework has
    // finished transitioning to using the Scribble channel.
    // https://github.com/flutter/flutter/pull/104128
  } else if ([method isEqualToString:kDeprecatedSetSelectionRectsMethod]) {
    [self setSelectionRects:args];
    result(nil);
  } else if ([method isEqualToString:kSetSelectionRectsMethod]) {
    [self setSelectionRects:args];
    result(nil);
  } else if ([method isEqualToString:kStartLiveTextInputMethod]) {
    [self startLiveTextInput];
    result(nil);
  } else if ([method isEqualToString:kUpdateConfigMethod]) {
    [self updateConfig:args];
    result(nil);
  } else if ([method isEqualToString:kOnInteractiveKeyboardPointerMoveMethod]) {
    CGFloat pointerY = (CGFloat)[args[@"pointerY"] doubleValue];
    [self handlePointerMove:pointerY];
    result(nil);
  } else if ([method isEqualToString:kOnInteractiveKeyboardPointerUpMethod]) {
    CGFloat pointerY = (CGFloat)[args[@"pointerY"] doubleValue];
    [self handlePointerUp:pointerY];
    result(nil);
  } else {
    result(FlutterMethodNotImplemented);
  }
}

- (void)handlePointerUp:(CGFloat)pointerY {
  if (_keyboardView.superview != nil) {
    // Done to avoid the issue of a pointer up done without a screenshot
    // View must be loaded at this point.
    UIScreen* screen = _viewController.flutterScreenIfViewLoaded;
    CGFloat screenHeight = screen.bounds.size.height;
    CGFloat keyboardHeight = _keyboardRect.size.height;
    // Negative velocity indicates a downward movement
    BOOL shouldDismissKeyboardBasedOnVelocity = _pointerYVelocity < 0;
    [UIView animateWithDuration:kKeyboardAnimationTimeToCompleteion
        animations:^{
          double keyboardDestination =
              shouldDismissKeyboardBasedOnVelocity ? screenHeight : screenHeight - keyboardHeight;
          _keyboardViewContainer.frame = CGRectMake(
              0, keyboardDestination, _viewController.flutterScreenIfViewLoaded.bounds.size.width,
              _keyboardViewContainer.frame.size.height);
        }
        completion:^(BOOL finished) {
          if (shouldDismissKeyboardBasedOnVelocity) {
            [self.textInputDelegate flutterTextInputView:self.activeView
                didResignFirstResponderWithTextInputClient:self.activeView.textInputClient];
            [self dismissKeyboardScreenshot];
          } else {
            [self showKeyboardAndRemoveScreenshot];
          }
        }];
  }
}

- (void)dismissKeyboardScreenshot {
  for (UIView* subView in _keyboardViewContainer.subviews) {
    [subView removeFromSuperview];
  }
}

- (void)showKeyboardAndRemoveScreenshot {
  [UIView setAnimationsEnabled:NO];
  [_cachedFirstResponder becomeFirstResponder];
  // UIKit does not immediately access the areAnimationsEnabled Boolean so a delay is needed before
  // returned
  dispatch_after(dispatch_time(DISPATCH_TIME_NOW, kKeyboardAnimationDelaySeconds * NSEC_PER_SEC),
                 dispatch_get_main_queue(), ^{
                   [UIView setAnimationsEnabled:YES];
                   [self dismissKeyboardScreenshot];
                 });
}

- (void)handlePointerMove:(CGFloat)pointerY {
  // View must be loaded at this point.
  UIScreen* screen = _viewController.flutterScreenIfViewLoaded;
  CGFloat screenHeight = screen.bounds.size.height;
  CGFloat keyboardHeight = _keyboardRect.size.height;
  if (screenHeight - keyboardHeight <= pointerY) {
    // If the pointer is within the bounds of the keyboard.
    if (_keyboardView.superview == nil) {
      // If no screenshot has been taken.
      [self takeKeyboardScreenshotAndDisplay];
      [self hideKeyboardWithoutAnimationAndAvoidCursorDismissUpdate];
    } else {
      [self setKeyboardContainerHeight:pointerY];
      _pointerYVelocity = _previousPointerYPosition - pointerY;
    }
  } else {
    if (_keyboardView.superview != nil) {
      // Keeps keyboard at proper height.
      _keyboardViewContainer.frame = _keyboardRect;
      _pointerYVelocity = _previousPointerYPosition - pointerY;
    }
  }
  _previousPointerYPosition = pointerY;
}

- (void)setKeyboardContainerHeight:(CGFloat)pointerY {
  CGRect frameRect = _keyboardRect;
  frameRect.origin.y = pointerY;
  _keyboardViewContainer.frame = frameRect;
}

- (void)hideKeyboardWithoutAnimationAndAvoidCursorDismissUpdate {
  [UIView setAnimationsEnabled:NO];
  _cachedFirstResponder = UIApplication.sharedApplication.keyWindow.flutterFirstResponder;
  _activeView.preventCursorDismissWhenResignFirstResponder = YES;
  [_cachedFirstResponder resignFirstResponder];
  _activeView.preventCursorDismissWhenResignFirstResponder = NO;
  [UIView setAnimationsEnabled:YES];
}

- (void)takeKeyboardScreenshotAndDisplay {
  // View must be loaded at this point
  UIScreen* screen = _viewController.flutterScreenIfViewLoaded;
  UIView* keyboardSnap = [screen snapshotViewAfterScreenUpdates:YES];
  keyboardSnap = [keyboardSnap resizableSnapshotViewFromRect:_keyboardRect
                                          afterScreenUpdates:YES
                                               withCapInsets:UIEdgeInsetsZero];
  _keyboardView = keyboardSnap;
  [_keyboardViewContainer addSubview:_keyboardView];
  if (_keyboardViewContainer.superview == nil) {
    [UIApplication.sharedApplication.delegate.window.rootViewController.view
        addSubview:_keyboardViewContainer];
  }
  _keyboardViewContainer.layer.zPosition = NSIntegerMax;
  _keyboardViewContainer.frame = _keyboardRect;
}

- (BOOL)showEditMenu:(NSDictionary*)args API_AVAILABLE(ios(16.0)) {
  if (!self.activeView.isFirstResponder) {
    return NO;
  }
  NSDictionary<NSString*, NSNumber*>* encodedTargetRect = args[@"targetRect"];
  CGRect globalTargetRect = CGRectMake(
      [encodedTargetRect[@"x"] doubleValue], [encodedTargetRect[@"y"] doubleValue],
      [encodedTargetRect[@"width"] doubleValue], [encodedTargetRect[@"height"] doubleValue]);
  CGRect localTargetRect = [self.hostView convertRect:globalTargetRect toView:self.activeView];
  [self.activeView showEditMenuWithTargetRect:localTargetRect];
  return YES;
}

- (void)hideEditMenu {
  [self.activeView hideEditMenu];
}

- (void)setEditableSizeAndTransform:(NSDictionary*)dictionary {
  NSArray* transform = dictionary[@"transform"];
  [_activeView setEditableTransform:transform];
  const int leftIndex = 12;
  const int topIndex = 13;
  if ([_activeView isScribbleAvailable]) {
    // This is necessary to set up where the scribble interactable element will be.
    _inputHider.frame =
        CGRectMake([transform[leftIndex] intValue], [transform[topIndex] intValue],
                   [dictionary[@"width"] intValue], [dictionary[@"height"] intValue]);
    _activeView.frame =
        CGRectMake(0, 0, [dictionary[@"width"] intValue], [dictionary[@"height"] intValue]);
    _activeView.tintColor = [UIColor clearColor];
  } else {
    // TODO(hellohuanlin): Also need to handle iOS 16 case, where the auto-correction highlight does
    // not match the size of text.
    // See https://github.com/flutter/flutter/issues/131695
    if (@available(iOS 17, *)) {
      // Move auto-correction highlight to overlap with the actual text.
      // This is to fix an issue where the system auto-correction highlight is displayed at
      // the top left corner of the screen on iOS 17+.
      // This problem also happens on iOS 16, but the size of highlight does not match the text.
      // See https://github.com/flutter/flutter/issues/131695
      // TODO(hellohuanlin): Investigate if we can use non-zero size.
      _inputHider.frame =
          CGRectMake([transform[leftIndex] intValue], [transform[topIndex] intValue], 0, 0);
    }
  }
}

- (void)updateMarkedRect:(NSDictionary*)dictionary {
  NSAssert(dictionary[@"x"] != nil && dictionary[@"y"] != nil && dictionary[@"width"] != nil &&
               dictionary[@"height"] != nil,
           @"Expected a dictionary representing a CGRect, got %@", dictionary);
  CGRect rect = CGRectMake([dictionary[@"x"] doubleValue], [dictionary[@"y"] doubleValue],
                           [dictionary[@"width"] doubleValue], [dictionary[@"height"] doubleValue]);
  _activeView.markedRect = rect.size.width < 0 && rect.size.height < 0 ? kInvalidFirstRect : rect;
}

- (void)setSelectionRects:(NSArray*)encodedRects {
  NSMutableArray<FlutterTextSelectionRect*>* rectsAsRect =
      [[NSMutableArray alloc] initWithCapacity:[encodedRects count]];
  for (NSUInteger i = 0; i < [encodedRects count]; i++) {
    NSArray<NSNumber*>* encodedRect = encodedRects[i];
    [rectsAsRect addObject:[FlutterTextSelectionRect
                               selectionRectWithRect:CGRectMake([encodedRect[0] floatValue],
                                                                [encodedRect[1] floatValue],
                                                                [encodedRect[2] floatValue],
                                                                [encodedRect[3] floatValue])
                                            position:[encodedRect[4] unsignedIntegerValue]
                                    writingDirection:[encodedRect[5] unsignedIntegerValue] == 1
                                                         ? NSWritingDirectionLeftToRight
                                                         : NSWritingDirectionRightToLeft]];
  }

  // TODO(hellohuanlin): Investigate why notifying the text input system about text changes (via
  // textWillChange and textDidChange APIs) causes a bug where we cannot enter text with IME
  // keyboards. Issue: https://github.com/flutter/flutter/issues/133908
  _activeView.selectionRects = rectsAsRect;
}

- (void)startLiveTextInput {
  if (@available(iOS 15.0, *)) {
    if (_activeView == nil || !_activeView.isFirstResponder) {
      return;
    }
    [_activeView captureTextFromCamera:nil];
  }
}

- (void)showTextInput {
  _activeView.viewResponder = _viewResponder;
  [self addToInputParentViewIfNeeded:_activeView];
  // Adds a delay to prevent the text view from receiving accessibility
  // focus in case it is activated during semantics updates.
  //
  // One common case is when the app navigates to a page with an auto
  // focused text field. The text field will activate the FlutterTextInputView
  // with a semantics update sent to the engine. The voiceover will focus
  // the newly attached active view while performing accessibility update.
  // This results in accessibility focus stuck at the FlutterTextInputView.
  if (!_enableFlutterTextInputViewAccessibilityTimer) {
    _enableFlutterTextInputViewAccessibilityTimer =
        [NSTimer scheduledTimerWithTimeInterval:kUITextInputAccessibilityEnablingDelaySeconds
                                         target:[FlutterTimerProxy proxyWithTarget:self]
                                       selector:@selector(enableActiveViewAccessibility)
                                       userInfo:nil
                                        repeats:NO];
  }
  [_activeView becomeFirstResponder];
}

- (void)enableActiveViewAccessibility {
  if (_activeView.isFirstResponder) {
    _activeView.accessibilityEnabled = YES;
  }
  [self removeEnableFlutterTextInputViewAccessibilityTimer];
}

- (void)hideTextInput {
  [self removeEnableFlutterTextInputViewAccessibilityTimer];
  _activeView.accessibilityEnabled = NO;
  [_activeView resignFirstResponder];
  [_activeView removeFromSuperview];
  [_inputHider removeFromSuperview];
}

- (void)triggerAutofillSave:(BOOL)saveEntries {
  [_activeView resignFirstResponder];

  if (saveEntries) {
    // Make all the input fields in the autofill context visible,
    // then remove them to trigger autofill save.
    [self cleanUpViewHierarchy:YES clearText:YES delayRemoval:NO];
    [_autofillContext removeAllObjects];
    [self changeInputViewsAutofillVisibility:YES];
  } else {
    [_autofillContext removeAllObjects];
  }

  [self cleanUpViewHierarchy:YES clearText:!saveEntries delayRemoval:NO];
  [self addToInputParentViewIfNeeded:_activeView];
}

- (void)setPlatformViewTextInputClient {
  // No need to track the platformViewID (unlike in Android). When a platform view
  // becomes the first responder, simply hide this dummy text input view (`_activeView`)
  // for the previously focused widget.
  [self removeEnableFlutterTextInputViewAccessibilityTimer];
  _activeView.accessibilityEnabled = NO;
  [_activeView removeFromSuperview];
  [_inputHider removeFromSuperview];
}

- (void)setTextInputClient:(int)client withConfiguration:(NSDictionary*)configuration {
  [self resetAllClientIds];
  // Hide all input views from autofill, only make those in the new configuration visible
  // to autofill.
  [self changeInputViewsAutofillVisibility:NO];

  // Update the current active view.
  switch (AutofillTypeOf(configuration)) {
    case kFlutterAutofillTypeNone:
      self.activeView = [self createInputViewWith:configuration];
      break;
    case kFlutterAutofillTypeRegular:
      // If the group does not involve password autofill, only install the
      // input view that's being focused.
      self.activeView = [self updateAndShowAutofillViews:nil
                                            focusedField:configuration
                                       isPasswordRelated:NO];
      break;
    case kFlutterAutofillTypePassword:
      self.activeView = [self updateAndShowAutofillViews:configuration[kAssociatedAutofillFields]
                                            focusedField:configuration
                                       isPasswordRelated:YES];
      break;
  }
  [_activeView setTextInputClient:client];
  [_activeView reloadInputViews];

  // Clean up views that no longer need to be in the view hierarchy, according to
  // the current autofill context. The "garbage" input views are already made
  // invisible to autofill and they can't `becomeFirstResponder`, we only remove
  // them to free up resources and reduce the number of input views in the view
  // hierarchy.
  //
  // The garbage views are decommissioned immediately, but the removeFromSuperview
  // call is scheduled on the runloop and delayed by 0.1s so we don't remove the
  // text fields immediately (which seems to make the keyboard flicker).
  // See: https://github.com/flutter/flutter/issues/64628.
  [self cleanUpViewHierarchy:NO clearText:YES delayRemoval:YES];
}

// Creates and shows an input field that is not password related and has no autofill
// info. This method returns a new FlutterTextInputView instance when called, since
// UIKit uses the identity of `UITextInput` instances (or the identity of the input
// views) to decide whether the IME's internal states should be reset. See:
// https://github.com/flutter/flutter/issues/79031 .
- (FlutterTextInputView*)createInputViewWith:(NSDictionary*)configuration {
  NSString* autofillId = AutofillIdFromDictionary(configuration);
  if (autofillId) {
    [_autofillContext removeObjectForKey:autofillId];
  }
  FlutterTextInputView* newView = [[FlutterTextInputView alloc] initWithOwner:self];
  [newView configureWithDictionary:configuration];
  [self addToInputParentViewIfNeeded:newView];

  for (NSDictionary* field in configuration[kAssociatedAutofillFields]) {
    NSString* autofillId = AutofillIdFromDictionary(field);
    if (autofillId && AutofillTypeOf(field) == kFlutterAutofillTypeNone) {
      [_autofillContext removeObjectForKey:autofillId];
    }
  }
  return newView;
}

- (FlutterTextInputView*)updateAndShowAutofillViews:(NSArray*)fields
                                       focusedField:(NSDictionary*)focusedField
                                  isPasswordRelated:(BOOL)isPassword {
  FlutterTextInputView* focused = nil;
  NSString* focusedId = AutofillIdFromDictionary(focusedField);
  NSAssert(focusedId, @"autofillId must not be null for the focused field: %@", focusedField);

  if (!fields) {
    // DO NOT push the current autofillable input fields to the context even
    // if it's password-related, because it is not in an autofill group.
    focused = [self getOrCreateAutofillableView:focusedField isPasswordAutofill:isPassword];
    [_autofillContext removeObjectForKey:focusedId];
  }

  for (NSDictionary* field in fields) {
    NSString* autofillId = AutofillIdFromDictionary(field);
    NSAssert(autofillId, @"autofillId must not be null for field: %@", field);

    BOOL hasHints = AutofillTypeOf(field) != kFlutterAutofillTypeNone;
    BOOL isFocused = [focusedId isEqualToString:autofillId];

    if (isFocused) {
      focused = [self getOrCreateAutofillableView:field isPasswordAutofill:isPassword];
    }

    if (hasHints) {
      // Push the current input field to the context if it has hints.
      _autofillContext[autofillId] = isFocused ? focused
                                               : [self getOrCreateAutofillableView:field
                                                                isPasswordAutofill:isPassword];
    } else {
      // Mark for deletion.
      [_autofillContext removeObjectForKey:autofillId];
    }
  }

  NSAssert(focused, @"The current focused input view must not be nil.");
  return focused;
}

// Returns a new non-reusable input view (and put it into the view hierarchy), or get the
// view from the current autofill context, if an input view with the same autofill id
// already exists in the context.
// This is generally used for input fields that are autofillable (UIKit tracks these veiws
// for autofill purposes so they should not be reused for a different type of views).
- (FlutterTextInputView*)getOrCreateAutofillableView:(NSDictionary*)field
                                  isPasswordAutofill:(BOOL)needsPasswordAutofill {
  NSString* autofillId = AutofillIdFromDictionary(field);
  FlutterTextInputView* inputView = _autofillContext[autofillId];
  if (!inputView) {
    inputView =
        needsPasswordAutofill ? [FlutterSecureTextInputView alloc] : [FlutterTextInputView alloc];
    inputView = [inputView initWithOwner:self];
    [self addToInputParentViewIfNeeded:inputView];
  }

  [inputView configureWithDictionary:field];
  return inputView;
}

// The UIView to add FlutterTextInputViews to.
- (UIView*)hostView {
  UIView* host = _viewController.view;
  NSAssert(host != nullptr,
           @"The application must have a host view since the keyboard client "
           @"must be part of the responder chain to function. The host view controller is %@",
           _viewController);
  return host;
}

// The UIView to add FlutterTextInputViews to.
- (NSArray<UIView*>*)textInputViews {
  return _inputHider.subviews;
}

// Removes every installed input field, unless it's in the current autofill context.
//
// The active view will be removed from its superview too, if includeActiveView is YES.
// When clearText is YES, the text on the input fields will be set to empty before
// they are removed from the view hierarchy, to avoid triggering autofill save.
// If delayRemoval is true, removeFromSuperview will be scheduled on the runloop and
// will be delayed by 0.1s so we don't remove the text fields immediately (which seems
// to make the keyboard flicker).
// See: https://github.com/flutter/flutter/issues/64628.

- (void)cleanUpViewHierarchy:(BOOL)includeActiveView
                   clearText:(BOOL)clearText
                delayRemoval:(BOOL)delayRemoval {
  for (UIView* view in self.textInputViews) {
    if ([view isKindOfClass:[FlutterTextInputView class]] &&
        (includeActiveView || view != _activeView)) {
      FlutterTextInputView* inputView = (FlutterTextInputView*)view;
      if (_autofillContext[inputView.autofillId] != view) {
        if (clearText) {
          [inputView replaceRangeLocal:NSMakeRange(0, inputView.text.length) withText:@""];
        }
        if (delayRemoval) {
          [inputView performSelector:@selector(removeFromSuperview) withObject:nil afterDelay:0.1];
        } else {
          [inputView removeFromSuperview];
        }
      }
    }
  }
}

// Changes the visibility of every FlutterTextInputView currently in the
// view hierarchy.
- (void)changeInputViewsAutofillVisibility:(BOOL)newVisibility {
  for (UIView* view in self.textInputViews) {
    if ([view isKindOfClass:[FlutterTextInputView class]]) {
      FlutterTextInputView* inputView = (FlutterTextInputView*)view;
      inputView.isVisibleToAutofill = newVisibility;
    }
  }
}

// Resets the client id of every FlutterTextInputView in the view hierarchy
// to 0.
// Called before establishing a new text input connection.
// For views in the current autofill context, they need to
// stay in the view hierachy but should not be allowed to
// send messages (other than autofill related ones) to the
// framework.
- (void)resetAllClientIds {
  for (UIView* view in self.textInputViews) {
    if ([view isKindOfClass:[FlutterTextInputView class]]) {
      FlutterTextInputView* inputView = (FlutterTextInputView*)view;
      [inputView setTextInputClient:0];
    }
  }
}

- (void)addToInputParentViewIfNeeded:(FlutterTextInputView*)inputView {
  if (![inputView isDescendantOfView:_inputHider]) {
    [_inputHider addSubview:inputView];
  }

  if (_viewController.view == nil) {
    // If view controller's view has detached from flutter engine, we don't add _inputHider
    // in parent view to fallback and avoid crash.
    // https://github.com/flutter/flutter/issues/106404.
    return;
  }
  UIView* parentView = self.hostView;
  if (_inputHider.superview != parentView) {
    [parentView addSubview:_inputHider];
  }
}

- (void)setTextInputEditingState:(NSDictionary*)state {
  [_activeView setTextInputState:state];
}

- (void)clearTextInputClient {
  [_activeView setTextInputClient:0];
  _activeView.frame = CGRectZero;
}

- (void)updateConfig:(NSDictionary*)dictionary {
  BOOL isSecureTextEntry = [dictionary[kSecureTextEntry] boolValue];
  for (UIView* view in self.textInputViews) {
    if ([view isKindOfClass:[FlutterTextInputView class]]) {
      FlutterTextInputView* inputView = (FlutterTextInputView*)view;
      // The feature of holding and draging spacebar to move cursor is affected by
      // secureTextEntry, so when obscureText is updated, we need to update secureTextEntry
      // and call reloadInputViews.
      // https://github.com/flutter/flutter/issues/122139
      if (inputView.isSecureTextEntry != isSecureTextEntry) {
        inputView.secureTextEntry = isSecureTextEntry;
        [inputView reloadInputViews];
      }
    }
  }
}

#pragma mark UIIndirectScribbleInteractionDelegate

- (BOOL)indirectScribbleInteraction:(UIIndirectScribbleInteraction*)interaction
                   isElementFocused:(UIScribbleElementIdentifier)elementIdentifier
    API_AVAILABLE(ios(14.0)) {
  return _activeView.scribbleFocusStatus == FlutterScribbleFocusStatusFocused;
}

- (void)indirectScribbleInteraction:(UIIndirectScribbleInteraction*)interaction
               focusElementIfNeeded:(UIScribbleElementIdentifier)elementIdentifier
                     referencePoint:(CGPoint)focusReferencePoint
                         completion:(void (^)(UIResponder<UITextInput>* focusedInput))completion
    API_AVAILABLE(ios(14.0)) {
  _activeView.scribbleFocusStatus = FlutterScribbleFocusStatusFocusing;
  [_indirectScribbleDelegate flutterTextInputPlugin:self
                                       focusElement:elementIdentifier
                                            atPoint:focusReferencePoint
                                             result:^(id _Nullable result) {
                                               _activeView.scribbleFocusStatus =
                                                   FlutterScribbleFocusStatusFocused;
                                               completion(_activeView);
                                             }];
}

- (BOOL)indirectScribbleInteraction:(UIIndirectScribbleInteraction*)interaction
         shouldDelayFocusForElement:(UIScribbleElementIdentifier)elementIdentifier
    API_AVAILABLE(ios(14.0)) {
  return NO;
}

- (void)indirectScribbleInteraction:(UIIndirectScribbleInteraction*)interaction
          willBeginWritingInElement:(UIScribbleElementIdentifier)elementIdentifier
    API_AVAILABLE(ios(14.0)) {
}

- (void)indirectScribbleInteraction:(UIIndirectScribbleInteraction*)interaction
          didFinishWritingInElement:(UIScribbleElementIdentifier)elementIdentifier
    API_AVAILABLE(ios(14.0)) {
}

- (CGRect)indirectScribbleInteraction:(UIIndirectScribbleInteraction*)interaction
                      frameForElement:(UIScribbleElementIdentifier)elementIdentifier
    API_AVAILABLE(ios(14.0)) {
  NSValue* elementValue = [_scribbleElements objectForKey:elementIdentifier];
  if (elementValue == nil) {
    return CGRectZero;
  }
  return [elementValue CGRectValue];
}

- (void)indirectScribbleInteraction:(UIIndirectScribbleInteraction*)interaction
              requestElementsInRect:(CGRect)rect
                         completion:
                             (void (^)(NSArray<UIScribbleElementIdentifier>* elements))completion
    API_AVAILABLE(ios(14.0)) {
  [_indirectScribbleDelegate
      flutterTextInputPlugin:self
       requestElementsInRect:rect
                      result:^(id _Nullable result) {
                        NSMutableArray<UIScribbleElementIdentifier>* elements =
                            [[NSMutableArray alloc] init];
                        if ([result isKindOfClass:[NSArray class]]) {
                          for (NSArray* elementArray in result) {
                            [elements addObject:elementArray[0]];
                            [_scribbleElements
                                setObject:[NSValue
                                              valueWithCGRect:CGRectMake(
                                                                  [elementArray[1] floatValue],
                                                                  [elementArray[2] floatValue],
                                                                  [elementArray[3] floatValue],
                                                                  [elementArray[4] floatValue])]
                                   forKey:elementArray[0]];
                          }
                        }
                        completion(elements);
                      }];
}

#pragma mark - Methods related to Scribble support

- (void)setUpIndirectScribbleInteraction:(id<FlutterViewResponder>)viewResponder {
  if (_viewResponder != viewResponder) {
    if (@available(iOS 14.0, *)) {
      UIView* parentView = viewResponder.view;
      if (parentView != nil) {
        UIIndirectScribbleInteraction* scribbleInteraction = [[UIIndirectScribbleInteraction alloc]
            initWithDelegate:(id<UIIndirectScribbleInteractionDelegate>)self];
        [parentView addInteraction:scribbleInteraction];
      }
    }
  }
  _viewResponder = viewResponder;
}

- (void)resetViewResponder {
  _viewResponder = nil;
}

#pragma mark -
#pragma mark FlutterKeySecondaryResponder

/**
 * Handles key down events received from the view controller, responding YES if
 * the event was handled.
 */
- (BOOL)handlePress:(nonnull FlutterUIPressProxy*)press API_AVAILABLE(ios(13.4)) {
  return NO;
}
@end

/**
 * Recursively searches the UIView's subviews to locate the First Responder
 */
@implementation UIView (FindFirstResponder)
- (id)flutterFirstResponder {
  if (self.isFirstResponder) {
    return self;
  }
  for (UIView* subView in self.subviews) {
    UIView* firstResponder = subView.flutterFirstResponder;
    if (firstResponder) {
      return firstResponder;
    }
  }
  return nil;
}
@end
