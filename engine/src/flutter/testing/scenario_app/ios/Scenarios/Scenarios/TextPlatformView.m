// Copyright 2018 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "TextPlatformView.h"

@protocol TestGestureRecognizerDelegate <NSObject>

- (void)gestureTouchesBegan;
- (void)gestureTouchesEnded;

@end

@interface TestTapGestureRecognizer : UITapGestureRecognizer

@property(weak, nonatomic)
    NSObject<TestGestureRecognizerDelegate>* testTapGestureRecognizerDelegate;

@end

@implementation TestTapGestureRecognizer

- (void)touchesBegan:(NSSet<UITouch*>*)touches withEvent:(UIEvent*)event {
  [self.testTapGestureRecognizerDelegate gestureTouchesBegan];
  [super touchesBegan:touches withEvent:event];
}

- (void)touchesEnded:(NSSet<UITouch*>*)touches withEvent:(UIEvent*)event {
  [self.testTapGestureRecognizerDelegate gestureTouchesEnded];
  [super touchesEnded:touches withEvent:event];
}

@end

@implementation TextPlatformViewFactory {
  NSObject<FlutterBinaryMessenger>* _messenger;
}

- (instancetype)initWithMessenger:(NSObject<FlutterBinaryMessenger>*)messenger {
  self = [super init];
  if (self) {
    _messenger = messenger;
  }
  return self;
}

- (NSObject<FlutterPlatformView>*)createWithFrame:(CGRect)frame
                                   viewIdentifier:(int64_t)viewId
                                        arguments:(id _Nullable)args {
  TextPlatformView* textPlatformView = [[TextPlatformView alloc] initWithFrame:frame
                                                                viewIdentifier:viewId
                                                                     arguments:args
                                                               binaryMessenger:_messenger];
  return textPlatformView;
}

- (NSObject<FlutterMessageCodec>*)createArgsCodec {
  return [FlutterStringCodec sharedInstance];
}

@end

@interface TextPlatformView () <TestGestureRecognizerDelegate>

@end

@implementation TextPlatformView {
  UIView* _containerView;
  UITextView* _textView;
  FlutterMethodChannel* _channel;
  BOOL _viewCreated;
}

- (instancetype)initWithFrame:(CGRect)frame
               viewIdentifier:(int64_t)viewId
                    arguments:(id _Nullable)args
              binaryMessenger:(NSObject<FlutterBinaryMessenger>*)messenger {
  self = [super init];
  if (self) {
    _containerView = [[UIView alloc] initWithFrame:CGRectMake(0, 0, 250, 100)];
    _containerView.backgroundColor = UIColor.lightGrayColor;
    _containerView.clipsToBounds = YES;
    _containerView.accessibilityIdentifier = @"platform_view";

    _textView = [[UITextView alloc] initWithFrame:CGRectMake(50.0, 50.0, 250, 100)];
    _textView.backgroundColor = UIColor.lightGrayColor;
    _textView.textColor = UIColor.blueColor;
    [_textView setFont:[UIFont systemFontOfSize:52]];
    _textView.text = args;
    _textView.autoresizingMask =
        (UIViewAutoresizingFlexibleWidth | UIViewAutoresizingFlexibleHeight);
    [_containerView addSubview:_textView];

    TestTapGestureRecognizer* gestureRecognizer =
        [[TestTapGestureRecognizer alloc] initWithTarget:self action:@selector(platformViewTapped)];

    [_textView addGestureRecognizer:gestureRecognizer];
    gestureRecognizer.testTapGestureRecognizerDelegate = self;

    _textView.accessibilityLabel = @"";

    _viewCreated = NO;
  }
  return self;
}

- (UIView*)view {
  // Makes sure the engine only calls this method once.
  if (_viewCreated) {
    abort();
  }
  _viewCreated = YES;
  return _containerView;
}

- (void)platformViewTapped {
  _textView.accessibilityLabel =
      [_textView.accessibilityLabel stringByAppendingString:@"-platformViewTapped"];
}

- (void)gestureTouchesBegan {
  _textView.accessibilityLabel =
      [_textView.accessibilityLabel stringByAppendingString:@"-gestureTouchesBegan"];
}

- (void)gestureTouchesEnded {
  _textView.accessibilityLabel =
      [_textView.accessibilityLabel stringByAppendingString:@"-gestureTouchesEnded"];
}

@end
