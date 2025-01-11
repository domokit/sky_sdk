// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "ShareViewController.h"

@interface ShareViewController ()

@end

@implementation ShareViewController

- (instancetype)init {
  FlutterEngine* engine = [[FlutterEngine alloc] initWithName:@"FlutterControllerTest" project:nil];
  [engine run];
  self = [self initWithEngine:engine nibName:nil bundle:nil];
  self.view.accessibilityIdentifier = @"flutter_view";

  [engine.binaryMessenger
      setMessageHandlerOnChannel:@"waiting_for_status"
            binaryMessageHandler:^(NSData* _Nullable message, FlutterBinaryReply _Nonnull reply) {
              FlutterMethodChannel* channel = [FlutterMethodChannel
                  methodChannelWithName:@"driver"
                        binaryMessenger:engine.binaryMessenger
                                  codec:[FlutterJSONMethodCodec sharedInstance]];
              [channel invokeMethod:@"set_scenario" arguments:@{@"name" : @"app_extension"}];
            }];
  return self;
}

@end
