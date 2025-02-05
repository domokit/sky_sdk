// Copyright 2019 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "ScreenBeforeFlutter.h"
#import "FlutterEngine+ScenariosTest.h"

@implementation ScreenBeforeFlutter

@synthesize engine = _engine;

- (id)initWithEngineRunCompletion:(dispatch_block_t)engineRunCompletion {
  self = [super init];
  _engine = [[FlutterEngine alloc] initWithScenario:@"poppable_screen"
                                     withCompletion:engineRunCompletion];
  return self;
}

- (void)viewDidLoad {
  [super viewDidLoad];
  self.view.backgroundColor = UIColor.grayColor;

  UIButton* showFlutterButton = [UIButton buttonWithType:UIButtonTypeSystem];
  showFlutterButton.translatesAutoresizingMaskIntoConstraints = NO;
  showFlutterButton.backgroundColor = UIColor.blueColor;
  [showFlutterButton setTitle:@"Show Flutter" forState:UIControlStateNormal];
  showFlutterButton.tintColor = UIColor.whiteColor;
  showFlutterButton.clipsToBounds = YES;
  [showFlutterButton addTarget:self
                        action:@selector(showFlutter:)
              forControlEvents:UIControlEventTouchUpInside];

  [self.view addSubview:showFlutterButton];
  [[showFlutterButton.centerXAnchor constraintEqualToAnchor:self.view.centerXAnchor] setActive:YES];
  [[showFlutterButton.centerYAnchor constraintEqualToAnchor:self.view.centerYAnchor] setActive:YES];
  [[showFlutterButton.heightAnchor constraintEqualToConstant:50] setActive:YES];
  [[showFlutterButton.widthAnchor constraintEqualToConstant:150] setActive:YES];

  [_engine runWithEntrypoint:nil];
}

- (FlutterViewController*)showFlutter:(dispatch_block_t)showCompletion {
  FlutterViewController* flutterVC = [[FlutterViewController alloc] initWithEngine:_engine
                                                                           nibName:nil
                                                                            bundle:nil];
  [self presentViewController:flutterVC animated:NO completion:showCompletion];
  return flutterVC;
}

- (FlutterEngine*)engine {
  return _engine;
}

@end
