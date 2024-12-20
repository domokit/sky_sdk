// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "ViewController.h"

@interface ViewController ()

@end

@implementation ViewController

- (void)viewDidLoad {
  [super viewDidLoad];
  UIButton* openShare =
      [UIButton systemButtonWithPrimaryAction:[UIAction actionWithHandler:^(
                                                            __kindof UIAction* _Nonnull action) {
                  UIActivityViewController* activityVC =
                      [[UIActivityViewController alloc] initWithActivityItems:@[ @"text to share" ]
                                                        applicationActivities:nil];
                  activityVC.excludedActivityTypes = @[
                    UIActivityTypePrint, UIActivityTypeCopyToPasteboard,
                    UIActivityTypeAssignToContact, UIActivityTypeSaveToCameraRoll
                  ];  // Exclude whichever aren't relevant
                  [self presentViewController:activityVC animated:YES completion:nil];
                }]];
  openShare.backgroundColor = [UIColor systemPinkColor];
  [openShare setTitle:@"Open Share" forState:UIControlStateNormal];
  [self.view addSubview:openShare];
  openShare.frame = CGRectMake(0, 0, 200, 200);
}

@end
