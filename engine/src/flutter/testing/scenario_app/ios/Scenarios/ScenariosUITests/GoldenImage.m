// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "GoldenImage.h"

#import <XCTest/XCTest.h>
#import <os/log.h>
#include <sys/sysctl.h>

@interface GoldenImage ()

@end

@implementation GoldenImage

- (instancetype)initWithGoldenNamePrefix:(NSString*)prefix {
  self = [super init];
  if (self) {
    _goldenName = [prefix stringByAppendingString:_platformName()];
    NSBundle* bundle = [NSBundle bundleForClass:[self class]];
    NSURL* goldenURL = [bundle URLForResource:_goldenName withExtension:@"png"];
    NSData* data = [NSData dataWithContentsOfURL:goldenURL];
    _image = [[UIImage alloc] initWithData:data];
  }
  return self;
}

- (BOOL)compareGoldenToImage:(UIImage*)image rmesThreshold:(double)rmesThreshold {
  if (!self.image || !image) {
    os_log_error(OS_LOG_DEFAULT, "GOLDEN DIFF FAILED: image does not exists.");
    return NO;
  }
  CGImageRef imageRefA = [self.image CGImage];
  CGImageRef imageRefB = [image CGImage];

  NSUInteger widthA = CGImageGetWidth(imageRefA);
  NSUInteger heightA = CGImageGetHeight(imageRefA);
  NSUInteger widthB = CGImageGetWidth(imageRefB);
  NSUInteger heightB = CGImageGetHeight(imageRefB);

  if (widthA != widthB || heightA != heightB) {
    os_log_error(OS_LOG_DEFAULT, "GOLDEN DIFF FAILED: images sizes do not match.");
    return NO;
  }
  NSUInteger bytesPerPixel = 4;
  NSUInteger size = widthA * heightA * bytesPerPixel;
  NSMutableData* rawA = [NSMutableData dataWithLength:size];
  NSMutableData* rawB = [NSMutableData dataWithLength:size];

  if (!rawA || !rawB) {
    os_log_error(OS_LOG_DEFAULT, "GOLDEN DIFF FAILED: image data length do not match.");
    return NO;
  }

  CGColorSpaceRef colorSpace = CGColorSpaceCreateDeviceRGB();

  NSUInteger bytesPerRow = bytesPerPixel * widthA;
  NSUInteger bitsPerComponent = 8;
  CGContextRef contextA =
      CGBitmapContextCreate(rawA.mutableBytes, widthA, heightA, bitsPerComponent, bytesPerRow,
                            colorSpace, kCGImageAlphaPremultipliedLast | kCGBitmapByteOrder32Big);

  CGContextDrawImage(contextA, CGRectMake(0, 0, widthA, heightA), imageRefA);
  CGContextRelease(contextA);

  CGContextRef contextB =
      CGBitmapContextCreate(rawB.mutableBytes, widthA, heightA, bitsPerComponent, bytesPerRow,
                            colorSpace, kCGImageAlphaPremultipliedLast | kCGBitmapByteOrder32Big);
  CGColorSpaceRelease(colorSpace);

  CGContextDrawImage(contextB, CGRectMake(0, 0, widthA, heightA), imageRefB);
  CGContextRelease(contextB);

  const char* apos = rawA.mutableBytes;
  const char* bpos = rawB.mutableBytes;
  double sum = 0.0;
  for (size_t i = 0; i < size; ++i, ++apos, ++bpos) {
    // Skip transparent pixels.
    if (*apos == 0 && *bpos == 0 && i % 4 == 0) {
      i += 3;
      apos += 3;
      bpos += 3;
    } else {
      double aval = *apos;
      double bval = *bpos;
      double diff = aval - bval;
      sum += diff * diff;
    }
  }
  double rmse = sqrt(sum / size);
  if (rmse > rmesThreshold) {
    os_log_error(
        OS_LOG_DEFAULT,
        "GOLDEN DIFF FAILED: image diff greater than threshold. Current diff: %@, threshold: %@",
        @(rmse), @(rmesThreshold));
    return NO;
  }
  return YES;
}

NS_INLINE NSString* _platformName(void) {
  NSString* systemVersion = UIDevice.currentDevice.systemVersion;
  NSString* simulatorName =
      [[NSProcessInfo processInfo].environment objectForKey:@"SIMULATOR_DEVICE_NAME"];
  if (simulatorName) {
    return [NSString stringWithFormat:@"%@_%@_simulator", simulatorName, systemVersion];
  }

  size_t size;
  sysctlbyname("hw.model", NULL, &size, NULL, 0);
  char* answer = malloc(size);
  sysctlbyname("hw.model", answer, &size, NULL, 0);

  NSString* results = [NSString stringWithUTF8String:answer];
  free(answer);
  return results;
}

@end
