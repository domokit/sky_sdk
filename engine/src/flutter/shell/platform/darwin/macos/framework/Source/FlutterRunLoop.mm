#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterRunLoop.h"
#include "fml/logging.h"

#include <vector>

namespace {
struct Task {
  void (^block)(void);
  CFAbsoluteTime target_time;
};

CFStringRef kFlutterRunLoopMode = CFSTR("FlutterRunLoopMode");

FlutterRunLoop* mainLoop;

}  // namespace

@implementation FlutterRunLoop (Private)

CFRunLoopRef _runLoop;
CFRunLoopSourceRef _source;
CFRunLoopTimerRef _timer;
std::vector<Task> _tasks;

@end

@implementation FlutterRunLoop

static void Perform(void* info) {
  FlutterRunLoop* runner = (__bridge FlutterRunLoop*)info;
  [runner performExpiredTasks];
}

static void PerformTimer(CFRunLoopTimerRef timer, void* info) {
  FlutterRunLoop* runner = (__bridge FlutterRunLoop*)info;
  [runner performExpiredTasks];
}

- (instancetype)init {
  if (self = [super init]) {
    _runLoop = CFRunLoopGetCurrent();
    CFRunLoopSourceContext sourceContext = {
        .info = (__bridge void*)self,
        .perform = Perform,
    };
    _source = CFRunLoopSourceCreate(kCFAllocatorDefault, 0, &sourceContext);
    CFRunLoopAddSource(_runLoop, _source, kCFRunLoopCommonModes);
    CFRunLoopAddSource(_runLoop, _source, kFlutterRunLoopMode);

    CFRunLoopTimerContext timerContext = {
        .info = (__bridge void*)self,
    };
    _timer = CFRunLoopTimerCreate(kCFAllocatorDefault, HUGE_VALF, HUGE_VALF, 0, 0, PerformTimer,
                                  &timerContext);
    CFRunLoopAddTimer(_runLoop, _timer, kCFRunLoopCommonModes);
    CFRunLoopAddTimer(_runLoop, _timer, kFlutterRunLoopMode);
  }
  return self;
}

- (void)dealloc {
  CFRunLoopTimerInvalidate(_timer);
  CFRunLoopRemoveTimer(_runLoop, _timer, kCFRunLoopCommonModes);
  CFRunLoopRemoveTimer(_runLoop, _timer, kFlutterRunLoopMode);
  CFRunLoopSourceInvalidate(_source);
  CFRunLoopRemoveSource(_runLoop, _source, kCFRunLoopCommonModes);
  CFRunLoopRemoveSource(_runLoop, _source, kFlutterRunLoopMode);
}

- (void)rearmTimer {
  CFAbsoluteTime nextFireTime = HUGE_VALF;
  for (const auto& task : _tasks) {
    nextFireTime = std::min(nextFireTime, task.target_time);
  }
  CFRunLoopTimerSetNextFireDate(_timer, nextFireTime);
}

- (void)performExpiredTasks {
  std::vector<Task> expiredTasks;
  @synchronized(self) {
    CFAbsoluteTime now = CFAbsoluteTimeGetCurrent();
    std::vector<Task>::iterator it = _tasks.begin();
    while (it != _tasks.end()) {
      if (it->target_time <= now) {
        expiredTasks.push_back(std::move(*it));
        it = _tasks.erase(it);
      } else {
        ++it;
      }
    }
    [self rearmTimer];
  }
  for (const auto& task : expiredTasks) {
    task.block();
  }
}

- (void)performBlock:(void (^)(void))block afterDelay:(NSTimeInterval)delay {
  @synchronized(self) {
    _tasks.push_back({block, CFAbsoluteTimeGetCurrent() + delay});
    if (delay > 0) {
      [self rearmTimer];
    } else {
      CFRunLoopSourceSignal(_source);
      CFRunLoopWakeUp(_runLoop);
    }
  }
}

- (void)performBlock:(void (^)(void))block {
  [self performBlock:block afterDelay:0];
}

+ (void)ensureMainLoopInitialized {
  FML_DCHECK([NSRunLoop currentRunLoop] == [NSRunLoop mainRunLoop]);
  if (mainLoop == nil) {
    mainLoop = [[FlutterRunLoop alloc] init];
  }
}

+ (FlutterRunLoop*)mainRunLoop {
  FML_DCHECK(mainLoop != nil);
  return mainLoop;
}

- (void)pollFlutterMessagesOnce {
  CFRunLoopRunInMode(kFlutterRunLoopMode, 0.1, YES);
}

@end
