#import <IOKit/hid/IOHIDLib.h>
#import <Foundation/NSObjCRuntime.h>
#import <Foundation/Foundation.h>

// for FrSky Taranis X9D Plus
#define COOKIE_THROTTLE (11)
#define COOKIE_ROLL (12)
#define COOKIE_PITCH (13)
#define COOKIE_YAW (14)

void gamepadWasAdded(void* inContext, IOReturn inResult, void* inSender, IOHIDDeviceRef device)
{
    // nothing
}

void gamepadWasRemoved(void* inContext, IOReturn inResult, void* inSender, IOHIDDeviceRef device) {
    // nothing
}

void gamepadAction(void* inContext, IOReturn inResult, void* inSender, IOHIDValueRef value)
{
    IOHIDElementRef element = IOHIDValueGetElement(value);
    int elementValue = IOHIDValueGetIntegerValue(value);
    int min = IOHIDElementGetLogicalMin(element);
    int max = IOHIDElementGetLogicalMax(element);
    IOHIDElementCookie input = IOHIDElementGetCookie(element);
    if (input == COOKIE_THROTTLE) {
        NSLog(@"THROTTLE = %d", elementValue);
    } else if (input == COOKIE_ROLL) {
        NSLog(@"ROLL = %d", elementValue);
    } else if (input == COOKIE_PITCH) {
        NSLog(@"PITCH = %d", elementValue);
    } else if (input == COOKIE_YAW) {
        NSLog(@"YAW = %d", elementValue);
    }
    // TODO
}

void setupGamepad()
{
    IOHIDManagerRef hidManager = IOHIDManagerCreate( kCFAllocatorDefault, kIOHIDOptionsTypeNone);
    NSMutableDictionary* criterion = [[NSMutableDictionary alloc] init];
    [criterion setObject: [NSNumber numberWithInt: kHIDPage_GenericDesktop] forKey: (NSString*)CFSTR(kIOHIDDeviceUsagePageKey)];
    [criterion setObject: [NSNumber numberWithInt: kHIDUsage_GD_GamePad] forKey: (NSString*)CFSTR(kIOHIDDeviceUsageKey)];
    IOHIDManagerSetDeviceMatching(hidManager, (__bridge CFDictionaryRef) criterion);
    IOHIDManagerRegisterDeviceMatchingCallback(hidManager, gamepadWasAdded, nil);
    IOHIDManagerRegisterDeviceRemovalCallback(hidManager, gamepadWasRemoved, nil);
    IOHIDManagerScheduleWithRunLoop(hidManager, CFRunLoopGetCurrent(), kCFRunLoopDefaultMode);
    IOReturn tIOReturn = IOHIDManagerOpen(hidManager, kIOHIDOptionsTypeNone);
    IOHIDManagerRegisterInputValueCallback(hidManager, gamepadAction, nil);
}

int main(int argc, const char *argv[]) {
    setupGamepad();
    CFRunLoopRun();
}
