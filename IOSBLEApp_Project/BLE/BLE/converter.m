//
//  NSObject+converter.m
//  BLE
//
//  Created by Oli Eydt on 2017-11-30.
//  Copyright © 2017 BroLoveEnergy. All rights reserved.
//

#import "converter.h"
@import CoreBluetooth;

@implementation NSObject (converter)

- (NSString *)representativeString:(CBUUID*)uuid
{
    NSData *data = [uuid data];
    
    NSUInteger bytesToConvert = [data length];
    
    const unsigned char *uuidBytes = [data bytes];
    NSMutableString *outputString = [NSMutableString stringWithCapacity:16];
    
    for (int currentByteIndex = bytesToConvert-1; currentByteIndex >= 0; currentByteIndex--)
    {
        switch (currentByteIndex)
        {
            case 12:
            case 10:
            case 8:
            case 6:[outputString appendFormat:@"%02x-", uuidBytes[currentByteIndex]]; break;
            default:[outputString appendFormat:@"%02x", uuidBytes[currentByteIndex]];
        }
        
    }
    
    return [outputString uppercaseString];
}

@end
