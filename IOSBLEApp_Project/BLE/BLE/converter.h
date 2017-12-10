//
//  NSObject+converter.h
//  BLE
//
//  Created by Oli Eydt on 2017-11-30.
//  Copyright © 2017 BroLoveEnergy. All rights reserved.
//

#import <Foundation/Foundation.h>
@import CoreBluetooth;

@interface NSObject (converter)

- (NSString *)representativeString:(CBUUID*)uuid;

@end
