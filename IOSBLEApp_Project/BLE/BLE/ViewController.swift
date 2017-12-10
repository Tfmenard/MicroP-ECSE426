//
//  ViewController.swift
//  BLE
//
//  Created by Oli Eydt on 2017-11-28.
//  Copyright © 2017 BroLoveEnergy. All rights reserved.
//

import UIKit
import CoreBluetooth
import FirebaseStorage

class ViewController: UIViewController, CBCentralManagerDelegate, CBPeripheralDelegate {

    @IBAction func stopBtn(_ sender: Any) {
        if(self.peripheral != nil){
            self.manager.cancelPeripheralConnection(self.peripheral)
        } else{
            self.manager.stopScan()
        }
        byteCounter = 0
        self.byteCounterLabel.text = "\(byteCounter) / 32512 bytes received."
        self.logLabel.text = "Disconnected"
    }
    @IBOutlet var byteCounterLabel: UITextView!
    @IBOutlet var logLabel: UITextView!
    @IBAction func startBtn(_ sender: Any) {
        manager = CBCentralManager(delegate: self, queue: nil)
    }
    var manager:CBCentralManager!
    var peripheral:CBPeripheral!
    var charact:CBCharacteristic!
    
    let ENVIRONMENTAL_SERVICE = "1BC5D5A5-0200-D082-E211-77E4401A8242"
    let HUMIDITY_CHAR = "1BC5D5A5-0200-73A0-E211-8CE4600BC501"
    let BEAN_NAME = "Celine"
    
    
    func sendData(data: Data){
        let storageRef = Storage.storage().reference()
        let homeImagesRef = storageRef.child("home/sample.wav")
        _ = homeImagesRef.putData(data, metadata: nil) { (metadata, error) in
            guard let metadata = metadata else {
                // Uh-oh, an error occurred!
                self.logLabel.text = "Error sending data."
                print(error.debugDescription)
                return
            }
        }
    }
    
    override func viewDidLoad() {
        super.viewDidLoad()
        // Do any additional setup after loading the view, typically from a nib.
    }

    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
        // Dispose of any resources that can be recreated.
    }
    
    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        if central.state == .poweredOn {
            self.logLabel.text = "Scanning..."
            print("scanning")
            central.scanForPeripherals(withServices: nil, options: nil)
        } else {
            print("Bluetooth not available.")
        }
    }
    
    func centralManager(_ central: CBCentralManager, didDiscover peripheral: CBPeripheral, advertisementData: [String : Any], rssi RSSI: NSNumber) {
        let device = (advertisementData as NSDictionary)
            .object(forKey: CBAdvertisementDataLocalNameKey)
            as? NSString
        if(device != nil){
            print(device)
        }
        if device?.contains(BEAN_NAME) == true {
            self.manager.stopScan()
            self.peripheral = peripheral
            self.logLabel.text = "Found Device!"
            print("Connecting...")
            self.peripheral.delegate = self
            manager.connect(peripheral, options: nil)
        }
    }
    
    func centralManager(_ central: CBCentralManager,
        didConnect peripheral: CBPeripheral) {
        //peripheral.delegate = self
        peripheral.discoverServices(nil)
        self.logLabel.text = "Connected."
        print("Connected.")
    }
    
    func centralManager(_ central: CBCentralManager, didDisconnectPeripheral peripheral: CBPeripheral, error: Error?) {
        print(error.debugDescription)
    }
    
    func centralManager(_ central: CBCentralManager, didFailToConnect peripheral: CBPeripheral, error: Error?) {
        print(error.debugDescription)
    }
    
    func peripheral(
        _ peripheral: CBPeripheral,
        didDiscoverServices error: Error?) {
        for service in peripheral.services! {
            let thisService = service as CBService
            //print("services")
            let serviceId = representativeString(service.uuid)!
            //print(serviceId)
            if serviceId == String(utf8String: ENVIRONMENTAL_SERVICE.cString(using: .utf8)!) {
                peripheral.discoverCharacteristics(
                    nil,
                    for: thisService
                )
            }
        }
    }
    
    func peripheral(
        _ peripheral: CBPeripheral,
        didDiscoverCharacteristicsFor service: CBService,
        error: Error?) {
        for characteristic in service.characteristics! {
            let thisCharacteristic = characteristic as CBCharacteristic
            //print("charact)")
            let charactId = representativeString(thisCharacteristic.uuid)!
            //print(charactId)
            if charactId == String(utf8String: HUMIDITY_CHAR.cString(using: .utf8)!) {
                self.peripheral.setNotifyValue(
                    true,
                    for: thisCharacteristic
                )
                //print("found charact")
                self.logLabel.text = "Waiting for first value."
                self.peripheral.readValue(for: thisCharacteristic)
                dataToSend = Data()
                dataEnd = Data()
                endOfMessage = 0
            }
        }
    }
    
    var dataToSend:Data!
    var dataEnd:Data!
    var endOfMessage = 0
    var hack = true
    var byteCounter = 0
    func peripheral(_ peripheral: CBPeripheral, didUpdateValueFor characteristic: CBCharacteristic, error: Error?) {
        //var count:UInt32 = 0;
        //if let value = String(data: characteristic.value!, encoding: .utf8){
        //    print(value)
        //}
        //characteristic.value!.getBytes(&count, length: sizeof(UInt8))
        //print(value)
        if representativeString(characteristic.uuid) == String(utf8String: HUMIDITY_CHAR.cString(using: .utf8)!) {
            //var num: Int = 0
            if(characteristic.value != nil){
                let data = characteristic.value!
                //print(data.count)
                //let array = [UInt8](data)
                //(data as NSData).getBytes(&num, length: MemoryLayout<Int>.size) //MemoryLayout<Int>.size
                if(hack){
                    updateByteCounter()
                    //print(data.count)
                    dataToSend.append(data)
                    //print(num)
                    if(byteCounter == 32512){
                        print(dataToSend.count)
                        sendData(data: dataToSend)
                        self.logLabel.text = "Sending data."
                        self.manager.cancelPeripheralConnection(self.peripheral)
                        return
                    }
                    self.peripheral.readValue(for: characteristic)
                }
                hack = !hack
                
                //data.getBytes(&num, length: 32512) //MemoryLayout<Int>.size
                //characteristic.value!.copyBytes(&num, length: MemoryLayout<Int>.size)
                //print(num)
            } else{
                self.logLabel.text = "Received nil"
            }
            
            //characteristic.value!.getBytes(&count, length: sizeof(UInt8))
            //print(value)
            //print(NSString(format: "%llu", count) as String)
        } else{
            print("Wrong charact")
        }
    }
    
    func updateByteCounter(){
        byteCounter += 16
        byteCounterLabel.text = "\(byteCounter) / 32512 bytes received."
        self.logLabel.text = "Receiving bytes"
    }
    

}

