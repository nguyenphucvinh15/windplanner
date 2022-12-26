#!/usr/bin/env python3

import can
import time

rpmLeftData = [0x00, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x00]
rpmRightData = [0x00, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x00]

def leftMotorCallBack(msg):
    rpmLeftData[0] = msg & 0xFF
    rpmLeftData[1] = msg >> 8 & 0xFF
    print(rpmLeftData)

def rightMotorCallBack(msg):
    rpmRightData[0] = msg & 0xFF
    rpmRightData[1] = msg >> 8 & 0xFF

def sendStartMessages(bus):
    startMessages = []
    startMessages.append(
        can.Message(
            arbitration_id=0x601, data=[0x2B, 0x40, 0x60, 0x00, 0x80, 0x00, 0x00, 0x00], is_extended_id=False
        )
    )
    startMessages.append(
        can.Message(
            arbitration_id=0x601, data=[0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00], is_extended_id=False
        )
    )
    startMessages.append(
        can.Message(
            arbitration_id=0x601, data=[0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00], is_extended_id=False
        )
    )
    startMessages.append(
        can.Message(
            arbitration_id=0x602, data=[0x2B, 0x40, 0x60, 0x00, 0x80, 0x00, 0x00, 0x00], is_extended_id=False
        )
    )
    startMessages.append(
        can.Message(
            arbitration_id=0x602, data=[0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00], is_extended_id=False
        )
    )
    startMessages.append(
        can.Message(
            arbitration_id=0x602, data=[0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00], is_extended_id=False
        )
    )
    startMessages.append(
        can.Message(
            arbitration_id=0x080, data=[], is_extended_id=False
        )
    )
    startMessages.append(
        can.Message(
            arbitration_id=0x000, data=[0x01, 0x00], is_extended_id=False
        )
    )
    
    for message in startMessages:
        bus.send(message)
        time.sleep(0.1)

def sendEndMessages(bus):
    startMessages = []
    startMessages.append(
        can.Message(
            arbitration_id=0x601, data=[0x2B, 0x40, 0x60, 0x00, 0x80, 0x00, 0x00, 0x00], is_extended_id=False
        )
    )
    startMessages.append(
        can.Message(
            arbitration_id=0x601, data=[0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00], is_extended_id=False
        )
    )
    startMessages.append(
        can.Message(
            arbitration_id=0x602, data=[0x2B, 0x40, 0x60, 0x00, 0x80, 0x00, 0x00, 0x00], is_extended_id=False
        )
    )
    startMessages.append(
        can.Message(
            arbitration_id=0x602, data=[0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00], is_extended_id=False
        )
    )
    startMessages.append(
        can.Message(
            arbitration_id=0x000, data=[], is_extended_id=False
        )
    )
    startMessages.append(
        can.Message(
            arbitration_id=0x080, data=[0x00, 0x00], is_extended_id=False
        )
    )
 
    for message in startMessages:
        bus.send(message)
        time.sleep(0.1)
        
def sendStopMessages(bus):
    startMessages = []
    startMessages.append(
        can.Message(
            arbitration_id=0x201, data=[0x00, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x00], is_extended_id=False
        )
    )
    startMessages.append(
        can.Message(
            arbitration_id=0x202, data=[0x00, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x00], is_extended_id=False
        )
    )
    
    for message in startMessages:
        bus.send(message)
        time.sleep(0.1)

def getMessages(bus):
    print(bus.recv(0.1))
      
def sendMessages(bus, dataLeft, dataRight):
    bus.send(can.Message(arbitration_id=0x201, data=dataLeft, is_extended_id=False))
    bus.send(can.Message(arbitration_id=0x202, data=dataRight, is_extended_id=False))

def main():
    with can.interface.Bus(bustype="socketcan", channel="can0", bitrate=500000) as bus:
        try:
            # sendStartMessages(bus)
            
            # leftMotorCallBack(2000)
            # rightMotorCallBack(2000)
    
            # sendMessages(bus, rpmLeftData, rpmRightData)
            
            # time.sleep(5)
            
            sendEndMessages(bus)
                
        except can.CanError:
            print("Message NOT Sent!")
    

if __name__ == "__main__":
    main()