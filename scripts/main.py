#!/usr/bin/env python3

import rospy
import asyncio
import can
import time
from typing import List
from std_msgs.msg import Int32


class ControllerProcess:
    def __init__(self) -> None:
        rospy.init_node('Communication', anonymous=False)

        rospy.Subscriber('/rpmLeft', Int32, self.leftMotorCallBack)
        rospy.Subscriber('/rpmRight', Int32, self.rightMotorCallBack)
        self.pubLeftEncoder = rospy.Publisher('/leftEncoder', Int32, queue_size=10)
        self.pubRightEncoder = rospy.Publisher('/rightEncoder', Int32, queue_size=10)
        
        self.rate = rospy.Rate(100)
        
        self.rpmLeftData = [0x00, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x00]
        self.rpmRightData = [0x00, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x00]

        self.leftEncoderMSG = Int32()
        self.rightEncoderMSG = Int32()
        self.leftEncoderMSG.data = 0
        self.rightEncoderMSG.data = 0

        filters = [
            {"can_id": 0x281, "can_mask": 0xFFF, "extended": False},
            {"can_id": 0x282, "can_mask": 0xFFF, "extended": False},
        ]

        self.bus = can.ThreadSafeBus(bustype="socketcan", channel="can0", bitrate=1000000, can_filters=filters)
        listeners: List[can.notifier.MessageRecipient] = [self.getMessages]
        
        self.notifier = can.Notifier(self.bus, listeners, 0.1)

    def leftMotorCallBack(self, msg) -> None:
        self.rpmLeftData[0] = msg.data & 0xFF
        self.rpmLeftData[1] = msg.data >> 8 & 0xFF
        
        if msg.data < 0:
            self.rpmLeftData[2] = 0xFF
            self.rpmLeftData[3] = 0xFF
        else:
            self.rpmLeftData[2] = 0x00
            self.rpmLeftData[3] = 0x00
        
        self.sendMessages(id=0x201, data=self.rpmLeftData)

    def rightMotorCallBack(self, msg) -> None:
        self.rpmRightData[0] = msg.data & 0xFF
        self.rpmRightData[1] = msg.data >> 8 & 0xFF
        
        if msg.data < 0:
            self.rpmRightData[2] = 0xFF
            self.rpmRightData[3] = 0xFF
        else:
            self.rpmRightData[2] = 0x00
            self.rpmRightData[3] = 0x00
        
        self.sendMessages(id=0x202, data=self.rpmRightData)

    def sendStartMessages(self) -> None:
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
            self.bus.send(message)
            time.sleep(0.1)

    def sendEndMessages(self) -> None:
        endMessages = []
        endMessages.append(
            can.Message(
                arbitration_id=0x601, data=[0x2B, 0x40, 0x60, 0x00, 0x80, 0x00, 0x00, 0x00], is_extended_id=False
            )
        )
        endMessages.append(
            can.Message(
                arbitration_id=0x601, data=[0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00], is_extended_id=False
            )
        )
        endMessages.append(
            can.Message(
                arbitration_id=0x602, data=[0x2B, 0x40, 0x60, 0x00, 0x80, 0x00, 0x00, 0x00], is_extended_id=False
            )
        )
        endMessages.append(
            can.Message(
                arbitration_id=0x602, data=[0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00], is_extended_id=False
            )
        )
        endMessages.append(
            can.Message(
                arbitration_id=0x000, data=[], is_extended_id=False
            )
        )
        endMessages.append(
            can.Message(
                arbitration_id=0x080, data=[0x00, 0x00], is_extended_id=False
            )
        )
    
        for message in endMessages:
            self.bus.send(message)
            time.sleep(0.1)
            
    def sendStopMessages(self) -> None:
        stopMessages = []
        stopMessages.append(
            can.Message(
                arbitration_id=0x201, data=[0x00, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x00], is_extended_id=False
            )
        )
        stopMessages.append(
            can.Message(
                arbitration_id=0x202, data=[0x00, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x00], is_extended_id=False
            )
        )
        
        for message in stopMessages:
            self.bus.send(message)
            time.sleep(0.1)
        
    def sendMessages(self, id, data) -> None:
        self.bus.send(can.Message(arbitration_id=id, data=data, is_extended_id=False))
        
    def getMessages(self, msg: can.Message) -> None:
        if msg is not None:
            if msg.arbitration_id == 0x281:
                self.leftEncoderMSG.data = self.hexToSignedInt(((msg.data[3] << 8 | msg.data[2]) << 8 | msg.data[1]) << 8 | msg.data[0])
            elif msg.arbitration_id == 0x282:
                self.rightEncoderMSG.data = self.hexToSignedInt(((msg.data[3] << 8 | msg.data[2]) << 8 | msg.data[1]) << 8 | msg.data[0])

    def main(self) -> None:
        while not rospy.is_shutdown():
            self.pubLeftEncoder.publish(self.leftEncoderMSG)
            self.pubRightEncoder.publish(self.rightEncoderMSG)
            
            self.rate.sleep()
        
        rospy.on_shutdown(self.callEnd)

    def callEnd(self) -> None:
        self.notifier.stop()
        self.sendStopMessages()
        self.sendEndMessages()
        
    def hexToSignedInt(self, value):
        return -(int(hex(value), 16) & 0x8000) | (int(hex(value), 16) & 0x7FFF)

if __name__ == "__main__":
    Controller = ControllerProcess()
    Controller.sendStartMessages()
    Controller.main()
        