#! /usr/bin/env python3
# encoding: utf-8

import serial

class osx001Driver():
    port = serial.Serial()

    def __init__(self, COMPort):
        self.port = serial.Serial(COMPort, 1000000, timeout=1)

    def calculateCheckSum(self, array, size):
        sum = 0
        for num in range(size - 1):
            sum += array[num]
        return sum & 0xFF

    def bootAllRobot(self):
        txdata = [0xFF, 0xFD, 0x03, 0x00, 0x01, 0x00, 0x00]
        txdata[6] = self.calculateCheckSum(txdata, 7)
        self.port.write(txdata)

    def writeMotorSpeed(self, id, right, left):
        right = int(right)
        left = int(left)
        if right < 0:
            right = right + 256
        if left < 0:
            left = left + 256
        txdata = [0xFF, 0xFD, 0x05, 0x00, 0x03, id, right, left, 0x00]
        txdata[8] = self.calculateCheckSum(txdata, 9)
        self.port.write(txdata)

    def resetIMU(self, id):
        txdata = [0xFF, 0xFD, 0x03, 0x00, 0x04, id, 0x00]
        txdata[6] = self.calculateCheckSum(txdata, 7)
        self.port.write(txdata)

    def shutdownAllRobot(self):
        txdata = [0xFF, 0xFD, 0x02, 0x00, 0x05, 0x00]
        txdata[5] = self.calculateCheckSum(txdata, 6)
        self.port.write(txdata)

    def setTargetPosition(self, id, x, y, move = 1):
        if x < 0:
            x = x + 65536
        if y < 0:
            y = y + 65536
        txdata = [0xFF, 0xFD, 0x08, 0x00, 0x07, id, (x & 0xFF), ((x >> 8) & 0xFF), (y & 0xFF), ((y >> 8) & 0xFF), move, 0x00]
        txdata[11] = self.calculateCheckSum(txdata, 12)
        self.port.write(txdata)

    def setTargetPositionWithOrientation(self, id, x, y, orientation, move = 1):
        if x < 0:
            x = x + 65536
        if y < 0:
            y = y + 65536
        if orientation < 0:
            orientation = orientation + 65536

        txdata = [0xFF, 0xFD, 0x0A, 0x00, 0x08, id, (x & 0xFF), ((x >> 8) & 0xFF), (y & 0xFF), ((y >> 8) & 0xFF), (orientation & 0xFF), ((orientation >> 8) & 0xFF), move, 0x00]
        txdata[13] = self.calculateCheckSum(txdata, 14)
        self.port.write(txdata)

    def setTargetID(self, ids):
        if len(ids) < 1 or len(ids) > 9:
            return
        txdata = [0xFF, 0xFD, 0x02 + len(ids), 0x00, 0x10]
        for id in ids:
            txdata.append(id)
        txdata.append(0)
        txdata[5 + len(ids)] = self.calculateCheckSum(txdata, 6 + len(ids))
        self.port.write(txdata)

    def getTargetID(self):
        txdata = [0xFF, 0xFD, 0x02, 0x00, 0x11, 0x00]
        txdata[5] = self.calculateCheckSum(txdata, 6)
        self.port.write(txdata)

    def calibrateMotor(self, id):
        txdata = [0xFF, 0xFD, 0x03, 0x00, 0x3A, id, 0x00]
        txdata[6] = self.calculateCheckSum(txdata, 7)
        self.port.write(txdata)

    def flushBuffer(self):
        txdata = [0xFF, 0xFD, 0x02, 0x00, 0x06, 0x00]
        txdata[5] = self.calculateCheckSum(txdata, 6)
        self.port.write(txdata)

    commandState = 0
    commandLength = 0
    commandInstruction = 0
    commandData = [0] * 100
    commandPosition = 0

    def checkStatusPacket(self, verbose=True):
        rcvArray = self.port.read_all()
        data_dict = {}

        for data in rcvArray:
            # Restore data
            if self.commandState == 0:
                self.commandPosition = 0
            self.commandData[self.commandPosition] = data
            self.commandPosition = self.commandPosition + 1

            # Check command
            if self.commandState == 0: # 0xFF
                if data == 0xFF:
                    self.commandState = 1

            elif self.commandState == 1: # 0xFD
                if data == 0xFD:
                    self.commandState = 2
                else:
                    self.commandState = 0

            elif self.commandState == 2: # LEN_L
                self.commandLength = data
                self.commandState = 3

            elif self.commandState == 3: # LEN_H
                self.commandLength = self.commandLength + (data << 8)
                self.commandState = 4

            elif self.commandState == 4: # Instruction
                self.commandInstruction = data
                if self.commandLength == 2:
                    self.commandState = 6
                else:
                    self.commandState = 5

            elif self.commandState == 5: # Param
                 if self.commandPosition == self.commandLength + 3:
                    self.commandState = 6

            elif self.commandState == 6: # CheckSum
                self.commandState = 0
                if data != self.calculateCheckSum(self.commandData, self.commandPosition):
                    continue

                # Status command
                if self.commandInstruction == 0x21:
                    id = (self.commandData[5] & 0b01111111)
                    x_position = (self.commandData[6] + (self.commandData[7] << 8))
                    y_position = (self.commandData[8] + (self.commandData[9] << 8))
                    degree  = (self.commandData[10] + (self.commandData[11] << 8) )
                    voltage = (self.commandData[12] + (self.commandData[13] << 8) )
                    yaw = (self.commandData[14] + (self.commandData[15] << 8) )
                    pitch = (self.commandData[16] + (self.commandData[17] << 8) )
                    roll = (self.commandData[18] + (self.commandData[19] << 8) )

                    if x_position > 32768:
                        x_position = x_position - 65536
                    if y_position > 32768:
                        y_position = y_position - 65536
                    if degree > 32768:
                        degree = degree - 65536
                    if yaw > 32768:
                        yaw = yaw - 65536
                    if pitch > 32768:
                        pitch = pitch - 65536
                    if roll > 32768:
                        roll = roll - 65536

                    if verbose:
                        print({'id':id, 'x':x_position, 'y':y_position, 'deg' : degree, 'v' : voltage, 'yaw' : yaw, 'pitch' : pitch, 'roll' : roll})

                    data_dict[id] = [x_position, y_position, degree, voltage, yaw, pitch, roll]

                elif self.commandInstruction == 0x28:
                    ids = []
                    for i in range(0, self.commandLength - 2):
                        ids.append(self.commandData[5 + i])
                    print(ids)
        return data_dict
