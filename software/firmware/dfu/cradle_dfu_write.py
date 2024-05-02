#! /usr/bin/env python3
# encoding: utf-8

import sys
import serial
from serial.tools import list_ports
import time

pageSize = 2048
keyCode = bytes([0xFF, 0xFD, 0x02, 0x00, 0xFA, 0xF8])

def calculateCheckSum(array):
    sum = 0
    for data in array:
        sum += data
    return sum & 0xFF

def main():
    args = sys.argv

    # Exit if no serial port and binary file specified
    if len(args) < 3:
        ports = list_ports.comports()
        devices = [info.device for info in ports]
        print("Specify serial port and binary file\r\n")
        print("e.g., python cradle_dfu_write.py COM3 firmware.bin\r\n")
        print("Port List : \r\n")
        for port in ports:
            print(port)
        sys.exit(0)

    # Firmware loading
    f = open(args[2], 'rb')
    flashBuffer = f.read()
    fileSize = len(flashBuffer)
    pageNum = int(fileSize / pageSize)
    if fileSize % pageSize != 0:
        pageNum = pageNum + 1
    writtenPage = 0

    # Preparing the serial port
    port = serial.Serial(args[1], 1000000, timeout=1)

    # Restart TM002 (cradle) in DFU mode
    rebootBoard = [0xFF, 0xFD, 0x02, 0x00, 0x02]
    rebootBoard.append(calculateCheckSum(rebootBoard))
    port.write(rebootBoard)

    receiveState = 0
    receiveFlag = 0

    while(True):
        # Waiting for request command reception
        rcvArray = port.read_all()

        for data in rcvArray:
            if data == keyCode[receiveState]:
                receiveState = receiveState + 1
            else:
                receiveState = 0

            if receiveState == 4:
                receiveState = 0
                receiveFlag = 1

        if receiveFlag == 0:
            continue

        receiveFlag = 0

        # Firmware writing
        # Header
        flashData = bytes([0xFF, 0xFD])
        flashArray = bytearray(flashData)

        # LEN_L, LEN_H
        pageLength = pageSize
        if fileSize - pageSize * writtenPage < pageSize:
            pageLength = fileSize - pageSize * writtenPage

        flashArray.append((pageLength + 2) & 0xFF)
        flashArray.append((pageLength + 2) >> 8)

        # Instruction
        if writtenPage < pageNum - 1:
            flashArray.append(0xFC)
        else:
            flashArray.append(0xFD)

        # Copy Parameters
        for num in range(pageLength):
            flashArray.append(flashBuffer[writtenPage * pageSize + num])

        # Check-Sum
        flashArray.append(calculateCheckSum(flashArray))
        flashData = bytes(flashArray)

        # Send Firmware
        port.write(flashData)

        writtenPage = writtenPage + 1

        print((int)(writtenPage / pageNum * 100), end='')
        print("% Complete...")
        if writtenPage == pageNum:
            break

if __name__ == '__main__':
    main()
