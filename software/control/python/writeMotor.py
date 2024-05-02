import keyboard
import sys
from maru import osx001Driver
from serial.tools import list_ports

# For macOS user, if you get an error with key assignment, enable the commented out codes and run the script with sudo.

def main():
    args = sys.argv
    targetId = 14

    # Exit if no serial port and id specified
    if len(args) < 3:
        ports = list_ports.comports()
        devices = [info.device for info in ports]
        print("Specify serial port and robot id\r\n")
        print("e.g., python writeMotor.py COM3 1\r\n")
        print("Port List : \r\n")
        for port in ports:
            print(port)
        sys.exit(0)

    # Initialization
    driver = osx001Driver(args[1])
    targetId = int(args[2])

    rightSpeed = 0
    leftSpeed = 0
    defaultSpeed = 80

    while(True):
        rightSpeed = leftSpeed = 0

        if keyboard.is_pressed("w"):
        # if keyboard.is_pressed(13):  # for macOS user
            rightSpeed = rightSpeed + defaultSpeed
            leftSpeed = leftSpeed + defaultSpeed

        if keyboard.is_pressed("d"):
        # if keyboard.is_pressed(2):  # for macOS user
            rightSpeed = rightSpeed - (defaultSpeed / 2)
            leftSpeed = leftSpeed + (defaultSpeed / 2)

        if keyboard.is_pressed("a"):
        # if keyboard.is_pressed(0):  # for macOS user
            rightSpeed = rightSpeed + (defaultSpeed / 2)
            leftSpeed = leftSpeed - (defaultSpeed / 2)

        if keyboard.is_pressed("s"):
        # if keyboard.is_pressed(1):  # for macOS user
            rightSpeed = rightSpeed - defaultSpeed
            leftSpeed = leftSpeed - defaultSpeed

        driver.writeMotorSpeed(targetId, rightSpeed, leftSpeed)
        driver.checkStatusPacket()

if __name__ == '__main__':
    main()
