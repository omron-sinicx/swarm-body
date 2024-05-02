import sys
from maru import osx001Driver
from serial.tools import list_ports

def main():
    args = sys.argv
    targetId = 11

    # Exit if no serial port and id specified
    if len(args) < 3:
        ports = list_ports.comports()
        devices = [info.device for info in ports]
        print("Specify serial port and robot id\r\n")
        print("e.g., python resetIMU.py COM3 1\r\n")
        print("Port List : \r\n")
        for port in ports:
            print(port)
        sys.exit(0)

    # initialization
    driver = osx001Driver(args[1])
    targetId = int(args[2])

    driver.resetIMU(targetId)

if __name__ == '__main__':
    main()
