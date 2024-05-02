import sys
import atexit
from maru import osx001Driver
from serial.tools import list_ports


def flush(driver):
    driver.flushBuffer()
    print("flush!!")

def main():
    args = sys.argv
    targetId = 11

    # Exit if no serial port, id and positions specified
    if len(args) < 5:
        ports = list_ports.comports()
        devices = [info.device for info in ports]
        print("Specify serial port, robot id, and target position (x and y).\r\n")
        print("e.g., python setTargetPosition.py COM3 1 100 100\r\n")
        print("Port List : \r\n")
        for port in ports:
            print(port)
        sys.exit(0)

    # Initialization
    driver = osx001Driver(args[1])
    targetId = int(args[2])
    x = int(args[3])
    y = int(args[4])
    driver.setTargetPosition(targetId, x, y)

    atexit.register(flush, driver)

    while(True):
        driver.checkStatusPacket()

if __name__ == '__main__':
    main()
