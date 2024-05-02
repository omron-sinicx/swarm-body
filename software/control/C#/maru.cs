using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace MARUDriver {
    /*
     * Parameters for robot control
     * deltaRelay   : Radius for determining the end of a relay point
     * deltaEnd     : Radius for determining the end of a destination point
     * deltaTheta   : Angle range for determining the end of direction specification
     *
     * speedPGain   : Distance Proportional (P) Gain
     * speedDGain   : Distance Derivative (D) Gain
     *
     * theta1PGain  : Angle Proportional (P) Gain 1
     * theta1DGain  : Angle Derivative (D) Gain 2
     *
     * theta1PGain  : Angle Proportional (P) Gain 1
     * theta1DGain  : Angle Derivative (D) Gain 2
     *
     * minPWM       : Minimum speed value during target point tracking (0 to 200)
     */
    public class Parameter {
        private int deltaRelay_ = 50;
        private int deltaEnd_ = 30;
        private int deltaTheta_ = 20;

        private double speedPGain_ = 0.5;
        private double speedDGain_ = 0;

        private double theta1PGain_ = 1.4;
        private double theta1DGain_ = 0.01;

        private double theta2PGain_ = 1.4;
        private double theta2DGain_ = 0.1;

        private int minPWM_ = 140;

        public Parameter (int deltaRelay = 50,
            int deltaEnd = 30,
            int deltaTheta = 20,
            double speedPGain = 0.5,
            double speedDGain = 0.0,
            double theta1PGain = 1.4,
            double theta1DGain = 0.01,
            double theta2PGain = 1.4,
            double theta2DGain = 0.1,
            int minPWM = 140) {
            deltaRelay_ = deltaRelay;
            deltaEnd_ = deltaEnd;
            deltaTheta_ = deltaTheta;
            speedPGain_ = speedPGain;
            speedDGain_ = speedDGain;
            theta1PGain_ = theta1PGain;
            theta1DGain_ = theta1DGain;
            theta2PGain_ = theta2PGain;
            theta2DGain_ = theta2DGain;
            minPWM_ = minPWM;
        }

        public int deltaRelay { get { return deltaRelay_; } set { deltaRelay_ = value; } }
        public int deltaEnd { get { return deltaEnd_; } set { deltaEnd_ = value; } }
        public int deltaTheta { get { return deltaTheta_; } set { deltaTheta_ = value; } }

        public double speedPGain { get { return speedPGain_; } set { speedPGain_ = value; } }
        public double speedDGain { get { return speedDGain_; } set { speedDGain_ = value; } }

        public double theta1PGain { get { return theta1PGain_; } set { theta1PGain_ = value; } }
        public double theta1DGain { get { return theta1DGain_; } set { theta1DGain_ = value; } }

        public double theta2PGain { get { return theta2PGain_; } set { theta2PGain_ = value; } }
        public double theta2DGain { get { return theta2DGain_; } set { theta2DGain_ = value; } }

        public int minPWM { get { return minPWM_; } set { minPWM_ = value; } }
    }

    /*
     * Information on coordinates and rotation angles obtainable from projector projection
     */
    public class Position {
        private int x_;
        private int y_;
        private int orientation_;

        public Position (int x, int y, int orientation) {
            this.x_ = x;
            this.y_ = y;
            this.orientation_ = orientation;
        }

        public int x { get { return x_; } set { x_ = value; } }
        public int y { get { return y_; } set { y_ = value; } }
        public int orientation { get { return orientation_; } set { orientation_ = value; } }
    }

    /*
     * Information on Yaw, Pitch, and Roll obtainable from an IMU
     */
    public class YawPitchRoll {
        private readonly double yaw_;
        private readonly double pitch_;
        private readonly double roll_;

        public YawPitchRoll (double yaw, double pitch, double roll) {
            yaw_ = yaw;
            pitch_ = pitch;
            roll_ = roll;
        }

        public double yaw { get { return yaw_; } }
        public double pitch { get { return pitch_; } }
        public double roll { get { return roll_; } }
    }

    /*
     * Arguments of the Status Packet Reception Event
     */
    public class MARUEventArgs : EventArgs {
        private readonly byte id_;
        private readonly bool chargeStatus_;
        private readonly double voltage_;
        private readonly Position position_;
        private readonly YawPitchRoll ypr_;

        public MARUEventArgs (byte id, bool chargeStatus, double voltage, Position position, YawPitchRoll ypr) {
            id_ = id;
            chargeStatus_ = chargeStatus;
            voltage_ = voltage;
            position_ = position;
            ypr_ = ypr;
        }

        public byte id { get { return id_; } }
        public bool chargeStatus { get { return chargeStatus_; } }
        public double voltage { get { return voltage_; } }
        public Position position { get { return position_; } }
        public YawPitchRoll ypr { get { return ypr_; } }
    }

    public class MARUDriver {
        private System.IO.Ports.SerialPort serialPort = new System.IO.Ports.SerialPort ();

        // For thread operations
        private CancellationTokenSource tokenSource = new CancellationTokenSource ();
        internal CancellationToken token;

        // Timer for timeout monitoring
        internal System.Timers.Timer timeoutTimer;
        bool timeoutFlag = false;

        // Callback Event Handler
        public delegate void StatusEventHandler (MARUEventArgs e);
        public delegate void IDEventHandler (List<int> idList);

        public event StatusEventHandler StatusEvent;
        public event IDEventHandler GetIDEvent;

        /*
         * Command content
         */
        private enum Command {
            bootRobot = 0x01,
            rebootBoard = 0x02,
            writeMotorSpeed = 0x03,
            resetIMU = 0x04,
            shutdown = 0x05,
            flushBuf = 0x06,
            setTargetPosition = 0x07,
            setTargetPositionWithOrientation = 0x08,

            setTargetId = 0x10,
            getTargetId = 0x11,

            calibrate = 0x3a,
            setParameter = 0x3d,

            selectTarget = 0xF6,
            writeFirmware = 0xFC,
            writeFirmwareEnd = 0xFD,

            status = 0x21,
            getTargetIDResponse = 0x28,
        }

        public MARUDriver () { }
        ~MARUDriver () { close (); }

        /*
         * Serial port initialization
         * COMPort: Serial port name (e.g., "COM3")
         */
        public void open (string COMPort) {
            serialPort.PortName = COMPort;
            serialPort.BaudRate = 1000000;
            serialPort.ReadTimeout = 100;
            serialPort.WriteBufferSize = 4096;
            serialPort.WriteTimeout = 100;
            serialPort.Open ();

            // Flush Buffer
            flushBuffer ();

            // Set a timer for timeout
            timeoutTimer = new System.Timers.Timer ();
            timeoutTimer.Interval = 200;
            timeoutTimer.Elapsed += async (s, e) => {
                await Task.Run (() => {
                    timeoutFlag = true;
                });
            };

            tokenSource = new CancellationTokenSource ();
            token = tokenSource.Token;
            Task.Run (() => { DataReceivedHandler (token); });
        }

        public void close () {
            flushBuffer ();
            tokenSource.Cancel ();
            if (serialPort.IsOpen) serialPort.Close ();
        }

        /*
         * Checksum calculation
         */
        private byte calculateCheckSum (byte[] data) {
            int sum = 0;
            for (int i = 0; i < data.Length - 1; i++) {
                sum += data[i];
            }

            return (byte) (sum & 0xFF);
        }

        byte calculateCheckSum (byte[] data, int length) {
            int sum = 0;
            for (int i = 0; i < length - 1; i++) {
                sum += data[i];
            }

            return (byte) (sum & 0xFF);
        }

        /*
         * Data transmission function
         */
        private void WriteSerial (byte[] data) {
            serialPort.Write (data, 0, data.Length);
        }

        /*
         * Starting the robot docked in the cradle
         */
        public void bootRobot () {
            byte[] data = new byte[6] { 0xFF, 0xFD, 0x02, 0x00, (byte) Command.bootRobot, 0x00 };
            data[5] = calculateCheckSum (data);
            WriteSerial (data);
        }

        /* Buffer clear */
        public void flushBuffer () {
            byte[] data = new byte[6] { 0xFF, 0xFD, 0x02, 0x00, (byte) Command.flushBuf, 0x00 };
            data[5] = calculateCheckSum (data);
            WriteSerial (data);
        }

        /*
         * Raw motor command instruction
         * id    : Target ID 1 to 15
         * right : Right side PWM Duty ratio 0 to 100%
         * left  : Left side PWM Duty ratio 0 to 100%
         */
        public void writeMotorSpeed (byte id, int right, int left) {
            byte rightMotor = (byte) (right);
            byte leftMotor = (byte) (left);

            byte[] data = new byte[9] { 0xFF, 0xFD, 0x05, 0x00, (byte) Command.writeMotorSpeed, id, rightMotor, leftMotor, 0x00 };
            data[8] = calculateCheckSum (data);
            WriteSerial (data);
        }

        /*
         * Reset the IMU's initial posture to the current posture
         * id : Target ID 1 to 15
         */
        public void resetIMU (byte id) {
            byte[] data = new byte[7] { 0xFF, 0xFD, 0x03, 0x00, (byte) Command.resetIMU, id, 0x00 };
            data[6] = calculateCheckSum (data);
            WriteSerial (data);
        }

        /*
         * Shutdown of the connected robot
         */
        public void shutdown () {
            byte[] data = new byte[6] { 0xFF, 0xFD, 0x02, 0x00, (byte) Command.shutdown, 0x00 };
            data[5] = calculateCheckSum (data);
            WriteSerial (data);
        }

        /*
         * Specifying the target coordinates for the robot (without direction specification)
         * id    : Target ID 1 to 15
         * x     : -456 to 456, stops if out of range
         * y     : -285 to 285, stops if out of range
         *
         * endFlag
         *   false : Relay point. Does not start moving until the end point is sent
         *   true  : End point
         *
         * flush
         *   false : Queue the target points
         *   true  : Clear the queue and set the target point sent this time as the latest target point
         *
         */
        public void setTargetPoint (byte id, int x, int y, bool endFlag = true, bool flush = true) {
            UInt16 tmpx = (UInt16) ((Int16) (x));
            UInt16 tmpy = (UInt16) ((Int16) (y));

            byte[] data = new byte[12] {
                0xFF,
                0xFD,
                0x08,
                0x00,
                (byte) Command.setTargetPosition,
                id,
                (byte) ((tmpx) & 0xFF),
                (byte) ((tmpx >> 8) & 0xFF),
                (byte) ((tmpy) & 0xFF),
                (byte) ((tmpy >> 8) & 0xFF),
                0b00,
                0x00
            };
            data[10] = (byte) ((endFlag ? 1 : 0) + (flush ? 0b10 : 0));
            data[11] = calculateCheckSum (data);

            WriteSerial (data);
        }

        /*
         * Specifying the robot's target coordinates (with direction specification)
         * id    : Target ID 1 to 15
         * x     : -456 to 456, stops if out of range
         * y     : -285 to 285, stops if out of range
         * orientation : -180 to 180 direction
         *
         * endFlag
         *   false : Relay point. Does not start moving until the end point is sent
         *   true  : End point
         *
         * flush
         *   false : Queue the target points
         *   true  : Clear the queue and set the target point sent this time as the latest target point
         */
        public void setTargetPointWithOrientation (byte id, int x, int y, int orientation, bool endFlag = false, bool flush = false) {
            UInt16 tmpx = (UInt16) ((Int16) (x));
            UInt16 tmpy = (UInt16) ((Int16) (y));
            UInt16 tmpo = (UInt16) ((Int16) (orientation));

            byte[] data = new byte[14] {
                0xFF,
                0xFD,
                0x0A,
                0x00,
                (byte) Command.setTargetPositionWithOrientation,
                id,
                (byte) ((tmpx) & 0xFF),
                (byte) ((tmpx >> 8) & 0xFF),
                (byte) ((tmpy) & 0xFF),
                (byte) ((tmpy >> 8) & 0xFF),
                (byte) ((tmpo) & 0xFF),
                (byte) ((tmpo >> 8) & 0xFF),
                0b00,
                0x00
            };
            data[12] = (byte) ((endFlag ? 1 : 0) + (flush ? 0b10 : 0));
            data[13] = calculateCheckSum (data);

            WriteSerial (data);
        }

        /*
         * Setting the ID of the communication target
         * idList  : List of target IDs
         */
        public void setTargetID (List<int> idList) {
            if (idList.Count == 0) return;

            byte[] data = new byte[6 + idList.Count];
            data[0] = 0xFF;
            data[1] = 0xFD;
            data[2] = (byte) (0x02 + idList.Count);
            data[3] = 0x00;
            data[4] = (byte) (Command.setTargetId);
            for (int i = 0; i < idList.Count; i++) {
                data[5 + i] = (byte) idList[i];
            }
            data[5 + idList.Count] = calculateCheckSum (data);
            WriteSerial (data);
        }

        /*
         * Retrieving the ID of the communication target
         */
        public void getTargetID () {
            byte[] data = new byte[6] { 0xFF, 0xFD, 0x02, 0x00, (byte) Command.getTargetId, 0x00 };
            data[5] = calculateCheckSum (data);
            WriteSerial (data);
        }

        /*
         * Starting the motor calibration
         * id    : Target ID 1 to 15
         */
        public void calibrateStart (byte id) {
            byte[] data = new byte[7] { 0xFF, 0xFD, 0x03, 0x00, (byte) Command.calibrate, id, 0x00 };
            data[6] = calculateCheckSum (data);
            WriteSerial (data);
        }

        /*
         * Setting parameters
         */
        public void setParameter (byte id, Parameter parameter) {
            int[] intParam = new int[10] {
                (int) parameter.deltaRelay,
                (int) parameter.deltaEnd,
                (int) parameter.deltaTheta,
                (int) (parameter.speedPGain * 1000.0),
                (int) (parameter.speedDGain * 1000.0),

                (int) (parameter.theta1PGain * 1000.0),
                (int) (parameter.theta1DGain * 1000.0),
                (int) (parameter.theta2PGain * 1000.0),
                (int) (parameter.theta2DGain * 1000.0),
                (int) parameter.minPWM
            };

            byte[] data = new byte[26] {
                0xFF,
                0xFD,
                22,
                0x00,
                (byte) Command.setParameter,
                id,
                (byte) ((int) intParam[0] & 0xFF),
                (byte) (intParam[0] >> 8),
                (byte) ((int) intParam[1] & 0xFF),
                (byte) (intParam[1] >> 8),
                (byte) ((int) intParam[2] & 0xFF),
                (byte) (intParam[2] >> 8),
                (byte) ((int) intParam[3] & 0xFF),
                (byte) (intParam[3] >> 8),
                (byte) ((int) intParam[4] & 0xFF),
                (byte) (intParam[4] >> 8),
                (byte) ((int) intParam[5] & 0xFF),
                (byte) (intParam[5] >> 8),
                (byte) ((int) intParam[6] & 0xFF),
                (byte) (intParam[6] >> 8),
                (byte) ((int) intParam[7] & 0xFF),
                (byte) (intParam[7] >> 8),
                (byte) ((int) intParam[8] & 0xFF),
                (byte) (intParam[8] >> 8),
                (byte) ((int) intParam[9] & 0xFF),
                0x00
            };
            data[25] = calculateCheckSum (data);
            WriteSerial (data);
        }

        /* Reception processing: To avoid dependency on the environment, do not use DataReceivedEvent */
        int rPos = 0, rStat = 0, rCommand = 0, rLength = 0;
        byte[] rData = new byte[200];

        private async void DataReceivedHandler (CancellationToken cancelToken) {
            while (true) {
                System.IO.Ports.SerialPort sp = serialPort;
                byte buf;
                byte[] rcvData = new byte[2000];
                int len = 0, pos = 0, tmpId;

                if (cancelToken.IsCancellationRequested) {
                    return;
                }

                if (!sp.IsOpen) continue;

                await Task.Run (() => {
                    try {
                        len = sp.Read (rcvData, 0, 2000);
                    } catch (Exception ex) { /* Error handling */ }
                });

                while (pos < len) {
                    // Checking for timeout
                    if (timeoutFlag) {
                        timeoutFlag = false;
                        rStat = 0;
                    }

                    // Data acquisition
                    buf = rcvData[pos++];
                    if (rStat == 0) rPos = 0;
                    rData[rPos++] = buf;

                    // Command processing
                    switch (rStat) {
                        case 0: // Header 0xFF
                            rStat = (buf == 0xFF) ? (rStat + 1) : 0;
                            break;

                        case 1: // Header 0xFD
                            rStat = (buf == 0xFD) ? (rStat + 1) : 0;
                            break;

                        case 2: // Length_L
                            rLength = buf;
                            rStat++;
                            break;
                        case 3: // Length_H
                            rLength += (buf << 8);
                            rStat++;
                            break;

                        case 4: // Instruction
                            rCommand = buf;
                            if (rLength == 2) rStat += 2;
                            else rStat++;
                            break;

                        case 5: // Param
                            if (rPos == rLength + 3) rStat++;
                            break;

                        case 6: // CheckSum
                            rStat = 0;
                            if (buf != calculateCheckSum (rData, rPos)) break;

                            switch (rCommand) {
                                case (byte) Command.status:
                                    MARUEventArgs arg = new MARUEventArgs (
                                        (byte) (rData[5] & 0b01111111), ((rData[5] & 0b10000000) != 0),
                                        ((Int16) (rData[12] + (rData[13] << 8))) / 1000.0,
                                        new Position (
                                            (Int16) (rData[6] + (rData[7] << 8)),
                                            (Int16) (rData[8] + (rData[9] << 8)),
                                            (Int16) (rData[10] + (rData[11] << 8))),
                                        new YawPitchRoll (
                                            ((Int16) (rData[14] + (rData[15] << 8))) / 100.0,
                                            ((Int16) (rData[16] + (rData[17] << 8))) / 100.0,
                                            ((Int16) (rData[18] + (rData[19] << 8))) / 100.0)
                                    );

                                    if (StatusEvent != null) StatusEvent (arg);
                                    break;

                                case (byte) Command.getTargetIDResponse:
                                    List<int> idList = new List<int> ();
                                    if (rLength > 2 && rLength <= 12) {
                                        for (int i = 0; i < rLength - 2; i++) {
                                            if (rData[5 + i] > 0 && rData[5 + i] <= 16) {
                                                idList.Add (rData[5 + i]);
                                            }
                                        }
                                    }
                                    if (idList.Count > 0 && GetIDEvent != null) GetIDEvent (idList);
                                    break;
                            }
                            break;
                    }
                }
            }
        }
    }
}
