**
 * Example HERO application can reads a serial port and echos the bytes back.
 * After deploying this application, the user can open a serial terminal and type while the HERO echoes the typed keys back.
 * Use a USB to UART (TTL) cable like the Adafruit Raspberry PI or FTDI-TTL cable.
 * Use device manager to figure out which serial port number to select in your PC terminal program.
 * HERO Gadgeteer Port 1 is used in this example, but can be changed at the top of Main().
 */
using System;
using System.Threading;
using Microsoft.SPOT;

namespace HERO_Serial_Example
{
    public class Program
    {

        /** Serial object, this is constructed on the serial number. */
        static System.IO.Ports.SerialPort _uart;
        /** Ring buffer holding the bytes to transmit. */
        static byte[] _tx = new byte[1024];
        static int _txIn = 0;
        static int _txOut = 0;
        static int _txCnt = 0;
        private static bool localDrive = true;

        static CTRE.Phoenix.Controller.GameController _gamepad = null;

        /** Cache for reading out bytes in serial driver. */
        static byte[] _rx = new byte[1024];
        /* initial message to send to the terminal */
        static byte[] _helloMsg = MakeByteArrayFromString("Hero-Ready\r\n");
        /** @return the maximum number of bytes we can read*/
        private static int CalcRemainingCap()
        {
            /* firs calc the remaining capacity in the ring buffer */
            int rem = _tx.Length - _txCnt;
            /* cap the return to the maximum capacity of the rx array */
            if (rem > _rx.Length)
                rem = _rx.Length;
            return rem;
        }
        /** @param received byte to push into ring buffer */
        private static void PushByte(byte datum)
        {
            _tx[_txIn] = datum;
            if (++_txIn >= _tx.Length)
                _txIn = 0;
            ++_txCnt;
        }
        /** 
         * Pop the oldest byte out of the ring buffer.
         * Caller must ensure there is at least one byte to pop out by checking _txCnt.
         * @return the oldest byte in buffer.
         */
        private static byte PopByte()
        {
            byte retval = _tx[_txOut];
            if (++_txOut >= _tx.Length)
                _txOut = 0;
            --_txCnt;
            return retval;
        }

        public static double[] parseData(String str)
        {

            int startIndex = str.IndexOf('<');
            int comma1Index = str.IndexOf(',', startIndex + 1);
            int comma2Index = str.IndexOf(',', comma1Index + 1);
            int comma3Index = str.IndexOf(',', comma2Index + 1);
            int comma4Index = str.IndexOf(',', comma3Index + 1);
            int comma5Index = str.IndexOf(',', comma4Index + 1);
            int endIndex = str.IndexOf('>', comma5Index + 1);

            double[] data = new double[6];
            int secondStartIndex = str.IndexOf('<', startIndex + 1);
            if (comma1Index == -1 || comma2Index == -1 || comma3Index == -1 || comma4Index == -1 || comma5Index == -1 || secondStartIndex > endIndex)
            {
                //Debug.Print(comma1Index.ToString()+ comma2Index.ToString()+ comma3Index.ToString()+ comma4Index.ToString()+ comma5Index.ToString());
                data[2] = -2;
                data[3] = -1;
                data[4] = -1;
                data[5] = -1;
                return data;
            }

            data[0] = Convert.ToDouble(str.Substring(startIndex + 1, comma1Index - startIndex - 1));
            data[1] = Convert.ToDouble(str.Substring(comma1Index + 1, comma2Index - comma1Index - 1));
            data[2] = Convert.ToDouble(str.Substring(comma2Index + 1, comma3Index - comma2Index - 1));
            data[3] = Convert.ToDouble(str.Substring(comma3Index + 1, comma4Index - comma3Index - 1));
            data[4] = Convert.ToDouble(str.Substring(comma4Index + 1, comma5Index - comma4Index - 1));
            data[5] = Convert.ToDouble(str.Substring(comma5Index + 1, endIndex - comma5Index - 1));
            Debug.Print("Data" + data[0].ToString() + data[1].ToString() + data[2].ToString() + data[3].ToString() + data[4].ToString() + data[5].ToString());

            return data;
        }
        /** entry point of the application */
        public static void Main()
        {
            double scaleTurn = 1.0f;
            int loops = 0;
            int turnCap = 15;
            CTRE.Phoenix.MotorControl.CAN.TalonSRX lf = new CTRE.Phoenix.MotorControl.CAN.TalonSRX(1);
            //CTRE.Phoenix.MotorControl.CAN.TalonSRX lm = new CTRE.Phoenix.MotorControl.CAN.TalonSRX(2);
            CTRE.Phoenix.MotorControl.CAN.TalonSRX lb = new CTRE.Phoenix.MotorControl.CAN.TalonSRX(3);
            CTRE.Phoenix.MotorControl.CAN.TalonSRX rf = new CTRE.Phoenix.MotorControl.CAN.TalonSRX(4);
            //CTRE.Phoenix.MotorControl.CAN.TalonSRX rm = new CTRE.Phoenix.MotorControl.CAN.TalonSRX(5);
            CTRE.Phoenix.MotorControl.CAN.TalonSRX rb = new CTRE.Phoenix.MotorControl.CAN.TalonSRX(6);

            // Configuring factory default
            lf.ConfigFactoryDefault();
            //lm.ConfigFactoryDefault();
            lb.ConfigFactoryDefault();
            rf.ConfigFactoryDefault();
            //rm.ConfigFactoryDefault();
            rb.ConfigFactoryDefault();

            // Configuring Deadband
            float deadband = .01f;
            lf.ConfigNeutralDeadband(deadband);
            //lm.ConfigNeutralDeadband(deadband);
            lb.ConfigNeutralDeadband(deadband);
            rf.ConfigNeutralDeadband(deadband);
           // rm.ConfigNeutralDeadband(deadband);
            rb.ConfigNeutralDeadband(deadband);

            lf.SetNeutralMode(CTRE.Phoenix.MotorControl.NeutralMode.Brake);
            //lm.SetNeutralMode(CTRE.Phoenix.MotorControl.NeutralMode.Brake);
            lb.SetNeutralMode(CTRE.Phoenix.MotorControl.NeutralMode.Brake);
            rf.SetNeutralMode(CTRE.Phoenix.MotorControl.NeutralMode.Brake);
            //rm.SetNeutralMode(CTRE.Phoenix.MotorControl.NeutralMode.Brake);
            rb.SetNeutralMode(CTRE.Phoenix.MotorControl.NeutralMode.Brake);


            // Configuring the Encoders
            //Debug.Print(lf.ConfigSelectedFeedbackSensor((CTRE.Phoenix.MotorControl.FeedbackDevice)CTRE.Phoenix.MotorControl.TalonSRXFeedbackDevice.QuadEncoder, 0, 50).ToString());
            lf.ConfigSelectedFeedbackSensor((CTRE.Phoenix.MotorControl.FeedbackDevice)CTRE.Phoenix.MotorControl.TalonSRXFeedbackDevice.QuadEncoder, 0, 0);
            //lm.ConfigSelectedFeedbackSensor((CTRE.Phoenix.MotorControl.FeedbackDevice)CTRE.Phoenix.MotorControl.TalonSRXFeedbackDevice.QuadEncoder, 0, 0);
            lb.ConfigSelectedFeedbackSensor((CTRE.Phoenix.MotorControl.FeedbackDevice)CTRE.Phoenix.MotorControl.TalonSRXFeedbackDevice.QuadEncoder, 0, 0);
            rf.ConfigSelectedFeedbackSensor((CTRE.Phoenix.MotorControl.FeedbackDevice)CTRE.Phoenix.MotorControl.TalonSRXFeedbackDevice.QuadEncoder, 0, 0);
            //rm.ConfigSelectedFeedbackSensor((CTRE.Phoenix.MotorControl.FeedbackDevice)CTRE.Phoenix.MotorControl.TalonSRXFeedbackDevice.QuadEncoder, 0, 0);
            rb.ConfigSelectedFeedbackSensor((CTRE.Phoenix.MotorControl.FeedbackDevice)CTRE.Phoenix.MotorControl.TalonSRXFeedbackDevice.QuadEncoder, 0, 0);

            lf.SelectProfileSlot(0, 0);
            //lm.SelectProfileSlot(0, 0);
            lb.SelectProfileSlot(0, 0);
            rf.SelectProfileSlot(0, 0);
            //rm.SelectProfileSlot(0, 0);
            rb.SelectProfileSlot(0, 0);

            // Set one side to inverted 

            // Set sensor phase allows you to set encoders to match inverted
            rf.SetInverted(true);
            rf.SetSensorPhase(true);

            //rm.SetSensorPhase(false);
            rb.SetInverted(false);
            rb.SetSensorPhase(false);
            //rm.SetInverted(true);
            //rb.SetInverted(true);
            //lf.SetSensorPhase(false);
            //lm.SetSensorPhase(false);
            lb.SetSensorPhase(true);
      
            

            lf.SetInverted(false);
            lf.SetSensorPhase(true);
            //lm.SetInverted(true);

            // information that you get from the controller
            double[] data = new double[6];
            double[] speeds = new double[6];
            // could map to other things
            data[3] = -1;
            data[4] = -1;
            data[5] = -1;
            // General approximation of if you put in a value of 10 how many signals does that send before a meter pases
            double pulsesPerMeter = 22000;

            // Tuned values for pid
            float pValue = 0;
            float iValue = 0.0001f;
            float dValue = 0;

            //Set new PID Values for each motor
            lf.Config_kP(0,-pValue, 0);
            lf.Config_kI(0, iValue, 0);
            lf.Config_kD(0, dValue, 0);

            //lm.Config_kP(0, pValue, 0);
            //lm.Config_kI(0, iValue, 0);
            //lm.Config_kD(0, dValue, 0);

            lb.Config_kP(0, pValue, 0);
            lb.Config_kI(0, iValue, 0);
            lb.Config_kD(0, dValue, 0);

            rf.Config_kP(0, pValue, 0);
            rf.Config_kI(0, iValue, 0);
            rf.Config_kD(0, dValue, 0);

            //rm.Config_kP(0, pValue, 0);
            //rm.Config_kI(0, iValue, 0);
            //rm.Config_kD(0, dValue, 0);

            rb.Config_kP(0, pValue, 0);
            rb.Config_kI(0, iValue, 0);
            rb.Config_kD(0, dValue, 0);

            /* temporary array */
            byte[] scratch = new byte[1];
            /* open the UART, select the com port based on the desired gadgeteer port.
             *   This utilizes the CTRE.IO Library.
             *   The full listing of COM ports on HERO can be viewed in CTRE.IO
             *   Uart connection is from audino
             *   
             */
            _uart = new System.IO.Ports.SerialPort(CTRE.HERO.IO.Port1.UART, 115200);
            _uart.Open();
            /* send a message to the terminal for the user to see */
            _uart.Write(_helloMsg, 0, _helloMsg.Length);
            /* loop forever */
            String str = "";
            char[] msg = new char[25];
            while (true)
            {
                loops++;
                //Debug.Print(loops.ToString());
                // Messages start with a Carrot < and have 6 comma seperated variable and end with >
                // Example: < 1, 2, 3, 4, 5, 6>
                int start = str.IndexOf('<');
                int secondstart = str.IndexOf('<', start + 1);
                // if you cant find a message
                if (start == -1)
                {
                    str = "";
                }
                else
                {
                    // taking the values
                    str = str.Substring(System.Math.Max(start, secondstart));
                }
                int end = str.IndexOf('>');
                /* read bytes out of uart */
                if (_uart.BytesToRead > 0)
                {

                    int readCnt = _uart.Read(_rx, 0, CalcRemainingCap());
                    for (int i = 0; i < readCnt; ++i)
                    {
                        PushByte(_rx[i]);
                        if (_rx[i] != '\n')
                        {
                            str = str + (((char)_rx[i]));
                        }
                    }
                    Debug.Print(str);
                }
                /* if there are bufferd bytes echo them back out */
                // You can get more messages at a time moving pass weird chars//
                if (_uart.CanWrite && (_txCnt > 0))
                {
                    scratch[0] = PopByte();
                    _uart.Write(scratch, 0, 1);

                }
                // if the end was found >
                if (end != -1 || localDrive)
                {

                    data = parseData(str);
                    if (data[2] != -1 && false) //P value
                    {
                        pValue = (float)data[2] / 100f;
                    }
                    if (data[3] != -1 && false) //I Value
                    {
                        iValue = (float)data[3] / 100f;
                    }
                    if (data[4] != -1 && false) //D Value
                    {
                        turnCap = (int)data[4];
                    }
                    if (data[5] != -1 && false) //PulsesPerMeter
                    {
                        pulsesPerMeter = data[5];
                    }

                    //Set new PID Values for each motor
                    lf.Config_kP(0, pValue, 0);
                    lf.Config_kI(0, iValue, 0);
                    lf.Config_kD(0, dValue, 0);

                    //lm.Config_kP(0, pValue, 0);
                    //lm.Config_kI(0, iValue, 0);
                    //lm.Config_kD(0, dValue, 0);

                    lb.Config_kP(0, pValue, 0);
                    lb.Config_kI(0, iValue, 0);
                    lb.Config_kD(0, dValue, 0);

                    rf.Config_kP(0, pValue, 0);
                    rf.Config_kI(0, iValue, 0);
                    rf.Config_kD(0, dValue, 0);

                    //rm.Config_kP(0, pValue, 0);
                    //rm.Config_kI(0, iValue, 0);
                    //rm.Config_kD(0, dValue, 0);

                    rb.Config_kP(0, pValue, 0);
                    rb.Config_kI(0, iValue, 0);
                    rb.Config_kD(0, dValue, 0);
                    
                    if (data[2] != -2)
                    {
                        localDrive = false;
                    }
                    if (data[2] == -3.0)
                    {
                        localDrive = true;
                    }
                    if(localDrive && _gamepad == null)
                    {
                        Debug.Print("hmmm");
                        _gamepad = new CTRE.Phoenix.Controller.GameController(CTRE.Phoenix.UsbHostDevice.GetInstance());
                    }
                    if (_gamepad == null)
                    {
                        Debug.Print("yup");
                    }
                    if (data[2] != -2 || localDrive)
                    {
                        loops = 0;
                        //double mag = System.Math.Sqrt(data[1] * data[1] + data[0] * data[0]);
                        //speeds[0] = data[0] / 200 + data[1] / 200;
                        //speeds[1] = data[0] / 200 + data[1] / 200;
                        //speeds[2] = data[0] / 200 + data[1] / 200;
                        //speeds[3] = data[0] / 200 - data[1] / 200;
                        //speeds[4] = data[0] / 200 - data[1] / 200;
                        //speeds[5] = data[0] / 200 - data[1] / 200;
                        if (localDrive) 
                        {
                            var forward = _gamepad.GetAxis(1);
                            var turn = _gamepad.GetAxis(2);
                            forward = deadZone(forward);
                            turn = deadZone(turn);

                            //Debug.Print(forward.ToString());
                            //Debug.Print(turn.ToString());

                            var localScale = 0.25f;

                            if (forward > 0.1)
                            {
                                data[0] = (forward - 0.1) * -100 * localScale;
                            }
                            else if (forward < -0.1)
                            {
                                data[0] = (forward + 0.1) * -100 * localScale;
                            }
                            if (turn > 0.1)
                            {
                                data[1] = (turn - 0.1) * 100 * localScale;
                            }
                            else if (turn < -0.1)
                            {
                                data[1] = (turn + 0.1) * 100 * localScale;
                            }
                            
                        }
                        // setting motor controllor speed
                        float scaleFactor = 0.3f;
                        speeds[0] = data[0] / 100 * pulsesPerMeter;
                        speeds[1] = data[0] / 100 * pulsesPerMeter;
                        speeds[2] = data[0] / 100 * pulsesPerMeter;
                        speeds[3] = data[0] / 100 * pulsesPerMeter;
                        speeds[4] = data[0] / 100 * pulsesPerMeter;
                        speeds[5] = data[0] / 100 * pulsesPerMeter;
                        //Debug.Print("Data" + speeds[0].ToString() + speeds[1].ToString() + speeds[2].ToString() + speeds[3].ToString() + speeds[4].ToString() + speeds[5].ToString());

                        // Gradual turning 
                        if (data[1] > turnCap)
                        {
                            speeds[3] -= ((data[1] - turnCap) / 100 * data[0] / 100) * pulsesPerMeter;
                            speeds[4] -= ((data[1] - turnCap) / 100 * data[0] / 100) * pulsesPerMeter;
                            speeds[5] -= ((data[1] - turnCap) / 100 * data[0] / 100) * pulsesPerMeter;
                        }
                        if (data[1] < -turnCap)
                        {
                            speeds[0] += ((data[1] + turnCap) / 100 * data[0] / 100) * pulsesPerMeter;
                            speeds[1] += ((data[1] + turnCap) / 100 * data[0] / 100) * pulsesPerMeter;
                            speeds[2] += ((data[1] + turnCap) / 100 * data[0] / 100) * pulsesPerMeter;
                        }
                        // Just turn
                        if (data[0] == 0)
                        {
                            speeds[0] = data[1] / 100 * scaleTurn * pulsesPerMeter;
                            speeds[1] = data[1] / 100 * scaleTurn * pulsesPerMeter;
                            speeds[2] = data[1] / 100 * scaleTurn * pulsesPerMeter;
                            speeds[3] = data[1] / 100 * -scaleTurn * pulsesPerMeter;
                            speeds[4] = data[1] / 100 * -scaleTurn * pulsesPerMeter;
                            speeds[5] = data[1] / 100 * -scaleTurn * pulsesPerMeter;
                        }
                    }
                    if(!localDrive)
                    {
                        //str = str.Substring(end);
                    }
                    
                }
                //Debug.Print(rf.GetSelectedSensorVelocity().ToString()+"      "+speeds[3].ToString());
                if (loops > 350)
                {
                    speeds[0] = 0;
                    speeds[1] = 0;
                    speeds[2] = 0;
                    speeds[3] = 0;
                    speeds[4] = 0;
                    speeds[5] = 0;

                    lf.Set(CTRE.Phoenix.MotorControl.ControlMode.PercentOutput, 0);
                    //lm.Set(CTRE.Phoenix.MotorControl.ControlMode.PercentOutput, 0);
                    lb.Set(CTRE.Phoenix.MotorControl.ControlMode.PercentOutput, 0);
                    rf.Set(CTRE.Phoenix.MotorControl.ControlMode.PercentOutput, 0);
                    //rm.Set(CTRE.Phoenix.MotorControl.ControlMode.PercentOutput, 0);
                    rb.Set(CTRE.Phoenix.MotorControl.ControlMode.PercentOutput, 0);
                    CTRE.Phoenix.Watchdog.Feed();
                }

                //Debug.Print(loops.ToString());

                if(speeds[0] == 0)
                {
                    lf.Set(CTRE.Phoenix.MotorControl.ControlMode.PercentOutput, speeds[0] / pulsesPerMeter);
                }
                else
                {
                    lf.Set(CTRE.Phoenix.MotorControl.ControlMode.Velocity, speeds[0]);
                }
                Debug.Print("Left front Motor: " + lf.GetSelectedSensorVelocity().ToString());
                //if (speeds[1] == 0)
                //{
                    //lm.Set(CTRE.Phoenix.MotorControl.ControlMode.PercentOutput, speeds[1] / pulsesPerMeter);
                //}
                //else
                //{
                    //lm.Set(CTRE.Phoenix.MotorControl.ControlMode.Velocity, speeds[1]);
                //}
                //Debug.Print("Left Middle Motor: " + lm.GetSelectedSensorVelocity().ToString());
                if (speeds[2] == 0)
                {
                    lb.Set(CTRE.Phoenix.MotorControl.ControlMode.PercentOutput, speeds[2] / pulsesPerMeter);
                }
                else
                {
                    lb.Set(CTRE.Phoenix.MotorControl.ControlMode.Velocity, speeds[2]);
                }
                Debug.Print("Left Back Motor: " + lb.GetSelectedSensorVelocity().ToString());
                if (speeds[3] == 0)
                {
                    rf.Set(CTRE.Phoenix.MotorControl.ControlMode.PercentOutput, speeds[3] / pulsesPerMeter);
                }
                else
                {
                    rf.Set(CTRE.Phoenix.MotorControl.ControlMode.Velocity, speeds[3]);
                }
                Debug.Print("Right Front Motor: " + rf.GetSelectedSensorVelocity().ToString());
                //if (speeds[4] == 0)
                //{
                //    rm.Set(CTRE.Phoenix.MotorControl.ControlMode.PercentOutput, speeds[4] / pulsesPerMeter);
                //}
                //else
                //{
                //    rm.Set(CTRE.Phoenix.MotorControl.ControlMode.Velocity, speeds[4]);
                //}
                //Debug.Print("Right Middle Motor: " + rm.GetSelectedSensorVelocity().ToString());
                if (speeds[5] == 0)
                {
                    rb.Set(CTRE.Phoenix.MotorControl.ControlMode.PercentOutput, speeds[5] / pulsesPerMeter);
                }
                else
                {
                    rb.Set(CTRE.Phoenix.MotorControl.ControlMode.Velocity, speeds[5]);
                }
                Debug.Print("Right Back Motor: " + rb.GetSelectedSensorVelocity().ToString());
                CTRE.Phoenix.Watchdog.Feed();

                /* wait a bit, keep the main loop time constant, this way you can add to this example (motor control for example). */

            }
        }
        /**
         * Helper routine for creating byte arrays from strings.
         * @param msg string message to covnert.
         * @return byte array version of string.
         */
        private static byte[] MakeByteArrayFromString(String msg)
        {
            byte[] retval = new byte[msg.Length];
            for (int i = 0; i < msg.Length; ++i)
                retval[i] = (byte)msg[i];
            return retval;
        }
        private static float deadZone(float value)
        {
            Debug.Print(value.ToString());
            if(value < 0.1 && value > -0.1)
            {
                return 0.0f;
            }
            else
            {
                return value;
            }
        }
    }
}