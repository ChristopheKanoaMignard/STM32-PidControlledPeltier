using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Threading;
using System.Runtime.InteropServices;
using System.IO.Ports;
using System.ComponentModel;
using System.Management;
using System.Configuration;
using System.Text.RegularExpressions;
using System.Collections.ObjectModel;

namespace PidControlledPeltierNS {

    class PidControlledPeltierComms
    {
        private System.IO.Ports.SerialPort thePort;
        private string portName = "";
        public static int tiltPoints = 4096;

        public class Comm_Exception : Exception
        {
            public Comm_Exception() { }
            public Comm_Exception(string message) : base(message) { }
        };

        public enum REGS
        {
            RegFirmWareVersion = 0,      //0, Version # set at top of main.c
            RegUniqueID,                 //1, a unique ID for each MCU (96 bits, but only returns lowest 16 bits)
            RegTick,                     //2, lower 16 bits of the 1mS system timer
            RegAdcTemp,                  //3, read temperature sensor internal to MCU, 0.76V @25degC + 2.5mV/degC, temp = 25+(nAdc*3.3/4095-0.76)/0.025, 1022 => 27.5degC
            RegAdcRef,                   //4, MCU internal reference voltage (1.2Vnom), should be around 1.2/3.3*4095 = 1500, can use to calculate voltage of 3.3V supply
            RegPeltierTempColdsideAdc,          //5, when read, takes an adc measurement of the thermister on cold side of heat pump
            RegPeltierTempColdsideCelsius,      //6	when read, takes an adc measurement of the thermister on cold side of heat pump and converts that temp into celsius !!currently broken. Will also need to handle negative values
            RegPeltierTempHotsideAdc,           //7, when read, takes an adc measurement of the thermister on hot side of heat pump
            RegPeltierTempHotsideCelsius,       //8	when read, takes an adc measurement of the thermister on hot side of heat pump and converts that temp into celsius !!currently broken. Will also need to handle negative values
            RegPeltierTempBoxAdc,               //9, when read, takes an adc measurement of the thermister inside the insulated box
            RegPeltierTempBoxCelsius,           //10 when read, takes an adc measurement of the thermister on cold side of heat pump and converts to celsius
            RegPeltierPWMDutyCycle,             //d11 when written to, changes the PWM duty cycle to whatever percentage was entered
            RegPeltierReverseMode,              //12, if set to 0 (default), run in normal mode; if 1, reverse heat transfer direction. Note that 100% duty cycle is maximum heat transfer in normal but slowest in reverse mode. 
            RegPeltierBangBangTempTargetAdc,    //13, enabled by uncommenting BangBangControl macro in main.h; Target adc value: if current temp adc is below this value PWM turns/stays on when met or exceeded PWM turns off.
            RegPeltierBangBangTempAllowanceAdc, //14, enabled by uncommenting BangBangControl macro in main.h; The interval below target adc where PWM will remain off even though current temp is below target. PWM will turn on again once current temp adc drops below Target-Allowance
            RegPeltierPidSetpointAdc,           //15, target temp adc for the pid loop to achieve
            RegPeltierPidDt,                    //16, units of ms; recalculate error value every dt ms
            RegPeltierPidKp,                    //17, constant of proporionality controlling the proportional term
            RegPeltierPidKi,                    //18, constant of proporionality controlling the integral term
            RegPeltierPidISum,
            RegPeltierPidISumMax,
            RegPeltierPidKd,                    //19, constant of proporionality controlling the derivative term
            RegPeltierPidMv,   //20, when written to, changes PWM's CCR1 reg to the written value: value = u(t) = Kp*e(t) + Ki*e(t)*dt + Kd*( e(t_n)-e(t_n-1) )/dt. e(t), error value, computed in GUI/Python
            RegPeltierForceTemp,                //21, when nonzero, overrides the PID controller to rush 100% to the setpoint. Then stops when it is within a few adc values, returning control to PID controller. Stopping override is handled by ReadReg(RegPeltierTempColdsideAdc)
            RegLast
        };

        public static bool RegSigned(REGS reg)
        {
            switch (reg)
            {
                /*
                  case REGS.RegPeltierTempColdsideAdc:
                      return true;
                  case REGS.RegPeltierTempHotsideAdc:
                      return true;
                */
            
                default:
                    return false;
            }
        }

        public static bool Reg32S(REGS reg)
        {
            switch (reg)
            {
                case REGS.RegPeltierPidISum:
                    return true;
                case REGS.RegPeltierPidISumMax:
                    return true;
                case REGS.RegPeltierPidMv:
                    return true;
                default:
                    return false;
            }
        }

        public static bool RegHex(REGS reg)
        {
            switch (reg)
            {
                default:
                    return false;
            }
        }

        public enum NVparams
        {
            NvStart,                     //0, start code (0x55a0)
            NvBoardType,                 //1, board type, 0=FlashController from 10/2020
            NvHwVersion,                 //2, hardware revision, 0=FlashController from 10/2020
            NvLast
        }

        public static bool NvpSigned(NVparams param)
        {
            switch (param)
            {
                case NVparams.NvLast:
                    return true;
                default:
                    return false;
            }
        }

        struct ComPort // custom struct with our desired values
        {
            public string name;
            public string vid;
            public string pid;
            public string description;
        }

        public PidControlledPeltierComms()
        {
            List<string> ports = PidControlledPeltierComms.GetPortNames();
            if (ports.Count > 0)
            {
                newPort(ports[0]);
                this.Open();
            }
        }

        public PidControlledPeltierComms(string port)
        {
            newPort(port);
            this.Open();
        }

        //From the com ports attached, select the ones with the correct VID/PID
        public static List<string> GetPortNames()
        {
            List<string> names = new List<string>();
            List<ComPort> portExtraInfo = PidControlledPeltierComms.GetSerialPorts();
            for (int i = 0; i < portExtraInfo.Count; ++i)
            {
                if ((portExtraInfo[i].vid == "0483") && (portExtraInfo[i].pid == "5740"))   //F7 board
                {
                    names.Add(portExtraInfo[i].name);
                }
                else if ((portExtraInfo[i].vid == "0483") && (portExtraInfo[i].pid == "5740"))  //F4 board. Note: convert PID VID into hex
                {
                    names.Add(portExtraInfo[i].name);
                }
            }
            return names;
        }

        //Get a list of all the com ports currently attached along with the VID/PID identifier for each one
        private const string vidPattern = @"VID_([0-9A-F]{4})";
        private const string pidPattern = @"PID_([0-9A-F]{4})";
        private static List<ComPort> GetSerialPorts()
        {
            using (var searcher = new ManagementObjectSearcher
                ("SELECT * FROM WIN32_SerialPort"))
            {
                var ports = searcher.Get().Cast<ManagementBaseObject>().ToList();
                return ports.Select(p =>
                {
                    ComPort c = new ComPort();
                    c.name = p.GetPropertyValue("DeviceID").ToString();
                    c.vid = p.GetPropertyValue("PNPDeviceID").ToString();
                    c.description = p.GetPropertyValue("Caption").ToString();

                    Match mVID = Regex.Match(c.vid, vidPattern, RegexOptions.IgnoreCase);
                    Match mPID = Regex.Match(c.vid, pidPattern, RegexOptions.IgnoreCase);

                    if (mVID.Success)
                        c.vid = mVID.Groups[1].Value;
                    if (mPID.Success)
                        c.pid = mPID.Groups[1].Value;

                    return c;

                }).ToList();
            }
        }

        public bool IsOpen
        {
            get
            {
                if (thePort == null)
                    return false;
                else
                    return thePort.IsOpen;
            }
        }

        private void newPort(string name)
        {
            portName = name;
            thePort = new SerialPort(name, 115200);
            thePort.ReadTimeout = 500;
            thePort.DataBits = 8;
            thePort.Parity = Parity.None;
            thePort.StopBits = StopBits.One;
            thePort.DtrEnable = false;
            thePort.Handshake = System.IO.Ports.Handshake.None;
            thePort.RtsEnable = false;
            thePort.ReadBufferSize = 2048;
            thePort.WriteBufferSize = 2048;
            thePort.WriteTimeout = 500;
            thePort.NewLine = "\n";
        }

        public void Close()
        {
            if ((thePort != null) && thePort.IsOpen)
            {
                thePort.Close();
            }
        }

        public void Open()
        {
            if ((thePort == null) || !thePort.IsOpen)
            {
                thePort.Open();
            }
        }

        public bool WriteLine(string str)
        {
            if ((thePort != null) && thePort.IsOpen)
            {
                thePort.WriteLine(str);
                return true;
            }
            return false;
        }

        public string ReadLine()
        {
            if ((thePort != null) && thePort.IsOpen)
            {
                return (thePort.ReadLine());
            }
            else
            {
                return ("\n");
            }
        }

        public void SetPort(string port)
        {
            bool wasOpen = this.IsOpen;

            if (this.IsOpen)
            {
                this.Close();
            }
            newPort(port);
            if (wasOpen)
            {
                this.Open();
            }
        }

        public bool SetNvParam(NVparams param, UInt16 value)
        {
            string str = "s" + ((UInt16)param).ToString("x") + "=" + value.ToString("x");
            this.WriteLine(str);
            string resStr = this.ReadLine();
            return true;
        }

        public UInt16 GetNvParam(NVparams param)
        {
            string str = "g" + ((UInt16)param).ToString("x");
            this.WriteLine(str);
            string resStr = this.ReadLine();
            UInt16 result = Convert.ToUInt16(resStr.Substring(resStr.IndexOf(@"=") + 1), 16);
            return result;
        }

        public UInt16[] ReadAllNvParams()
        {
            UInt16[] result = new UInt16[(int)NVparams.NvLast];
            for (int i = 0; i < (int)NVparams.NvLast; i++)
            {
                string str = "g" + i.ToString("x");
                this.WriteLine(str);
                string resStr = this.ReadLine();
                result[i] = Convert.ToUInt16(resStr.Substring(resStr.IndexOf(@"=") + 1), 16);
            }

            return result;
        }

        public bool FlashNvParams()
        {
            string str = "f";
            this.WriteLine(str);
            string resStr = this.ReadLine();
            return true;
        }
        public bool SetReg(REGS reg, UInt16 value)
        {
            string str = "w" + ((UInt16)reg).ToString("x") + "=" + value.ToString("x");
            this.WriteLine(str + "\n");
            string resStr = this.ReadLine();
            return true;
        }

        public bool SetReg32S(REGS reg, Int32 value)
        {
            string str = "W" + ((UInt16)reg).ToString("x") + "=" + value.ToString("x");
            this.WriteLine(str + "\n");
            string resStr = this.ReadLine();    //Currently cannot write a value larger than 65535. Limitation in SerialPort file? 
            return true;
        }

        public UInt16 GetReg(REGS reg)
        {
            string str = "r" + ((UInt16)reg).ToString("x");
            this.WriteLine(str);
            string resStr = this.ReadLine();
            UInt16 result = 0;
            try
            {
                result = Convert.ToUInt16(resStr.Substring(resStr.IndexOf(@"=") + 1), 16);
            }
            catch
            {
                result = UInt16.MaxValue;
            }
            return result;
        }

        public Int32 GetReg32S(REGS reg)
        {
            string str = "R" + ((UInt16)reg).ToString("x");
            this.WriteLine(str);
            string resStr = this.ReadLine();
            Int32 result = 0;
            try
            {
                result = Convert.ToInt32(resStr.Substring(resStr.IndexOf(@"=") + 1), 16);
            }
            catch
            {
                result = Int32.MaxValue;
            }
            return result;
        }

        public UInt16[] GetReg(REGS[] reg)
        {
            UInt16[] result = new UInt16[(int)REGS.RegLast];
            for (int i = 0; i < (int)REGS.RegLast; i++)
            {
                result[i] = GetReg(reg[i]);
            }
            return result;
        }

        public Int32[] GetReg32S(REGS[] reg)
        {
            Int32[] result = new Int32[(int)REGS.RegLast];
            for (int i = 0; i < (int)REGS.RegLast; i++)
            {
                if (PidControlledPeltierComms.Reg32S(reg[i]))
                {
                    result[i] = GetReg32S(reg[i]);
                }
                else
                {
                    result[i] = GetReg(reg[i]);
                }
            }
            return result;
        }

        public Int32 ConvertTo32s(UInt16 high, UInt16 low)
        {
            Int32 full = (Int32)(high) * (Int32)Math.Pow(2, 16) + (Int32)low;
            return full;
        }
    }
}
