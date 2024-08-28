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

namespace ImuTest01NS {

    class ImuTest01Comms {
        private System.IO.Ports.SerialPort thePort;
        private string portName = "";
        public static int tiltPoints = 4096;

        public class Comm_Exception : Exception {
            public Comm_Exception() { }
            public Comm_Exception( string message ) : base( message ) { }
        };

        public enum REGS
        {
            RegFirmWareVersion = 0,      //0, Version # set at top of main.c
            RegUniqueID,                 //1, a unique ID for each MCU (96 bits, but only returns lowest 16 bits)
            RegTick,                     //2, lower 16 bits of the 1mS system timer
            RegAdcTemp,                  //3, read temperature sensor internal to MCU, 0.76V @25degC + 2.5mV/degC, temp = 25+(nAdc*3.3/4095-0.76)/0.025, 1022 => 27.5degC
            RegAdcRef,                   //4, MCU internal reference voltage (1.2Vnom), should be around 1.2/3.3*4095 = 1500, can use to calculate voltage of 3.3V supply
            RegPeltierTempColdsideAdc,          //5,
            RegPeltierTempColdsideCelsius,      //6	
            RegPeltierTempHotsideAdc,           //7,
            RegPeltierTempHotsideCelsius,       //8	
            RegPeltierPWMDutyCycle,             //9
            RegPeltierReverseMode,              //10, if set to 0, run in normal mode, if 1, reverse heat transfer direction. Note that 100%duty cycle is fastest heat transfer in normal but slowest in reverse mode. 
            RegPeltierBangBangTempTargetAdc,    //11,
            RegPeltierBangBangTempAllowanceAdc, //12,
            RegPeltierPidSetpointAdc,           //13, target temp adc for the pid loop to achieve
            RegPeltierPidDt,                    //14, units of ms; recalculate error value every dt ms
            RegPeltierPidKp,                    //15, constant of proporionality controlling the proportional term
            RegPeltierPidKi,                    //16, constant of proporionality controlling the integral term
            RegPeltierPidKd,                    //17, constant of proporionality controlling the derivative term
            RegPeltierPidManipulatedVariable,   //18, when written to, changes PWM's CCR1 reg to the written value: value = u(t) = Kp*e(t) + Ki*e(t)*dt + Kd*( e(t_n)-e(t_n-1) )/dt. e(t), error value, computed in GUI/Python
            RegLast
        };

        public static bool RegSigned(REGS reg)
        {
            switch (reg)
            {
              
                //case REGS.RegImuGzOffset:
                    //return true;
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

        public ImuTest01Comms()
        {
            List<string> ports = ImuTest01Comms.GetPortNames();
            if (ports.Count > 0)
            {
                newPort(ports[0]);
                this.Open();
            }
        }

        public ImuTest01Comms(string port)
        {
            newPort(port);
            this.Open();
        }

        //From the com ports attached, select the ones with the correct VID/PID
        public static List<string> GetPortNames()
        {
            List<string> names = new List<string>();
            List<ComPort> portExtraInfo = ImuTest01Comms.GetSerialPorts();
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

        public bool SetNvParam( NVparams param, UInt16 value ) {
            string str = "s" + ((UInt16)param).ToString("x") + "=" + value.ToString("x");
            this.WriteLine(str);
            string resStr = this.ReadLine();
            return true;
        }

        public UInt16 GetNvParam( NVparams param ) {
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

        public bool FlashNvParams() {
            string str = "f";
            this.WriteLine(str);
            string resStr = this.ReadLine(); 
            return true;
        }
        public bool SetReg(REGS reg, UInt16 value)
        {
            string str = "w" + ((UInt16)reg).ToString("x") + "=" + value.ToString("x");
            this.WriteLine(str+"\n");
            string resStr = this.ReadLine();
            return true;
        }

        public UInt16 GetReg( REGS reg ) {
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

        public UInt16[] GetReg(REGS[] reg)
        {
            UInt16[] result = new UInt16[(int)REGS.RegLast];
            for (int i = 0; i < (int)REGS.RegLast; i++)
            {
                result[i] = GetReg(reg[i]);
            }
            return result;
        }
/*
        public Double[] GetFullTilt()
        {
            Double[] result = new Double[tiltPoints];
            int idx = 0;
            do
            {
                string str = "z";
                this.WriteLine(str);
                //System.Threading.Thread.Sleep(10);
                string resStr = this.ReadLine();
                string[] nums = resStr.Split(',');

                for (int j = 0; j < nums.Length; ++j)
                {
                    result[idx++] = 0.0180 / Math.PI * Convert.ToDouble(nums[j]);
                }
            }
            while (idx < tiltPoints);
            this.SetReg(REGS.RegFullIndex, 0);
            return result;
        }

        public Double[] GetSmoothTilt()
        {
            Double[] result = new Double[tiltPoints];
            int idx = 0;
            do
            {
                string str = "y";
                this.WriteLine(str);
                //System.Threading.Thread.Sleep(10);
                string resStr = this.ReadLine();
                string[] nums = resStr.Split(',');

                for (int j = 0; j < nums.Length; ++j)
                {
                    result[idx++] = 0.0180 / Math.PI * Convert.ToDouble(nums[j]);
                }
            }
            while (idx < tiltPoints);
            this.SetReg(REGS.RegFullIndex, 0);
            return result;
        }
*/

    }
}
