using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.IO;
using System.Text;
using System.Threading;
using System.Windows.Forms;
using System.Windows.Forms.DataVisualization.Charting;
using static System.Net.Mime.MediaTypeNames;

namespace ImuTest01NS {
    public partial class MainForm : Form
    {
        ImuTest01Comms theBoard;
        int fwRev = 0;
        private bool boardConnected = false;
        bool boardInit = false;
 

        public MainForm()
        {
            InitializeComponent();
            guiRev.Text = "Rev XX";
            try
            {
                theBoard = new ImuTest01Comms();
                PopulatePorts();
                //Console.WriteLine(" in MainForm, is the board open?: " + theBoard.IsOpen);
                checkBox_Connect.Checked = theBoard.IsOpen;
                InitBoard();
            }
            catch (ImuTest01Comms.Comm_Exception)
            {
                Console.Write("caught comm_exception");
            }
        }

        private void InitBoard()
        {
            try
            {
                //theBoard.WriteLine("slj");
                fwRev = theBoard.GetReg(ImuTest01Comms.REGS.RegFirmWareVersion);
            }
            catch (Exception)
            {
                fwRev = 0;
                boardConnected = false;
            }



            if (fwRev > 0)
            {
                boardConnected = true;
                if (boardInit)
                {
                    return;
                }
                initRegs();
                InitParams();
                updateRegs();
                //updateStuff();
               
                boardInit = true;
            }

        }

        private void MainForm_Shown(object sender, EventArgs e)
        {
            //InitBoard();
            tabControl_ctrl.Enabled = true;
        }

        private void quit_Click(object sender, EventArgs e)
        {
            timer1.Enabled = false;
            if (boardConnected) theBoard.Close();
            //Application.Exit();                       //Error! Had to comment out for some reason
        }

        private void PopulatePorts()
        {
            object selItem = comboBox_CommPort.SelectedItem;

            List<string> portNames = ImuTest01Comms.GetPortNames();
            comboBox_CommPort.Enabled = false;
            comboBox_CommPort.Items.Clear();
            comboBox_CommPort.Items.AddRange(portNames.ToArray());
            if (portNames.Count > 0)
            {
                if (selItem != null)
                {
                    if (comboBox_CommPort.Items.Contains(selItem))
                    {
                        comboBox_CommPort.SelectedIndex = comboBox_CommPort.Items.IndexOf(selItem);
                    }
                    else
                    {
                        comboBox_CommPort.SelectedIndex = 0;
                        //SelectPort();
                    }
                }
                else
                { // Nothing was previously selected, so we are using the first item on the list
                    comboBox_CommPort.SelectedIndex = 0;
                }
                SelectPort();
                comboBox_CommPort.Enabled = true;
            }
        }

        private void SelectPort()
        {
            theBoard.SetPort((string)comboBox_CommPort.SelectedItem);
            Console.WriteLine("in Select Port, after setting port, is it open?: " + theBoard.IsOpen);
            theBoard.Open();
            Console.WriteLine("in Select Port, after theboard.open(), is it open?: " + theBoard.IsOpen);
            //Console.WriteLine("Port Open");
            //System.Threading.Thread.Sleep(300);
        }


        private void checkBox_Connect_CheckedChanged(object sender, EventArgs e)
        {
            if (checkBox_Connect.Checked)
            {
                SelectPort();
            }
            else
            {
                theBoard.Close();
            }
        }

        private void comboBox_CommPort_SelectedIndexChanged(object sender, EventArgs e)
        {
            //TODO: having some odd interactions here that need to fix, so commented it out for a temporary fix
            //if(theBoard.IsOpen)
            //{
            //    theBoard.Close();
            //}
        }

        private void tabControl1_SelectedIndexChanged(object sender, EventArgs e)
        {
            if (tabControl_ctrl.SelectedTab == regsPage)
            {
                if (boardConnected)
                {
                    updateRegs();
                    //updateStuff();
                }
            }
            else if (tabControl_ctrl.SelectedTab == nvParamsPage)
            {
                if (boardConnected)
                {
                    ReadParameters();
                }
            }
        }

        int nLogged = 100000;

        private void read_Click(object sender, EventArgs e)
        {
            if (nSamples.Value == 1M)
            {
                updateRegs();
            }
            else
            {
                if (logFile != null)
                {
                    logFile.Close();
                    logFile = null;
                }
                OpenLog();
                nLogged = 0;
                timer1.Enabled = true;
            }
        }

        private void period_ValueChanged(object sender, EventArgs e)
        {
            timer1.Interval = (int)period.Value;
        }

        System.IO.StreamWriter logFile;
        int lastLogTick = 0;
        int tickOffset = 0;
        bool tickChecked = false;

        private void WriteHeaders()
        {
            tickChecked = false;
            for (ImuTest01Comms.REGS i = (ImuTest01Comms.REGS)0; i < ImuTest01Comms.REGS.RegLast; i++)
            {
                if (statusList.Rows[(int)i].Cells[0].Value.ToString() == true.ToString())
                {
                    logFile.Write(i.ToString() + ',');
                    if (i == ImuTest01Comms.REGS.RegTick)
                    {
                        tickChecked = true;
                    }
                }
            }
            logFile.WriteLine();
            if (!tickChecked)
            {
                lastLogTick = 0;
                tickOffset = 0;
            }
        }

        private void OpenLog()
        {
            if (logFile == null)
            {
                //TODO: This fails if folder "c:\temp" does not exist. Should make this more forgiving.
                string logFileName = "c:\\temp\\" + System.DateTime.Now.ToString("yyyy-MM-dd_HH-mm-ss") + ".csv";
                logFile = new System.IO.StreamWriter(logFileName);
                lastLogTick = 0;
                tickOffset = 0;
                WriteHeaders();
            }
        }

        private void plot_CheckedChanged(object sender, EventArgs e)
        {
            timer1.Interval = (int)period.Value;
            timer1.Enabled = plot.Checked | log.Checked;
        }

        private void log_CheckedChanged(object sender, EventArgs e)
        {
            if (log.Checked)
            {
                if (cellsChecked)
                {
                    timer1.Interval = (int)period.Value;
                    timer1.Enabled = plot.Checked;
                    OpenLog();
                }
            }
            else if (logFile != null)
            {
                timer1.Enabled = plot.Checked;
                logFile.Close();
                logFile = null;
            }
            timer1.Enabled = plot.Checked | log.Checked;
        }

        private int getRegs(ImuTest01Comms.REGS i)
        {
            UInt16 value = theBoard.GetReg(i);
            int result = value;
            if (ImuTest01Comms.RegSigned(i))
            {
                result = (Int16)value;
            }
            return result;
        }

        private int[] getRegs(ImuTest01Comms.REGS[] i)
        {
            int[] result = new int[i.Length];
            UInt16[] uResult = theBoard.GetReg(i);
            for (int j = 0; j < i.Length; j++)
            {
                if (ImuTest01Comms.RegSigned(i[j]))
                {
                    result[j] = (Int16)uResult[j];
                }
                else
                {
                    result[j] = uResult[j];
                }
            }
            return result;
        }

        ImuTest01Comms.REGS[] allRegs;

        private void initRegs()
        {
            statusList.Enabled = true;
            checkedCells = new List<ImuTest01Comms.REGS>();
            statusList.Rows.Clear();
            allRegs = new ImuTest01Comms.REGS[(int)ImuTest01Comms.REGS.RegLast];
            for (ImuTest01Comms.REGS i = (ImuTest01Comms.REGS)0; i < ImuTest01Comms.REGS.RegLast; i++)
            {
                allRegs[(int)i] = i;
            }
            try
            {
                int[] values = getRegs(allRegs);
                for (ImuTest01Comms.REGS i = (ImuTest01Comms.REGS)0; i < ImuTest01Comms.REGS.RegLast; i++)
                {
                    string[] labels = new string[3];
                    labels[0] = "False";
                    labels[1] = "(" + ((int)i).ToString() + ") " + i.ToString();
                    labels[2] = values[(int)i].ToString();
                    statusList.Rows.Add(labels);
                }
            }
            catch (ImuTest01Comms.Comm_Exception) { }
            statusList.Rows[(int)ImuTest01Comms.REGS.RegTick].Cells[2].ToolTipText = "ms";
        }

        private void UpdateRegsPlot()
        {
            try
            {
                int[] values = getRegs(allRegs);

                for (ImuTest01Comms.REGS i = (ImuTest01Comms.REGS)0; i < ImuTest01Comms.REGS.RegLast; i++)
                {
                    if (ImuTest01Comms.RegHex(i))
                    {
                        statusList.Rows[(int)i].Cells[2].Value = "0x"+values[(int)i].ToString("x");
                    }
                    else
                    {
                        statusList.Rows[(int)i].Cells[2].Value = values[(int)i].ToString();
                    }
                    if (statusList.Rows[(int)i].Cells[0].Value.ToString() == true.ToString())
                    {
                        int value = values[(int)i];
                        if (plot.Checked || ((nSamples.Value > 1M) && (((decimal)nLogged) < nSamples.Value)))
                        {
                            chart1.Series[i.ToString()].Points.AddY(value);
                            if (chart1.Series[i.ToString()].Points.Count > (int)widX.Value)
                            {
                                chart1.Series[i.ToString()].Points.RemoveAt(0);
                            }
                        }

                        if (logFile != null)
                        {
                            if (i == ImuTest01Comms.REGS.RegTick)
                            {
                                if (lastLogTick > value)
                                {
                                    tickOffset += 65536;
                                }
                                lastLogTick = value;
                                value += tickOffset;
                            }
                            logFile.Write(value.ToString() + ',');
                        }
                    }
                }

                if (logFile != null)
                {
                    logFile.WriteLine();
                    if ((nSamples.Value > 1M) && (((decimal)nLogged) < nSamples.Value))
                    {
                        nLogged++;
                        if (((decimal)nLogged) >= nSamples.Value)
                        {
                            nLogged = 100000;
                            logFile.Close();
                            logFile = null;
                            if (log.Checked)
                            {
                                if (cellsChecked)
                                {
                                    OpenLog();
                                }
                            }
                            timer1.Enabled = plot.Checked | log.Checked;
                        }
                    }
                }
            }
            catch (ImuTest01Comms.Comm_Exception)
            {
                for (ImuTest01Comms.REGS i = (ImuTest01Comms.REGS)0; i < ImuTest01Comms.REGS.RegLast; i++)
                {
                    statusList.Rows[(int)i].Cells[2].Value = "";
                }
            }
        }

        private void updateRegs()
        {
            try
            {
                int[] values = getRegs(allRegs);
                for (ImuTest01Comms.REGS i = (ImuTest01Comms.REGS)0; i < ImuTest01Comms.REGS.RegLast; i++)
                {
                    if (ImuTest01Comms.RegHex(i))
                    {
                        statusList.Rows[(int)i].Cells[2].Value = "0x" + values[(int)i].ToString("x");
                    }
                    else
                    {
                        statusList.Rows[(int)i].Cells[2].Value = values[(int)i].ToString();
                    }
                }
            }
            catch (ImuTest01Comms.Comm_Exception)
            {
                for (ImuTest01Comms.REGS i = (ImuTest01Comms.REGS)0; i < ImuTest01Comms.REGS.RegLast; i++)
                {
                    statusList.Rows[(int)i].Cells[2].Value = "xxx";
                }
            }
        }

        private void timer1_Tick(object sender, EventArgs e)
        {
            UpdateRegsPlot();
        }

        bool cellsChecked = false;
        List<ImuTest01Comms.REGS> checkedCells;

        private void statusList_CellValueChanged(object sender, DataGridViewCellEventArgs e)
        {
            if (!statusList.Enabled) return;
            if (e.ColumnIndex == statusList.Columns.IndexOf(selected))
            {
                if (e.RowIndex >= (int)ImuTest01Comms.REGS.RegLast)
                {
                    statusList.Rows[e.RowIndex].Cells[0].Value = false;
                    return;
                }
                chart1.Series.Clear();
                cellsChecked = false;
                checkedCells.Clear();
                for (ImuTest01Comms.REGS i = (ImuTest01Comms.REGS)0; i < ImuTest01Comms.REGS.RegLast; i++)
                {
                    if (statusList.Rows[(int)i].Cells[0].Value.ToString() == true.ToString())
                    {
                        cellsChecked = true;
                        chart1.Series.Add(i.ToString());
                        chart1.Series[i.ToString()].ChartType = System.Windows.Forms.DataVisualization.Charting.SeriesChartType.Line;
                        checkedCells.Add(i);
                    }
                }
                if (log.Checked)
                {
                    if (cellsChecked)
                    {
                        if (logFile != null)
                        {
                            WriteHeaders();
                        }
                        else
                        {
                            OpenLog();
                        }
                    }
                }
            }
            else if (e.ColumnIndex == statusList.Columns.IndexOf(newVal))
            {
                if (e.RowIndex < 0 || e.RowIndex >= (int)ImuTest01Comms.REGS.RegLast)
                {
                    return;
                }
                int newvalue;
                try
                {
                    newvalue = Convert.ToInt32((string)(statusList[e.ColumnIndex, e.RowIndex].Value), 10);
                }
                catch (Exception)
                {
                    statusList[statusList.Columns.IndexOf(newVal), e.RowIndex].Value = "";
                    return;
                }
                try
                {
                    if (ImuTest01Comms.RegSigned((ImuTest01Comms.REGS)e.RowIndex))
                    {
                        if (newvalue >= -32768 && newvalue <= 32767)
                        {
                            bool success = theBoard.SetReg((ImuTest01Comms.REGS)e.RowIndex, (ushort)newvalue);
                        }
                        newvalue = (int)((short)theBoard.GetReg((ImuTest01Comms.REGS)e.RowIndex));
                    }
                    else
                    {
                        if (newvalue >= 0 && newvalue <= 0xFFFF)
                        {
                            bool success = theBoard.SetReg((ImuTest01Comms.REGS)e.RowIndex, (ushort)newvalue);
                        }
                        newvalue = (int)theBoard.GetReg((ImuTest01Comms.REGS)e.RowIndex);
                    }
                    statusList[statusList.Columns.IndexOf(value), e.RowIndex].Value = newvalue.ToString();
                }
                catch (ImuTest01Comms.Comm_Exception)
                {
                    statusList[statusList.Columns.IndexOf(value), e.RowIndex].Value = "";
                }
                statusList[statusList.Columns.IndexOf(newVal), e.RowIndex].Value = "";
            }
        }

        private void unCheckAll_Click(object sender, EventArgs e)
        {
            statusList.Enabled = false;
            log.Checked = false;
            plot.Checked = false;
            cellsChecked = false;
            checkedCells.Clear();
            if (logFile != null)
            {
                logFile.Close();
                logFile = null;
            }
            nLogged = 100000;
            timer1.Enabled = plot.Checked | log.Checked;
            for (int i = 0; i < statusList.RowCount; i++)
            {
                statusList.Rows[i].Cells[0].Value = false;
            }
            statusList.Enabled = true;
        }

        private void maxY_ValueChanged(object sender, EventArgs e)
        {
            if (autoScale.Checked) return;
            if (minY.Value > maxY.Value - 1)
            {
                minY.Value = maxY.Value - 1;
            }
            minY.Maximum = maxY.Value - 1;
            chart1.ChartAreas[0].AxisY.Maximum = (double)maxY.Value;
        }

        private void minY_ValueChanged(object sender, EventArgs e)
        {
            if (autoScale.Checked) return;
            if (maxY.Value < minY.Value + 1)
            {
                maxY.Value = minY.Value + 1;
            }
            maxY.Minimum = minY.Value + 1;
            chart1.ChartAreas[0].AxisY.Minimum = (double)minY.Value;
        }

        private void widX_ValueChanged(object sender, EventArgs e)
        {
            chart1.ChartAreas[0].AxisX.Maximum = (double)widX.Value;
        }

        private void autoScale_CheckedChanged(object sender, EventArgs e)
        {
            if (autoScale.Checked)
            {
                maxY.Enabled = false;
                minY.Enabled = false;
                chart1.ChartAreas[0].AxisY.Maximum = double.NaN;
                chart1.ChartAreas[0].AxisY.Minimum = double.NaN;
            }
            else
            {
                maxY.Enabled = true;
                minY.Enabled = true;
                chart1.ChartAreas[0].AxisY.Minimum = (double)minY.Value;
                chart1.ChartAreas[0].AxisY.Maximum = (double)maxY.Value;
            }
        }

        private void InitParams()
        {
            parameters.Rows.Clear();
            for (ImuTest01Comms.NVparams i = (ImuTest01Comms.NVparams)0; i < ImuTest01Comms.NVparams.NvLast; i++)
            {
                string[] labels = new string[4];
                labels[0] = ((int)i).ToString("D");
                labels[1] = i.ToString("f");
                labels[2] = "";
                labels[3] = "";

                parameters.Rows.Add(labels);
            }
        }

        private void ReadParameters()
        {
            for (ImuTest01Comms.NVparams i = (ImuTest01Comms.NVparams)0; i < ImuTest01Comms.NVparams.NvLast; i++)
            {
                try
                {
                    int value;
                    if (ImuTest01Comms.NvpSigned(i))
                    {
                        value = (int)((short)theBoard.GetNvParam(i));
                    }
                    else
                    {
                        value = (int)theBoard.GetNvParam(i);
                    }

                    parameters[parameters.Columns.IndexOf(pValue), (int)i].Value = value.ToString("D");
                    parameters[parameters.Columns.IndexOf(newValue), (int)i].Value = "";
                }
                catch (ImuTest01Comms.Comm_Exception)
                {
                    parameters[parameters.Columns.IndexOf(pValue), (int)i].Value = "";
                    parameters[parameters.Columns.IndexOf(newValue), (int)i].Value = "";
                }
            }
        }

        private void parameters_CellValueChanged(object sender, DataGridViewCellEventArgs e)
        {
            if (e.ColumnIndex != parameters.Columns.IndexOf(newValue) || e.RowIndex < 0 || e.RowIndex >= parameters.RowCount)
            {
                return;
            }
            int value;
            try
            {
                value = Convert.ToInt32((string)(parameters[e.ColumnIndex, e.RowIndex].Value), 10);
            }
            catch (Exception)
            {
                parameters[parameters.Columns.IndexOf(newValue), e.RowIndex].Value = "";
                return;
            }
            try
            {
                if (ImuTest01Comms.NvpSigned((ImuTest01Comms.NVparams)e.RowIndex))
                {
                    if (value >= -32768 && value <= 32767)
                    {
                        bool success = theBoard.SetNvParam((ImuTest01Comms.NVparams)e.RowIndex, (ushort)value);
                    }
                    value = (int)((short)theBoard.GetNvParam((ImuTest01Comms.NVparams)e.RowIndex));
                }
                else
                {
                    if (value >= 0 && value <= 0xFFFF)
                    {
                        bool success = theBoard.SetNvParam((ImuTest01Comms.NVparams)e.RowIndex, (ushort)value);
                    }
                    value = (int)theBoard.GetNvParam((ImuTest01Comms.NVparams)e.RowIndex);
                }
                parameters[parameters.Columns.IndexOf(pValue), e.RowIndex].Value = value.ToString("D");
            }
            catch (ImuTest01Comms.Comm_Exception)
            {
                parameters[parameters.Columns.IndexOf(pValue), e.RowIndex].Value = "";
            }
            parameters[parameters.Columns.IndexOf(newValue), e.RowIndex].Value = "";
        }

        private void toFlash_Click(object sender, EventArgs e)
        {
            bool success = theBoard.FlashNvParams();
        }

        private void saveParametersToFile()
        {
            saveFileDialog1.FileName = "parameters";
            DialogResult result = saveFileDialog1.ShowDialog();
            if (result == DialogResult.OK)
            {
                System.IO.StreamWriter strWri = new System.IO.StreamWriter(saveFileDialog1.FileName, false, Encoding.ASCII);
                strWri.WriteLine("ID, Parameter, Value");
                for (ImuTest01Comms.NVparams i = (ImuTest01Comms.NVparams)0; i < ImuTest01Comms.NVparams.NvLast; i++)
                {
                    try
                    {
                        strWri.WriteLine(((int)i).ToString() + "," + i.ToString() + "," + ((int)theBoard.GetNvParam(i)).ToString("D"));
                    }
                    catch (ImuTest01Comms.Comm_Exception)
                    {
                        strWri.WriteLine(((int)i).ToString() + "," + i.ToString() + "," + "XXX");
                    }
                }
                strWri.Close();
            }
        }

        private void readParametersFromFile()
        {
            DialogResult result = openFileDialog1.ShowDialog();
            if (result == DialogResult.OK)
            {
                bool success = true;
                System.IO.StreamReader strRead = null;
                try
                {
                    strRead = new System.IO.StreamReader(openFileDialog1.FileName, Encoding.ASCII);
                    strRead.ReadLine();
                    for (ImuTest01Comms.NVparams i = (ImuTest01Comms.NVparams)0; i < ImuTest01Comms.NVparams.NvLast; i++)
                    {
                        string line = strRead.ReadLine();
                        string[] tokens = line.Split(',');
                        success &= theBoard.SetNvParam(i, Convert.ToUInt16(tokens[2]));
                    }
                }
                catch
                {
                    success = false;
                }
                finally
                {
                    if (strRead != null) strRead.Close();
                }
            }
        }

























        private void toFile_Click(object sender, EventArgs e)
        {
            saveParametersToFile();
            ReadParameters();
        }

        private void fromFile_Click(object sender, EventArgs e)
        {
            readParametersFromFile();
            ReadParameters();
        }




        private void checkBoxImuDataPlot_CheckedChanged(object sender, EventArgs e)
        {
        
        }
        private void chartImuData_Click(object sender, EventArgs e)
        {

        }
        private void checkBoxImuDataAx_CheckedChanged(object sender, EventArgs e)
        {

        }

        private void checkBoxImuDataAy_CheckedChanged(object sender, EventArgs e)
        {

        }

        private void checkBoxImuDataAz_CheckedChanged(object sender, EventArgs e)
        {

        }

        private void checkBoxImuDataGx_CheckedChanged(object sender, EventArgs e)
        {

        }

        private void checkBoxImuDataGy_CheckedChanged(object sender, EventArgs e)
        {

        }

        private void checkBoxImuDataGz_CheckedChanged(object sender, EventArgs e)
        {

        }

        private void labelAnglex_Click(object sender, EventArgs e)
        {

        }

        private void labelAngley_Click(object sender, EventArgs e)
        {

        }

        private void labelAnglez_Click(object sender, EventArgs e)
        {

        }

  


    }

}