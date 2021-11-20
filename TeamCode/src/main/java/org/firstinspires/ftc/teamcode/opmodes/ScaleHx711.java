package org.firstinspires.ftc.teamcode.opmodes;



import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDeviceWithParameters;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

import org.jetbrains.annotations.NotNull;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

//import com.qualcomm.robotcore.util.TypeConversion;


// YANCUI: Referencing https://github.com/cdjq/DFRobot_HX711_I2C/blob/main/DFRobot_HX711_I2C.cpp
@SuppressWarnings({"WeakerAccess", "unused"}) // Ignore access and unused warnings
// Both driver classes cannot register the sensor at the same time. One driver should have the
// sensor registered, and the other should be commented out

@I2cDeviceType
@DeviceProperties(name = "BF Robot HX711 Weight Sensor", description = "a BF Robot HX711 weight sensor", xmlTag = "scaleHX711")
public class ScaleHx711 extends I2cDeviceSynchDeviceWithParameters<I2cDeviceSynch, ScaleHx711.Parameters>
{
    // private variables
    float calibration;
    long  offset;
    boolean debug = false;
    ////////////////////////////////////////////////////////////////////////////////////////////////
    // User Methods
    ////////////////////////////////////////////////////////////////////////////////////////////////

    public float readWeight(int times)
    {
        long value = average(times);
        byte ppFlag = peelFlag();
        float final_value;

        if (ppFlag == 1) {
            this.offset = average(times);
        } else if (ppFlag == 2) {
            this.calibration = getCalibration();
        }

        if (value == 0) {
            if (debug) {
                telemetry.addData("readWeight", "calibration is 0");
                telemetry.update();
            }
            return 0;
        } else {
            final_value = ((float)value - this.offset)/this.calibration;
            if (this.debug) {
                telemetry.addData("readWeight : ", final_value);
            }
            return (final_value);
        }

    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // helper functions
    ////////////////////////////////////////////////////////////////////////////////////////////////
    public boolean getCalFlag()
    {
        byte ppFlag = peelFlag();
        return ppFlag == 2;
    }
    public void enableCal()
    {
        byte[] data = {0};
        writeReg(Register.REG_CLICK_RST, data);
    }
    public void peel()
    {
        this.offset = average(15);
        byte[] data = {0};
        writeReg(Register.REG_CLICK_RST, data);
    }

    public long getOffset() {
        return this.offset;
    }

    public long average (int times)
    {
        long sum = 0;
        long value, cnt=0;
        while (cnt < times)
        {
            value = getValue();
            if (value != 0) {
                sum += value;
                cnt += 1;
            }
        }
        return sum/cnt;
    }
    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Calibration and peelFlag
    ////////////////////////////////////////////////////////////////////////////////////////////////
    public float getCalibration()
    {
        byte[] data = new byte[4];
        int value;
        readReg(Register.REG_DATA_GET_CALIBRATION, data,4);

        value = data[0];
        for (int i=1; i<4; i++) {
            value = (value << 8) | data[i];
        }

        //float * cal = (float *)&value;
        //return * cal;
        float f = ByteBuffer.wrap(data).order(ByteOrder.BIG_ENDIAN).getFloat();
        if (this.debug) {
            telemetry.addData("getCalibration", "value hex = 0x%0h, float = %f", value, f);
        }
        return f;
    }

    public void setCalibration(float value)
    {
        this.calibration = value;
    }

    public byte peelFlag()
    {
        byte[] data = new byte[1];
        readReg(Register.REG_DATA_GET_PEEL_FLAG, data, 1);
        if (data[0] == 1 || data[0] == 2) {
            return data[0];
        } else {
            return 0;
        }
    }

    public long getValue()
    {
        byte[] data = new byte[8];
        long value = 0;
        data = deviceClient.read(Register.REG_DATA_GET_RAM_DATA.bVal, 8);
        //readReg(Register.REG_DATA_GET_RAM_DATA, data,8);
        /*
        value = data[0];
        value = (value << 8) | data[1];
        value = (value << 8) | data[2];
        value = (value << 8) | data[3];
        return value;
        */

        //data0 = data[0];
        //data0 = 19;
        /*
        if (data[0] + 128 == 192) { //0xc0
            value = data[1] + 128;
            for (int i=2; i<8; i++) {
                value = (value << 8) + (data[i] + 128);
            }
            return (value); // ^ 0x800000);
        } else {
        */

        if (data[2] == 0x12) {
            value = (data[3] >= 0) ? data[3] : (256 + data[3]);



            //for (int i = 3; i < 6; i++) {
            //    int byte_val = (data[i] < 0) ? data[i] : (256 + data[i]);
            //    if (data[i] >= 0) {
            //        byte_val= data[i];
            //    } else {
            //        byte_val= data[i]+256;
            //    }
            //    value = (value << 8) + byte_val;
            //}


        }
        return value;



    }

    public void setThreashold(short threshold)
    {
        byte[] data;
        data = new byte[2];
        data[0] = (byte)(threshold >> 8);
        data[1] = (byte)(threshold & 0xFF);
        writeReg(Register.REG_SET_CAL_THRESHOLD, data);
        //return;
    }

    public void setCalWeight(short triWeight)
    {
        byte[] data;
        data = new byte[2];
        data[0] = (byte)(triWeight >> 8);
        data[1] = (byte)(triWeight & 0xFF);
        writeReg(Register.REG_SET_TRIGGER_WEIGHT, data);
        //return;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Read and Write Methods
    ////////////////////////////////////////////////////////////////////////////////////////////////

    protected void writeReg(final Register reg, byte[] data)
    {
        deviceClient.write(reg.bVal, data);
    }


    protected void readReg(final Register reg, byte[] data, int size)
    {
        byte[] tmp_data;
        tmp_data = deviceClient.read(reg.bVal, size);
        for (int i = 0; i < size; i++) {
            data[i] = tmp_data[i];
            if (this.debug) {
                telemetry.addData("readReg ", "reg 0x%0h, data[%0d] = 0x%0h", reg.bVal, i, data[i] );
                telemetry.update();
            }
        }

        //return 0;
        /*
        if (size == 1 || size == 2 || size == 4) {
            return TypeConversion.byteArrayToLong(deviceClient.read(reg.bVal, size));
        } else {
            return 0;
        }
         */
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Registers and Config Settings
    ////////////////////////////////////////////////////////////////////////////////////////////////

    public enum Register
    {

        FIRST(0x65),
        REG_CLEAR_REG_STATE(0x65),
        REG_DATA_GET_RAM_DATA(0x66),
        REG_DATA_GET_CALIBRATION(0x67),
        REG_DATA_GET_PEEL_FLAG(0x69),
        REG_DATA_INIT_SENSOR(0x70),
        REG_SET_CAL_THRESHOLD(0x71),
        REG_SET_TRIGGER_WEIGHT(0x72),
        REG_CLICK_RST(0x73),
        REG_CLICK_CAL(0x74),
        LAST(REG_CLICK_CAL.bVal);


        /*
        FIRST(0x1),
        REG_CLEAR_REG_STATE(0x1),
        REG_DATA_GET_RAM_DATA(0x2),
        REG_DATA_GET_CALIBRATION(0x3),
        REG_DATA_GET_PEEL_FLAG(0x5),
        REG_DATA_INIT_SENSOR(0xC),
        REG_SET_CAL_THRESHOLD(0xD),
        REG_SET_TRIGGER_WEIGHT(0xE),
        REG_CLICK_RST(0xF),
        REG_CLICK_CAL(0x10),
        LAST(REG_CLICK_CAL.bVal);

         */

        public int bVal;

        Register(int bVal)
        {
            this.bVal = bVal;
        }
    }


    // More settings are available on the sensor, but not included here. Could be added later

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Construction and Initialization
    ////////////////////////////////////////////////////////////////////////////////////////////////

    public final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x64);

    public ScaleHx711(I2cDeviceSynch deviceClient)
    {
        super(deviceClient, true, new Parameters());

        //this.setOptimalReadWindow();
        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);

        super.registerArmingStateCallback(false); // Deals with USB cables getting unplugged
        // Sensor starts off disengaged so we can change things like I2C address. Need to engage
        this.deviceClient.engage();

        //this.offset = average(10);


    }

    protected void setOptimalReadWindow()
    {
        // Sensor registers are read repeatedly and stored in a register. This method specifies the
        // registers and repeat read mode
        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(
                Register.FIRST.bVal,
                Register.LAST.bVal - Register.FIRST.bVal + 1,
                I2cDeviceSynch.ReadMode.REPEAT);
        this.deviceClient.setReadWindow(readWindow);
    }


    @Override
    protected synchronized boolean internalInitialize(@NonNull Parameters params)
    {

        this.parameters = params.clone();

        byte[] initSeq = new byte[]{(byte)0x65};
        deviceClient.write(Register.REG_DATA_INIT_SENSOR.bVal, initSeq);

        return true;
    }

    public static class Parameters implements Cloneable
    {
        I2cAddr i2cAddr = ADDRESS_I2C_DEFAULT;

        // All settings available
        //Hysteresis hysteresis = Hysteresis.HYST_0;
        //AlertControl alertControl = AlertControl.ALERT_DISABLE;

        @NotNull
        public Parameters clone()
        {
            try
            {
                return (Parameters) super.clone();
            }
            catch(CloneNotSupportedException e)
            {
                throw new RuntimeException("Internal Error: Parameters not cloneable");
            }
        }
    }

    @Override
    public Manufacturer getManufacturer()
    {
        //return Manufacturer.Adafruit;
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName()
    {
        return "DF Robot HX711 Weight Sensor";
    }
}