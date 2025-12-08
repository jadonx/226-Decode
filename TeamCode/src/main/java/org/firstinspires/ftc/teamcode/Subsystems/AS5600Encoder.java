package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

@I2cDeviceType
@DeviceProperties(name = "AS5600 Encoder", xmlTag = "AS5600Encoder")
public class AS5600Encoder extends I2cDeviceSynchDevice<I2cDeviceSynch> {
    private static final I2cAddr ADDRESS_AS5600 = I2cAddr.create7bit(0x36);

    private static final int RAW_ANGLE_HIGH = 0x0C;
    private static final int RAW_ANGLE_LOW  = 0x0D;

    private double lastRawAngle = 0.0;
    private double continuousAngle = 0.0;
    private boolean firstRead = true;

    public AS5600Encoder(I2cDeviceSynch deviceSynch) {
        super(deviceSynch, true);
        this.deviceClient.setI2cAddress(ADDRESS_AS5600);
        this.deviceClient.engage();
    }

    @Override
    protected boolean doInitialize() {
        deviceClient.setReadWindow(
                new I2cDeviceSynch.ReadWindow(
                        RAW_ANGLE_HIGH, // start here
                        2,              // read 0x0C and 0x0D
                        I2cDeviceSynch.ReadMode.REPEAT
                )
        );

        return true;
    }

    // Get raw 12 bits
    private int getRaw12() {
        byte[] data = deviceClient.read(RAW_ANGLE_HIGH, 2);

        int high = data[0] & 0x0F;  // lower 4 bits are valid
        int low  = data[1] & 0xFF;

        return (high << 8) | low;
    }

    public double getWrappedAngle() {
        return getRaw12() * (360.0 / 4096.0);
    }

    public double getContinuousAngle() {
        double raw = getWrappedAngle();

        // First read: don't compute delta, just initialize
        if (firstRead) {
            lastRawAngle = raw;
            firstRead = false;
            continuousAngle = raw;
            return raw;  // typically 0
        }

        double delta = raw - lastRawAngle;

        // Fix discontinuity around 0/360 boundary:
        if (delta > 180) delta -= 360;
        if (delta < -180) delta += 360;

        continuousAngle += delta;
        lastRawAngle = raw;

        return continuousAngle;
    }

    public void rebaseContinuousAngle() {
        continuousAngle = getWrappedAngle();
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "AS5600 Magnetic Encoder";
    }
}