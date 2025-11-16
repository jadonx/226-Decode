package org.firstinspires.ftc.teamcode.Subsystems.Encoder;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

@I2cDeviceType
@DeviceProperties(name = "AS5600 Encoder", xmlTag = "AS5600Encoder")
public class AS5600Encoder extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    private static final I2cAddr ADDRESS_AS5600 = I2cAddr.create7bit(0x36);
    private static final int ANGLE_REG = 0x0E;

    private double lastAngle = 0;
    private double turretAngle = 0;



    public AS5600Encoder(I2cDeviceSynch deviceSynch) {
        super(deviceSynch, true);
        this.deviceClient.setI2cAddress(ADDRESS_AS5600);
        this.deviceClient.engage();
    }

    @Override
    protected boolean doInitialize() {
        return true;
    }

    public double getAngleDegrees() {
        byte[] rawBytes = deviceClient.read(ANGLE_REG, 2);
        int raw = ((rawBytes[0] & 0x0F) << 8) | (rawBytes[1] & 0xFF);
        return raw * (360.0 / 4096.0);
    }

    public double getTurretAngle() {
        double currentAngle = getAngleDegrees();
        double deltaAngle = currentAngle - lastAngle;

        if (deltaAngle > 180) {
            deltaAngle -= 360;
        } else if (deltaAngle < -180) {
            deltaAngle += 360;
        }

        turretAngle += deltaAngle;
        lastAngle = currentAngle;

        return turretAngle;
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