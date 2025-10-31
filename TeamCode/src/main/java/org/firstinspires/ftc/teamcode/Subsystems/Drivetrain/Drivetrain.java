package org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

import dev.nextftc.core.commands.Command;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.driving.FieldCentric;
import dev.nextftc.hardware.driving.HolonomicMode;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.MotorEx;

public class Drivetrain extends NextFTCOpMode {
    private MotorEx frontLeftMotor;
    private MotorEx frontRightMotor;
    private MotorEx backLeftMotor;
    private MotorEx backRightMotor;

    public IMUEx imu;
    public Command driverControlled;
    @Override public void onInit() {
        frontLeftMotor = new MotorEx(DrivetrainConstant.frontLeftName);
        backLeftMotor = new MotorEx(DrivetrainConstant.backLeftName);
        backRightMotor = new MotorEx(DrivetrainConstant.backRightName);
        frontRightMotor = new MotorEx(DrivetrainConstant.frontRightName);

        imu = new IMUEx(DrivetrainConstant.IMUName, Direction.UP, Direction.FORWARD).zeroed();
    }

    @Override public void onStartButtonPressed() {
        driverControlled = new MecanumDriverControlled(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX(),
                new FieldCentric(imu)
                );
        driverControlled.schedule();
    }
}
