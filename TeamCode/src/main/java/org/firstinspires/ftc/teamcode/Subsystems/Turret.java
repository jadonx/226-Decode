package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;

@Config
public class Turret {
    //Hardware
    private CRServo turretLeft, turretRight;
    private IMU turretEncoder;
    private PIDController turretPID;

    public static double kP = 0.0; // Proportional gain on position error
    public static double kI = 0.0; // Integral gain on position error
    public static double kD = 0.0; // Derivative gain on position error

    public Turret(HardwareMap hardwareMap) {
        turretRight = hardwareMap.get(CRServo.class, Constants.HMServoTurretRight);
        turretLeft = hardwareMap.get(CRServo.class, Constants.HMServoTurretLeft);

        turretEncoder = hardwareMap.get(IMU.class, Constants.HMTurretEncoder);

        IMU.Parameters turretParameters = new IMU.Parameters(new RevHubOrientationOnRobot( RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP));

        turretEncoder.initialize(turretParameters);

        turretEncoder.resetYaw();

        turretPID = new PIDController(kP, kI, kD);
    }

    public void goToAngle(double targetAngle) {
        double currentAngle = getTurretAngle();
        double power = turretPID.calculate(currentAngle, targetAngle);
        setPower(power);
    }

    public double getTurretAngle() {
        return turretEncoder.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void setPower(double power) {
        turretLeft.setPower(power);
        turretRight.setPower(power);
    }
}
