package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
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
    private PIDFController pid;

    public static double kP = 0.02; // Proportional gain on position error
    public static double kI = 0.0; // Integral gain on position error
    public static double kD = 0.0005; // Derivative gain on position error
    public static double kF = 0.0015; // Feedforward to overcome static friction

    public static double RIGHT_LIMIT = 140;
    public static double LEFT_LIMIT  = -220;

    public Turret(HardwareMap hardwareMap) {
        turretRight = hardwareMap.get(CRServo.class, Constants.HMServoTurretRight);
        turretLeft = hardwareMap.get(CRServo.class, Constants.HMServoTurretLeft);

        turretEncoder = hardwareMap.get(IMU.class, Constants.HMTurretEncoder);

        IMU.Parameters turretParameters = new IMU.Parameters(new RevHubOrientationOnRobot( RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));

        turretEncoder.initialize(turretParameters);

        turretEncoder.resetYaw();

        pid = new PIDFController(kP, kI, kD, kF);
    }

    public void goToAngle(double targetAngle) {
        pid.setPIDF(kP, kI, kD, kF);
        double currentAngle = getTurretAngle();

        double error = wrapDegrees(targetAngle - currentAngle);

        double power = pid.calculate(0, error);

        power = Math.max(Math.min(power, 1), -1);

        if (pid.atSetPoint()) {
            setPower(0);
            return;
        }
        setPower(power);
    }

    public double getTurretAngle() {
        return -1*turretEncoder.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    private double wrapDegrees(double angle) {
        while (angle <= -180) angle += 360;
        while (angle > 180) angle -= 360;
        return angle;
    }


    public void setPower(double power) {
        turretLeft.setPower(-power);
        turretRight.setPower(-power);
    }
}
