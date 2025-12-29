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

    public static double kP = 0.0065; // Proportional gain on position error
    public static double kI = 0.0; // Integral gain on position error
    public static double kD = 0.0006; // Derivative gain on position error
    public static double kF = 0.05; // Feedforward to overcome static friction

    double lastError = 0;
    long lastTime = System.nanoTime();

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

        double currentAngle = getTurretAngle();
        double error = wrapDegrees(targetAngle - currentAngle);

        long currentTime = System.nanoTime();
        double deltaTime = (currentTime - lastTime) / 1.0e9;
        double derivative = 0;
        if (deltaTime > 0) {
            derivative = (error - lastError) / deltaTime;
        }
        double pTerm = kP * error;
        double dTerm = kD * derivative;
        double fTerm = Math.signum(error) * kF; // Static friction boooost!!!!
        double power = pTerm + dTerm + fTerm;


        power = Math.max(Math.min(power, 1), -1);


        if (Math.abs(error) < 0.5) {
            setPower(0);
        } else {
            setPower(power);
        }


        lastError = error;
        lastTime = currentTime;


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
