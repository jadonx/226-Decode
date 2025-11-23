package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Constants;

@Config
public class Turret {
    private CRServo turretLeft, turretRight;
    private PIDController pid;

    private IMU turretEncoder;

    private static double kP = 0.02;
    private static double kI = 0.0;
    private static double kD = 0.0008;

    double turretTargetAngle = 0;

    private Limelight3A limelight;

    public Turret(HardwareMap hardwareMap) {
        turretRight = hardwareMap.get(CRServo.class, Constants.HMServoTurretRight);
        turretLeft = hardwareMap.get(CRServo.class, Constants.HMServoTurretLeft);

        limelight = hardwareMap.get(Limelight3A.class, Constants.HMLimelight);
        limelight.pipelineSwitch(1);
        limelight.start();

        pid = new PIDController(kP, kI, kD);
        pid.setTolerance(0.5);

        IMU.Parameters turretParameters = new IMU.Parameters(new RevHubOrientationOnRobot( RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP));
        turretEncoder.initialize(turretParameters);
        turretEncoder.resetYaw();
    }

    public void alignTurret() {
        LLResult result = limelight.getLatestResult();

        // If target is found, use limelight tracking to align turret, if not, aim to 0 angle
        if (result.isValid()) {
            double tX = result.getTx();
            trackingWithLL(tX);
        } else {
            AimToAngle(turretTargetAngle);
        }
    }

    public void trackingWithLL(double tX) {
        pid.setPID(kP, kI, kD);
        double correction = pid.calculate(-tX, 0);
        if (pid.atSetPoint()) {
            setPower(0);
            return;
        }
        setPower(correction);
    }

    public void AimToAngle(double ta) {
        pid.setPID(kP, kI, kD);
        double turretAngle = turretEncoder.getRobotYawPitchRollAngles().getYaw();
        double error = ta - turretAngle;
        double power = pid.calculate(error, 0);
        if (pid.atSetPoint()) {
            setPower(0);
            return;
        }
        setPower(power);
    }

    private void setPower(double power) {
        turretLeft.setPower(-power);
        turretRight.setPower(-power);
    }

    public void stopTurert() {
        setPower(0);
    }

}
