package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Constants;

@Config
public class TurretTester   {
    private CRServo turretLeft, turretRight;
    private OpMode opMode;
    private IMU turretEncoder;
    private PIDController pid;
    private PIDController pidAngle;

    public static double kP = 0.02;
    public static double kI = 0.0;
    public static double kD = 0.0008;
    public static double kPA = 0.02;
    public static double kIA = 0.0;
    public static double kDA = 0.0008;
    public static double tolerance = 0.5;

    public TurretTester(OpMode _opMode) {
        opMode = _opMode;
    }

    public void init() {
        opMode.telemetry = new MultipleTelemetry(opMode.telemetry, FtcDashboard.getInstance().getTelemetry());
        turretLeft = opMode.hardwareMap.get(CRServo.class, Constants.HMServoTurretLeft);
        turretRight = opMode.hardwareMap.get(CRServo.class, Constants.HMServoTurretRight);
        turretEncoder = opMode.hardwareMap.get(IMU.class, Constants.HMTurretEncoder);

        IMU.Parameters turretParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot( RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP));
        turretEncoder.initialize(turretParameters);
        turretEncoder.resetYaw();

        pid = new PIDController(kP, kI, kD);
        pid.setTolerance(tolerance);
        pidAngle = new PIDController(kPA, kIA, kDA);
        pid.setTolerance(tolerance);
    }

    public void trackTarget(double tX) {
        pid.setPID(kP, kI, kD);

        double correction = pid.calculate(tX, 0);

        correction = Math.max(Math.min(correction, 0.8), -0.8);

        if (pid.atSetPoint()) {
            setPower(0);
            return;
        }
        setPower(-correction);
        opMode.telemetry.addData("tX", tX);
        opMode.telemetry.addData("Correction", correction);
        opMode.telemetry.addData("Error", pid.getPositionError());
        opMode.telemetry.addData("kP", kP);
        opMode.telemetry.addData("kI", kI);
        opMode.telemetry.addData("kD", kD);
        opMode.telemetry.update();
    }
    private void setPower(double power) {
        turretLeft.setPower(-power);
        turretRight.setPower(-power);
    }
    public void manualPower(double power) {
        setPower(power);
    }

    public void AimToAngle(double ta) {
        pidAngle.setPID(kPA, kIA, kDA);
        double turretAngle = turretEncoder.getRobotYawPitchRollAngles().getYaw();
        double error = ta - turretAngle;
        double power = pidAngle.calculate(error, 0);

        power = Math.max(Math.min(power, 0.8), -0.8);

        if (pidAngle.atSetPoint()) {
            setPower(0);
            return;
        }
        setPower(power);

        opMode.telemetry.addData("Turret Angle", turretEncoder.getRobotYawPitchRollAngles().getYaw());
        opMode.telemetry.update();

    }
}