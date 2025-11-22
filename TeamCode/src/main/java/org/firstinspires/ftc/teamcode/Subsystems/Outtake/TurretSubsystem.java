package org.firstinspires.ftc.teamcode.Subsystems.Outtake;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;


@Config
public class TurretSubsystem {
    private CRServo turretLeft, turretRight;
    private OpMode opMode;
    private PIDController pid;
    public static double kP = 0.02;
    public static double kI = 0.0;
    public static double kD = 0.0008;

    public TurretSubsystem(OpMode _opMode) {
        opMode = _opMode;
    }

    public void init() {
        opMode.telemetry = new MultipleTelemetry(opMode.telemetry, FtcDashboard.getInstance().getTelemetry());
        turretLeft = opMode.hardwareMap.get(CRServo.class, TurretConstant.HMServoTurretLeft);
        turretRight = opMode.hardwareMap.get(CRServo.class, TurretConstant.HMServoTurretRight);

        pid = new PIDController(kP, kI, kD);
        pid.setTolerance(0.5);
    }

    public void trackTarget(double tX) {
        pid.setPID(kP, kI, kD);

        double correction = pid.calculate(tX, 0);

        correction = Math.max(Math.min(correction, 0.6), -0.6);

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
}
