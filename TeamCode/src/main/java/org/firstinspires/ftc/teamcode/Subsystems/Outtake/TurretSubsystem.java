package org.firstinspires.ftc.teamcode.Subsystems.Outtake;

import static org.firstinspires.ftc.teamcode.Subsystems.Outtake.TurretConstant.kD;
import static org.firstinspires.ftc.teamcode.Subsystems.Outtake.TurretConstant.kI;
import static org.firstinspires.ftc.teamcode.Subsystems.Outtake.TurretConstant.kP;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Subsystems.Encoder.AS5600Encoder;
import org.firstinspires.ftc.teamcode.Subsystems.Limelight.LLSubsystem;


public class TurretSubsystem {
    private CRServo turretLeft, turretRight;
    private AS5600Encoder turretEncoder;
    private OpMode opMode;
    private IMU imu;
    private PIDController pid;
    private LLSubsystem limelight;

    public TurretSubsystem(OpMode _opMode, LLSubsystem _limelight) {
        opMode = _opMode;
        limelight = _limelight;
    }

    public void init() {
        turretLeft = opMode.hardwareMap.get(CRServo.class, TurretConstant.HMServoTurretLeft);
        turretRight = opMode.hardwareMap.get(CRServo.class, TurretConstant.HMServoTurretRight);
        turretEncoder = opMode.hardwareMap.get(AS5600Encoder.class, TurretConstant.HMEncoder);

        imu = opMode.hardwareMap.get(IMU.class, TurretConstant.HMIMU);

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);

        pid = new PIDController(kP, kI, kD);
        pid.setTolerance(0.5);
    }

    public void trackTarget() {
        limelight.update();

        if (!limelight.hasTarget()) {
            setPower(0);
            return;
        }

        double tx = limelight.getTargetX();
        double correction = pid.calculate(tx, 0);

        correction = Math.max(Math.min(correction, 0.6), -0.6);

        if (pid.atSetPoint()) {
            setPower(0);
            return;
        }


        setPower(correction);
    }

    private void setPower(double power) {
        turretLeft.setPower(power);
        turretRight.setPower(-power);
    }

    public void manualPower(double power) {
        setPower(power);
    }


}
