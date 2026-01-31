package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

public class Intake {
    private DcMotor intake;
    private final float MAX_POWER = 1.0f;

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, Constants.HMMotorIntake);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void runIntake(float intakePower) {
        intake.setPower(Math.min(intakePower, MAX_POWER));
    }

    public void reverseIntake(float intakePower) {
        intake.setPower(Math.max(-intakePower, -MAX_POWER));
    }

    public void stopIntake() {
        intake.setPower(0);
    }
}