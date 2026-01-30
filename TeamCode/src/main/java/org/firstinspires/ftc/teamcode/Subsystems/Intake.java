package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

public class Intake {
    private DcMotor intake;
    private boolean isSpinningAtRest = false;
    private final float MAX_POWER = 0.7f;
    private final float SPIN_AT_REST_POWER = 0.5f;

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

    public void toggleSpinningAtRest() {
        isSpinningAtRest = !isSpinningAtRest;
    }

    public void stopIntake() {
        if (isSpinningAtRest) {
            intake.setPower(SPIN_AT_REST_POWER);
        }
        else {
            intake.setPower(0);
        }
    }
}