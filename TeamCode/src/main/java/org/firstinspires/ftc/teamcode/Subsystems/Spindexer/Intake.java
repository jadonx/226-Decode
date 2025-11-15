package org.firstinspires.ftc.teamcode.Subsystems.Spindexer;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private DcMotor intake;
    private boolean isIntaking;

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "");
        isIntaking = false;
    }

    public void runIntake() {
        intake.setPower(0.8);
        isIntaking = true;
    }

    public void unjamIntake() {
        intake.setPower(-0.8);
    }

    public void stopIntake() {
        intake.setPower(0);
        isIntaking = false;
    }

    public boolean isIntaking() {
        return isIntaking;
    }
}
