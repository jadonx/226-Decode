package org.firstinspires.ftc.teamcode.Subsystems.Outtake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

public class Launcher {
    private DcMotor launcher1, launcher2;

    public Launcher(HardwareMap hardwareMap) {
        launcher1 = hardwareMap.get(DcMotor.class, Constants.HMMotorShooter1);
        launcher2 = hardwareMap.get(DcMotor.class, Constants.HMMotorShooter2);
    }

    public double calculateTargetVelocity(double distance) {
        return 0.0;
    }

    public double getCurrentVelocity() {
        return 0.0;
    }

    public void setVelocity(double target) {
        return;
    }

    public boolean atTargetVelocity(double target) {
        return Math.abs(getCurrentVelocity() - target) < 10;
    }

    public void stopLauncher() {
        // CHANGE TO VELOCITY
        launcher1.setPower(0);
        launcher2.setPower(0);
    }
}