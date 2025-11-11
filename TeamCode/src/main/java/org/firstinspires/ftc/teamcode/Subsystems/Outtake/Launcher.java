package org.firstinspires.ftc.teamcode.Subsystems.Outtake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Launcher {
    private DcMotor shooter1, shooter2;

    // RPM Calculations
    private int last = 0;
    private final double ticksPerRev = 28;
    private ElapsedTime rpmTimer;

    // RPM PID
    private double kP, kD;
    private double lastError = 0;
    private ElapsedTime pidTimer;

    public Launcher(HardwareMap hardwareMap) {
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");

        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rpmTimer = new ElapsedTime();

        pidTimer = new ElapsedTime();
    }

    public void setPower(double power) {
        shooter1.setPower(power);
        shooter2.setPower(power);
    }

    public double currentRPM() {
        int current = shooter1.getCurrentPosition();
        int deltaTicks = current - last;
        last = current;

        double revsPerSecond = (deltaTicks / ticksPerRev) / rpmTimer.seconds();
        rpmTimer.reset();

        return revsPerSecond * 60.0;
    }

    public double updatePID(int targetRPM) {
        double error = targetRPM - currentRPM();

        double derivative = (error - lastError) / pidTimer.seconds();
        lastError = error;

        return kP * error + kD * derivative;
    }

    public void updatePIDValues(double kP, double kD) {
        this.kP = kP;
        this.kD = kD;
    }
}
