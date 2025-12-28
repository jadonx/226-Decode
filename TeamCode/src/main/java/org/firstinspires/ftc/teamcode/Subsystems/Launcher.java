package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

public class Launcher {
    // Hardware
    private DcMotorEx launcher1, launcher2;
    private Servo cover;

    // Launcher PID Values
    private double kS = 0.12;     // Static friction feedforward
    private double kV = 0.000395;  // Velocity feedforward (power per RPM)
    private double kP = 0.005;   // Proportional gain on velocity error

    private double targetVelocity = 0;
    private double targetCoverAngle = 0;

    public Launcher(HardwareMap hardwareMap) {
        launcher1 = hardwareMap.get(DcMotorEx.class, Constants.HMMotorShooter1);
        launcher2 = hardwareMap.get(DcMotorEx.class, Constants.HMMotorShooter2);

        launcher1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcher2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        launcher1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcher2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        cover = hardwareMap.get(Servo.class, Constants.HMServobackSpin);
    }
    public double getVelocity() {
        return (launcher1.getVelocity() + launcher2.getVelocity()) / 2.0;
    }

    public double getVelocity1() { return launcher1.getVelocity(); }

    public double getVelocity2() { return launcher2.getVelocity(); }

    public void setTargetVelocity(int targetVelocity) {
        this.targetVelocity = targetVelocity;
    }

    public void calculateTargetVelocity(double distance){
        targetVelocity = (0.0000014237*Math.pow(distance,4))-(0.000303373*Math.pow(distance,3))+(0.0297095*Math.pow(distance,2))+(1.67866*distance)+1134.53147;
    }

    public void calculateTargetAngle(double distance){
        if(distance > 75){
            targetCoverAngle = 0;
        }
        else {
            targetCoverAngle = -0.012064*(distance)+1.25891;
        }
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getTargetCoverAngle() {
        return targetCoverAngle;
    }

    public void updatePIDValues(double kS, double kV, double kP) {
        this.kS = kS;
        this.kV = kV;
        this.kP = kP;
    }

    public void update(double distance) {
        calculateTargetVelocity(distance);
        calculateTargetAngle(distance);

        cover.setPosition(targetCoverAngle);

        // Velocity Control
        double error = targetVelocity - getVelocity();

        // Feedforward
        double ff = kS * Math.signum(targetVelocity) + kV * targetVelocity;

        // Proportional correction
        double p = kP * error;

        double result = clamp(ff + p);

        launcher1.setPower(result);
        launcher2.setPower(result);
    }

    public void stopLauncher() {
        launcher1.setPower(0);
        launcher2.setPower(0);
    }

    private double clamp(double value) {
        return Math.max(-1.0, Math.min(1.0, value));
    }

    public void moveCover(double pos) {
        cover.setPosition(pos);
    }
}