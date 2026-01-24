
package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants;
@Config
public class Launcher {
    // Hardware
    private DcMotorEx launcher1, launcher2;
    private Servo cover;

    // Launcher PID Values
    public static double kS = 0.12;     // Static friction feedforward
    public static double kV = 0.000395;  // Velocity feedforward (power per RPM)
    public static double kP = 0.005;   // Proportional gain on velocity error

    private double currentVelocity1;
    private double currentVelocity2;

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

    public void update() {
        currentVelocity1 = launcher1.getVelocity();
        currentVelocity2 = launcher2.getVelocity();

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

    public void calculateTargetVelocity(double distance){
        targetVelocity = (-0.00000380044*Math.pow(distance,4))+(0.00147547*Math.pow(distance,3))-(0.16748*Math.pow(distance,2))+(12.18982*distance)+896.3041;
    }

    public void calculateTargetAngle(double distance){
        if(distance > 110){
            targetCoverAngle = 0;
        }
        else {
            targetCoverAngle = 1.07204 / (1 + Math.pow(Math.E, -(-0.0689279 * distance + 4.59372)));
        }

        targetCoverAngle = Range.clip(targetCoverAngle, 0, 1);

        cover.setPosition(targetCoverAngle);
    }

    public void stopLauncher() {
        launcher1.setPower(0);
        launcher2.setPower(0);
    }

    public boolean atTargetVelocity(int threshold) {
        return Math.abs(getVelocity() - getTargetVelocity()) < threshold;
    }

    private double clamp(double value) {
        return Math.max(-1.0, Math.min(1.0, value));
    }

    /** SETTER AND GETTER METHODS */

    public double getVelocity() {
        return (currentVelocity1 + currentVelocity2) / 2.0;
    }

    public double getPower(){
        return (launcher1.getPower() + launcher2.getPower())/ 2.0;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getTargetCoverAngle() {
        return targetCoverAngle;
    }

    public void setTargetVelocity(int targetVelocity) {
        this.targetVelocity = targetVelocity;
    }

    public void setTargetCoverAngle(double targetCoverAngle) {
        this.targetCoverAngle = targetCoverAngle;
        cover.setPosition(targetCoverAngle);
    }
}
