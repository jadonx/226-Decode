package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.TeleOps.ScrimmageTeleOp.xValueBot;
import static org.firstinspires.ftc.teamcode.TeleOps.ScrimmageTeleOp.xValueGoal;
import static org.firstinspires.ftc.teamcode.TeleOps.ScrimmageTeleOp.yValueBot;
import static org.firstinspires.ftc.teamcode.TeleOps.ScrimmageTeleOp.yValueGoal;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;

public class Launcher {
    // LAUNCHER
    private DcMotorEx launcher1, launcher2;
    private double shooterSpeed = 1;

    // COVER
    private Servo cover;
    private double anglePos, angleDegree;

    // LIMELIGHT
    private Limelight3A limelight;

    private double distance = 0;
    double pinPointDistance = 0;

    private final double CAM_DEG = 29.78;
    private final double CAM_H = 12;
    private final double TARGET_H = 29;

    Pose2d BLUE_GOALPose = new Pose2d(xValueGoal, yValueGoal, 0);

    public Launcher(HardwareMap hardwareMap) {
        launcher1 = hardwareMap.get(DcMotorEx.class, Constants.HMMotorShooter1);
        launcher2 = hardwareMap.get(DcMotorEx.class, Constants.HMMotorShooter2);

        launcher1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        cover = hardwareMap.get(Servo.class, Constants.HMServobackSpin);

        limelight = hardwareMap.get(Limelight3A.class, Constants.HMLimelight);
        limelight.pipelineSwitch(1);
        limelight.start();
        Pose2d initialPose = new Pose2d(xValueBot, yValueBot, Math.toRadians(180));


    }

    // Returns [target velocity, target angle]
    public double[] getVelocityAndAngle(Pose2d currentPose) {
        double targetVelocity;
        double coverPos;
        calculateLimelightDistance();
        calculatePinPointDistance(currentPose);
        if(distance < 29.71 && distance > 29.70 ){
            distance = pinPointDistance - 9;
        }
        if (distance > 76){
            coverPos = 0.05;
            if (distance > 100){
                targetVelocity = 2800;
            }
            else {
                targetVelocity = calculateTargetVelocity();
            }
        }
        /*
        else if (distance < 29.70772 && distance > 29.7077){
            coverPos = 1;
            targetVelocity = 1400;
        }

         */
        else {
            coverPos = calculateCoverAngle();
            targetVelocity = calculateTargetVelocity();
        }

        return new double[] {targetVelocity, coverPos};
    }

    public void setVelocity(double targetVelocity) {
        launcher1.setVelocity(targetVelocity);
        launcher2.setVelocity(targetVelocity);
    }

    public void setCoverAngle(double targetAngle) {
        cover.setPosition(targetAngle);
    }

    public double getCurrentVelocity() {
        return launcher1.getVelocity();
    }

    public boolean atTargetVelocity(double targetVelocity) {
        return Math.abs(getCurrentVelocity() - targetVelocity) < 10;
    }

    public void stopLauncher() {
        // CHANGE TO VELOCITY
        launcher1.setVelocity(0);
        launcher2.setVelocity(0);
    }

    // CALCULATE DISTANCE USING LIMELIGHT
    private void calculateLimelightDistance() {
        LLResult result = limelight.getLatestResult();

        if(result != null){
            distance = getDistanceInches(result.getTy());
        }
    }

    private void calculatePinPointDistance(Pose2d currentPose){
        // Use currentPose instead of drive_roadrunner.localizer.getPose()
        pinPointDistance = Math.sqrt(
                Math.pow(BLUE_GOALPose.position.y - currentPose.position.y, 2) +
                        Math.pow(BLUE_GOALPose.position.x - currentPose.position.x, 2)
        );
    }

    public double getPinPointDistance(Pose2d currentPose){
        calculatePinPointDistance(currentPose);
        return pinPointDistance;
    }

    // CALCULATE TARGET VELOCITY
    private double calculateTargetVelocity() {
        if (distance < 80){
            return (0.0353161*(Math.pow(distance,2)))+(0.991859*distance)+1388.8866;
        } else if(distance > 130){
            return 2800;
        } else{
            return (-0.47286*Math.pow(distance,2))+(109.73693*distance)-3946.15385;
        }
    }

    // CALCULATE TARGET ANGLE
    private double calculateCoverAngle(){
        return (-0.0117319 * distance) + 0.956034;
    }

    // CALCULATE DISTANCE FROM BOT TO APRIL TAG
    private double getDistanceInches(double tyDegrees) {
        double totalAngleDeg = CAM_DEG + tyDegrees;
        double totalAngleRad = Math.toRadians(totalAngleDeg);

        return (TARGET_H - CAM_H) / Math.tan(totalAngleRad);
    }
}