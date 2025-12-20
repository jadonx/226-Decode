package org.firstinspires.ftc.teamcode.Subsystems;
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
    private LimeLight limelight2;

    private double distance = 0;
    double pinPointDistance = 0;

    private final double CAM_DEG = 29.78;
    private final double CAM_H = 12;
    private final double TARGET_H = 29;

    // Pose2d BLUE_GOALPose = new Pose2d(xValueGoal, yValueGoal, 0);

    public Launcher(HardwareMap hardwareMap) {
        launcher1 = hardwareMap.get(DcMotorEx.class, Constants.HMMotorShooter1);
        launcher2 = hardwareMap.get(DcMotorEx.class, Constants.HMMotorShooter2);

        launcher1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        cover = hardwareMap.get(Servo.class, Constants.HMServobackSpin);
    }
}