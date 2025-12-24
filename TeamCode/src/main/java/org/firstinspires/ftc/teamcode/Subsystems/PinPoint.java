package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;

@Config
public class PinPoint {
    MecanumDrive drive_roadrunner;

    public static double goal_x = 62.883723702017726;
    public static double goal_y = 66.55849186454233;

    public static int bot_heading = 90;

    Pose2d GOAL_RED = new Pose2d(goal_x, goal_y, Math.toRadians(0));
    Pose2d GOAL_BLUE = new Pose2d(36, 60, Math.toRadians(0));

    Pose2d GOAL_POSE;

    Pose2d BOT_RED = new Pose2d(0, 0, Math.toRadians(bot_heading));
    Pose2d BOT_BLUE = new Pose2d(0, 0, Math.toRadians(180));

    public enum AllianceColor {
        RED,
        BLUE,
    }

    public PinPoint(HardwareMap hardwareMap, AllianceColor allianceColor) {
        if (allianceColor == AllianceColor.RED) {
            GOAL_POSE = GOAL_RED;
            drive_roadrunner = new MecanumDrive(hardwareMap, BOT_RED);
        } else {
            GOAL_POSE = GOAL_BLUE;
            drive_roadrunner = new MecanumDrive(hardwareMap, BOT_BLUE);
        }
    }
    public double getDistanceToGoal() {
        Pose2d currentPose = drive_roadrunner.localizer.getPose();
        return Math.hypot(GOAL_POSE.position.x - currentPose.position.x, GOAL_POSE.position.y - currentPose.position.y);
    }

    public Pose2d getPose() {
        return drive_roadrunner.localizer.getPose();
    }

    public void updatePose() {
        drive_roadrunner.updatePoseEstimate();
    }

    public Pose2d getPoseGoal() {
        return GOAL_POSE;
    }

    public double getAngleToGoal() {
        Pose2d currentPose = drive_roadrunner.localizer.getPose();
        return Math.toDegrees(
                Math.atan2
                        (GOAL_POSE.position.y - currentPose.position.y,
                                GOAL_POSE.position.x - currentPose.position.x)
        );
    }
}
