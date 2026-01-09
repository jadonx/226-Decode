package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;

public class RoadRunnerPinPoint {
    MecanumDrive drive_roadrunner;

    private final double RED_GOAL_X = -67; private final double RED_GOAL_Y = 67;
    private final double BLUE_GOAL_X = -67; private final double BLUE_GOAL_Y = -67;

    private double GOAL_POS_X;
    private double GOAL_POS_Y;

    public enum AllianceColor { RED, BLUE }
    AllianceColor allianceColor;

    public RoadRunnerPinPoint(HardwareMap hardwareMap, AllianceColor allianceColor, Pose2d startPose) {
        drive_roadrunner = new MecanumDrive(hardwareMap, startPose);
        this.allianceColor = allianceColor;

        if (allianceColor == AllianceColor.RED) {
            GOAL_POS_X = RED_GOAL_X; GOAL_POS_Y = RED_GOAL_Y;
        }
        else if (allianceColor == AllianceColor.BLUE) {
            GOAL_POS_X = BLUE_GOAL_X; GOAL_POS_Y = BLUE_GOAL_Y;
        }
    }

    public void updatePose() {
        drive_roadrunner.updatePoseEstimate();
    }

    public Pose2d getPose() {
        return drive_roadrunner.localizer.getPose();
    }

    public double getAngleToGoal() {
        Pose2d currentPose = getPose();

        // We do 90 plus because we found that it had to be flipped
        return 90 - Math.toDegrees(
                Math.atan2(
                        GOAL_POS_X - getPose().position.x,
                        GOAL_POS_Y - getPose().position.y
                )
        );
    }

    public double getDistanceToGoal() {
        Pose2d currentPose = getPose();
        return Math.hypot(GOAL_POS_X - currentPose.position.x, GOAL_POS_Y - currentPose.position.y);
    }
}
