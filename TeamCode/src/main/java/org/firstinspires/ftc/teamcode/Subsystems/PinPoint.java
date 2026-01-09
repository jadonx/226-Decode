package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;

@Config
public class PinPoint {
    GoBildaPinpointDriver pinpoint;

    public static int bot_heading = 90;

    //Remember to put X value to Y and Y value to X for Pose2D to work.
    Pose2D GOAL_RED = new Pose2D(DistanceUnit.INCH, 66.950425996555126, -67.60601854699804, AngleUnit.DEGREES, 0);
    Pose2D GOAL_BLUE = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);

    Pose2D GOAL_POSE;

    public enum AllianceColor {
        RED,
        BLUE,
    }

    public PinPoint(HardwareMap hardwareMap, AllianceColor allianceColor, double botPosX, double botPosY, double botHeading) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Constants.HMPinPointer);
        configurePinPoint();

        if (allianceColor == AllianceColor.RED) {
            GOAL_POSE = GOAL_RED;
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, botPosY, botPosX, AngleUnit.DEGREES, botHeading));
        } else {
            GOAL_POSE = GOAL_BLUE;
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, botPosY, botPosX, AngleUnit.DEGREES, botHeading));
        }
    }

    public void configurePinPoint() {
        pinpoint.setOffsets(-195.325, 2.671, DistanceUnit.MM);

        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        pinpoint.resetPosAndIMU();
    }

    public double getDistanceToGoal() {
        Pose2D currentPose = getPose();
        return Math.hypot(getXCoordinate(GOAL_POSE, DistanceUnit.INCH) - getXCoordinate(currentPose, DistanceUnit.INCH),
                getYCoordinate(GOAL_POSE, DistanceUnit.INCH) - getYCoordinate(currentPose, DistanceUnit.INCH));
    }


    public Pose2D getPoseGoal() {
        return GOAL_POSE;
    }

    public double getAngleToGoal() {
        Pose2D currentPose = getPose();
        return Math.toDegrees(
                Math.atan2
                        (getYCoordinate(GOAL_POSE, DistanceUnit.INCH) - getYCoordinate(currentPose, DistanceUnit.INCH),
                                getXCoordinate(GOAL_POSE, DistanceUnit.INCH) - getXCoordinate(currentPose, DistanceUnit.INCH))
        );
    }

    public void updatePose() {
        pinpoint.update();
    }

    public Pose2D getPose() {
        return pinpoint.getPosition();
    }

    public void setPose(Pose2D pose){
        if(pose!= null){
            pinpoint.setPosition(pose);
        }
    }

    public double getHeading() {
        return -1*getPose().getHeading(AngleUnit.DEGREES);
    }

    public double getXCoordinate(Pose2D pose, DistanceUnit unit) {
        return -1*pose.getY(unit);
    }

    public double getYCoordinate(Pose2D pose, DistanceUnit unit) {
        return pose.getX(unit);
    }
}
