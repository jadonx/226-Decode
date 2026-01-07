package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Constants;

public class BetterPinPoint {
    GoBildaPinpointDriver pinpoint;

    private final double RED_GOAL_X = 66.950425996555126; private final double RED_GOAL_Y = 67.60601854699804;
    private final double BLUE_GOAL_X = -66.950425996555126; private final double BLUE_GOAL_Y = 67.60601854699804;

    private double GOAL_POS_X;
    private double GOAL_POS_Y;

    public enum AllianceColor {
        RED,
        BLUE,
    }
    AllianceColor allianceColor;

    public BetterPinPoint(HardwareMap hardwareMap, AllianceColor allianceColor) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Constants.HMPinPointer);

        pinpoint.setOffsets(-195.325, 2.671, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();

        this.allianceColor = allianceColor;

        if (allianceColor == AllianceColor.RED) {
            GOAL_POS_X = RED_GOAL_X;
            GOAL_POS_Y = RED_GOAL_Y;
        }
        else if (allianceColor == AllianceColor.BLUE) {
            GOAL_POS_X = BLUE_GOAL_X;
            GOAL_POS_Y = BLUE_GOAL_Y;
        }
    }

    public void update() {
        pinpoint.update();
    }

    /**
     * Correct X points towards the goals
     * Correct-Y points towards the red goal
     * Pos-90 degrees points towards the red goal side
     * */
    public double getCorrectX() {
        return -getRawPos().getY(DistanceUnit.INCH);
    }

    public double getCorrectY() {
        return getRawPos().getX(DistanceUnit.INCH);
    }

    public double getCorrectHeading() {
        return -getRawPos().getHeading(AngleUnit.DEGREES);
    }

    public void setPosition(double x, double y, double heading) {
        // X and Y are flipped because of pinpoints weird conventions
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, y, -x, AngleUnit.DEGREES, -heading));
    }

    public double getAngleToGoal() {
        return Math.toDegrees(
                Math.atan2(
                        GOAL_POS_Y - getCorrectY(),
                        GOAL_POS_X - getCorrectX()
                )
        );
    }

    public double getDistanceToGoal() {
        return Math.hypot(GOAL_POS_X - getCorrectX(),GOAL_POS_Y - getCorrectY());
    }

    /** getRawPos returns the position with x and y swapped, don't use this one */
    public Pose2D getRawPos() {
        return pinpoint.getPosition();
    }
}
