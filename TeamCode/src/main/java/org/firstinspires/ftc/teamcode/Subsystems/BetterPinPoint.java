package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Constants;

public class BetterPinPoint {
    GoBildaPinpointDriver pinpoint;

    public BetterPinPoint(HardwareMap hardwareMap) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Constants.HMPinPointer);

        pinpoint.setOffsets(-195.325, 2.671, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
    }

    public void update() {
        pinpoint.update();
    }

    /** getFilteredPos returns the position (x, y, heading)
     * Pos-Y points towards the goals
     * Pos-X points towards the red goal
     * Pos-90 degrees points towards the red goal side
     * */
    public Pose2D getFilteredPos() {
        Pose2D pose2D = getRawPos();
        double filteredX = -pose2D.getY(DistanceUnit.INCH);
        double filteredY = pose2D.getX(DistanceUnit.INCH);
        double filteredAngle = -pose2D.getHeading(AngleUnit.DEGREES);

        return new Pose2D(DistanceUnit.INCH, filteredX, filteredY, AngleUnit.DEGREES, filteredAngle);
    }

    public void setPosition(double x, double y, double heading) {
        // X and Y are flipped because of pinpoints weird conventions
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, y, -x, AngleUnit.DEGREES, -heading));
    }

    /** getRawPos returns the position with x and y swapped, don't use this one */
    public Pose2D getRawPos() {
        return pinpoint.getPosition();
    }
}
