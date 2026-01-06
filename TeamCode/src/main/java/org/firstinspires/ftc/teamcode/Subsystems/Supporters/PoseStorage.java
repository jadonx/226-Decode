package org.firstinspires.ftc.teamcode.Subsystems.Supporters;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class PoseStorage {
    public static double xCoord;
    public static double yCoord;
    public static double heading;

    public PoseStorage(double x, double y, double head) {
        xCoord = x;
        yCoord = y;
        heading = head;
    }

    public static void updatePose(double x, double y, double head) {
        xCoord = x;
        yCoord = y;
        heading = head;
    }

    public static double getX() {
        return xCoord;
    }

    public static double getY() {
        return yCoord;
    }

    public static double getHeading() {
        return heading;
    }
}
