package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Constants;

@Config
@TeleOp(name="PinPointTest")
public class PinPointTest extends OpMode {
    GoBildaPinpointDriver pinpoint;

    public static double xOffset = -7.55;
    public static double yOffset = 0.25;

    @Override
    public void init() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Constants.HMPinPointer);
        pinpoint.setOffsets(xOffset, yOffset, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
    }

    @Override
    public void loop() {
        pinpoint.update();
        Pose2D pose2d = pinpoint.getPosition();

        telemetry.addData("x-value ", pose2d.getY(DistanceUnit.INCH));
        telemetry.addData("y-value ", pose2d.getX(DistanceUnit.INCH));

        telemetry.update();
    }
}
