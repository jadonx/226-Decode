package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.BetterPinPoint;

@TeleOp(name="BetterPinPointTest", group="Test")
public class BetterPinPointTest extends OpMode {
    BetterPinPoint betterPinPoint;

    @Override
    public void init() {
        betterPinPoint = new BetterPinPoint(hardwareMap, BetterPinPoint.AllianceColor.RED);
    }

    @Override
    public void loop() {
        betterPinPoint.update();

        telemetry.addData("X ", betterPinPoint.getCorrectX());
        telemetry.addData("Y ", betterPinPoint.getCorrectY());
        telemetry.addData("Heading ", betterPinPoint.getCorrectHeading());

        if (gamepad1.aWasPressed()) {
            betterPinPoint.setPosition(6,7, 67);
        }

        telemetry.addData("Goal Distance ", betterPinPoint.getDistanceToGoal());
        telemetry.addData("Angle to Goal ", betterPinPoint.getAngleToGoal());
        telemetry.addData("Angle to Goal - 90 ", 90 - betterPinPoint.getAngleToGoal());

        telemetry.update();
    }
}
