package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Archived.BetterPinPoint;
import org.firstinspires.ftc.teamcode.Subsystems.FieldCentricDrive;

@TeleOp(name="BetterPinPointTest", group="Test")
public class BetterPinPointTest extends OpMode {
    BetterPinPoint betterPinPoint;
    FieldCentricDrive drive;

    @Override
    public void init() {
        betterPinPoint = new BetterPinPoint(hardwareMap, BetterPinPoint.AllianceColor.RED);
        drive = new FieldCentricDrive(hardwareMap);
    }

    @Override
    public void loop() {
        betterPinPoint.update();
        drive.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        if (gamepad1.xWasPressed()) {
            drive.resetIMU();
        }

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
