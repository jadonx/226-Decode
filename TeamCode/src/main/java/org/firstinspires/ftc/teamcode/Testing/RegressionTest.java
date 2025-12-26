package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Subsystems.PinPoint;

@TeleOp(name="RegressionTest", group="Test")
public class RegressionTest extends OpMode {
    PinPoint pinpoint;
    Launcher launcher;
    FieldCentricDrive drive;

    @Override
    public void init() {
        pinpoint = new PinPoint(hardwareMap, PinPoint.AllianceColor.RED);
        launcher = new Launcher(hardwareMap);
        drive = new FieldCentricDrive(hardwareMap);
    }

    @Override
    public void loop() {
        drive.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        if (gamepad1.x) {
            drive.resetIMU();
        }

        double distance = pinpoint.getDistanceToGoal();

        telemetry.addData("location ", pinpoint.getPose());
        telemetry.addData("distance from goal ", distance);
        telemetry.addData("calculated velocity ", launcher.calculateVelocity(distance));
        telemetry.addData("calculated angle ", launcher.calculateAngle(distance));

        telemetry.update();
    }
}
