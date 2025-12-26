package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Subsystems.NewPinPoint;
import org.firstinspires.ftc.teamcode.Subsystems.PinPoint;

@TeleOp(name="RegressionTest", group="Test")
public class RegressionTest extends OpMode {
    NewPinPoint pinpoint;
    Launcher launcher;
    FieldCentricDrive drive;

    @Override
    public void init() {
        pinpoint = new NewPinPoint(hardwareMap);
        launcher = new Launcher(hardwareMap);
        drive = new FieldCentricDrive(hardwareMap);
    }

    @Override
    public void loop() {
        drive.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        pinpoint.update();

        if (gamepad1.x) {
            drive.resetIMU();
        }

        telemetry.addData("x-value ", pinpoint.getPose().getX(DistanceUnit.INCH));
        telemetry.addData("y-value ", pinpoint.getPose().getY(DistanceUnit.INCH));
        telemetry.addData("rotation ", pinpoint.getPose().getHeading(AngleUnit.DEGREES));

        telemetry.update();
    }
}
