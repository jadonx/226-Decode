package org.firstinspires.ftc.teamcode.Subsystems.Spindexer;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.FieldCentricDrive;

@TeleOp(name="Drive Unjam Test")
public class DriveUnjamTeleOp extends OpMode {
    FieldCentricDrive drive;

    Intake intake;
    Spindexer spindexer;
    UnjammerSystem unjamSystem;

    @Override
    public void init() {
        drive = new FieldCentricDrive(hardwareMap);

        intake = new Intake(hardwareMap);
        spindexer = new Spindexer(hardwareMap);
        unjamSystem = new UnjammerSystem(intake, spindexer);
    }

    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        drive.drive(y, x, rx);

        if (gamepad1.a) {
            drive.resetIMU();
        }

        if (gamepad1.right_trigger > 0.5) {
            unjamSystem.periodic();
        }

        telemetry.addData("imu ", drive.getYaw());
    }
}
