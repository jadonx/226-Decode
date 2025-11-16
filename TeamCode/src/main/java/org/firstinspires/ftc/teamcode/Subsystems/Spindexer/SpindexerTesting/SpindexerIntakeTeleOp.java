package org.firstinspires.ftc.teamcode.Subsystems.Spindexer.SpindexerTesting;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer.LaunchArtifactCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer.Spindexer;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer.UnjammerSystem;

@TeleOp(name="SpindexerIntakeTeleOp")
public class SpindexerIntakeTeleOp extends OpMode {
    FieldCentricDrive drive;

    Intake intake;
    Spindexer spindexer;
    UnjammerSystem unjamSystem;

    LaunchArtifactCommand launchArtifactCommand;

    @Override
    public void init() {
        drive = new FieldCentricDrive(hardwareMap);

        intake = new Intake(hardwareMap);
        spindexer = new Spindexer(hardwareMap);
        unjamSystem = new UnjammerSystem(intake, spindexer);
    }

    @Override
    public void loop() {
        // DRIVE LOGIC
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        drive.drive(y, x, rx);
        telemetry.addData("imu ", drive.getYaw());

        if (gamepad1.x) {
            drive.resetIMU();
        }

        // INTAKE/SPINDEXER LOGIC
        if (gamepad1.right_trigger > 0.1) {
            unjamSystem.periodic(gamepad1.right_trigger);
        }
        else {
            unjamSystem.stopIntakeSpindexer();
        }

        // SPINDEXER LAUNCH LOGIC
        if (gamepad1.a) {
            launchArtifactCommand = new LaunchArtifactCommand(spindexer);
            launchArtifactCommand.start();
        }

        if (launchArtifactCommand != null && !launchArtifactCommand.isFinished()) {
            launchArtifactCommand.update(telemetry);
        }

        telemetry.addData("spindexer current angle ", spindexer.getAngle());
        telemetry.update();
    }
}
