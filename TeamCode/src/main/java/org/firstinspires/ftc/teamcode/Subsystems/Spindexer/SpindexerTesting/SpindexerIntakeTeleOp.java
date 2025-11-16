package org.firstinspires.ftc.teamcode.Subsystems.Spindexer.SpindexerTesting;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer.LaunchArtifactCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer.Popper;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer.Spindexer;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer.UnjammerSystem;

@Config
@TeleOp(name="SpindexerIntakeTeleOp")
public class SpindexerIntakeTeleOp extends OpMode {
    FieldCentricDrive drive;

    Intake intake;
    Spindexer spindexer;
    UnjammerSystem unjamSystem;
    Popper popper;

    LaunchArtifactCommand launchArtifactCommand;

    public static double kP, kI, kD;

    TelemetryPacket packet;
    FtcDashboard dashboard;

    @Override
    public void init() {
        drive = new FieldCentricDrive(hardwareMap);

        intake = new Intake(hardwareMap);
        spindexer = new Spindexer(hardwareMap);
        unjamSystem = new UnjammerSystem(intake, spindexer);
        popper = new Popper(hardwareMap);

        packet = new TelemetryPacket();
        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void loop() {
        // DRIVE LOGIC
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        drive.drive(y, x, rx);
        packet.put("imu ", drive.getYaw());

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
            launchArtifactCommand = new LaunchArtifactCommand(spindexer, popper);
            launchArtifactCommand.start();
        }

        if (launchArtifactCommand != null && !launchArtifactCommand.isFinished()) {
            launchArtifactCommand.update(packet);
        }

        packet.put("spindexer current angle ", spindexer.getAngle());
        dashboard.sendTelemetryPacket(packet);

        // UPDATE SPINDEXER PID VALUES
        spindexer.updatePID(kP, kI, kD);
    }

    @Override
    public void stop() {
        unjamSystem.stopIntakeSpindexer();
        drive.stopDrive();
        popper.deactivatePopper();
    }
}
