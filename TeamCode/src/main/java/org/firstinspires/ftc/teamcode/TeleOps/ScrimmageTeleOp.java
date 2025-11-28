package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Commands.LaunchArtifactCommand;
import org.firstinspires.ftc.teamcode.Subsystems.LimeLight;
import org.firstinspires.ftc.teamcode.Subsystems.Popper;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Subsystems.UnjammerSystem;

@Config
@TeleOp(name="ScrimmageTeleOp", group="!TeleOp")
public class ScrimmageTeleOp extends OpMode {
    FieldCentricDrive drive;
    MecanumDrive drive_roadrunner;
    Pose2d initialPose = new Pose2d(0, 0, 0);
    Pose2d BLUE_GOALPose = new Pose2d(-50, 50, 0);

    LimeLight limelight;

    Intake intake;
    Spindexer spindexer;
    UnjammerSystem unjamSystem;
    Popper popper;
    Launcher launcher;
    Turret turret;
    LaunchArtifactCommand launchArtifactCommand;

    boolean isIntaking = false;
    boolean isUsingLL = false;

    TelemetryPacket packet;
    FtcDashboard dashboard;

    @Override
    public void init() {
        drive = new FieldCentricDrive(hardwareMap);

        //Roadrunner Initialization
        drive_roadrunner = new MecanumDrive(hardwareMap, initialPose);

        limelight = new LimeLight(hardwareMap);

        intake = new Intake(hardwareMap);
        spindexer = new Spindexer(hardwareMap);
        unjamSystem = new UnjammerSystem(intake, spindexer);
        popper = new Popper(hardwareMap);
        launcher = new Launcher(hardwareMap);
        turret = new Turret(hardwareMap);

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
        packet.put("IMU: ", drive.getYaw());

        //Limelight Logic
        limelight.getResult();
        packet.put("Limelight tX: ", limelight.getTX());
        packet.put("Limelight tY: ", limelight.getTY());
        packet.put("LimeLight distance: ", limelight.getDistance());

        if (gamepad1.x) {
            drive.resetIMU();
        }

        if (gamepad1.right_trigger > 0.1) {
            unjamSystem.periodic(gamepad1.right_trigger);
            launchArtifactCommand = null;
            launcher.stopLauncher();
            popper.deactivatePopper();
        }
        else {
            unjamSystem.stopIntakeSpindexer();
        }

        // SPINDEXER LAUNCH LOGIC
        if (gamepad1.a) {
            launchArtifactCommand = new LaunchArtifactCommand(spindexer, popper, launcher);
            // launchArtifactCommand.startPID();
            launchArtifactCommand.start();
        }

        // FAR LAUNCHING
        if (gamepad1.b) {
            launchArtifactCommand = new LaunchArtifactCommand(spindexer, popper, launcher);
            launchArtifactCommand.startFar();
        }

        if (launchArtifactCommand != null && !launchArtifactCommand.isFinished()) {
            // launchArtifactCommand.updatePID(packet);
            launchArtifactCommand.update(packet);
        }

        if (launchArtifactCommand != null && launchArtifactCommand.isFinished()) {
            launchArtifactCommand = null;
            launcher.stopLauncher();
            popper.deactivatePopper();
        }

        if(gamepad1.dpad_up && !isUsingLL) {
            isUsingLL = true;
            packet.put("Using Limelight for turret targeting", "");
        } else if (gamepad1.dpad_down && isUsingLL) {
            isUsingLL = false;
            packet.put("Using Angle Calculation for turret targeting", "");
        }

        //Turret Logic
        if (isUsingLL) {
            turret.trackAprilTag(limelight.getTX());
        } else {
            // turret.trackTargetAngle(turret.angleBotToGoal(BLUE_GOALPose.position.y - drive_roadrunner.localizer.getPose().position.y, BLUE_GOALPose.position.x - drive_roadrunner.localizer.getPose().position.x));
        }

        packet.put("Spindexer current angle: ", spindexer.getAngle());
        //Theoretical Angle Calculation
        drive_roadrunner.updatePoseEstimate();
        packet.put("Angle from bot to goal: ", turret.angleBotToGoal(BLUE_GOALPose.position.y - drive_roadrunner.localizer.getPose().position.y, BLUE_GOALPose.position.x - drive_roadrunner.localizer.getPose().position.x));
        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void stop() {
        unjamSystem.stopIntakeSpindexer();
        drive.stopDrive();
        popper.deactivatePopper();
        launcher.stopLauncher();
    }
}
