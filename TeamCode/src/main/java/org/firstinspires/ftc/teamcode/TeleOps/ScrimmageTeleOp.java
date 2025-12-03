package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Commands.SpindexerColorIntakeCommand;
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

    public static double xValueGoal = -60;
    public static double yValueGoal = 60;
    public static double xValueBot = 0;
    public static double yValueBot = 0;

    public double pinPointDistance;

    FieldCentricDrive drive;
    MecanumDrive drive_roadrunner;
    Pose2d initialPose = new Pose2d(xValueBot, yValueBot, Math.toRadians(180));
    Pose2d BLUE_GOALPose = new Pose2d(xValueGoal, yValueGoal, 0);

    LimeLight limelight;

    Intake intake;
    Spindexer spindexer;
    public static double kP, kS;
    public static int slowingThreshold, stoppingThreshold;
    public static double slowingMultiplier;
    Popper popper;
    Launcher launcher;
    Turret turret;

    // COMMANDS
    LaunchArtifactCommand launchArtifactCommand;
    SpindexerColorIntakeCommand spindexerColorSensorIntakeCommand;

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
        kP = 0.005; kS = 0.12;
        slowingThreshold = 15; slowingMultiplier = 0.75; stoppingThreshold = 5;
        popper = new Popper(hardwareMap);
        launcher = new Launcher(hardwareMap);
        turret = new Turret(hardwareMap);

        spindexerColorSensorIntakeCommand = new SpindexerColorIntakeCommand(spindexer);
        spindexerColorSensorIntakeCommand.start();

        packet = new TelemetryPacket();
        dashboard = FtcDashboard.getInstance();

        spindexerColorSensorIntakeCommand = new SpindexerColorIntakeCommand(spindexer);
        spindexerColorSensorIntakeCommand.start();

        drive.resetIMU();
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
//        packet.put("Limelight tX: ", limelight.getTX());
//        packet.put("Limelight tY: ", limelight.getTY());
//        packet.put("LimeLight distance: ", limelight.getDistance());

        if (gamepad1.x) {
            drive.resetIMU();
        }

        if (gamepad1.right_trigger > 0.1) {
            // Canceling launch command sequence
            launchArtifactCommand = null;
            launcher.stopLauncher();
            popper.deactivatePopper();

            // Intake logic
            intake.runIntake(gamepad1.right_trigger);
        }
        else {
            intake.stopIntake();
        }

        // If launch sequence not engaged, set spindexer to intake mode
        if (launchArtifactCommand == null) {
            spindexerColorSensorIntakeCommand.update(telemetry);
        }

        // SPINDEXER LAUNCH LOGIC
        if (gamepad1.a) {
            launchArtifactCommand = new LaunchArtifactCommand(spindexer, popper, launcher, drive_roadrunner);
            launchArtifactCommand.start();
        }

        if (launchArtifactCommand != null && !launchArtifactCommand.isFinished()) {
            launchArtifactCommand.update(packet);
        }

        if (launchArtifactCommand != null && launchArtifactCommand.isFinished()) {
            launchArtifactCommand = null;
            launcher.stopLauncher();
            popper.deactivatePopper();

            // Reset holder statuses
            spindexerColorSensorIntakeCommand.resetHolderStatuses();
        }

        spindexer.updatePID(kP, kS, slowingThreshold, slowingMultiplier, stoppingThreshold);

        // TURRET LOGIC

        double ta = turret.angleBotToGoal(
                BLUE_GOALPose.position.y - drive_roadrunner.localizer.getPose().position.y,
                BLUE_GOALPose.position.x - drive_roadrunner.localizer.getPose().position.x,
                Math.toDegrees(drive_roadrunner.localizer.getPose().heading.log()));


        turret.setRobotHeading(Math.toDegrees(drive_roadrunner.localizer.getPose().heading.log()));

        if (gamepad1.dpad_down) {
            turret.zeroTurretRelativeToRobot();
        }

        if (limelight.getTX() != 0) {
            turret.trackAprilTag(limelight.getTX());
            packet.put("Turret Mode: ", "LimeLight");
        } else {
            turret.setPower(0);
        }

        // Calculate Theoretical Angle to Goal

        //Theoretical Angle Calculation
        drive_roadrunner.updatePoseEstimate();
        packet.put("Is resulted: ", limelight.isResulted());
        packet.put("LimeLight Tx: ", limelight.getTX());
        packet.put("1 Turret Angle: ", turret.getTurretAngle());
        packet.put("2 Desired Angle: ", ta);

        // packet.put("Bot Position X: ", drive_roadrunner.localizer.getPose().position.x);
        // packet.put("Bot Position Y: ", drive_roadrunner.localizer.getPose().position.y);
        packet.put("3 Bot Position Heading log: ", Math.toDegrees(drive_roadrunner.localizer.getPose().heading.log()));
        packet.put("4 Offset: ", turret.getTurretZeroOffsetField());
        packet.put("5 Robot Angle at init", turret.getRobotHeadingDeg());

        telemetry.addData("1 Turret Angle: ", turret.getTurretAngle());
        telemetry.addData("2 Desired Angle: ", ta);
        telemetry.addData("3 Bot Position Heading log: ", Math.toDegrees(drive_roadrunner.localizer.getPose().heading.log()));
        telemetry.addData("4 Offset: ", turret.getTurretZeroOffsetField());
        telemetry.addData("5 Robot Angle at init", turret.getRobotHeadingDeg());
        telemetry.addData("Launcher Actual Speed", launcher.getVelocity());
        telemetry.addData("Limelight Distance", limelight.getDistance());
        telemetry.addData("Pinpoint Distance", launcher.getPinPointDistance(drive_roadrunner.localizer.getPose())- 9);

        telemetry.update();

        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void stop() {
        intake.stopIntake();
        spindexer.stopSpindexer();
        drive.stopDrive();
        popper.deactivatePopper();
        launcher.stopLauncher();
        turret.stopTurret();
    }
}
