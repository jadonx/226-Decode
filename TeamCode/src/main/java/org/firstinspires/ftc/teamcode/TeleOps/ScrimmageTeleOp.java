package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    // SUBSYSTEMS
    FieldCentricDrive drive;
    MecanumDrive drive_roadrunner;
    Pose2d initialPose = new Pose2d(0, 0, 0);
    Pose2d BLUE_GOALPose = new Pose2d(-50, 50, 0);

    LimeLight limelight;

    Intake intake;
    Spindexer spindexer;
    public static double kP, kD, kS;
    Popper popper;
    Launcher launcher;
    Turret turret;

    // COMMANDS
    LaunchArtifactCommand launchArtifactCommand;
    SpindexerColorIntakeCommand spindexerColorSensorIntakeCommand;

    boolean isIntaking = false;
    boolean isUsingLL = false;

    TelemetryPacket packet;
    FtcDashboard dashboard;

    ElapsedTime loopTime;

    @Override
    public void init() {
        drive = new FieldCentricDrive(hardwareMap);

        //Roadrunner Initialization
        drive_roadrunner = new MecanumDrive(hardwareMap, initialPose);

        limelight = new LimeLight(hardwareMap);

        intake = new Intake(hardwareMap);
        spindexer = new Spindexer(hardwareMap);
        popper = new Popper(hardwareMap);
        launcher = new Launcher(hardwareMap);
        turret = new Turret(hardwareMap);

        // COMMANDS
        spindexerColorSensorIntakeCommand = new SpindexerColorIntakeCommand(spindexer);
        spindexerColorSensorIntakeCommand.start();

        packet = new TelemetryPacket();
        dashboard = FtcDashboard.getInstance();

        // 0.0005
        kP = 0.009; kD = 0; kS = 0.12;

        loopTime = new ElapsedTime();
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

        /*
        OLD INTAKE LOGIC
        if (gamepad1.right_trigger > 0.1) {
            unjamSystem.periodic(gamepad1.right_trigger);
            launchArtifactCommand = null;
            launcher.stopLauncher();
            popper.deactivatePopper();
        }
        else {
            unjamSystem.stopIntakeSpindexer();
        }
         */

        /*
        NEW INTAKE LOGIC
         */
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

        /*
        AUTONOMOUS SPINDEXER INTAKE LOGIC
         */
        // If launch sequence not engaged, set spindexer to intake mode
        if (launchArtifactCommand == null) {
            spindexerColorSensorIntakeCommand.update(telemetry);
        }

        /*
        SPINDEXER LAUNCH LOGIC
         */
        if (gamepad1.a && launchArtifactCommand == null) {
            launchArtifactCommand = new LaunchArtifactCommand(spindexer, popper, launcher);
            launchArtifactCommand.start();
        }

        // FAR LAUNCHING
        if (gamepad1.b) {
            launchArtifactCommand = new LaunchArtifactCommand(spindexer, popper, launcher);
            launchArtifactCommand.startFar();
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

        // UPDATING SPINDEXER PID FOR TESTING
        packet.put("Spindexer current angle: ", spindexer.getAngle());
        spindexer.updatePID(kP, kD, kS);

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

        //Theoretical Angle Calculation
        drive_roadrunner.updatePoseEstimate();
        packet.put("Angle from bot to goal: ", turret.angleBotToGoal(BLUE_GOALPose.position.y - drive_roadrunner.localizer.getPose().position.y, BLUE_GOALPose.position.x - drive_roadrunner.localizer.getPose().position.x));

        telemetry.addData("LOOP TIME ", loopTime.seconds());
        loopTime.reset();

        // FTC Dashboard/Telemetry Update
        dashboard.sendTelemetryPacket(packet);
        telemetry.update();
    }

    @Override
    public void stop() {
        intake.stopIntake();
        spindexer.stopSpindexer();
        drive.stopDrive();
        popper.deactivatePopper();
        launcher.stopLauncher();
    }
}
