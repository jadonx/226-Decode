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
@TeleOp(name="Blue", group="!TeleOp")
public class BlueTeleOp extends OpMode {
    public double pinPointDistance;

    FieldCentricDrive drive;
    LimeLight limelight;
    public double motifID = -1;
    public double aprilTagID = -1;
    Intake intake;
    Spindexer spindexer;
    public static double kP, kD, kS;
    public static int slowingThreshold;
    public static double slowingMultiplier;
    public static double target;
    Popper popper;
    Launcher launcher;
    Turret turret;
    ElapsedTime offSetTurretTime = new ElapsedTime();
    public static double xValueGoal = -60;
    public static double yValueGoal = 60;
    public static double xValueBot = -2;
    public static double yValueBot = -40;
    public static double robotHeading = 120;
    boolean isUsingTurret = false;
    boolean prevDpadUp = false;
    MecanumDrive drive_roadrunner;
    Pose2d initialPose = new Pose2d(xValueBot, yValueBot, Math.toRadians(robotHeading));
    Pose2d BLUE_GOALPose = new Pose2d(xValueGoal, yValueGoal, 0);


    // COMMANDS
    LaunchArtifactCommand launchArtifactCommand;
    SpindexerColorIntakeCommand spindexerColorSensorIntakeCommand;

    TelemetryPacket packet;
    FtcDashboard dashboard;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        drive = new FieldCentricDrive(hardwareMap);

        limelight = new LimeLight(hardwareMap);

        intake = new Intake(hardwareMap);
        spindexer = new Spindexer(hardwareMap);

        popper = new Popper(hardwareMap);
        launcher = new Launcher(hardwareMap);
        turret = new Turret(hardwareMap);
        drive_roadrunner = new MecanumDrive(hardwareMap, initialPose);
        offSetTurretTime.reset();

        spindexerColorSensorIntakeCommand = new SpindexerColorIntakeCommand(spindexer);
        spindexerColorSensorIntakeCommand.start();

        packet = new TelemetryPacket();
        dashboard = FtcDashboard.getInstance();

        drive.resetIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        limelight();

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        if (gamepad1.left_bumper) {
            drive.driveSlow(y, x, rx);
        }
        else {
            drive.drive(y, x, rx);
        }

        if (gamepad1.x) {
            drive.resetIMU();
        }

        intake();

        colorSensorIntake();

        launchCommand();

        if (gamepad1.dpadDownWasPressed()) {
            launchArtifactCommand = null;
            spindexer.resetHolderStatuses();
            spindexerColorSensorIntakeCommand.start();
        }

        // spindexer.goToAngle(target);
        // spindexer.updatePIDValues(kP, kD, kS, slowingThreshold, slowingMultiplier);

        packet.put("current ", spindexer.getWrappedAngle());
        packet.put("target ", target);

        spindexerTelemetry();

        turret();

        telemetry.update();
        dashboard.sendTelemetryPacket(packet);
    }

    public void drive() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        drive.drive(y, x, rx);

        if (gamepad1.x) {
            drive.resetIMU();
        }
    }

    public void intake() {
        if (gamepad1.right_trigger > 0.1) {
            // Canceling launch command sequence
            launchArtifactCommand = null;
            launcher.stopLauncher();
            popper.deactivatePopper();

            // Intake logic
            intake.runIntake(gamepad1.right_trigger);
        } else if (gamepad1.left_trigger > 0.1) {
            intake.reverseIntake(gamepad1.left_trigger);
        } else {
            intake.stopIntake();
        }
    }

    public void launchCommand() {
        if (gamepad1.a) {
            launchArtifactCommand = new LaunchArtifactCommand(spindexer, popper, launcher, drive_roadrunner);
            launchArtifactCommand.start();
        }

        if (launchArtifactCommand != null && !launchArtifactCommand.isFinished()) {
            launchArtifactCommand.update(packet);
        }

        if (launchArtifactCommand != null && launchArtifactCommand.isFinished()) {
            launchArtifactCommand = null;
            spindexerColorSensorIntakeCommand.start();
            launcher.stopLauncher();
            popper.deactivatePopper();
        }
    }

    public void colorSensorIntake() {
        if (launchArtifactCommand == null) {
            spindexerColorSensorIntakeCommand.update(telemetry);
        }
    }

    public void spindexerTelemetry() {
        telemetry.addData("spindexer ", spindexer.getWrappedAngle());
        telemetry.addData("holder 1 ", spindexer.getHolderStatus()[0]);
        telemetry.addData("holder 2 ", spindexer.getHolderStatus()[1]);
        telemetry.addData("holder 3", spindexer.getHolderStatus()[2]);
        telemetry.addData("hue ", spindexer.getHSVRev()[0]);
    }

    public void turret() {
        // TURRET LOGIC
        drive_roadrunner.updatePoseEstimate();
        turret.setRobotHeading(Math.toDegrees(drive_roadrunner.localizer.getPose().heading.log()));
        double targetangle = turret.angleBotToGoal(
                BLUE_GOALPose.position.y - drive_roadrunner.localizer.getPose().position.y,
                BLUE_GOALPose.position.x - drive_roadrunner.localizer.getPose().position.x,
                Math.toDegrees(drive_roadrunner.localizer.getPose().heading.log()));

        if (offSetTurretTime.seconds() < 2) {
            turret.zeroTurretRelativeToRobot();
        }
        boolean current = gamepad1.dpad_up;
        if (current && !prevDpadUp) {
            isUsingTurret = !isUsingTurret;
        }
        prevDpadUp = current;
        if(turret.getTurretZeroOffsetField() != 0.0) {
            if (isUsingTurret) {
                if (limelight.isResulted()) {
                    if (limelight.getAprilTagID() == 20) {
                        turret.trackAprilTag(limelight.getTX());
                        packet.put("Turret Mode: ", "LimeLight");
                    }
                } else {
                    packet.put("Turret Mode: ", "PinPoint");
                    turret.trackTargetAngle(targetangle);
                }
                packet.put("Turret Mode: ", "ON");
            } else {
                turret.setPower(0);
                packet.put("Turret Mode: ", "OFF");
            }
        }

        packet.put("Turret Desired Angle: ", targetangle);
        packet.put("Turret Angle Raw: ", turret.getTurretAngleRaw());
        packet.put("Turret Angle: ", turret.getTurretAngle());

    }

    public void limelight() {
        limelight.getResult();
        aprilTagID = limelight.getAprilTagID();
        motifID = limelight.getMotifID();
        packet.put("AprilTag ID: ", limelight.getAprilTagID());
        packet.put("Motif ID: ", limelight.getMotifID());
    }

    @Override
    public void stop() {
        intake.stopIntake();
        spindexer.stopSpindexer();
        drive.stopDrive();
        popper.deactivatePopper();
        launcher.stopLauncher();
        turret.stopTurret();
        limelight.stopLimeLight();
    }
}