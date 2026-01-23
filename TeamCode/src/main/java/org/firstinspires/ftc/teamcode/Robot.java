package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Commands.ColorIntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.LaunchCommand;
import org.firstinspires.ftc.teamcode.Subsystems.ColorSensor;
import org.firstinspires.ftc.teamcode.Subsystems.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Subsystems.LimeLight;
import org.firstinspires.ftc.teamcode.Subsystems.Popper;
import org.firstinspires.ftc.teamcode.Subsystems.RGBIndicator;
import org.firstinspires.ftc.teamcode.Subsystems.RoadRunnerPinPoint;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.Subsystems.Supporters.PoseStorage;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

import java.util.List;

public class Robot {
    private final FieldCentricDrive drive;
    private final Intake intake;
    private final Launcher launcher;
    private final Popper popper;
    private final Spindexer spindexer;
    private final ColorSensor colorSensor;
    private final Turret turret;
    private final LimeLight limelight;
    private final RGBIndicator light;

    private final RoadRunnerPinPoint pinpoint;

    private final Telemetry telemetry;

    private ColorIntakeCommand colorIntakeCommand;
    private LaunchCommand launchCommand;

    private int numLoops;
    private ElapsedTime loopTimer;

    private boolean isUsingTurret;

    private Gamepad gamepad1;
    private Gamepad gamepad2;

    // Bulk caching
    List<LynxModule> hubs;

    public Robot(HardwareMap hardwareMap, RoadRunnerPinPoint.AllianceColor allianceColor, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        drive = new FieldCentricDrive(hardwareMap);
        intake = new Intake(hardwareMap);
        launcher = new Launcher(hardwareMap);
        popper = new Popper(hardwareMap);
        spindexer = new Spindexer(hardwareMap);
        colorSensor = new ColorSensor(hardwareMap);
        turret = new Turret(hardwareMap);
        limelight = new LimeLight(hardwareMap, allianceColor);
        light = new RGBIndicator(hardwareMap);

        Pose2d startPose = new Pose2d(PoseStorage.getX(), PoseStorage.getY(), Math.toRadians(PoseStorage.getHeading()));
        pinpoint = new RoadRunnerPinPoint(hardwareMap, allianceColor, startPose);

        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;
        loopTimer = new ElapsedTime();

        hubs = hardwareMap.getAll(LynxModule.class);
    }

    public void start() {
        colorIntakeCommand = new ColorIntakeCommand(spindexer, colorSensor);
        colorIntakeCommand.start();

        launchCommand = null;

        loopTimer.reset();

        isUsingTurret = false;
    }

    public void update() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        updateDrive();
        updateLauncherCover();
        updatePinPoint();
        updateTurret();
        updateStoredPosition();
        updateIntake();
        updateLight();

        if (gamepad1.dpadLeftWasPressed()) {
            spindexer.toggleUnjam();
        }

        if (gamepad1.left_trigger > 0.1 && launchCommand != null) {
            stopLaunchCommand();
            colorIntakeCommand.start();
        }

        if (launchCommand == null) {
            if (gamepad1.aWasPressed() && launchCommand == null) {
                launchCommand = new LaunchCommand(spindexer, popper, launcher, pinpoint, intake);
                launchCommand.start();
            }
        }

        if (launchCommand == null || launchCommand.getCurrentState() == LaunchCommand.State.PRIME_SHOOTER) {
            colorIntakeCommand.update();
        }

        if (launchCommand != null) {
            if (gamepad1.bWasPressed()) {
                launchCommand.startShootingSequence();
            }

            if (!launchCommand.isFinished()) {
                launchCommand.update();
            }

            if (launchCommand.isFinished()) {
                colorIntakeCommand.start();
                stopLaunchCommand();
            }
        }

        numLoops++;
        telemetry.addData("Average Loop Times", ((double) loopTimer.milliseconds())/numLoops);
        updateTelemetry();
        telemetry.update();

        if (numLoops > 150) {
            numLoops = 0;
            loopTimer.reset();
        }
    }

    private void updateDrive() {
        drive.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        if (gamepad1.x) {
            drive.resetIMU();
        }
    }

    private void updateIntake() {
        if (gamepad1.right_trigger > 0.1) {
            intake.runIntake(gamepad1.right_trigger);
        }
        else if (gamepad1.left_trigger > 0.1) {
            intake.reverseIntake(gamepad1.left_trigger);
        }
        else {
            intake.stopIntake();
        }
    }

    private void updateLight() {
        if (spindexer.getHolderStatus(2) != Spindexer.HolderStatus.NONE) {
            light.setLightValue(0.5);
        }
        else if (spindexer.getHolderStatus(1) != Spindexer.HolderStatus.NONE) {
            light.setLightValue(0.37);
        }
        else if (spindexer.getHolderStatus(0) != Spindexer.HolderStatus.NONE) {
            light.setLightValue(0.28);
        }
        else {
            light.setLightValue(1);
        }
    }

    private void updateLauncherCover() {
        launcher.calculateTargetAngle(pinpoint.getDistanceToGoal());
    }

    private void updatePinPoint() {
        pinpoint.updatePose();
    }

    private void updateTurret() {
        double TURRET_MIN = -160.0;
        double TURRET_MAX = 160.0;
        double heading = turret.wrapDegRobot(Math.toDegrees(pinpoint.getPose().heading.toDouble()));
        double desired = turret.wrapDegRobot(pinpoint.getAngleToGoal());
        double turretRel = turret.deltaDeg(turret.getTurretAngle(), heading);
        double desiredRel = turret.deltaDeg(desired, heading);
        desiredRel = turret.clamp(desiredRel, TURRET_MIN, TURRET_MAX);
        double target = turret.wrapDegRobot(heading + desiredRel);

        turret.update();
        limelight.getResult();
        limelight.getAprilTagID();

        if (isUsingTurret) {
            if (Math.abs(turretRel) > 135) {
                turret.setMode(Turret.TurretMode.PINPOINT);
                turret.setTarget(heading);
                isUsingTurret = false;
            } else {
                if (limelight.isResulted() && limelight.isGoalTargeted()) {
                    turret.setMode(Turret.TurretMode.LIMELIGHT);
                    turret.setLimelightError(-limelight.getTX());
                } else {
                    turret.setMode(Turret.TurretMode.PINPOINT);
                    turret.setTarget(target);
                }
            }
        } else {
            turret.setMode(Turret.TurretMode.PINPOINT);
            turret.setTarget(heading);
        }

        if (gamepad1.rightBumperWasPressed() || gamepad1.dpadUpWasPressed()) {
            isUsingTurret = !isUsingTurret;
        }
    }

    private void updateStoredPosition() {
        Pose2d storedPose = pinpoint.getPose();
        PoseStorage.updatePose(storedPose.position.x, storedPose.position.y, Math.toDegrees(storedPose.heading.toDouble()));
    }

    private void updateTelemetry() {
//        telemetry.addData("Desired Angle", (90 - pinpoint.getAngleToGoal()));
//        telemetry.addData("Actual Angle", (turret.getTurretAngle()));
//        telemetry.addData("Robot Angle", (Math.abs(pinpoint.getPose().getHeading(AngleUnit.DEGREES))) - turret.getTurretAngle());
        // Spindexer
        telemetry.addData("Spindexer Mode ", spindexer.getMode());
        String holderStatuses = String.format("[%s, %s, %s]", spindexer.getHolderStatus(0), spindexer.getHolderStatus(1), spindexer.getHolderStatus(2));
        telemetry.addData("Spindexer Holders ", holderStatuses + "\n");

//        telemetry.addData("Spindexer wrapped pos ", spindexer.getWrappedAngle());
//        telemetry.addData("Spindexer unwrapped pos ", spindexer.getUnwrappedAngle());

//        telemetry.addData("Turret mode ", turret.getMode());
//        telemetry.addData("Turret target ", turret.getTarget());
//        telemetry.addData("Turret error ", turret.getError());

        // Launcher
//        telemetry.addData("Target velocity ", launcher.getTargetVelocity());
//        telemetry.addData("Current velocity ", launcher.getVelocity());
//        telemetry.addData("Current Power ", launcher.getPower());
//        telemetry.addData("Target cover angle ", launcher.getTargetCoverAngle() + "\n");
//
//        // Pinpoint
//        telemetry.addData("Pinpoint Position ", pinpoint.getPose().position.x + ", " + pinpoint.getPose().position.y);
//        telemetry.addData("Rotation ", Math.toDegrees(pinpoint.getPose().heading.toDouble()));
//        telemetry.addData("Goal Distance ", pinpoint.getDistanceToGoal() + "\n");

//        telemetry.addData("Desired Angle", pinpoint.getAngleToGoal());
//        telemetry.addData("Actual Angle", (turret.getTurretAngle()));
//
//        // Color intake command
//        telemetry.addData("Intake Command State ", colorIntakeCommand.getCurrentState() + "\n");
//
//        // Launch command
//        if (launchCommand != null) {
//            telemetry.addData("Launch Command State ", launchCommand.getCurrentState() + "\n");
//        }
//        else {
//            telemetry.addData("Launch Command State ", "Null \n");
//        }
    }

    private void stopLaunchCommand() {
        launchCommand = null;
        launcher.stopLauncher();
        popper.deactivatePopper();
    }

    public double getLauncherVel() {
        return launcher.getVelocity();
    }

    public double getLauncherTargetVel() {
        return launcher.getTargetVelocity();
    }

    public double getSpindexerWrapped() {
        return spindexer.getWrappedAngle();
    }

    public double getSpindexerUnwrapped() {
        return spindexer.getUnwrappedAngle();
    }
}