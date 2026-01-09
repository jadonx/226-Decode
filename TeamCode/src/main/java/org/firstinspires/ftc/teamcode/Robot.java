package org.firstinspires.ftc.teamcode;

import androidx.xr.runtime.math.Pose;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Commands.ColorIntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.LaunchCommand;
import org.firstinspires.ftc.teamcode.Subsystems.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Subsystems.PinPoint;
import org.firstinspires.ftc.teamcode.Subsystems.Popper;
import org.firstinspires.ftc.teamcode.Subsystems.RoadRunnerPinPoint;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.Subsystems.Supporters.PoseStorage;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

public class Robot {
    private final FieldCentricDrive drive;
    private final Intake intake;
    private final Launcher launcher;
    private final Popper popper;
    private final Spindexer spindexer;
    private final Turret turret;

    private final RoadRunnerPinPoint pinpoint;

    private final Telemetry telemetry;

    private boolean isUsingTurret;

    private ColorIntakeCommand colorIntakeCommand;
    private LaunchCommand launchCommand;

    private Gamepad gamepad1;

    public Robot(HardwareMap hardwareMap, RoadRunnerPinPoint.AllianceColor allianceColor, Gamepad gamepad1, Telemetry telemetry) {
        drive = new FieldCentricDrive(hardwareMap);
        intake = new Intake(hardwareMap);
        launcher = new Launcher(hardwareMap);
        popper = new Popper(hardwareMap);
        spindexer = new Spindexer(hardwareMap);
        turret = new Turret(hardwareMap);

        Pose2d startPose = new Pose2d(PoseStorage.getX(), PoseStorage.getY(), PoseStorage.getHeading());
        pinpoint = new RoadRunnerPinPoint(hardwareMap, allianceColor, startPose);

        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;
    }

    public void start() {
        isUsingTurret = false;

        colorIntakeCommand = new ColorIntakeCommand(spindexer);
        colorIntakeCommand.start();

        launchCommand = null;
    }

    public void update() {
        updateDrive();
        updateLauncherCover();
        updatePinPoint();
        updateTurret();
        updateTelemetry();

        if (gamepad1.right_trigger > 0.1) {
            stopLaunchCommand();
        }

        if (launchCommand == null) {
            colorIntakeCommand.update();
            updateIntake();

            if (gamepad1.a && launchCommand == null) {
                launchCommand = new LaunchCommand(spindexer, popper, launcher, pinpoint, intake);
                launchCommand.start();
            }
        }

        if (launchCommand != null) {
            if (!launchCommand.isFinished()) {
                launchCommand.update();
            }

            if (launchCommand.isFinished()) {
                colorIntakeCommand.start();
                stopLaunchCommand();
            }
        }

        if (gamepad1.bWasPressed()) {
            spindexer.resetHolderStatuses();
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

    private void updateLauncherCover() {
        launcher.calculateTargetAngle(pinpoint.getDistanceToGoal());
    }

    private void updatePinPoint() {
        pinpoint.updatePose();
    }

    private void updateTurret() {
        if (isUsingTurret) {
            double desired = pinpoint.getAngleToGoal();
            turret.goToAngle(desired);
        } else {
            turret.goToAngle(Math.toDegrees(pinpoint.getPose().heading.toDouble()));
        }

        if (gamepad1.dpadUpWasPressed()) {
            isUsingTurret = !isUsingTurret;
        }
    }

    private void updateTelemetry() {
        // Spindexer
//        telemetry.addData("Spindexer Mode ", spindexer.getMode());
//        String holderStatuses = String.format("[%s, %s, %s]", spindexer.getHolderStatus(0), spindexer.getHolderStatus(1), spindexer.getHolderStatus(2));
//        telemetry.addData("Spindexer Holders ", holderStatuses + "\n");

        // Launcher
        telemetry.addData("Target velocity ", launcher.getTargetVelocity());
        telemetry.addData("Current velocity ", launcher.getVelocity());
        telemetry.addData("Current Power ", launcher.getPower());
        telemetry.addData("Target cover angle ", launcher.getTargetCoverAngle() + "\n");

        // Pinpoint
        telemetry.addData("Pinpoint Position ", pinpoint.getPose().position.x + ", " + pinpoint.getPose().position.y);
        telemetry.addData("Rotation ", Math.toDegrees(pinpoint.getPose().heading.toDouble()));
        telemetry.addData("Goal Distance ", pinpoint.getDistanceToGoal() + "\n");

        telemetry.addData("Desired Angle", pinpoint.getAngleToGoal());
        telemetry.addData("Actual Angle", (turret.getTurretAngle()));

        // Color intake command
        telemetry.addData("Intake Command State ", colorIntakeCommand.getCurrentState() + "\n");

        // Launch command
        if (launchCommand != null) {
            telemetry.addData("Launch Command State ", launchCommand.getCurrentState() + "\n");
        }
        else {
            telemetry.addData("Launch Command State ", "Null \n");
        }

        telemetry.update();
    }

    private void stopLaunchCommand() {
        launchCommand = null;
        launcher.stopLauncher();
        popper.deactivatePopper();
    }
}