package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Commands.ColorIntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.LaunchCommand;
import org.firstinspires.ftc.teamcode.Subsystems.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Subsystems.PinPoint;
import org.firstinspires.ftc.teamcode.Subsystems.Popper;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

public class Robot {
    private final FieldCentricDrive drive;
    private final Intake intake;
    private final Launcher launcher;
    private final Popper popper;
    private final Spindexer spindexer;
    private final Turret turret;
    private final PinPoint pinpoint;

    private ColorIntakeCommand colorIntakeCommand;
    private LaunchCommand launchCommand;

    private Gamepad gamepad1;

    private enum RobotState {
        INTAKE_STATE,
        LAUNCH_STATE
    }
    private RobotState robotState;

    public Robot(HardwareMap hardwareMap, PinPoint.AllianceColor allianceColor, Gamepad gamepad1) {
        drive = new FieldCentricDrive(hardwareMap);
        intake = new Intake(hardwareMap);
        launcher = new Launcher(hardwareMap);
        popper = new Popper(hardwareMap);
        spindexer = new Spindexer(hardwareMap);
        turret = new Turret(hardwareMap);
        pinpoint = new PinPoint(hardwareMap, allianceColor);

        this.gamepad1 = gamepad1;
    }

    public void start() {
        robotState = RobotState.INTAKE_STATE;

        colorIntakeCommand = new ColorIntakeCommand(spindexer);
        colorIntakeCommand.start();

        launchCommand = null;
    }

    public void update() {
        updateDrive();
        updateIntake();

        if (launchCommand == null) {
            colorIntakeCommand.update();

            if (gamepad1.a && launchCommand == null) {
                launchCommand = new LaunchCommand(spindexer, popper, launcher, pinpoint);
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
    }

    private void updateDrive() {
        drive.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        if (gamepad1.x) {
            drive.resetIMU();
        }
    }

    private void updateIntake() {
        if (gamepad1.right_trigger > 0.1) {
            stopLaunchCommand();
            intake.runIntake(gamepad1.right_trigger);
        }
        else if (gamepad1.left_trigger > 0.1) {
            stopLaunchCommand();
            intake.reverseIntake(gamepad1.left_trigger);
        }
        else {
            intake.stopIntake();
        }
    }

    private void stopLaunchCommand() {
        launchCommand = null;
        launcher.stopLauncher();
        launcher.setTargetCoverAngle(0.5);
        popper.deactivatePopper();
    }
}
