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
        updateIntake();

        switch (robotState) {
            case INTAKE_STATE:
                break;
            case LAUNCH_STATE:
                break;
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

    private void stopLaunchCommand() {
        launchCommand = null;
        launcher.stopLauncher();
        popper.deactivatePopper();
    }
}
