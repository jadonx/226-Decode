package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.ColorIntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.LaunchCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Subsystems.PinPoint;
import org.firstinspires.ftc.teamcode.Subsystems.Popper;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;

@Disabled
@TeleOp(name="CommandTest", group="Test")
public class CommandTest extends OpMode {
    ColorIntakeCommand colorIntakeCommand;
    LaunchCommand launchCommand;

    Spindexer spindexer;
    Popper popper;
    Launcher launcher;
    PinPoint pinpoint;
    Intake intake;

    @Override
    public void init() {
        spindexer = new Spindexer(hardwareMap);
        popper = new Popper(hardwareMap);
        launcher = new Launcher(hardwareMap);
        pinpoint = new PinPoint(hardwareMap, PinPoint.AllianceColor.RED, 0, 0, 0);
        intake = new Intake(hardwareMap);

        colorIntakeCommand = new ColorIntakeCommand(spindexer);
        colorIntakeCommand.start();
    }

    @Override
    public void loop() {
        if (gamepad1.right_trigger > 0.1) {
            // Canceling launch command sequence
            launchCommand = null;
            launcher.stopLauncher();
            popper.deactivatePopper();

            // Intake logic
            intake.runIntake(gamepad1.right_trigger);
        } else if (gamepad1.left_trigger > 0.1) {
            intake.reverseIntake(gamepad1.left_trigger);
        } else {
            intake.stopIntake();
        }

        if (launchCommand == null) {
            colorIntakeCommand.update();
        }

        if (gamepad1.aWasPressed() && launchCommand == null) {
            // launchCommand = new LaunchCommand(spindexer, popper, launcher, pinpoint);
            // launchCommand.start();
        }

        if (launchCommand != null && !launchCommand.isFinished()) {
            launchCommand.update();
        }

        if (launchCommand != null && launchCommand.isFinished()) {
            launchCommand = null;
            colorIntakeCommand.start();
            launcher.stopLauncher();
            popper.deactivatePopper();
        }
    }
}
