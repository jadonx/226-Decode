package org.firstinspires.ftc.teamcode.Testing.Spindexer;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.ColorIntakeCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;

@TeleOp(name="SpindexerSortingTest", group="Test")
public class SpindexerSortingTest extends OpMode {
    Spindexer spindexer;
    ColorIntakeCommand colorIntakeCommand;

    Intake intake;

    @Override
    public void init() {
        spindexer = new Spindexer(hardwareMap);
        colorIntakeCommand = new ColorIntakeCommand(spindexer);
        colorIntakeCommand.start();
        intake = new Intake(hardwareMap);
    }

    @Override
    public void loop() {
        colorIntakeCommand.update();

        intake.runIntake(gamepad1.right_trigger);

        if (gamepad1.dpadLeftWasPressed()) {
            spindexer.setMotifPattern(Spindexer.HolderStatus.GREEN, Spindexer.HolderStatus.PURPLE, Spindexer.HolderStatus.PURPLE);
        }
        if (gamepad1.dpadUpWasPressed()) {
            spindexer.setMotifPattern(Spindexer.HolderStatus.PURPLE, Spindexer.HolderStatus.GREEN, Spindexer.HolderStatus.PURPLE);
        }
        if (gamepad1.dpadRightWasPressed()) {
            spindexer.setMotifPattern(Spindexer.HolderStatus.PURPLE, Spindexer.HolderStatus.PURPLE, Spindexer.HolderStatus.GREEN);
        }

        if (gamepad1.aWasPressed()) {
            spindexer.setTargetAngle(spindexer.getSortedPosition());
        }

        telemetry.addData("Holder statuses ", spindexer.getHolderStatus(0) + " " + spindexer.getHolderStatus(1) + " " + spindexer.getHolderStatus(2));
        telemetry.addData("Pattern ", spindexer.getMotifPattern()[0] + " " + spindexer.getMotifPattern()[1] + " " + spindexer.getMotifPattern()[2]);
        telemetry.addData("Target angle ", spindexer.getTargetAngle());
        telemetry.update();
    }
}
