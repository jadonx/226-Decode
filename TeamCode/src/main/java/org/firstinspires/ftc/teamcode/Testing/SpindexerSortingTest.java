package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.ColorIntakeCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;

@TeleOp(name="SpindexerSortingTest", group="Test")
public class SpindexerSortingTest extends OpMode {
    Spindexer spindexer;
    ColorIntakeCommand colorIntakeCommand;
    Spindexer.HolderStatus[] motifPattern;

    Intake intake;

    @Override
    public void init() {
        spindexer = new Spindexer(hardwareMap);
        colorIntakeCommand = new ColorIntakeCommand(spindexer);
        colorIntakeCommand.start();
        intake = new Intake(hardwareMap);
        motifPattern = new Spindexer.HolderStatus[] {Spindexer.HolderStatus.NONE, Spindexer.HolderStatus.NONE, Spindexer.HolderStatus.NONE};
    }

    @Override
    public void loop() {
        colorIntakeCommand.update();

        intake.runIntake(gamepad1.right_trigger);

        if (gamepad1.dpadLeftWasPressed()) {
            motifPattern[0] = Spindexer.HolderStatus.GREEN;
            motifPattern[1] = Spindexer.HolderStatus.PURPLE;
            motifPattern[2] = Spindexer.HolderStatus.PURPLE;
        }
        if (gamepad1.dpadUpWasPressed()) {
            motifPattern[0] = Spindexer.HolderStatus.PURPLE;
            motifPattern[1] = Spindexer.HolderStatus.GREEN;
            motifPattern[2] = Spindexer.HolderStatus.PURPLE;
        }
        if (gamepad1.dpadRightWasPressed()) {
            motifPattern[0] = Spindexer.HolderStatus.PURPLE;
            motifPattern[1] = Spindexer.HolderStatus.PURPLE;
            motifPattern[2] = Spindexer.HolderStatus.GREEN;
        }

        if (gamepad1.aWasPressed()) {
            spindexer.setTargetAngle(spindexer.getSortedPosition(motifPattern));
        }

        telemetry.addData("Pattern ", motifPattern[0] + " " + motifPattern[1] + " " + motifPattern[2]);
        telemetry.addData("Target angle ", spindexer.getTargetAngle());
        telemetry.update();
    }
}
