package org.firstinspires.ftc.teamcode.Subsystems.Commands;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;

public class AutonomousIntakeCommand {
    private final Spindexer spindexer;
    private final Intake intake;

    private ElapsedTime timer;
    private int intakeTimeMilliseconds;

    /*
    TESTING POWER VALUES FOR INTAKE TIMING
     */
    private float spindexerPower;
    private float intakePower;

    private boolean isFinished = false;

    public AutonomousIntakeCommand (HardwareMap hardwareMap) {
        spindexer = new Spindexer(hardwareMap);
        intake = new Intake(hardwareMap);
    }

    public void start(float spindexerPower, float intakePower) {
        timer = new ElapsedTime();

        spindexer.runSpindexer(spindexerPower);
        intake.runIntake(intakePower);
    }

    public void update() {
        if (timer.milliseconds() > intakeTimeMilliseconds) {
            isFinished = true;
            spindexer.stopSpindexer();
            intake.stopIntake();
        }
    }

    public boolean isFinished() {
        return isFinished;
    }
}
