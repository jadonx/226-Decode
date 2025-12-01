package org.firstinspires.ftc.teamcode.Subsystems.Commands;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;

public class AutonomousIntakeCommand {
    private final Spindexer spindexer;
    private final Intake intake;

    private int intakeTimeMilliseconds;

    public AutonomousIntakeCommand (HardwareMap hardwareMap) {
        spindexer = new Spindexer(hardwareMap);
        intake = new Intake(hardwareMap);
    }

    public void update() {

    }
}
