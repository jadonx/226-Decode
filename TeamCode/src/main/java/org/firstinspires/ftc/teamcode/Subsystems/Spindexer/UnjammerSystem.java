package org.firstinspires.ftc.teamcode.Subsystems.Spindexer;

import com.qualcomm.robotcore.util.ElapsedTime;

public class UnjammerSystem {
    private final Intake intake;
    private final Spindexer spindexer;

    private boolean isJammed;
    private double jamStartTime = -1;
    private double unjamStartTime = -1;
    private double lastAngle;

    private ElapsedTime timer;

    public UnjammerSystem(Intake intake, Spindexer spindexer) {
        this.intake = intake;
        this.spindexer = spindexer;

        timer = new ElapsedTime();
    }

    public void runIntakeSpindexer(float intakePower) {
        intake.runIntake(intakePower);
        spindexer.runSpindexer();
    }

    public void stopIntakeSpindexer() {
        intake.stopIntake();
        spindexer.stopSpindexer();
    }

    public void periodic(float intakePower) {

        if (!isJammed) {
            runIntakeSpindexer(intakePower);
        }

        runIntakeSpindexer(intakePower);


        // Checks whether intake is intaking and is not jammed
        if (intake.isIntaking() && !isJammed) {
            // Checks if spindexer is stuck
            if (Math.abs(spindexer.getAngle() - lastAngle) < 2) {
                if (jamStartTime == -1) {
                    jamStartTime = timer.milliseconds();
                }
                else if (timer.milliseconds() - jamStartTime > 150) {
                    isJammed = true;
                }
            }
            else {
                jamStartTime = -1;
            }
        }

        // Checks if jammed, then run unjam logic
        if (isJammed) {
            spindexer.unjamSpindexer();
            intake.unjamIntake();
            if (unjamStartTime == -1) {
                unjamStartTime = timer.milliseconds();
            }
            if (timer.milliseconds() - unjamStartTime > 120) {
                isJammed = false;
                jamStartTime = -1;
                unjamStartTime = -1;
            }
        }

        if (!isJammed) {
            lastAngle = spindexer.getAngle();
        }
    }
}