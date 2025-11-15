package org.firstinspires.ftc.teamcode.Subsystems.Spindexer;

import com.qualcomm.robotcore.util.ElapsedTime;

public class UnjammerSystem {
    private Intake intake;
    private Spindexer spindexer;

    private boolean isJammed;
    private double jamStartTime;
    private double unjamStartTime;
    private double lastAngle;

    private ElapsedTime timer;

    public UnjammerSystem(Intake intake, Spindexer spindexer) {
        this.intake = intake;
        this.spindexer = spindexer;

        timer = new ElapsedTime();
    }

    public void runIntakeSpindexer() {
        intake.runIntake();
        spindexer.runSpindexer();
    }

    public void periodic() {
        if (!isJammed) {
            runIntakeSpindexer();
        }

        // Checks whether intake is intaking and is not jammed
        if (intake.isIntaking() && !isJammed) {
            // Checks if spindexer is stuck
            if (Math.abs(spindexer.getAngle() - lastAngle) < 3) {
                if (jamStartTime == -1) {
                    jamStartTime = timer.milliseconds();
                }
                else if (timer.milliseconds() - jamStartTime > 100) {
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
            if (timer.milliseconds() - unjamStartTime > 250) {
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
