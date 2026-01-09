package org.firstinspires.ftc.teamcode.Autonomous.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.Commands.ColorIntakeCommand;
import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Subsystems.LimeLight;
import org.firstinspires.ftc.teamcode.Subsystems.PinPoint;
import org.firstinspires.ftc.teamcode.Subsystems.Popper;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

public class AutonomousActions {
    MecanumDrive drive;
    LimeLight limelight;
    Intake intake;
    Turret turret;
    Spindexer spindexer;
    Launcher launcher;
    Popper popper;

    ColorIntakeCommand colorIntakeCommand;

    public AutonomousActions(MecanumDrive drive, LimeLight limelight, Intake intake, Turret turret, Spindexer spindexer, Launcher launcher, Popper popper) {
        this.drive = drive;
        this.limelight = limelight;
        this.intake = intake;
        this.turret = turret;
        this.spindexer = spindexer;
        this.launcher = launcher;
        this.popper = popper;
    }

    public class UpdateLauncher implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                launcher.setTargetVelocity(1330);
                initialized = true;
            }

            launcher.update();

            return true;
        }
    }

    public Action updateLauncher() {
        return new UpdateLauncher();
    }
}
