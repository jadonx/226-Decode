package org.firstinspires.ftc.teamcode.Autonomous.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.Autonomous.RedClose;
import org.firstinspires.ftc.teamcode.Autonomous.RedClosePath;
import org.firstinspires.ftc.teamcode.Commands.ColorIntakeCommand;
import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Subsystems.LimeLight;
import org.firstinspires.ftc.teamcode.Subsystems.PinPoint;
import org.firstinspires.ftc.teamcode.Subsystems.Popper;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.Subsystems.Supporters.PoseStorage;
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

    public AutonomousActions(MecanumDrive drive, LimeLight limelight, Intake intake, Turret turret, Spindexer spindexer, Launcher launcher, Popper popper, ColorIntakeCommand colorIntakeCommand) {
        this.drive = drive;
        this.limelight = limelight;
        this.intake = intake;
        this.turret = turret;
        this.spindexer = spindexer;
        this.launcher = launcher;
        this.popper = popper;
        this.colorIntakeCommand = colorIntakeCommand;
    }

    public class UpdateBotPosition implements Action {
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            PoseStorage.updatePose(drive.localizer.getPose().position.x, drive.localizer.getPose().position.y, drive.localizer.getPose().heading.toDouble());

            return true;
        }
    }
    public Action updateBotPosition() {return new UpdateBotPosition();}

    public class MoveCover implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            launcher.setTargetCoverAngle(0);
            return false;
        }
    }
    public Action moveCover() {
        return new MoveCover();
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

    public class RunPopper implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            popper.setTargetVelocity(1800);
            return false;
        }
    }
    public Action runPopper() {
        return new RunPopper();
    }

    public class PushInPopper implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            popper.pushInPopper();
            return false;
        }
    }
    public Action pushInPopper() {
        return new PushInPopper();
    }

    public class DeactivatePopper implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            popper.deactivatePopper();
            return false;
        }
    }
    public Action deactivatePopper() {
        return new DeactivatePopper();
    }

    public class SpindexerFullRotation implements Action {
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                spindexer.setMode(Spindexer.SpindexerMode.LAUNCH_MODE);
                spindexer.setSpeed(0.2);
                initialized = true;
            }

            spindexer.update();

            return !spindexer.atTargetAngle(0);
        }
    }
    public Action spindexerFullRotation() {
        return new SpindexerFullRotation();
    }

    public class MoveToSortedPosition implements Action {
        private final Spindexer.HolderStatus[] motifPattern;
        private boolean initialized = false;

        public MoveToSortedPosition(Spindexer.HolderStatus[] motifPattern) {
            this.motifPattern = motifPattern;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                spindexer.setMode(Spindexer.SpindexerMode.INTAKE_MODE);
                spindexer.setTargetAngle(spindexer.getSortedPosition(motifPattern));
                initialized = true;
            }

            spindexer.update();
            return !spindexer.atTargetAngle(10);
        }
    }
    public Action moveToSortedPosition(Spindexer.HolderStatus[] motifPattern) {
        return new MoveToSortedPosition(motifPattern);
    }

    public class RunIntake implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.runIntake(1);
            return false;
        }
    }
    public Action runIntake() {
        return new RunIntake();
    }

    public class StopSpindexer implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            spindexer.setPower(0);
            return false;
        }
    }
    public Action stopSpindexer() {
        return new StopSpindexer();
    }

    public class StopIntake implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.stopIntake();
            return false;
        }
    }
    public Action stopIntake() {
        return new StopIntake();
    }
}
