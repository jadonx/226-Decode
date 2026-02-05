package org.firstinspires.ftc.teamcode.Autonomous.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.Commands.ColorIntakeCommand;
import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Subsystems.LimeLight;
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

    public static final Spindexer.HolderStatus[] GPP = new Spindexer.HolderStatus[] {Spindexer.HolderStatus.GREEN, Spindexer.HolderStatus.PURPLE, Spindexer.HolderStatus.PURPLE};
    public static final Spindexer.HolderStatus[] PGP = new Spindexer.HolderStatus[] {Spindexer.HolderStatus.PURPLE, Spindexer.HolderStatus.GREEN, Spindexer.HolderStatus.PURPLE};
    public static final Spindexer.HolderStatus[] PPG = new Spindexer.HolderStatus[] {Spindexer.HolderStatus.PURPLE, Spindexer.HolderStatus.PURPLE, Spindexer.HolderStatus.GREEN};

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
            PoseStorage.updatePose(drive.localizer.getPose().position.x, drive.localizer.getPose().position.y, Math.toDegrees(drive.localizer.getPose().heading.toDouble()));

            return true;
        }
    }
    public Action updateBotPosition() {return new UpdateBotPosition();}

    public class UpdateTurret implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            turret.update();
            return true;
        }
    }
    public Action updateTurret() {
        return new UpdateTurret();
    }

    public class SetTurretTarget implements Action {
        private double targetAngle;

        public SetTurretTarget(double targetAngle) {
            this.targetAngle = targetAngle;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            turret.setTarget(targetAngle);
            return false;
        }
    }
    public Action setTurretTarget(double targetAngle) {
        return new SetTurretTarget(targetAngle);
    }

    public class MoveCover implements Action {
        private double hoodPos;

        public MoveCover(double hoodPos) {
            this.hoodPos = hoodPos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            launcher.setTargetCoverAngle(hoodPos);
            return false;
        }
    }
    public Action moveCover(double hoodPos) {
        return new MoveCover(hoodPos);
    }

    public class UpdateLauncher implements Action {
        private boolean initialized = false;
        private int velocity;

        public UpdateLauncher(int velocity) {
            this.velocity = velocity;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                launcher.setTargetVelocity(velocity);
                initialized = true;
            }

            launcher.update();

            return true;
        }
    }
    public Action updateLauncher(int velocity) {
        return new UpdateLauncher(velocity);
    }

    public class AtLauncherTargetVelocity implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return !launcher.atTargetVelocity(20);
        }
    }
    public Action atLauncherTargetVelocity() {
        return new AtLauncherTargetVelocity();
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

    public class AutoColorIntakeCommand implements Action {
        private final ColorIntakeCommand colorIntakeCommand;

        private boolean initialized = false;

        public AutoColorIntakeCommand(ColorIntakeCommand colorIntakeCommand) {
            this.colorIntakeCommand = colorIntakeCommand;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                colorIntakeCommand.start();
                initialized = true;
            }
            colorIntakeCommand.update();

            return true;
        }
    }
    public Action autoColorIntakeCommand(ColorIntakeCommand colorIntakeCommand) {
        return new AutoColorIntakeCommand(colorIntakeCommand);
    }

    public class SpindexerFullRotation implements Action {
        private boolean initialized = false;
        private double speed;

        public SpindexerFullRotation(double speed) {
            this.speed = speed;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                spindexer.setMode(Spindexer.SpindexerMode.LAUNCH_MODE);
                spindexer.setSpeed(speed);
                initialized = true;
            }

            spindexer.update();

            return !spindexer.atTargetAngle(0);
        }
    }
    public Action spindexerFullRotation(double speed) {
        return new SpindexerFullRotation(speed);
    }

    public class SetSpindexerHolderStatuses implements Action {
        private Spindexer.HolderStatus[] holderStatuses;

        public SetSpindexerHolderStatuses(Spindexer.HolderStatus[] holderStatuses) {
            this.holderStatuses = holderStatuses;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            spindexer.setHolderStatus(0, holderStatuses[0]);
            spindexer.setHolderStatus(1, holderStatuses[1]);
            spindexer.setHolderStatus(2, holderStatuses[2]);
            return false;
        }
    }
    public Action setSpindexerHolderStatuses(Spindexer.HolderStatus[] holderStatuses) {
        return new SetSpindexerHolderStatuses(holderStatuses);
    }

    public class MoveToSortedPosition implements Action {
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                spindexer.setMode(Spindexer.SpindexerMode.INTAKE_MODE);
                spindexer.setTargetAngle(spindexer.getSortedPosition());
                initialized = true;
            }

            spindexer.update();
            return !spindexer.atTargetAngle(5);
        }
    }
    public Action moveToSortedPosition() {
        return new MoveToSortedPosition();
    }

    public class RunIntake implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.runIntake(0.7f);
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
