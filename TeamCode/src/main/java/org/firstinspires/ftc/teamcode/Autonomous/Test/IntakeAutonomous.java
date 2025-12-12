package org.firstinspires.ftc.teamcode.Autonomous.Test;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Commands.SpindexerColorIntakeCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Commands.LaunchArtifactCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Popper;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.Subsystems.UnjammerSystem;

@Config
@Autonomous(name = "IntakeAutonomous", group = "Autonomous")
public class IntakeAutonomous extends LinearOpMode {
    Intake intake;
    Spindexer spindexer;
    UnjammerSystem unjamSystem;
    Popper popper;
    Launcher launcher;

    LaunchArtifactCommand launchArtifactCommand;
    SpindexerColorIntakeCommand spindexerColorIntakeCommand;

    /*
    ACTIONS
     */
    public class ShootArtifacts implements Action {
        private final LaunchArtifactCommand launcherArtifactCommand;
        private boolean initialized = false;

        public ShootArtifacts(LaunchArtifactCommand launchArtifactCommand) {
            this.launcherArtifactCommand = launchArtifactCommand;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                launcherArtifactCommand.start();
                initialized = true;
            }

            // launcherArtifactCommand.update(telemetry);

            if (launcherArtifactCommand.isFinished()) {
                launcherArtifactCommand.stop();
                return false;
            }

            return true;
        }
    }
    public Action shootArtifacts(LaunchArtifactCommand launchArtifactCommand) {
        return new ShootArtifacts(launchArtifactCommand);
    }

    public class IntakeArtifacts implements Action {
        private final SpindexerColorIntakeCommand spindexerColorIntakeCommand;
        private final Intake intake;

        private boolean initialized = false;

        public IntakeArtifacts(SpindexerColorIntakeCommand spindexerColorIntakeCommand, Intake intake) {
            this.spindexerColorIntakeCommand = spindexerColorIntakeCommand;
            this.intake = intake;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                spindexerColorIntakeCommand.start();
                initialized = true;
            }

            intake.runIntake(1);
            spindexerColorIntakeCommand.update(telemetry);

            telemetry.update();

            return true;
        }
    }
    public Action intakeArtifacts(SpindexerColorIntakeCommand spindexerColorIntakeCommand, Intake intake) {
        return new IntakeArtifacts(spindexerColorIntakeCommand, intake);
    }

    public class StopIntakeSpindexer implements Action {
        private final Spindexer spindexer;
        private final Intake intake;

        public StopIntakeSpindexer(Spindexer spindexer, Intake intake) {
            this.spindexer = spindexer;
            this.intake = intake;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            spindexer.stopSpindexer();
            intake.stopIntake();
            return false;
        }
    }
    public Action stopIntakeSpindexer(Spindexer spindexer, Intake intake) {
        return new StopIntakeSpindexer(spindexer, intake);
    }

    public class GoToFirstColorHolder implements Action {
        private final Spindexer spindexer;
        private final Spindexer.HolderStatus[] motifPattern;

        private int targetAngle;

        public GoToFirstColorHolder(Spindexer spindexer, Spindexer.HolderStatus[] motifPattern) {
            this.spindexer = spindexer;
            this.motifPattern = motifPattern;

            targetAngle = spindexer.getLaunchPositionsColor(motifPattern)[0];
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            spindexer.goToAngle(targetAngle);

            if (spindexer.reachedTarget(spindexer.getWrappedAngle(), targetAngle)) {
                return false;
            }

            return true;
        }
    }
    public Action goToFirstColorHolder(Spindexer spindexer, Spindexer.HolderStatus[] motifPattern) {
        return new GoToFirstColorHolder(spindexer, motifPattern);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        /*
        SUBSYSTEMS AND COMMANDS
         */
        intake = new Intake(hardwareMap);
        spindexer = new Spindexer(hardwareMap);
        unjamSystem = new UnjammerSystem(intake, spindexer);
        popper = new Popper(hardwareMap);
        launcher = new Launcher(hardwareMap);

        /*
        TRAJECTORIES
         */

        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // Commands
        launchArtifactCommand = new LaunchArtifactCommand(spindexer, popper, launcher, drive);
        spindexerColorIntakeCommand = new SpindexerColorIntakeCommand(spindexer);

        // Trajectory Variables
        TrajectoryActionBuilder intakeBallPath = drive.actionBuilder(initialPose)
                .lineToX(40, new TranslationalVelConstraint(5));

        TrajectoryActionBuilder moveAfterIntake = intakeBallPath.endTrajectory().fresh()
                .lineToX(-20, new TranslationalVelConstraint(50.0));

        waitForStart();

        if (isStopRequested()) return;

        /*
        Actions.runBlocking(
                new SequentialAction(
                        intakeBallPath.build(),
                        moveAfterIntake.build()
                )
        );
        */

        Actions.runBlocking(
                new SequentialAction(
                        new RaceAction(
                                intakeBallPath.build(),
                                intakeArtifacts(spindexerColorIntakeCommand, intake)
                        ),
                        stopIntakeSpindexer(spindexer, intake),
                        moveAfterIntake.build()
                )
        );
    }
}
