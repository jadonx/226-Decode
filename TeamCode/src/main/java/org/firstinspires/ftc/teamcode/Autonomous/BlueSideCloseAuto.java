package org.firstinspires.ftc.teamcode.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
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
@Autonomous(name = "BlueSideCloseAuto", group = "Autonomous")
public class BlueSideCloseAuto extends LinearOpMode {
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

            launcherArtifactCommand.update(telemetryPacket);

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
            spindexerColorIntakeCommand.update(telemetryPacket);

            return true;
        }
    }
    public Action intakeArtifacts(SpindexerColorIntakeCommand spindexerColorIntakeCommand, Intake intake) {
        return new IntakeArtifacts(spindexerColorIntakeCommand, intake);
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

        Pose2d initialPose = new Pose2d(-48, -48, Math.toRadians(225));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // Commands
        launchArtifactCommand = new LaunchArtifactCommand(spindexer, popper, launcher, drive);
        spindexerColorIntakeCommand = new SpindexerColorIntakeCommand(spindexer);

        // Trajectory Variables
        TrajectoryActionBuilder moveToShootArtifacts = drive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(45))
                .lineToY(-20);

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        moveToShootArtifacts.build(),
                        shootArtifacts(launchArtifactCommand)
                )
        );
    }
}
