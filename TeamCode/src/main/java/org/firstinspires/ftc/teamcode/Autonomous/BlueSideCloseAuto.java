package org.firstinspires.ftc.teamcode.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous.Test.IntakeAutonomous;
import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Commands.SpindexerColorIntakeCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Commands.LaunchArtifactCommand;
import org.firstinspires.ftc.teamcode.Subsystems.LimeLight;
import org.firstinspires.ftc.teamcode.Subsystems.Popper;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Subsystems.UnjammerSystem;

@Config
@Autonomous(name = "BlueSideCloseAuto", group = "Autonomous")
public class BlueSideCloseAuto extends LinearOpMode {
    MecanumDrive drive;
    Intake intake;
    Spindexer spindexer;
    UnjammerSystem unjamSystem;
    Popper popper;
    Launcher launcher;
    Turret turret;
    LimeLight limeLight;

    LaunchArtifactCommand launchArtifactCommand;
    SpindexerColorIntakeCommand spindexerColorIntakeCommand;

    public double aprilTagID = -1;
    public double motifID = -1;
    public Spindexer.HolderStatus[] motifPattern;

    /*
    ACTIONS
     */
    public class GetMotifAndAimToAprilTag implements Action {
        private int goalTagID = 20;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            limeLight.getResult();
            if (motifID != -1) {
                motifPattern = limeLight.getMotif();
            }
            motifID = limeLight.getMotifID();
            aprilTagID = limeLight.getAprilTagID();
            telemetryPacket.put("Motif ID", motifID);
            telemetryPacket.put("April Tag ID", aprilTagID);
            if (limeLight.isResulted() && aprilTagID == goalTagID) {
                turret.trackAprilTag(limeLight.getTX());
            } else {
                turret.trackTargetAngleAuto(65);
            }

            return true;
        }
    }
    public Action turretGetMotifAndAimAprilTag() {
        return new GetMotifAndAimToAprilTag();
    }

    public class SpindexerColorIntake implements Action {
        private final SpindexerColorIntakeCommand spindexerColorIntakeCommand;

        private boolean initialized = false;

        public SpindexerColorIntake(SpindexerColorIntakeCommand spindexerColorIntakeCommand) {
            this.spindexerColorIntakeCommand = spindexerColorIntakeCommand;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                spindexerColorIntakeCommand.start();
                initialized = true;
            }
            spindexerColorIntakeCommand.update(telemetry);

            telemetry.update();

            return true;
        }
    }
    public Action spindexerColorIntake(SpindexerColorIntakeCommand spindexerColorIntakeCommand) {
        return new SpindexerColorIntake(spindexerColorIntakeCommand);
    }

    public class IntakeCommand implements Action {
        private final Intake intake;

        public IntakeCommand(Intake intake) {
            this.intake = intake;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.runIntake(1);
            return true;
        }
    }
    public Action intakeCommand(Intake intake) {
        return new IntakeCommand(intake);
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

    public class ShootArtifacts implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                launchArtifactCommand = new LaunchArtifactCommand(spindexer, popper, launcher, drive);
                launchArtifactCommand.start();
                initialized = true;
            }

            if (launchArtifactCommand != null && !launchArtifactCommand.isFinished()) {
                launchArtifactCommand.update(telemetry);
            }

            if (launchArtifactCommand != null && launchArtifactCommand.isFinished()) {
                launchArtifactCommand = null;
                launcher.stopLauncher();
                return false;
            }

            return true;
        }
    }
    public Action shootArtifacts() {
        return new ShootArtifacts();
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
        turret = new Turret(hardwareMap);
        limeLight = new LimeLight(hardwareMap);

        /*
        DRIVE AND TRAJECTORIES
         */
        Pose2d startPose = new Pose2d(-60, -20, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        TrajectoryActionBuilder firstLaunch = drive.actionBuilder(startPose).strafeToLinearHeading(new Vector2d(-10,-40), Math.toRadians(270));
        TrajectoryActionBuilder firstPickup = firstLaunch.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-10,-40)).strafeToConstantHeading(new Vector2d(-10,-55), new TranslationalVelConstraint(7));
        TrajectoryActionBuilder secondLaunch = firstPickup.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-7,-35));
        TrajectoryActionBuilder secondPickup = secondLaunch.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(15, -40)).strafeToConstantHeading(new Vector2d(15,-55) , new TranslationalVelConstraint(7));
        TrajectoryActionBuilder thirdLaunch = secondPickup.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-7,-35));
        TrajectoryActionBuilder thirdPickup = thirdLaunch.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(39, -40)).strafeToConstantHeading(new Vector2d(39,-55), new TranslationalVelConstraint(7));
        TrajectoryActionBuilder fourthLaunch = thirdPickup.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-7,-35));
        TrajectoryActionBuilder park = fourthLaunch.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-2,-40));


        /** Commands */
        spindexerColorIntakeCommand = new SpindexerColorIntakeCommand(spindexer);
        launchArtifactCommand = new LaunchArtifactCommand(spindexer, popper, launcher, drive);

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new ParallelAction(
                        turretGetMotifAndAimAprilTag(),
                        new SequentialAction(
                                firstLaunch.build(),
//                            new RaceAction(
//                                    firstLaunch.build(),
//                                    spindexerColorIntake(spindexerColorIntakeCommand)
//                            )
//                            shootArtifacts(),
//                            new RaceAction(
//                                    firstPickup.build(),
//                                    spindexerColorIntake(spindexerColorIntakeCommand),
//                                    intakeCommand(intake)
//                            )
//                            stopIntakeSpindexer(spindexer, intake),
                                firstPickup.build(),
                                secondLaunch.build(),
//                            shootArtifacts(),
                                secondPickup.build(),
                                thirdLaunch.build(),
                                thirdPickup.build(),
                                fourthLaunch.build(),
                                park.build()
                        )
                )
        );
    }
}
