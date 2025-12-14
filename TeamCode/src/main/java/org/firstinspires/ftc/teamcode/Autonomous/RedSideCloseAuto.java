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
@Autonomous(name = "RedSideCloseAuto", group = "Autonomous")
public class RedSideCloseAuto extends LinearOpMode {
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
    public boolean hasReadMotif = false;
    public Spindexer.HolderStatus[] motifPattern = new Spindexer.HolderStatus[]{Spindexer.HolderStatus.NONE, Spindexer.HolderStatus.NONE, Spindexer.HolderStatus.NONE};

    /*
    ACTIONS
     */
    public class GetMotifAndAimToAprilTag implements Action {
        private int goalTagID = 24;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            limeLight.getResult();
            if (!hasReadMotif) {
                if (motifID != -1) {
                    motifPattern = limeLight.getMotif();
                    hasReadMotif = true;
                } else {
                    motifPattern = new Spindexer.HolderStatus[]{Spindexer.HolderStatus.GREEN, Spindexer.HolderStatus.PURPLE, Spindexer.HolderStatus.PURPLE};
                }
            }
            motifID = limeLight.getMotifID();
            aprilTagID = limeLight.getAprilTagID();
            telemetryPacket.put("Motif ID", motifID);
            telemetryPacket.put("April Tag ID", aprilTagID);
            if (limeLight.isResulted() && aprilTagID == goalTagID) {
                    turret.trackAprilTag(limeLight.getTX());
            } else {
                turret.trackTargetAngleAuto(-65);
            }

            return true;
        }
    }
    public Action turretGetMotifAndAimAprilTag() {
        return new GetMotifAndAimToAprilTag();
    }


    public class FirstSpindexerColorIntake implements Action {
        private final SpindexerColorIntakeCommand spindexerColorIntakeCommand;

        private boolean initialized = false;

        public FirstSpindexerColorIntake(SpindexerColorIntakeCommand spindexerColorIntakeCommand) {
            this.spindexerColorIntakeCommand = spindexerColorIntakeCommand;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                spindexerColorIntakeCommand.startAuto();
                initialized = true;
            }
            telemetry.update();
            return true;
        }
    }
    public Action firstSpindexerColorIntake(SpindexerColorIntakeCommand spindexerColorIntakeCommand) {
        return new FirstSpindexerColorIntake(spindexerColorIntakeCommand);
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
            return false;
        }
    }
    public Action intakeCommand(Intake intake) {
        return new IntakeCommand(intake);
    }

    public class ReverseIntakeCommand implements Action {
        private final Intake intake;

        public ReverseIntakeCommand(Intake intake) {
            this.intake = intake;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.runIntake(-1);
            return false;
        }
    }
    public Action reverseIntakeCommand(Intake intake) {
        return new ReverseIntakeCommand(intake);
    }

    public class StopIntake implements Action {
        private final Intake intake;

        public StopIntake(Intake intake) {
            this.intake = intake;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.stopIntake();
            return false;
        }
    }
    public Action stopIntake(Intake intake) {
        return new StopIntake(intake);
    }

    public class StopSpindexer implements Action {
        private final Spindexer spindexer;

        public StopSpindexer(Spindexer spindexer) {
            this.spindexer = spindexer;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            spindexer.stopSpindexer();
            return false;
        }
    }
    public Action stopSpindexer(Spindexer spindexer) {
        return new StopSpindexer(spindexer);
    }

    public class ShootArtifacts implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                launchArtifactCommand = new LaunchArtifactCommand(spindexer, popper, launcher, drive);
                launchArtifactCommand.autoColorStart(motifPattern);
                initialized = true;
            }

            if (launchArtifactCommand != null && !launchArtifactCommand.isFinished()) {
                launchArtifactCommand.update(telemetryPacket);
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
        Pose2d startPose = new Pose2d(-60, 50, Math.toRadians(90));
        drive = new MecanumDrive(hardwareMap, startPose);
        TrajectoryActionBuilder getMotif = drive.actionBuilder(startPose).strafeToLinearHeading(new Vector2d(-40,30), Math.toRadians(90));
        TrajectoryActionBuilder firstLaunch = getMotif.endTrajectory().fresh().strafeToLinearHeading(new Vector2d(-13,40), Math.toRadians(90));
        TrajectoryActionBuilder firstPickup = firstLaunch.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-12,40)).strafeToConstantHeading(new Vector2d(-12,60), new TranslationalVelConstraint(6));
        TrajectoryActionBuilder secondLaunch = firstPickup.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-9,35));
        TrajectoryActionBuilder secondPickup = secondLaunch.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(13.5, 40)).strafeToConstantHeading(new Vector2d(13.5,60) , new TranslationalVelConstraint(6));
        TrajectoryActionBuilder thirdLaunch = secondPickup.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-9,35));
        TrajectoryActionBuilder thirdPickup = thirdLaunch.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(39, 40)).strafeToConstantHeading(new Vector2d(39,59), new TranslationalVelConstraint(6));
        TrajectoryActionBuilder fourthLaunch = thirdPickup.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-8,35));
        TrajectoryActionBuilder park = fourthLaunch.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-2,40));
        /** Commands */
        spindexerColorIntakeCommand = new SpindexerColorIntakeCommand(spindexer);
        launchArtifactCommand = new LaunchArtifactCommand(spindexer, popper, launcher, drive);

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new ParallelAction(
                        turretGetMotifAndAimAprilTag(),
                        new SequentialAction(
                            getMotif.build(),
                            new RaceAction(
                                    firstLaunch.build(),
                                    firstSpindexerColorIntake(spindexerColorIntakeCommand)
                            ),
                            shootArtifacts(),
                            intakeCommand(intake),
                            new RaceAction(
                                    firstPickup.build(),
                                    spindexerColorIntake(spindexerColorIntakeCommand)
                            ),
                            stopIntake(intake),
                            stopSpindexer(spindexer),
                            secondLaunch.build(),
                            shootArtifacts(),
                            intakeCommand(intake),
                            new RaceAction(
                                    secondPickup.build(),
                                    spindexerColorIntake(spindexerColorIntakeCommand)
                            ),
                            stopIntake(intake),
                            stopSpindexer(spindexer),
                            thirdLaunch.build(),
                            shootArtifacts(),
                            park.build()
                        )
                )
        );
    }
}
