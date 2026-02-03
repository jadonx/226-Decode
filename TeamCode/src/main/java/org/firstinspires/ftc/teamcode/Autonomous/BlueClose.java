package org.firstinspires.ftc.teamcode.Autonomous;

import androidx.annotation.NonNull;

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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autonomous.Actions.AutonomousActions;
import org.firstinspires.ftc.teamcode.Commands.ColorIntakeCommand;
import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.ColorSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Subsystems.LimeLight;
import org.firstinspires.ftc.teamcode.Subsystems.Popper;
import org.firstinspires.ftc.teamcode.Subsystems.RoadRunnerPinPoint;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.Subsystems.Supporters.PoseStorage;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;


@Autonomous (name="BlueClose", group="Autonomous")
public class BlueClose extends LinearOpMode {
    MecanumDrive drive;
    LimeLight limelight;
    Intake intake;
    Turret turret;
    Spindexer spindexer;
    ColorSensor colorSensor;
    Launcher launcher;
    Popper popper;

    Spindexer.HolderStatus[] motif;
    double turretAngle;

    ColorIntakeCommand colorIntakeCommand;

    AutonomousActions autonomousActions;

    public class LimeLightDetectMotif implements Action {
        private ElapsedTime detectionTimer = new ElapsedTime();
        private boolean intialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!intialized) {
                detectionTimer.reset();
                intialized = true;
            }

            limelight.getResult();
            limelight.getAprilTagID();

            if (limelight.hasMotif()) {
                telemetry.addData("Status: ", "Motif detected " + limelight.getMotifID());
                motif = limelight.getMotif();
                spindexer.setMotifPattern(motif[0], motif[1], motif[2]);
                telemetry.addData("Motif: ", motif[0] + " " + motif[1] + " " + motif[2]);
            }
            else {
                telemetry.addData("Status: ", "nothing detected");
            }

            telemetry.addData("Target angle ", turretAngle);
            telemetry.update();

            return !(detectionTimer.milliseconds() > 1500 || limelight.hasMotif());
        }
    }
    public Action limeLightDetectMotif() {
        return new LimeLightDetectMotif();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-61, -39, Math.toRadians(-90));
        PoseStorage.updatePose(-61, -39, -90);
        drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder firstLaunch = drive.actionBuilder(initialPose).strafeToConstantHeading(new Vector2d(-14,-22));
        TrajectoryActionBuilder firstPickup = firstLaunch.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-11,-31)).strafeToConstantHeading(new Vector2d(-11,-48), new TranslationalVelConstraint(5.5));
        TrajectoryActionBuilder openGate = firstPickup.endTrajectory().fresh().strafeToLinearHeading(new Vector2d(-4, -55.5), Math.toRadians(180)).waitSeconds(0.5);
        TrajectoryActionBuilder secondLaunch = openGate.endTrajectory().fresh().strafeToLinearHeading(new Vector2d(-14,-22), Math.toRadians(-90));
        TrajectoryActionBuilder secondPickup = secondLaunch.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(13, -31)).strafeToConstantHeading(new Vector2d(13,-48) , new TranslationalVelConstraint(5.5));
        TrajectoryActionBuilder thirdLaunch = secondPickup.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-14,-22));
        TrajectoryActionBuilder thirdPickup = thirdLaunch.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(36.5, -31.5)).strafeToConstantHeading(new Vector2d(36.5,-48), new TranslationalVelConstraint(5.5));
        TrajectoryActionBuilder fourthLaunch = thirdPickup.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-14,-22));
        TrajectoryActionBuilder park = fourthLaunch.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-7,-34));

        intake = new Intake(hardwareMap);
        turret = new Turret(hardwareMap);
        spindexer = new Spindexer(hardwareMap);
        colorSensor = new ColorSensor(hardwareMap);
        launcher = new Launcher(hardwareMap);
        popper = new Popper(hardwareMap);
        limelight = new LimeLight(hardwareMap, RoadRunnerPinPoint.AllianceColor.BLUE);

        colorIntakeCommand = new ColorIntakeCommand(spindexer, colorSensor);

        autonomousActions = new AutonomousActions(drive, limelight, intake, turret, spindexer, launcher, popper, colorIntakeCommand);

        Spindexer.HolderStatus[] motif = new Spindexer.HolderStatus[]{Spindexer.HolderStatus.PURPLE, Spindexer.HolderStatus.PURPLE, Spindexer.HolderStatus.GREEN};

        turret.resetTurretIMU();
        turret.setTarget(-160);
        while (opModeInInit()) {
            turret.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new ParallelAction(
                        autonomousActions.updateTurret(),
                        autonomousActions.updateBotPosition(),
                        autonomousActions.updateLauncher(1330),
                        new SequentialAction(
                                autonomousActions.moveCover(0.25),
                                autonomousActions.runPopper(),
                                autonomousActions.setTurretTarget(160),
                                autonomousActions.pushInPopper(),
                                new ParallelAction(
                                        firstLaunch.build(),
                                        new SequentialAction(
                                                limeLightDetectMotif(),
                                                autonomousActions.setTurretTarget(-135)
                                        )
                                ),
                                autonomousActions.spindexerFullRotation(0.2),
                                autonomousActions.runIntake(),
                                autonomousActions.deactivatePopper(),
                                new RaceAction(
                                        firstPickup.build(),
                                        autonomousActions.autoColorIntakeCommand(colorIntakeCommand)
                                ),
                                autonomousActions.stopSpindexer(),
                                openGate.build(),
                                autonomousActions.runPopper(),
                                autonomousActions.setSpindexerHolderStatuses(AutonomousActions.PPG),
                                new ParallelAction(
                                        secondLaunch.build(),
                                        new SequentialAction(
                                                autonomousActions.moveToSortedPosition(),
                                                autonomousActions.stopSpindexer(),
                                                autonomousActions.pushInPopper()
                                        )
                                ),
                                autonomousActions.stopIntake(),
                                autonomousActions.spindexerFullRotation(0.2),
                                autonomousActions.deactivatePopper(),
                                autonomousActions.runIntake(),
                                new RaceAction(
                                        secondPickup.build(),
                                        autonomousActions.autoColorIntakeCommand(colorIntakeCommand)
                                ),
                                autonomousActions.stopSpindexer(),
                                autonomousActions.runPopper(),
                                autonomousActions.setSpindexerHolderStatuses(AutonomousActions.PGP),
                                new ParallelAction(
                                        thirdLaunch.build(),
                                        new SequentialAction(
                                                autonomousActions.moveToSortedPosition(),
                                                autonomousActions.stopSpindexer(),
                                                autonomousActions.pushInPopper()
                                        )
                                ),
                                autonomousActions.stopIntake(),
                                autonomousActions.spindexerFullRotation(0.2),
                                autonomousActions.deactivatePopper(),
                                autonomousActions.runIntake(),
                                new RaceAction(
                                        thirdPickup.build(),
                                        autonomousActions.autoColorIntakeCommand(colorIntakeCommand)
                                ),
                                autonomousActions.stopSpindexer(),
                                autonomousActions.runPopper(),
                                autonomousActions.setSpindexerHolderStatuses(AutonomousActions.GPP),
                                new ParallelAction(
                                        fourthLaunch.build(),
                                        new SequentialAction(
                                                autonomousActions.moveToSortedPosition(),
                                                autonomousActions.stopSpindexer(),
                                                autonomousActions.pushInPopper()
                                        )
                                ),
                                autonomousActions.stopIntake(),
                                autonomousActions.spindexerFullRotation(0.2),
                                autonomousActions.deactivatePopper(),
                                park.build()
                        )
                )
        );
    }
}
