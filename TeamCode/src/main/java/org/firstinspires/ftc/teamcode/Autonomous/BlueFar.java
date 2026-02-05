package org.firstinspires.ftc.teamcode.Autonomous;

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


@Autonomous (name="BlueFar", group="Autonomous")
public class BlueFar extends LinearOpMode {
    MecanumDrive drive;
    LimeLight limelight;
    Intake intake;
    Turret turret;
    Spindexer spindexer;
    ColorSensor colorSensor;
    Launcher launcher;
    Popper popper;

    ColorIntakeCommand colorIntakeCommand;

    AutonomousActions autonomousActions;

    private final double spindexerSpeed = 0.15;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(62, -14, Math.toRadians(180));
        PoseStorage.updatePose(62, -14, 180);
        drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder firstLaunch = drive.actionBuilder(initialPose).strafeToLinearHeading(new Vector2d(51,-18), Math.toRadians(-90));
        TrajectoryActionBuilder firstPickup = firstLaunch.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(36,-31)).strafeToConstantHeading(new Vector2d(36,-48), new TranslationalVelConstraint(5.5));
        TrajectoryActionBuilder secondLaunch = firstPickup.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(51,-18));
        TrajectoryActionBuilder secondPickup = secondLaunch.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(12, -31)).strafeToConstantHeading(new Vector2d(12,-48) , new TranslationalVelConstraint(5.5));
        TrajectoryActionBuilder thirdLaunch = secondPickup.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(51,-18));
        TrajectoryActionBuilder park = thirdLaunch.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(45,-30));

        intake = new Intake(hardwareMap);
        turret = new Turret(hardwareMap);
        spindexer = new Spindexer(hardwareMap);
        colorSensor = new ColorSensor(hardwareMap);
        launcher = new Launcher(hardwareMap);
        popper = new Popper(hardwareMap);
        limelight = new LimeLight(hardwareMap, RoadRunnerPinPoint.AllianceColor.BLUE);
        limelight.setPipeline(1);

        colorIntakeCommand = new ColorIntakeCommand(spindexer, colorSensor);

        autonomousActions = new AutonomousActions(drive, limelight, intake, turret, spindexer, launcher, popper, colorIntakeCommand);

        Spindexer.HolderStatus[] motif = new Spindexer.HolderStatus[]{Spindexer.HolderStatus.PURPLE, Spindexer.HolderStatus.PURPLE, Spindexer.HolderStatus.GREEN};

        turret.resetTurretIMU();
        turret.setTarget(-170);
        boolean reachedMiddlePoint = false;
        while (opModeInInit()) {
            turret.update();

            if (turret.getTurretAngle() < -160 && !reachedMiddlePoint) {
                turret.setTarget(170);
                reachedMiddlePoint = true;
            }

            limelight.getResult();
            limelight.getAprilTagID();

            telemetry.addData("Status: ", limelight.getMotifID());
            motif = limelight.getMotif();
            spindexer.setMotifPattern(motif[0], motif[1], motif[2]);
            telemetry.addData("Motif: ", motif[0] + " " + motif[1] + " " + motif[2]);
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new ParallelAction(
                        autonomousActions.updateLauncher(1750),
                        autonomousActions.updateBotPosition(),
                        autonomousActions.updateTurret(),
                        new SequentialAction(
                                /** Setup Sequence */
                                autonomousActions.setSpindexerHolderStatuses(AutonomousActions.GPP),
                                autonomousActions.moveCover(0),
                                autonomousActions.runPopper(),
                                autonomousActions.setTurretTarget(-157),
                                /** First Shooting Sequence */
                                new ParallelAction(
                                        firstLaunch.build(),
                                        new SequentialAction(
                                                autonomousActions.moveToSortedPosition(),
                                                autonomousActions.stopSpindexer(),
                                                autonomousActions.pushInPopper()
                                        )
                                ),
                                autonomousActions.atLauncherTargetVelocity(),
                                autonomousActions.spindexerFullRotation(spindexerSpeed),
                                /** First Intake Sequence */
                                autonomousActions.deactivatePopper(),
                                autonomousActions.runIntake(),
                                new RaceAction(
                                        firstPickup.build(),
                                        autonomousActions.autoColorIntakeCommand(colorIntakeCommand)
                                ),
                                autonomousActions.stopSpindexer(),
                                autonomousActions.setSpindexerHolderStatuses(AutonomousActions.GPP),
                                /** Second Shooting Sequence */
                                autonomousActions.runPopper(),
                                new ParallelAction(
                                        secondLaunch.build(),
                                        new SequentialAction(
                                                autonomousActions.moveToSortedPosition(),
                                                autonomousActions.stopSpindexer(),
                                                autonomousActions.pushInPopper()
                                        )
                                ),
                                autonomousActions.stopIntake(),
                                autonomousActions.spindexerFullRotation(spindexerSpeed),
                                /** Second Intake Sequence */
                                autonomousActions.deactivatePopper(),
                                autonomousActions.runIntake(),
                                new RaceAction(
                                        secondPickup.build(),
                                        autonomousActions.autoColorIntakeCommand(colorIntakeCommand)
                                ),
                                autonomousActions.stopSpindexer(),
                                autonomousActions.setSpindexerHolderStatuses(AutonomousActions.PGP),
                                /** Third Shooting Sequence */
                                autonomousActions.runPopper(),
                                new ParallelAction(
                                        thirdLaunch.build(),
                                        new SequentialAction(
                                                autonomousActions.moveToSortedPosition(),
                                                autonomousActions.stopSpindexer(),
                                                autonomousActions.pushInPopper()
                                        )
                                ),
                                autonomousActions.stopIntake(),
                                autonomousActions.spindexerFullRotation(spindexerSpeed),
                                autonomousActions.deactivatePopper(),
                                park.build()
                        )
                )
        );
    }
}
