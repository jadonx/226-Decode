package org.firstinspires.ftc.teamcode.Autonomous;

import android.graphics.Color;

import androidx.annotation.ColorInt;
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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
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


@Autonomous (name="RedClose", group="Autonomous")
public class RedClose extends LinearOpMode {
    MecanumDrive drive;
    PinPoint pinpoint;
    LimeLight limelight;
    Intake intake;
    Turret turret;
    Spindexer spindexer;
    Launcher launcher;
    Popper popper;

    ColorIntakeCommand colorIntakeCommand;

    public class UpdateBotPosition implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            Pose2d currentPoseRR = drive.localizer.getPose();
            double botXCoord = pinpoint.getXCoordinate(pinpoint.getPose(), DistanceUnit.INCH);
            double botYCoord = pinpoint.getYCoordinate(pinpoint.getPose(), DistanceUnit.INCH);
            double botHead = pinpoint.getHeading();
            telemetryPacket.put("PinPoint X", botXCoord);
            telemetryPacket.put("PinPoint Y", botYCoord);
            telemetryPacket.put("PinPoint Heading", botHead);

            PoseStorage.updatePose(botXCoord, botYCoord, botHead);
            return true;
        }
    }
    public Action updateBotPosition() {return new UpdateBotPosition();}

    public class UpdatePinPoint implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            pinpoint.updatePose();
            return true;
        }
    }
    public Action updatePinPoint( ) {
        return new UpdatePinPoint();
    }

    public class TurretTrackingAngle implements Action {
        double targetAngle;
        public TurretTrackingAngle(double tA) {
            targetAngle = tA;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            turret.goToAngle(targetAngle);
            return true;
        }
    }
    public Action turretTrackingAngle(double tA) {
        return new TurretTrackingAngle(tA);
    }

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

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-63, 35, Math.toRadians(90));
        drive = new MecanumDrive(hardwareMap, initialPose);
        pinpoint = new PinPoint(hardwareMap, PinPoint.AllianceColor.RED, 40, 61, 0);

        TrajectoryActionBuilder firstLaunch = drive.actionBuilder(initialPose).strafeToLinearHeading(new Vector2d(-14,22), Math.toRadians(90));
        TrajectoryActionBuilder firstPickup = firstLaunch.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-14,25)).strafeToConstantHeading(new Vector2d(-14,42), new TranslationalVelConstraint(5.5));
        TrajectoryActionBuilder secondLaunch = firstPickup.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-14,22));
        TrajectoryActionBuilder secondPickup = secondLaunch.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(11, 25)).strafeToConstantHeading(new Vector2d(11,42) , new TranslationalVelConstraint(5.5));
        TrajectoryActionBuilder thirdLaunch = secondPickup.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-14,22));
        TrajectoryActionBuilder thirdPickup = thirdLaunch.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(34, 25)).strafeToConstantHeading(new Vector2d(34,42), new TranslationalVelConstraint(5.5));
        TrajectoryActionBuilder fourthLaunch = thirdPickup.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-14,22));
        TrajectoryActionBuilder park = fourthLaunch.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-7,34));

        intake = new Intake(hardwareMap);
        turret = new Turret(hardwareMap);
        spindexer = new Spindexer(hardwareMap);
        launcher = new Launcher(hardwareMap);
        popper = new Popper(hardwareMap);
        limelight = new LimeLight(hardwareMap);

        colorIntakeCommand = new ColorIntakeCommand(spindexer);

        Spindexer.HolderStatus[] motif = new Spindexer.HolderStatus[]{Spindexer.HolderStatus.PURPLE, Spindexer.HolderStatus.PURPLE, Spindexer.HolderStatus.GREEN};

//        while (opModeInInit()) {
//            turret.goToAngle(-80);
//            limelight.getResult();
//            limelight.getAprilTagID();
//
//            if (limelight.hasMotif()) {
//                telemetry.addData("Status:", "Motif Detected: " + limelight.getMotifID());
//                motif = limelight.getMotif();
//                telemetry.addData("Motif: ", motif[0] + ", " + motif[1] + ", " + motif[2]);
//                telemetry.update();
//            }
//        }

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new ParallelAction(
                        updatePinPoint(),
                        updateBotPosition(),
                        turretTrackingAngle(-53),
                        updateLauncher(),
                        new SequentialAction(
                                moveCover(),
                                runPopper(),
                                pushInPopper(),
                                firstLaunch.build(),
                                spindexerFullRotation(),
                                deactivatePopper(),
                                runIntake(),
                                new RaceAction(
                                        firstPickup.build(),
                                        autoColorIntakeCommand(colorIntakeCommand)
                                ),
                                stopSpindexer(),
                                runPopper(),
                                new ParallelAction(
                                        secondLaunch.build(),
                                        new SequentialAction(
                                                moveToSortedPosition(motif),
                                                stopSpindexer()
                                        )
                                ),
                                pushInPopper(),
                                stopIntake(),
                                spindexerFullRotation(),
                                deactivatePopper(),
                                runIntake(),
                                new RaceAction(
                                        secondPickup.build(),
                                        autoColorIntakeCommand(colorIntakeCommand)
                                ),
                                stopSpindexer(),
                                runPopper(),
                                new ParallelAction(
                                        thirdLaunch.build(),
                                        new SequentialAction(
                                                moveToSortedPosition(motif),
                                                stopSpindexer()
                                        )
                                ),
                                pushInPopper(),
                                stopIntake(),
                                spindexerFullRotation(),
                                deactivatePopper(),
                                runIntake()
                        )
                )
        );
    }
}
