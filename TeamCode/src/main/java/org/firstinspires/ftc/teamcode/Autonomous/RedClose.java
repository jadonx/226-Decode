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

    PoseStorage poseStorage;

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

            poseStorage.updatePose(botXCoord, botYCoord, botHead);

            telemetryPacket.put("Bot X", currentPoseRR.position.x);
            telemetryPacket.put("Bot Y", currentPoseRR.position.y);
            telemetryPacket.put("Bot Heading", Math.toDegrees(currentPoseRR.heading.log()));
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
            launcher.setTargetCoverAngle(0.05);
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
                launcher.setTargetVelocity(1350);
                initialized = true;
            }

            launcher.update();

            return true;
        }
    }
    public Action updateLauncher() {
        return new UpdateLauncher();
    }

    public class ActivatePopper implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            popper.setTargetVelocity(1800);
            popper.pushInPopper();
            return false;
        }
    }
    public Action activatePopper() {
        return new ActivatePopper();
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

    public class StopIntakeSpindexer implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.stopIntake();
            spindexer.setPower(0);
            return false;
        }
    }
    public Action stopIntakeSpindexer() {
        return new StopIntakeSpindexer();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status:", "Initializing");
        telemetry.update();

        Pose2d initialPose = new Pose2d(-63, 35, Math.toRadians(90));
        drive = new MecanumDrive(hardwareMap, initialPose);

        intake = new Intake(hardwareMap);
        turret = new Turret(hardwareMap);
        pinpoint = new PinPoint(hardwareMap, PinPoint.AllianceColor.RED, 40, 61, 0);
        spindexer = new Spindexer(hardwareMap);
        launcher = new Launcher(hardwareMap);
        popper = new Popper(hardwareMap);
        limelight = new LimeLight(hardwareMap);
        poseStorage = new PoseStorage(botPosX, botPosY, 0);

        colorIntakeCommand = new ColorIntakeCommand(spindexer);

        // TrajectoryActionBuilder getMotif = drive.actionBuilder(initialPose).strafeToLinearHeading(new Vector2d(-40,30), Math.toRadians(90));
        TrajectoryActionBuilder firstLaunch = drive.actionBuilder(initialPose).strafeToLinearHeading(new Vector2d(-9,35), Math.toRadians(90));
        TrajectoryActionBuilder firstPickup = firstLaunch.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-11,40)).strafeToConstantHeading(new Vector2d(-10,60), new TranslationalVelConstraint(6));
        TrajectoryActionBuilder secondLaunch = firstPickup.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-9,35));
        TrajectoryActionBuilder secondPickup = secondLaunch.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(13.5, 40)).strafeToConstantHeading(new Vector2d(13.5,60) , new TranslationalVelConstraint(6));
        TrajectoryActionBuilder thirdLaunch = secondPickup.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-9,35));
        TrajectoryActionBuilder thirdPickup = thirdLaunch.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(39, 40)).strafeToConstantHeading(new Vector2d(39,59), new TranslationalVelConstraint(6));
        TrajectoryActionBuilder fourthLaunch = thirdPickup.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-8,35));
        TrajectoryActionBuilder park = fourthLaunch.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-2,40));

        telemetry.addData("Status:", "Subsystems Initialized");
        telemetry.addData("Status:", "Path Built");
        telemetry.update();
        telemetry.addData("Status:", "Getting Motif");
        Spindexer.HolderStatus[] motif;
        while (opModeInInit() && !limelight.hasMotif()) {
            turret.goToAngle(-80);
            limelight.getResult();
            limelight.getAprilTagID();
        }

        if (limelight.hasMotif()) {
            telemetry.addData("Status:", "Motif Detected: " + limelight.getMotifID());
            motif = limelight.getMotif();
            telemetry.addData("Motif: ", motif[0] + ", " + motif[1] + ", " + motif[2]);
            telemetry.update();
        } else {
            telemetry.addData("Motif: ", "Defaulting to PPG");
            telemetry.update();
        }

        telemetry.update();

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
                                activatePopper(),
                                firstLaunch.build(),
                                spindexerFullRotation(),
                                deactivatePopper(),
                                runIntake(),
                                new RaceAction(
                                        firstPickup.build(),
                                        autoColorIntakeCommand(colorIntakeCommand)
                                ),
                                stopIntakeSpindexer(),
                                activatePopper(),
                                secondLaunch.build(),
                                spindexerFullRotation(),
                                deactivatePopper(),
                                runIntake(),
                                new RaceAction(
                                        secondPickup.build(),
                                        autoColorIntakeCommand(colorIntakeCommand)
                                ),
                                stopIntakeSpindexer(),
                                activatePopper(),
                                thirdLaunch.build(),
                                spindexerFullRotation()
                        )
                )
        );

    }
}
