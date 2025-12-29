package org.firstinspires.ftc.teamcode.Autonomous;

import androidx.annotation.ColorInt;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Commands.ColorIntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.LaunchCommand;
import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Subsystems.PinPoint;
import org.firstinspires.ftc.teamcode.Subsystems.Popper;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;


@Autonomous (name="RedClose", group="Autonomous")
public class RedClose extends LinearOpMode {
    Turret turret;
    MecanumDrive drive;
    PinPoint pinpoint;

    Spindexer spindexer;
    Launcher launcher;
    Popper popper;

    LaunchCommand launchCommand;
    ColorIntakeCommand colorIntakeCommand;

    /** Turret Actions */
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
            return !turret.atTargetAngle(targetAngle);
        }
    }
    public Action turretTrackingAngle(double tA) {
        return new TurretTrackingAngle(tA);
    }

    public class TurretTrackingGoal implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            turret.goToAngle(pinpoint.getAngleToGoal());
            return true;
        }
    }
    public Action turretTrackingGoal() {
        return new TurretTrackingGoal();
    }

    /** Shoot Artifact Actions */
    public class ShootArtifacts implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                launchCommand = new LaunchCommand(spindexer, popper, launcher, pinpoint);
                launchCommand.startAuto();
                initialized = true;
            }

            if (launchCommand != null && !launchCommand.isFinished()) {
                launchCommand.update();
            }

            if (launchCommand != null && launchCommand.isFinished()) {
                launchCommand = null;
                launcher.stopLauncher();
                launcher.setTargetCoverAngle(0.5);
                popper.deactivatePopper();
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
        double botPosX = -60;
        double botPosY = 50;
        double botHeading = 0;
        Pose2d initialPose = new Pose2d(botPosX, botPosY, Math.toRadians(botHeading));
        drive = new MecanumDrive(hardwareMap, initialPose);
        turret = new Turret(hardwareMap);
        pinpoint = new PinPoint(hardwareMap, PinPoint.AllianceColor.RED, botPosX, botPosY, botHeading);
        spindexer = new Spindexer(hardwareMap);
        launcher = new Launcher(hardwareMap);
        popper = new Popper(hardwareMap);

        TrajectoryActionBuilder getMotif = drive.actionBuilder(initialPose).strafeToLinearHeading(new Vector2d(-40,30), Math.toRadians(90));
        TrajectoryActionBuilder firstLaunch = getMotif.endTrajectory().fresh().strafeToLinearHeading(new Vector2d(-13,40), Math.toRadians(90));
        TrajectoryActionBuilder firstPickup = firstLaunch.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-12,40)).strafeToConstantHeading(new Vector2d(-12,60), new TranslationalVelConstraint(6));
        TrajectoryActionBuilder secondLaunch = firstPickup.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-9,35));
        TrajectoryActionBuilder secondPickup = secondLaunch.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(13.5, 40)).strafeToConstantHeading(new Vector2d(13.5,60) , new TranslationalVelConstraint(6));
        TrajectoryActionBuilder thirdLaunch = secondPickup.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-9,35));
        TrajectoryActionBuilder thirdPickup = thirdLaunch.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(39, 40)).strafeToConstantHeading(new Vector2d(39,59), new TranslationalVelConstraint(6));
        TrajectoryActionBuilder fourthLaunch = thirdPickup.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-8,35));
        TrajectoryActionBuilder park = fourthLaunch.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-2,40));

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new ParallelAction(
                        updatePinPoint(),
                        turretTrackingGoal(),
                        new SequentialAction(
                                // turretTrackingAngle(0)
                                getMotif.build(),
                                firstLaunch.build(),
                                shootArtifacts(),
                                firstPickup.build()
                        )
                )
        );

    }
}
