package org.firstinspires.ftc.teamcode.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Commands.LaunchArtifactCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Popper;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Subsystems.UnjammerSystem;

@Config
@Autonomous(name = "RedSideCloseAuto", group = "Autonomous")
public class RedSideCloseAuto extends LinearOpMode {
    Intake intake;
    Spindexer spindexer;
    UnjammerSystem unjamSystem;
    Popper popper;
    Launcher launcher;
    Turret turret;

    LaunchArtifactCommand launchArtifactCommand;

    /*
    ACTIONS
     */
    public class ShootArtifacts implements Action {
        private final LaunchArtifactCommand launcherArtifactCommand;
        private boolean initialized = false;
        private boolean isTurretReadyToShoot = false;
        private double taShooting = 45; // Angle to aim turret for shooting

        public ShootArtifacts(LaunchArtifactCommand launchArtifactCommand) {
            this.launcherArtifactCommand = launchArtifactCommand;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                if (!isTurretReadyToShoot) {
                    turret.trackTargetAngle(taShooting); // Aim turret to shooting angle
                    isTurretReadyToShoot = true;
                }
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

        launchArtifactCommand = new LaunchArtifactCommand(spindexer, popper, launcher);

        /*
        TRAJECTORIES
         */

        Pose2d startPose = new Pose2d(-48, 48, Math.toRadians(128));

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        // Trajectory Variables
        TrajectoryActionBuilder firstLaunch = drive.actionBuilder(startPose)
                .strafeToLinearHeading(new Vector2d(-11.5,11.5), Math.toRadians(90))
                .waitSeconds(3);

        TrajectoryActionBuilder firstPickup = firstLaunch.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-11.5, 30))
                .lineToY(44.5)
                ;


        TrajectoryActionBuilder secondLaunch = firstPickup.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-11.5,11.5))
                .waitSeconds(3);

        TrajectoryActionBuilder secondPickup = secondLaunch.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(11.5, 30))
                .strafeToConstantHeading(new Vector2d(11.5,44.5));


        TrajectoryActionBuilder thirdLaunch = secondPickup.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-11.5,11.5))
                .waitSeconds(3);

        TrajectoryActionBuilder thirdPickup = thirdLaunch.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(36, 30))
                .strafeToConstantHeading(new Vector2d(36,44.5))
                .waitSeconds(0.3);

        TrajectoryActionBuilder fourthLaunch = thirdPickup.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-11.5,11.5))
                .waitSeconds(3);

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        firstLaunch.build(),
                        firstPickup.build(),
                        secondLaunch.build(),
                        secondPickup.build(),
                        thirdLaunch.build(),
                        thirdPickup.build(),
                        fourthLaunch.build()
                )
        );
    }
}
