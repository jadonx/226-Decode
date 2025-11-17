package org.firstinspires.ftc.teamcode.Operation.Autonomous;

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
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer.LaunchArtifactCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer.Popper;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer.Spindexer;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer.UnjammerSystem;

@Config
@Autonomous(name = "Blue Autonomous", group = "Autonomous")
public class BlueAutonomous extends LinearOpMode {
    Intake intake = new Intake(hardwareMap);;
    Spindexer spindexer = new Spindexer(hardwareMap);
    UnjammerSystem unjamSystem = new UnjammerSystem(intake, spindexer);
    Popper popper = new Popper(hardwareMap);

    /*
    ACTIONS
     */
    public class ShootArtifacts implements Action {
        LaunchArtifactCommand launchArtifactCommand = new LaunchArtifactCommand(spindexer, popper);

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            launchArtifactCommand.update(telemetryPacket);

            return launchArtifactCommand.isFinished();
        }
    }
    public Action shootArtifacts() {
        return new ShootArtifacts();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        /*
        TRAJECTORIES
         */

        Pose2d initialPose = new Pose2d(-50, -50, Math.toRadians(229));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder shooting1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-12, -10));

        TrajectoryActionBuilder intake1 = shooting1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-12, -20), Math.toRadians(270))
                .waitSeconds(0.2)
                .strafeToLinearHeading(new Vector2d(-12, -50), Math.toRadians(270));

        TrajectoryActionBuilder shooting2 = intake1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-12, -10), Math.toRadians(229));

        TrajectoryActionBuilder intake2 = shooting2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(12, -20), Math.toRadians(270))
                .waitSeconds(.2)
                .strafeToLinearHeading(new Vector2d(12, -50), Math.toRadians(270));

        TrajectoryActionBuilder shooting3 = intake2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(35.5, -20), Math.toRadians(270));

        TrajectoryActionBuilder intake3 = shooting3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(35.5, -20), Math.toRadians(270))
                .waitSeconds(.2)
                .strafeToLinearHeading(new Vector2d(35.5, -50), Math.toRadians(270));

        TrajectoryActionBuilder shooting4 = intake3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-12, -10), Math.toRadians(229));

        waitForStart();

        if (isStopRequested()) return;
        Actions.runBlocking(
                new SequentialAction(
                        shooting1.build(),
                        intake1.build(),
                        shooting2.build(),
                        intake2.build(),
                        shooting3.build(),
                        intake3.build(),
                        shooting4.build()
                )

        );
    }
}
