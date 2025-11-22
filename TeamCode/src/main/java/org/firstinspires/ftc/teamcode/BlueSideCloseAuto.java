package org.firstinspires.ftc.teamcode;

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
import org.firstinspires.ftc.teamcode.Subsystems.Outtake.Launcher;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer.LaunchArtifactCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer.Popper;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer.Spindexer;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer.UnjammerSystem;

@Config
@Autonomous(name = "BlueSideCloseAuto", group = "Autonomous")
public class BlueSideCloseAuto extends LinearOpMode {
    Intake intake = new Intake(hardwareMap);;
    Spindexer spindexer = new Spindexer(hardwareMap);
    UnjammerSystem unjamSystem = new UnjammerSystem(intake, spindexer);
    Popper popper = new Popper(hardwareMap);
    Launcher launcher = new Launcher(hardwareMap);

    /*
    ACTIONS
     */
    public class ShootArtifacts implements Action {
        LaunchArtifactCommand launchArtifactCommand = new LaunchArtifactCommand(spindexer, popper, launcher);

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

        Pose2d initialPose = new Pose2d(-48, -48, Math.toRadians(54));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // Trajectory Variables
        TrajectoryActionBuilder shiftForward = drive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(90))
                .lineToYLinearHeading(-44, Math.toRadians(90));

        TrajectoryActionBuilder shootZone = shiftForward.endTrajectory().fresh()
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-20, -20, Math.toRadians(270)), Math.toRadians(0.00))
                .waitSeconds(2);

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        shiftForward.build(),
                        shootZone.build(),
                        shootArtifacts()
                )
        );
    }
}
