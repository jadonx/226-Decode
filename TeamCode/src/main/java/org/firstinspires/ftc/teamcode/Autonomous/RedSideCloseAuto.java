package org.firstinspires.ftc.teamcode.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
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

import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;
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
    Intake intake;
    Spindexer spindexer;
    UnjammerSystem unjamSystem;
    Popper popper;
    Launcher launcher;
    Turret turret;
    LimeLight limeLight;
    LaunchArtifactCommand launchArtifactCommand;

    public double aprilTagID = -1;
    public double motifID = -1;

    /*
    ACTIONS
     */
    public class GetMotifAndAimToAprilTag implements Action {
        private int goalTagID = 24;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            limeLight.getResult();
            motifID = limeLight.getMotifID();
            aprilTagID = limeLight.getAprilTagID();
            telemetryPacket.put("Motif ID", motifID);
            telemetryPacket.put("April Tag ID", aprilTagID);
            if (limeLight.isResulted() && aprilTagID == goalTagID) {
                    turret.trackAprilTag(limeLight.getTX());
            } else {
                turret.trackTargetAngleAuto(-65);
            }

//            if (isMotifAvailable && motifID != -1) {
//                isMotifAvailable = false;
//                isAprilTagAvailable = true;
//            }
//            if (isAprilTagAvailable) {
//                if (limeLight.isResulted()) {
//                    if (aprilTagID == goalTagID) {
//                        turret.trackAprilTag(limeLight.getTX());
//                    }
//                } else {
//                    turret.trackTargetAngle(45);
//                }
//            }
            return true;
        }
    }
    public Action turretGetMotifAndAimAprilTag() {
        return new GetMotifAndAimToAprilTag();
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
        Pose2d startPose = new Pose2d(-60, 20, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        launchArtifactCommand = new LaunchArtifactCommand(spindexer, popper, launcher, drive);
        TrajectoryActionBuilder firstLaunch = drive.actionBuilder(startPose).strafeToLinearHeading(new Vector2d(-10,35), Math.toRadians(90));
        TrajectoryActionBuilder firstPickup = firstLaunch.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-10,60), new TranslationalVelConstraint(10));
        TrajectoryActionBuilder secondLaunch = firstPickup.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-7,35));
        TrajectoryActionBuilder secondPickup = secondLaunch.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(15, 35)).strafeToConstantHeading(new Vector2d(15,60) , new TranslationalVelConstraint(10));
        TrajectoryActionBuilder thirdLaunch = secondPickup.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-7,35));
        TrajectoryActionBuilder thirdPickup = thirdLaunch.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(39, 35)).strafeToConstantHeading(new Vector2d(39,60), new TranslationalVelConstraint(10));
        TrajectoryActionBuilder fourthLaunch = thirdPickup.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-7,35));

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new ParallelAction(
                        turretGetMotifAndAimAprilTag(),
                        new SequentialAction(
                            firstLaunch.build(),
                            firstPickup.build(),
                            secondLaunch.build(),
                            secondPickup.build(),
                            thirdLaunch.build(),
                            thirdPickup.build(),
                            fourthLaunch.build()
//
                        )

//
                )
        );
    }
}
