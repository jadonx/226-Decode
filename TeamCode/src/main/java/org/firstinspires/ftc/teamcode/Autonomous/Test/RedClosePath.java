package org.firstinspires.ftc.teamcode.Autonomous.Test;

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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Supporters.PoseStorage;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

@Disabled
@Autonomous(name="RedClosePath")
public class RedClosePath extends LinearOpMode {
    MecanumDrive drive;
    Turret turret;

    public class UpdateBotPosition implements Action {
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            PoseStorage.updatePose(drive.localizer.getPose().position.x, drive.localizer.getPose().position.y, Math.toDegrees(drive.localizer.getPose().heading.toDouble()));

            telemetry.addData("Storage x ", PoseStorage.getX());
            telemetry.addData("Storage y ", PoseStorage.getY());
            telemetry.addData("Storage heading ", PoseStorage.getHeading());

            telemetry.update();

            return true;
        }
    }
    public Action updateBotPosition() {return new UpdateBotPosition();}

    @Override
    public void runOpMode() throws InterruptedException {
        turret = new Turret(hardwareMap);
        turret.resetTurretIMU();

        Pose2d initialPose = new Pose2d(-62, 40, Math.toRadians(90));

        drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder firstLaunch = drive.actionBuilder(initialPose).strafeToLinearHeading(new Vector2d(-14,22), Math.toRadians(90));
        TrajectoryActionBuilder firstPickup = firstLaunch.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-12.5,33)).strafeToConstantHeading(new Vector2d(-12.5,48), new TranslationalVelConstraint(5.5));
        TrajectoryActionBuilder openGate = firstPickup.endTrajectory().fresh().strafeToLinearHeading(new Vector2d(-4, 54), Math.toRadians(180));
        TrajectoryActionBuilder secondLaunch = openGate.endTrajectory().fresh().strafeToLinearHeading(new Vector2d(-14,22), Math.toRadians(90));
        TrajectoryActionBuilder secondPickup = secondLaunch.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(12, 33)).strafeToConstantHeading(new Vector2d(12,48) , new TranslationalVelConstraint(5.5));
        TrajectoryActionBuilder thirdLaunch = secondPickup.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-14,22));
        TrajectoryActionBuilder thirdPickup = thirdLaunch.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(36, 33)).strafeToConstantHeading(new Vector2d(36,48), new TranslationalVelConstraint(5.5));
        TrajectoryActionBuilder fourthLaunch = thirdPickup.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-14,22));
        TrajectoryActionBuilder park = fourthLaunch.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-7,34));

        // turret.goToAngle(170);

        waitForStart();

        if (isStopRequested()) return;

        try {
            Actions.runBlocking(
                    new ParallelAction(
                            updateBotPosition(),
                            new SequentialAction(
                                    firstLaunch.build(),
                                    firstPickup.build(),
                                    openGate.build(),
                                    secondLaunch.build(),
                                    secondPickup.build(),
                                    thirdLaunch.build(),
                                    thirdPickup.build(),
                                    fourthLaunch.build(),
                                    park.build()
                            )
                    )
            );
        }
        finally {

        }
    }
}
