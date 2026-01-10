package org.firstinspires.ftc.teamcode.Autonomous;

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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.BetterPinPoint;
import org.firstinspires.ftc.teamcode.Subsystems.PinPoint;
import org.firstinspires.ftc.teamcode.Subsystems.Supporters.PoseStorage;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

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
        Pose2d initialPose = new Pose2d(-61, 39, Math.toRadians(90));
        drive = new MecanumDrive(hardwareMap, initialPose);
        turret = new Turret(hardwareMap);
        turret.resetTurretIMU();

        TrajectoryActionBuilder firstLaunch = drive.actionBuilder(initialPose).strafeToLinearHeading(new Vector2d(-14,22), Math.toRadians(90));
        TrajectoryActionBuilder firstPickup = firstLaunch.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-14,25)).strafeToConstantHeading(new Vector2d(-14,42), new TranslationalVelConstraint(5.5));
        TrajectoryActionBuilder secondLaunch = firstPickup.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-14,22));
        TrajectoryActionBuilder secondPickup = secondLaunch.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(11, 25)).strafeToConstantHeading(new Vector2d(11,42) , new TranslationalVelConstraint(5.5));
        TrajectoryActionBuilder thirdLaunch = secondPickup.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-14,22));
        TrajectoryActionBuilder thirdPickup = thirdLaunch.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(34, 25)).strafeToConstantHeading(new Vector2d(34,42), new TranslationalVelConstraint(5.5));
        TrajectoryActionBuilder fourthLaunch = thirdPickup.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-14,22));
        TrajectoryActionBuilder park = fourthLaunch.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-7,34));

        // turret.goToAngle(170);

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new ParallelAction(
                        updateBotPosition(),
                        new SequentialAction(
                                firstLaunch.build(),
                                firstPickup.build(),
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
}
