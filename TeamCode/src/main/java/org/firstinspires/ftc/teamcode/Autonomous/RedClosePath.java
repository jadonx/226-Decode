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

@Autonomous(name="RedClosePath")
public class RedClosePath extends LinearOpMode {
    MecanumDrive drive;
    PinPoint pinpoint;
    BetterPinPoint betterPinPoint;

    public class UpdateBotPosition implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            betterPinPoint.update();
            PoseStorage.updatePose(betterPinPoint.getCorrectX(), betterPinPoint.getCorrectY(), betterPinPoint.getCorrectHeading());

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
        Pose2d initialPose = new Pose2d(-63, 35, Math.toRadians(90));
        drive = new MecanumDrive(hardwareMap, initialPose);

        betterPinPoint = new BetterPinPoint(hardwareMap, BetterPinPoint.AllianceColor.RED);

        TrajectoryActionBuilder firstLaunch = drive.actionBuilder(initialPose).strafeToLinearHeading(new Vector2d(-16,26), Math.toRadians(90));
        TrajectoryActionBuilder firstPickup = firstLaunch.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-14,28)).strafeToConstantHeading(new Vector2d(-14,48), new TranslationalVelConstraint(6));
        TrajectoryActionBuilder secondLaunch = firstPickup.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-16,26));
        TrajectoryActionBuilder secondPickup = secondLaunch.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(12, 28)).strafeToConstantHeading(new Vector2d(12,48) , new TranslationalVelConstraint(6));
        TrajectoryActionBuilder thirdLaunch = secondPickup.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-16,26));
        TrajectoryActionBuilder thirdPickup = thirdLaunch.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(34, 28)).strafeToConstantHeading(new Vector2d(34,48), new TranslationalVelConstraint(6));
        TrajectoryActionBuilder fourthLaunch = thirdPickup.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-16,26));
        TrajectoryActionBuilder park = fourthLaunch.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-7,34));

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
