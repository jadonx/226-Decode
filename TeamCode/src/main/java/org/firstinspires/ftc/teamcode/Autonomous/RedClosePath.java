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
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                PoseStorage.updatePose(40, 61, 90);
            }

            betterPinPoint.update();
            PoseStorage.updatePose(PoseStorage.getX()+betterPinPoint.getCorrectX(), PoseStorage.getY()+betterPinPoint.getCorrectY(), PoseStorage.getHeading()+betterPinPoint.getCorrectHeading());

            telemetry.addData("Pinpoint x ", betterPinPoint.getCorrectX());
            telemetry.addData("Pinpoint y ", betterPinPoint.getCorrectY());
            telemetry.addData("Pinpoint heading ", betterPinPoint.getCorrectHeading());
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

        TrajectoryActionBuilder firstLaunch = drive.actionBuilder(initialPose).strafeToLinearHeading(new Vector2d(-14,22), Math.toRadians(90));
        TrajectoryActionBuilder firstPickup = firstLaunch.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-14,25)).strafeToConstantHeading(new Vector2d(-14,42), new TranslationalVelConstraint(5.5));
        TrajectoryActionBuilder secondLaunch = firstPickup.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-14,22));
        TrajectoryActionBuilder secondPickup = secondLaunch.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(11, 25)).strafeToConstantHeading(new Vector2d(11,42) , new TranslationalVelConstraint(5.5));
        TrajectoryActionBuilder thirdLaunch = secondPickup.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-14,22));
        TrajectoryActionBuilder thirdPickup = thirdLaunch.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(34, 25)).strafeToConstantHeading(new Vector2d(34,42), new TranslationalVelConstraint(5.5));
        TrajectoryActionBuilder fourthLaunch = thirdPickup.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-14,22));
        TrajectoryActionBuilder park = fourthLaunch.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-7,34));

        telemetry.addData("Pinpoint x (init) ", betterPinPoint.getCorrectX());
        telemetry.addData("Pinpoint y (init) ", betterPinPoint.getCorrectY());
        telemetry.addData("Pinpoint heading (init) ", betterPinPoint.getCorrectHeading());
        telemetry.update();

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
