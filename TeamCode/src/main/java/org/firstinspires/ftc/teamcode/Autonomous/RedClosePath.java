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
import org.firstinspires.ftc.teamcode.Subsystems.PinPoint;
import org.firstinspires.ftc.teamcode.Subsystems.Supporters.PoseStorage;

@Autonomous(name="RedClosePath")
public class RedClosePath extends LinearOpMode {
    MecanumDrive drive;
    PinPoint pinpoint;

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

            PoseStorage.updatePose(botXCoord, botYCoord, botHead);

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

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-63, 35, Math.toRadians(90));
        drive = new MecanumDrive(hardwareMap, initialPose);

        pinpoint = new PinPoint(hardwareMap, PinPoint.AllianceColor.RED, 40, 61, 0);

        TrajectoryActionBuilder firstLaunch = drive.actionBuilder(initialPose).strafeToLinearHeading(new Vector2d(-16,26), Math.toRadians(90));
        TrajectoryActionBuilder firstPickup = firstLaunch.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-11,40)).strafeToConstantHeading(new Vector2d(-11,48), new TranslationalVelConstraint(6));
        TrajectoryActionBuilder secondLaunch = firstPickup.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-16,26));
        TrajectoryActionBuilder secondPickup = secondLaunch.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(12, 28)).strafeToConstantHeading(new Vector2d(12,48) , new TranslationalVelConstraint(6));
        TrajectoryActionBuilder thirdLaunch = secondPickup.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-16,26));
        TrajectoryActionBuilder thirdPickup = thirdLaunch.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(36, 28)).strafeToConstantHeading(new Vector2d(36,48), new TranslationalVelConstraint(6));
        TrajectoryActionBuilder fourthLaunch = thirdPickup.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-16,26));
        TrajectoryActionBuilder park = fourthLaunch.endTrajectory().fresh().strafeToConstantHeading(new Vector2d(-7,34));

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new ParallelAction(
                        updatePinPoint(),
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
