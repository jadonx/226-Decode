package org.firstinspires.ftc.teamcode.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.PinPoint;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;


@Autonomous (name="RedClose", group="Autonomous")
public class RedClose extends LinearOpMode {
    Turret turret;
    MecanumDrive drive;
    PinPoint pinpoint;

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

    @Override
    public void runOpMode() throws InterruptedException {
        double botPosX = 0;
        double botPosY = 0;
        double botHeading = 90;
        Pose2d initialPose = new Pose2d(botPosX, botPosY, Math.toRadians(botHeading));
        drive = new MecanumDrive(hardwareMap, initialPose);
        turret = new Turret(hardwareMap);
        pinpoint = new PinPoint(hardwareMap, PinPoint.AllianceColor.RED, botPosX, botPosY, botHeading);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;


        Actions.runBlocking(
                new ParallelAction(
                        updatePinPoint(),
                        new SequentialAction(
                                turretTrackingAngle(0)
                        )
                )
        );

    }
}
