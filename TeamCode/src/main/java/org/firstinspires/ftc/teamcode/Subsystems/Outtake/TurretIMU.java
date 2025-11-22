package org.firstinspires.ftc.teamcode.Subsystems.Outtake;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;

@Config
@TeleOp (name = "Turret IMU")
public class TurretIMU extends OpMode {
    private CRServo turretLeft;
    private CRServo turretRight;
    private IMU imu;
    private IMU turretEncoder;

    private MecanumDrive drive;
    private Pose2d botPose;
    private PIDController pid;
    private Pose2d blueGoalPose;

    public static double kP = 0.02;
    public static double kI = 0.0;
    public static double kD = 0.0008;

    public static double x_position = 0;
    public static double y_position = 0;

    public static double targetAngle = 0;
    double power;
    double error;
    double turretAngle;
    double dx;
    double dy;

    boolean toggle = false;

    @Override
    public void init() {
        turretLeft = hardwareMap.get(CRServo.class, Constants.HMServoTurretLeft);
        turretRight = hardwareMap.get(CRServo.class, Constants.HMServoTurretRight);

        botPose = new Pose2d(0, 0, 0);
        pid = new PIDController(kP, kI, kD);
        pid.setTolerance(0.5);



        blueGoalPose = new Pose2d(x_position, y_position, 0);
        drive = new MecanumDrive(hardwareMap, botPose);

        imu = hardwareMap.get(IMU.class, Constants.HMimu);
        turretEncoder = hardwareMap.get(IMU.class, Constants.HMTurretEncoder);

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));

        IMU.Parameters turretParameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));

        imu.initialize(parameters);
        turretEncoder.initialize(turretParameters);
        imu.resetYaw();
        turretEncoder.resetYaw();
    }

    @Override
    public void loop() {
        drive.updatePoseEstimate();
        targetAngle = getRobotToGoalAngle(blueGoalPose, drive.localizer.getPose(), imu.getRobotYawPitchRollAngles().getYaw());

        if (gamepad1.a && !toggle) {
            toggle = true;
        } else if (gamepad1.a && toggle) {
            toggle = false;
        }

        if (!toggle) {
            telemetry.addData("desired angle", targetAngle);
            telemetry.addData("Bot Pose X", drive.localizer.getPose());
            telemetry.addData("dx", dx);
            telemetry.addData("dy", dy);
        } else {
            AimToAngle(targetAngle);
            telemetry.addData("Turret Angle", turretAngle);
            telemetry.addData("Error", error);
            telemetry.addData("Turret Power", power);
            telemetry.addData("desired angle", targetAngle);
            telemetry.addData("dx", dx);
            telemetry.addData("dy", dy);
        }

        telemetry.update();
    }

    public void setPower(double power) {
        turretLeft.setPower(-power);
        turretRight.setPower(-power);
    }

    public void AimToAngle(double ta) {
        pid.setPID(kP, kI, kD);
        double turretAngle = turretEncoder.getRobotYawPitchRollAngles().getYaw();
        error = ta - turretAngle;
        power = pid.calculate(error, 0);
        if (pid.atSetPoint()) {
            setPower(0);
            return;
        }
        setPower(power);
    }

    public double getRobotToGoalAngle(Pose2d goal, Pose2d bot, double robotHeadingDeg) {
        dy = goal.position.x - bot.position.x;
        dx = goal.position.y - bot.position.y;

        double angleToGoalRad = Math.atan2(dy, dx);
        double angleToGoalDeg = Math.toDegrees(angleToGoalRad) - 90;
        return angleToGoalDeg - robotHeadingDeg;
    }

}
