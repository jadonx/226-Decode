package org.firstinspires.ftc.teamcode.Subsystems.Outtake;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Core.Controller.PIDFController;
import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Encoder.AS5600Encoder;

@TeleOp (name = "Turret IMU")
public class TurretIMU extends OpMode {
    private CRServo turretLeft;
    private CRServo turretRight;
    private IMU imu;
    private AS5600Encoder turretEncoder;
    private MecanumDrive drive;
    private Pose2d botPose;
    private PIDFController pid;
    private Pose2d blueGoalPose;

    public static final double TURRET_KP = 0.01;
    public static final double TURRET_KI = 0.0;
    public static final double TURRET_KD = 0.0;
    public static final double TURRET_KF = 0.0;
    public static final double TURRET_MIN = 0.6;

    @Override
    public void init() {
        turretLeft = hardwareMap.get(CRServo.class, Constants.HMServoTurretLeft);
        turretRight = hardwareMap.get(CRServo.class, Constants.HMServoTurretRight);
        turretEncoder = hardwareMap.get(AS5600Encoder.class, Constants.HMTurretEncoder);


        botPose = new Pose2d(0, 0, 0);
        pid = new PIDFController(TURRET_KP, TURRET_KI, TURRET_KD, TURRET_KF);
        blueGoalPose = new Pose2d(-150, 130, 0);
        drive = new MecanumDrive(hardwareMap, botPose);

        imu = hardwareMap.get(IMU.class, Constants.HMimu);

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));

        imu.initialize(parameters);
    }


    public void AimTo(Pose2d GOAL, Pose2d BOT) {
        double desiredAngle = AngleUnit.normalizeDegrees(Math.toDegrees(Math.atan2(GOAL.position.y - BOT.position.y, GOAL.position.x - BOT.position.x)));
        double botHeading = AngleUnit.normalizeDegrees(imu.getRobotYawPitchRollAngles().getYaw());
        double turretAngle = AngleUnit.normalizeDegrees(turretEncoder.getAngleDegrees());

        double targetTurretAngle = AngleUnit.normalizeDegrees(desiredAngle - botHeading);
        double error = AngleUnit.normalizeDegrees(targetTurretAngle - turretAngle);

        pid.setSetpoint(error);
        double power = pid.calculate(0);

        if (Math.abs(power) < TURRET_MIN && Math.abs(error) > 1) {
            power = Math.signum(power) * TURRET_MIN;
        }

        power = Math.max(-1, Math.min(1, power));

        setPower(power);

        telemetry.addData("Desired Angle", desiredAngle);
        telemetry.addData("Bot Heading", botHeading);
        telemetry.addData("Turret Angle", turretAngle);
        telemetry.addData("Target Turret Angle", targetTurretAngle);
        telemetry.addData("Turret Power", power);
        telemetry.update();
    }


    @Override
    public void loop() {
        drive.updatePoseEstimate();
        AimTo(blueGoalPose, drive.localizer.getPose());
    }

    public void setPower(double power) {
        turretLeft.setPower(power);
        turretRight.setPower(power);
    }
}
