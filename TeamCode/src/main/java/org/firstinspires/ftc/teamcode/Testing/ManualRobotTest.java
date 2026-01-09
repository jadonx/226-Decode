package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Subsystems.PinPoint;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

@Config
@TeleOp(name="ManualRobotTest", group = "Test")
public class ManualRobotTest extends OpMode {
    public static int targetVelocity;
    public static double hoodAngle;
    public static double spindexerSpeedShoot;
    public static double spindexerSpeedStop;
    public static double spindexerSpeedIntake;
    public static double popperPos;
    public static double popperVelocity;
    public static double intakeSpeed;
    public static double distance = 0;
    public static double turretAngle;


    // Turret/Spindexer Servos
    CRServo leftServo, rightServo;
    CRServo spindexerServo;

    Servo popperServo;
    Servo hoodServo;

    // Launcher Motors
    DcMotorEx popper, intake;

    // Drive Motors
    IMU imu;

    float[] hsv = new float[3];

    FtcDashboard dashboard;
    TelemetryPacket packet;

    FieldCentricDrive drive;

    Spindexer spindexer;

    Launcher launcher;

    PinPoint pinpoint;

    Turret turret;

    public static boolean isShooting = false;
    public static boolean spindexerShooting = false;
    public static boolean isReadyToShoot = false;
    public static boolean isUsingTurret = false;



    @Override
    public void init() {
        spindexer = new Spindexer(hardwareMap);

        popper = hardwareMap.get(DcMotorEx.class, Constants.HMMotorPopper);
        popper.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        popper.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = hardwareMap.get(DcMotorEx.class, Constants.HMMotorIntake);

        spindexerServo = hardwareMap.get(CRServo.class, Constants.HMServospinDexer);
        popperServo = hardwareMap.get(Servo.class, Constants.HMServoPopper);
        hoodServo = hardwareMap.get(Servo.class, Constants.HMServobackSpin);

        launcher = new Launcher(hardwareMap);
        turret = new Turret(hardwareMap);

        intakeSpeed = -0.5;
        popperPos = 0.007;
        popperVelocity = 2300;
        spindexerSpeedShoot = 0.2;
        spindexerSpeedIntake = 0.15;
        spindexerSpeedStop = 0;
        targetVelocity = 0;

        drive = new FieldCentricDrive(hardwareMap);
        drive.resetIMU();

        pinpoint = new PinPoint(hardwareMap, PinPoint.AllianceColor.RED, 0, 0, 0);

        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
    }

    @Override
    public void loop() {
        drive.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        pinpoint.updatePose();

        if (isReadyToShoot) {
            popper.setVelocity(popperVelocity); // 2300
            intake.setPower(intakeSpeed); // -0.5
            popperServo.setPosition(popperPos); // 0.007
        } else {
            popper.setVelocity(0);
            intake.setPower(0);
            popperServo.setPosition(0);
        }
        if (gamepad1.bWasPressed()) {
            isReadyToShoot = !isReadyToShoot;
        }

        if (gamepad1.x) {
            drive.resetIMU();
        }

        if (isShooting) {
            //launcher.update(pinpoint.getDistanceToGoal());
        } else {
            launcher.stopLauncher();
        }
        if (gamepad1.aWasPressed()) {
            isShooting = !isShooting;
        }

        if (spindexerShooting) {
            spindexerServo.setPower(spindexerSpeedShoot); // 0.25 or 0.2
        } else {
            spindexerServo.setPower(spindexerSpeedStop);
        }

        if (gamepad1.yWasPressed()) {
            spindexerShooting = !spindexerShooting;
        }

        if (isUsingTurret) {
            double desired = 90-pinpoint.getAngleToGoal();
            turret.goToAngle(desired);
        } else {
            turret.goToAngle(pinpoint.getHeading());
        }

        if (gamepad1.dpadUpWasPressed()) {
            isUsingTurret = !isUsingTurret;
        }


//        packet.put("X Position", pinpoint.getXCoordinate(pinpoint.getPose(), DistanceUnit.INCH));
//        packet.put("Y Position", pinpoint.getYCoordinate(pinpoint.getPose(), DistanceUnit.INCH));
//        packet.put("GOAL X Position", pinpoint.getXCoordinate(pinpoint.getPoseGoal(), DistanceUnit.INCH));
//        packet.put("GOAL Y Position", pinpoint.getYCoordinate(pinpoint.getPoseGoal(), DistanceUnit.INCH));

//        packet.put("Distance from bot to goal", pinpoint.getDistanceToGoal());
        packet.put("bot heading - angle to goal ", pinpoint.getHeading() - pinpoint.getAngleToGoal());
        packet.put("robot heading: ", pinpoint.getHeading());
        packet.put("Desired Angle", pinpoint.getAngleToGoal());
        packet.put("Turret Current Angle: ", turret.getTurretAngle());


//        launcher.calculateTargetVelocity(pinpoint.getDistanceToGoal());
//        launcher.calculateTargetAngle(pinpoint.getDistanceToGoal());
//
//        packet.put("Target velocity ", launcher.getTargetVelocity());
//        packet.put("Target angle ", launcher.getTargetCoverAngle());
//
        dashboard.sendTelemetryPacket(packet);
    }
}