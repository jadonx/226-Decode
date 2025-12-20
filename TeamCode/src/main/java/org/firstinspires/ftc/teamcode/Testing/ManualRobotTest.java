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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.FieldCentricDrive;

@Config
@TeleOp(name="ManualRobotTest", group = "Test")
public class ManualRobotTest extends OpMode {
    public static double shooterSpeed1;
    public static double shooterSpeed2;
    public static double spinSpeed;

    public static double turretSpeed;

    public static double spindexerSpeed;

    public static double intakeSpeed;

    public static double popperPos;

    // Turret/Spindexer Servos
    CRServo leftServo, rightServo;
    CRServo spindexerServo;

    Servo popperServo;

    // Launcher Motors
    DcMotorEx shooter1, shooter2, spinner, intake;

    // Drive Motors
    DcMotor frontLeft, frontRight, backLeft, backRight;
    IMU imu;

    float[] hsv = new float[3];

    FtcDashboard dashboard;
    TelemetryPacket packet;

    FieldCentricDrive drive;

    @Override
    public void init() {
        shooter1 = hardwareMap.get(DcMotorEx.class, Constants.HMMotorShooter1);
        shooter2 = hardwareMap.get(DcMotorEx.class, Constants.HMMotorShooter2);

        shooter1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        spinner = hardwareMap.get(DcMotorEx.class, Constants.HMMotorPopper);
        spinner.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = hardwareMap.get(DcMotorEx.class, Constants.HMMotorIntake);

        spindexerServo = hardwareMap.get(CRServo.class, Constants.HMServospinDexer);
        popperServo = hardwareMap.get(Servo.class, Constants.HMServoPopper);

        frontLeft = hardwareMap.get(DcMotor.class, Constants.HMMotorFrontLeft);
        frontRight = hardwareMap.get(DcMotor.class, Constants.HMMotorFrontRight);
        backLeft = hardwareMap.get(DcMotor.class, Constants.HMMotorBackLeft);
        backRight = hardwareMap.get(DcMotor.class, Constants.HMMotorBackRight);

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterSpeed1 = 0; shooterSpeed2 = 0;
        spinSpeed = 0;
        turretSpeed = 0;
        spindexerSpeed = 0;
        intakeSpeed = 0;

        drive = new FieldCentricDrive(hardwareMap);

        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
    }

    @Override
    public void loop() {
        shooter1.setVelocity(shooterSpeed1);
        shooter2.setVelocity(shooterSpeed2);

        spinner.setPower(spinSpeed);

        intake.setPower(intakeSpeed);

        spindexerServo.setPower(spindexerSpeed);

        popperServo.setPosition(popperPos);

        drive.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        if (gamepad1.x) {
            drive.resetIMU();
        }
    }
}