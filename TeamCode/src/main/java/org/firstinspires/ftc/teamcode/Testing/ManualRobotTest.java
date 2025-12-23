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
import org.firstinspires.ftc.teamcode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;

@Config
@TeleOp(name="ManualRobotTest", group = "Test")
public class ManualRobotTest extends OpMode {
    public static int targetVelocity;
    public static double hoodAngle;
    public static double spindexerSpeed;
    public static double popperPos;
    public static double popperVelocity;
    public static double intakeSpeed;
    public static double distance = 0;

    // Turret/Spindexer Servos
    CRServo leftServo, rightServo;
    CRServo spindexerServo;

    Servo popperServo;
    Servo hoodServo;

    // Launcher Motors
    DcMotorEx shooter1, shooter2, popper, intake;

    // Drive Motors
    DcMotor frontLeft, frontRight, backLeft, backRight;
    IMU imu;

    float[] hsv = new float[3];

    FtcDashboard dashboard;
    TelemetryPacket packet;

    FieldCentricDrive drive;

    Spindexer spindexer;

    Launcher launcher;

    @Override
    public void init() {
        spindexer = new Spindexer(hardwareMap);
        shooter1 = hardwareMap.get(DcMotorEx.class, Constants.HMMotorShooter1);
        shooter2 = hardwareMap.get(DcMotorEx.class, Constants.HMMotorShooter2);
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooter1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        popper = hardwareMap.get(DcMotorEx.class, Constants.HMMotorPopper);
        popper.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        popper.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = hardwareMap.get(DcMotorEx.class, Constants.HMMotorIntake);

        spindexerServo = hardwareMap.get(CRServo.class, Constants.HMServospinDexer);
        popperServo = hardwareMap.get(Servo.class, Constants.HMServoPopper);
        hoodServo = hardwareMap.get(Servo.class, Constants.HMServobackSpin);

        frontLeft = hardwareMap.get(DcMotor.class, Constants.HMMotorFrontLeft);
        frontRight = hardwareMap.get(DcMotor.class, Constants.HMMotorFrontRight);
        backLeft = hardwareMap.get(DcMotor.class, Constants.HMMotorBackLeft);
        backRight = hardwareMap.get(DcMotor.class, Constants.HMMotorBackRight);

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        launcher = new Launcher(hardwareMap);

        intakeSpeed = -0.5;
        popperPos = 0.007;
        popperVelocity = 2300;
        spindexerSpeed = 0;
        targetVelocity = 0;

        drive = new FieldCentricDrive(hardwareMap);

        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
    }

    @Override
    public void loop() {
        setRobotDistance(distance);

        popper.setVelocity(popperVelocity); // 2300

        intake.setPower(intakeSpeed); // -0.5

        spindexerServo.setPower(spindexerSpeed); // 0.25 or 0.2

        hoodServo.setPosition(hoodAngle);

        popperServo.setPosition(popperPos); // 0.007

        drive.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        if (gamepad1.x) {
            drive.resetIMU();
        }

        launcher.setTargetVelocity(targetVelocity);
        launcher.update();

        packet.put("Launcher1 Velocity ", launcher.getVelocity1());
        packet.put("Launcher2 Velocity ", launcher.getVelocity2());
        packet.put("Average Velocity ", launcher.getVelocity());
        packet.put("Target Velocity ", targetVelocity);

        dashboard.sendTelemetryPacket(packet);
    }

    public void setRobotDistance(double distance) {
        targetVelocity = (int) ((0.0000014237*Math.pow(distance,4))-(0.000303373*Math.pow(distance,3))+(0.0297095*Math.pow(distance,2))+(1.67866*distance)+1134.53147);; // regression
        if(distance > 75){
            hoodAngle = 0;
        }else{
            hoodAngle = -0.012064*(distance)+1.25891; // regression
        }

    }
}