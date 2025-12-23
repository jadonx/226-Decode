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
import org.firstinspires.ftc.teamcode.Subsystems.PinPoint;
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

    PinPoint pinpoint;

    public static boolean isShooting = false;

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

        intakeSpeed = -0.5;
        popperPos = 0.007;
        popperVelocity = 2300;
        spindexerSpeed = 0;
        targetVelocity = 0;

        drive = new FieldCentricDrive(hardwareMap);

        pinpoint = new PinPoint(hardwareMap, PinPoint.AllianceColor.RED);

        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
    }

    @Override
    public void loop() {
        pinpoint.updatePose();

        popper.setVelocity(popperVelocity); // 2300

        intake.setPower(intakeSpeed); // -0.5

        spindexerServo.setPower(spindexerSpeed); // 0.25 or 0.2

        popperServo.setPosition(popperPos); // 0.007

        drive.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        if (gamepad1.x) {
            drive.resetIMU();
        }

        if (isShooting) {
            double targetVel = launcher.calculateTargetVelocity(pinpoint.getDistanceToGoal());
            double targetAng = launcher.calculateTargetAngle(pinpoint.getDistanceToGoal());
            hoodServo.setPosition(targetAng);
            launcher.setTargetVelocity((int)targetVel);
            launcher.update();
        } else {
            hoodServo.setPosition(0.0);
            launcher.setTargetVelocity(0);
            launcher.update();
        }

        if (gamepad1.aWasPressed()) {
            isShooting = !isShooting;
        }

        packet.put("X Position", pinpoint.getPose().position.x);
        packet.put("Y Position", pinpoint.getPose().position.y);

        packet.put("GOAL X Position", pinpoint.getPoseGoal().position.x);
        packet.put("GOAL Y Position", pinpoint.getPoseGoal().position.y);

        packet.put("Distance from bot to goal", pinpoint.getDistanceToGoal());

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