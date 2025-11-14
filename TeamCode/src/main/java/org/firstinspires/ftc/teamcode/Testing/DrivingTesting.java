package org.firstinspires.ftc.teamcode.Testing;

import static org.firstinspires.ftc.teamcode.Constants.HMLimelight;
import static org.firstinspires.ftc.teamcode.Constants.HMMotorIntake;
import static org.firstinspires.ftc.teamcode.Constants.HMMotorBackLeft;
import static org.firstinspires.ftc.teamcode.Constants.HMMotorBackRight;
import static org.firstinspires.ftc.teamcode.Constants.HMMotorFrontLeft;
import static org.firstinspires.ftc.teamcode.Constants.HMMotorFrontRight;
import static org.firstinspires.ftc.teamcode.Constants.HMServospinDexer;
import static org.firstinspires.ftc.teamcode.Constants.HMSpindexerEncoder;
import static org.firstinspires.ftc.teamcode.Constants.HMimu;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Encoder.AS5600Encoder;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake.TurretSubsystem;

@TeleOp(name="Drive Testing")
public class DrivingTesting extends OpMode {
    DcMotor frontLeft, frontRight, backLeft, backRight,intake;
    CRServo bigSpin;
    IMU imu;
    private TurretSubsystem turret;
    private FtcDashboard dashboard;
    private Limelight3A limelight;
    private double tX;
    double bigSpinSpeed =0;
    public double jamStart = -1;
    public double jamCool = -1;
    public double lastAngle = -1;
    public boolean isJammed;

    public static double unJamTime = 250;
    public static double jamThreshold = 100;
    public static double angleDiff = 3;
    private ElapsedTime runtime = new ElapsedTime();
    AS5600Encoder spinEncoder;

    private boolean trackingEnabled = false;
    private boolean togglePressed = false;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, HMMotorFrontLeft);
        frontRight = hardwareMap.get(DcMotor.class, HMMotorFrontRight);
        backLeft = hardwareMap.get(DcMotor.class, HMMotorBackLeft);
        backRight = hardwareMap.get(DcMotor.class, HMMotorBackRight);
        intake = hardwareMap.get(DcMotorEx.class, HMMotorIntake);
        bigSpin = hardwareMap.get(CRServo.class, HMServospinDexer);
        spinEncoder = hardwareMap.get(AS5600Encoder.class, HMSpindexerEncoder);

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, HMimu);

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));

        imu.initialize(parameters);

//        turret = new TurretSubsystem(this);
//        turret.init();

        limelight = hardwareMap.get(Limelight3A.class, HMLimelight);
        limelight.pipelineSwitch(1);
        limelight.start();

        telemetry.addLine("Turret Initialized");
        telemetry.addLine("Press [X] to toggle auto-tracking mode");
        telemetry.update();
    }

    @Override
    public void loop() {
        double y = gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        if (gamepad1.a) {
            imu.resetYaw();
        }
        if(gamepad1.b){
            bigSpin.setPower(1);
            intake.setPower(1);
        }

        if(gamepad1.y){
            bigSpin.setPower(0);
            intake.setPower(0);
        }


        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);


        if(gamepad1.x){
            intake.setPower(1);
        }

        if(gamepad1.dpad_right){
            bigSpinSpeed = 1;
            bigSpin.setPower(bigSpinSpeed);
        }

        if(Math.abs(bigSpinSpeed) > 0.05 && !isJammed){
            if(Math.abs(spinEncoder.getAngleDegrees() - lastAngle) < angleDiff){
                if(jamStart == -1){
                    jamStart = runtime.milliseconds();
                }

                if(runtime.milliseconds() - jamStart > jamThreshold){
                    isJammed = true;


                }
            } else{
                jamStart = -1;
            }
        }

        if(isJammed){
            if(jamCool == -1){
                jamCool = runtime.milliseconds();
            }
            bigSpin.setPower(-0.5);
            intake.setPower(-1);
            if(runtime.milliseconds() - jamCool > unJamTime){
                bigSpin.setPower(bigSpinSpeed);
                intake.setPower(1);
                jamCool = -1;
                isJammed = false;
                jamStart = -1;
            }
        }


        if(!isJammed){
            lastAngle = spinEncoder.getAngleDegrees();
        }


        /*
        if (trackingEnabled) {
            turret.trackTarget(tX);
            telemetry.addLine("Mode: Tracking Apriltag");
            telemetry.addData("Bot Rotation: ", botHeading);
            telemetry.addData("Tracking Enabled", trackingEnabled);

        } else {
            double manualPower = gamepad2.right_stick_x * 0.5; // reduce sensitivity
            turret.manualPower(manualPower);
            telemetry.addLine("Mode: Manual Control");
            telemetry.addData("Bot Rotation: ", botHeading);
            telemetry.addData("Tracking Enabled", trackingEnabled);

        }
        */

        telemetry.addData("spin Angle", spinEncoder.getAngleDegrees ());
        telemetry.addData("isJammed?", isJammed);
        telemetry.addData("runtime", runtime.milliseconds());
        telemetry.update();


    }
}
