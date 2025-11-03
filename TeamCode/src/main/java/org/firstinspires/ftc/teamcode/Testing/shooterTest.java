package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.AS5600Encoder;

@Config
@TeleOp(name = "Shooter test")
public class shooterTest extends OpMode {
    public static double shooterSpeed;
    public static double spinSpeed;
    public static double coverPos;
    public static double spinDexerPos;
    DcMotorEx shooter1, shooter2, spinner;

    Servo spinDexer, cover;
    AS5600Encoder turretEncoder, spinEncoder;

    CRServo bigSpin, turret1, turret2;




    public void init(){
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        spinner = hardwareMap.get(DcMotorEx.class, "spinner");
        spinDexer = hardwareMap.get(Servo.class, "spinDexerServo");
        cover = hardwareMap.get(Servo.class, "cover");
        bigSpin = hardwareMap.get(CRServo.class, "leftCRServo");
        turret1 = hardwareMap.get(CRServo.class, "turret1");
        turret2 = hardwareMap.get(CRServo.class, "turret2");
        turretEncoder = hardwareMap.get(AS5600Encoder.class, "turretEncoder");
        spinEncoder = hardwareMap.get(AS5600Encoder.class, "spinEncoder");
        shooterSpeed = 0;
        coverPos = 0;
    }

    public void loop(){
        if(gamepad1.a){
            shooterSpeed = 0;
            shooter1.setPower(shooterSpeed);
            shooter2.setPower(shooterSpeed);
        }

        if(gamepad1.b){
            spinSpeed = -1;
            spinner.setPower(spinSpeed);
        }

        if(gamepad1.x){
            shooter1.setPower(shooterSpeed);
            shooter2.setPower(shooterSpeed);
        }

        if(gamepad1.y){
            shooterSpeed = 1;
            shooter1.setPower(shooterSpeed);
            shooter2.setPower(shooterSpeed);
        }

        if(gamepad1.dpad_up){
            spinDexer.setPosition(0.75);
        }

        if(gamepad1.dpad_down){
            spinDexer.setPosition(0);
        }

        if(gamepad2.dpad_down){
            coverPos += 0.05;
            cover.setPosition(coverPos);
        }

        if(gamepad2.dpad_up){
            coverPos -= 0.05;
            cover.setPosition(coverPos);
        }

        if(gamepad2.a){

            turret1.setPower(1);
            turret2.setPower(1);
        }

        if(gamepad2.b){
            turret1.setPower(-1);
            turret2.setPower(-1);
        }

        if(gamepad1.dpad_right){
            bigSpin.setPower(1);
        }

        if(gamepad1.dpad_left){
            bigSpin.setPower(-1);
        }

        telemetry.addData("Speed", shooterSpeed);
        telemetry.addData("Angle", turretEncoder.getAngleDegrees());
        telemetry.addData("Angle", spinEncoder.getAngleDegrees ());
    }
}
