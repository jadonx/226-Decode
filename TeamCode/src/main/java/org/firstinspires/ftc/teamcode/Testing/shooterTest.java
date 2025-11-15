package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Encoder.AS5600Encoder;

@Config
@TeleOp(name = "Shooter test")
public class shooterTest extends OpMode {
    public static double shooterSpeed;
    public static double spinSpeed;
    public static double bigSpinSpeed;
    public static double coverPos;
    public static double spinDexerPos =0;
    public double jamStart = -1;
    public double jamCool = -1;
    public double lastAngle = -1;
    public boolean isJammed;

    public static double unJamTime = 250;
    public static double jamThreshold = 100;
    public static double angleDiff = 3;

    private ElapsedTime runtime = new ElapsedTime();
    DcMotorEx shooter1, shooter2, spinner, intake;

    Servo spinDexer, cover;
    AS5600Encoder turretEncoder, spinEncoder;

    CRServo bigSpin, turret1, turret2;




    public void init(){
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        spinner = hardwareMap.get(DcMotorEx.class, "spinner");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
//        spinDexer = hardwareMap.get(Servo.class, "spinDexerServo");
        cover = hardwareMap.get(Servo.class, "cover");
        bigSpin = hardwareMap.get(CRServo.class, "leftCRServo");
        turret1 = hardwareMap.get(CRServo.class, "turret");
        turret2 = hardwareMap.get(CRServo.class, "TR");
        turretEncoder = hardwareMap.get(AS5600Encoder.class, "turretEncoder");
        spinEncoder = hardwareMap.get(AS5600Encoder.class, "spinEncoder");
        shooterSpeed = 0;
        coverPos = 0;
//        spinDexer.setPosition(0);
        runtime.reset();
        bigSpinSpeed =0;
        intake.setPower(0);
        bigSpin.setPower(0);
        spinSpeed = 0;
        isJammed = false;
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

//        if(gamepad1.dpad_up){
//            spinDexer.setPosition(0.75);
//        }
//
//        if(gamepad1.dpad_down){
//            spinDexer.setPosition(0);
//        }

        if(gamepad2.dpad_down && coverPos < 1){
            coverPos += 0.005;
            cover.setPosition(coverPos);
        }

        if(gamepad2.dpad_up){

            coverPos -= 0.005;
            cover.setPosition(coverPos);
        }

        if(gamepad2.a){

            turret1.setPower(1);
            turret2.setPower(1);
        }


        if(gamepad2.y){
            turret1.setPower(0);
            turret2.setPower(0);
        }

        if(gamepad2.x){
            intake.setPower(1);

        }

        if(gamepad1.dpad_right){
            bigSpinSpeed = 1;
            bigSpin.setPower(bigSpinSpeed);
        }

        if(gamepad1.dpad_left){
            bigSpinSpeed = -1;
            bigSpin.setPower(-1);
        }

        if(gamepad1.right_bumper){
            bigSpinSpeed = 0;
            bigSpin.setPower(0);
        }

        if(Math.abs(bigSpinSpeed) > 0.05 && !isJammed){
            if(Math.abs(spinEncoder.getAngleDegrees() - lastAngle) < angleDiff){
                if(jamStart == -1){
                    jamStart = runtime.milliseconds();
                }

                if(runtime.milliseconds() - jamStart > jamThreshold){
                    isJammed = true;

                    intake.setPower(-1);
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




        telemetry.addData("Speed", shooterSpeed);
        telemetry.addData("cover pos", coverPos);
        telemetry.addData("turret Angle", turretEncoder.getAngleDegrees());
        telemetry.addData("spin Angle", spinEncoder.getAngleDegrees ());
        telemetry.addData("isJammed?", isJammed);
        telemetry.addData("runtime", runtime.milliseconds());
        telemetry.addData("bigSPinSpeed", bigSpinSpeed);
    }



}