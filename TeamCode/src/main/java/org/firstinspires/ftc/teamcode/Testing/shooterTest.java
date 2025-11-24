package org.firstinspires.ftc.teamcode.Testing;

import static org.firstinspires.ftc.teamcode.Constants.HMMotorIntake;
import static org.firstinspires.ftc.teamcode.Constants.HMMotorPopper;
import static org.firstinspires.ftc.teamcode.Constants.HMMotorShooter1;
import static org.firstinspires.ftc.teamcode.Constants.HMMotorShooter2;
import static org.firstinspires.ftc.teamcode.Constants.HMServoPopper;
import static org.firstinspires.ftc.teamcode.Constants.HMServoTurretLeft;
import static org.firstinspires.ftc.teamcode.Constants.HMServoTurretRight;
import static org.firstinspires.ftc.teamcode.Constants.HMServobackSpin;
import static org.firstinspires.ftc.teamcode.Constants.HMServospinDexer;
import static org.firstinspires.ftc.teamcode.Constants.HMSpindexerEncoder;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
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

    public static double unJamTime = 150;
    public static double jamThreshold = 120;
    public static double angleDiff = 2;

    private ElapsedTime runtime = new ElapsedTime();
    DcMotorEx shooter1, shooter2, spinner, intake;

    Servo popper, cover;
    AS5600Encoder spinEncoder;

    CRServo bigSpin, turret1, turret2;






    public void init(){
        shooter1 = hardwareMap.get(DcMotorEx.class, HMMotorShooter1);
        shooter2 = hardwareMap.get(DcMotorEx.class, HMMotorShooter2 );
        spinner = hardwareMap.get(DcMotorEx.class, HMMotorPopper);
        intake = hardwareMap.get(DcMotorEx.class, HMMotorIntake);
        popper = hardwareMap.get(Servo.class, HMServoPopper);
        cover = hardwareMap.get(Servo.class, HMServobackSpin);
        bigSpin = hardwareMap.get(CRServo.class, HMServospinDexer );
        turret1 = hardwareMap.get(CRServo.class, HMServoTurretLeft );
        turret2 = hardwareMap.get(CRServo.class, HMServoTurretRight);
        spinEncoder = hardwareMap.get(AS5600Encoder.class, HMSpindexerEncoder );
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterSpeed = 0;
        coverPos = 1;
        cover.setPosition(coverPos);
        popper.setPosition(0.45);
        runtime.reset();
        bigSpinSpeed =0;
        intake.setPower(0);
        bigSpin.setPower(0);
        spinSpeed = 0;
        isJammed = false;

    }

    public void loop(){
        if(gamepad1.a){
            shooter1.setVelocity(shooterSpeed);
            shooter2.setVelocity(shooterSpeed);
            spinner.setVelocity(-2800);
        }

        if(gamepad1.b){
            spinSpeed = -2800;
            spinner.setVelocity(spinSpeed);
        }





        if(gamepad1.y && (spinEncoder.getAngleDegrees() > 290.05 || spinEncoder.getAngleDegrees() < 289.5)){
            bigSpin.setPower(0.025);
        }

        if(gamepad1.dpad_up){
            popper.setPosition(0.485);
        }

        if(gamepad1.dpad_down){
            popper.setPosition(0.45);
        }

        if(gamepad2.dpad_down && coverPos < 0.96){
            coverPos += 0.03;
            cover.setPosition(coverPos);
        }

        if(gamepad2.dpad_up && coverPos > 0){

            coverPos -= 0.03;
            cover.setPosition(coverPos);
        }

        if(gamepad2.a){
            if(turret1.getPower() < 0.05 && turret2.getPower() > -0.05){
                turret1.setPower(1);
                turret2.setPower(1);
            } else{
                turret1.setPower(-turret1.getPower());
                turret2.setPower(-turret2.getPower());
            }

        }



        if(gamepad2.y){
            turret1.setPower(0);
            turret2.setPower(0);
        }

        if(gamepad2.x){
            intake.setPower(0.8);

        }

        if(gamepad1.dpad_right){
            bigSpinSpeed = 1;
            bigSpin.setPower(bigSpinSpeed);
        }

        if(gamepad1.dpad_left){
            bigSpinSpeed = -1;
            bigSpin.setPower(bigSpinSpeed);
        }

        if(gamepad1.right_bumper){
            bigSpinSpeed = 0;
            bigSpin.setPower(0);
        }

        if(Math.abs(bigSpinSpeed) > 0.05 && !isJammed && intake.getPower() > 0.05 ){
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
            intake.setPower(0);
            if(runtime.milliseconds() - jamCool > unJamTime){
                 bigSpin.setPower(bigSpinSpeed);
                 intake.setPower(0.8);
                 jamCool = -1;
                isJammed = false;
                jamStart = -1;
            }
        }


        if(!isJammed){
            lastAngle = spinEncoder.getAngleDegrees();
        }




        telemetry.addData("Power", shooterSpeed);
        telemetry.addData("Velocity", shooter1.getVelocity());
        telemetry.addData("Popper Spinner Speed", spinner.getVelocity());
        telemetry.addData("cover pos", coverPos);
        telemetry.addData("spin Angle", spinEncoder.getAngleDegrees ());
        telemetry.addData("isJammed?", isJammed);
        telemetry.addData("runtime", runtime.milliseconds());
        telemetry.addData("bigSPinSpeed", bigSpinSpeed);
        telemetry.update();

    }



}
