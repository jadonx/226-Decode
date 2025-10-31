package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "Shooter test")
public class shooterTest extends OpMode {
    public static double shooterSpeed;
    public static double spinSpeed;
    public static double coverPos;
    public static double basePos;
    DcMotorEx shooter1, shooter2, spinner;

    Servo base;



    public void init(){
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        spinner = hardwareMap.get(DcMotorEx.class, "spinner");
        base = hardwareMap.get(Servo.class, "base");
        shooterSpeed = 0;
    }

    public void loop(){
        if(gamepad1.a){
            shooterSpeed = 0;
            shooter1.setPower(shooterSpeed);
            shooter2.setPower(shooterSpeed);
        }

        if(gamepad1.b){
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

        if(gamepad1.dpad_down){
            base.setPosition(basePos);
        }

        if(gamepad1.dpad_up){
            base.setPosition(0);
        }

        telemetry.addData("Speed", shooterSpeed);
    }
}
