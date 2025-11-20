package org.firstinspires.ftc.teamcode.Testing;

import static org.firstinspires.ftc.teamcode.Constants.HMMotorPopper;
import static org.firstinspires.ftc.teamcode.Constants.HMServoPopper;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "popper tester")
public class popperTester extends OpMode {
    public static double servoPos = 0.5;
    Servo popper;
    DcMotorEx spinner;

    public void init(){
        popper = hardwareMap.get(Servo.class, HMServoPopper);
        spinner = hardwareMap.get(DcMotorEx.class, HMMotorPopper);
    }
    public void loop(){
        if(gamepad1.a){
            popper.setPosition(servoPos);
        }

        if(gamepad1.b){
            popper.setPosition(0.475);
        }

        if(gamepad1.x){
            spinner.setPower(-1);
        }

        if(gamepad1.y){
            popper.setPosition(0.45);
            spinner.setPower(0);
        }

        telemetry.addData("pos", servoPos);
    }
}
