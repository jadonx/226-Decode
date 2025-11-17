package org.firstinspires.ftc.teamcode.Testing;

import static org.firstinspires.ftc.teamcode.Constants.HMMotorPopper;
import static org.firstinspires.ftc.teamcode.Constants.HMServoPopper;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "popper tester")
public class popperTester extends OpMode {

    Servo popper;
    DcMotorEx spinner;

    public void init(){
        popper = hardwareMap.get(Servo.class, HMServoPopper);
        spinner = hardwareMap.get(DcMotorEx.class, HMMotorPopper);
    }
    public void loop(){
        if(gamepad1.b){
            popper.setPosition(0.75);
        }

        if(gamepad1.x){
            spinner.setPower(1);
        }

        if(gamepad1.y){
            popper.setPosition(0.4);
            spinner.setPower(0);
        }
    }
}
