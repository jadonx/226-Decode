package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

public class Popper {
    private Servo popperServo;
    private DcMotorEx popperMotor;

    public Popper(HardwareMap hardwareMap) {
        popperMotor = hardwareMap.get(DcMotorEx.class, Constants.HMMotorPopper);
        popperMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        popperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        popperMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        popperServo = hardwareMap.get(Servo.class, Constants.HMServoPopper);
    }

    public void pushInPopper() {
        popperServo.setPosition(0.007);
    }

    public void pushOutPopper() {
        popperServo.setPosition(0.0);
    }

    public void spinPopper() {
        popperMotor.setVelocity(22300);
    }

    public void stopPopper() {
        popperMotor.setVelocity(0);
    }

    public void deactivatePopper() {
        pushOutPopper();
        stopPopper();
    }
}