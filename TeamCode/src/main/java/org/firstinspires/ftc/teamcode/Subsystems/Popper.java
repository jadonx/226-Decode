package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

public class Popper {
    private Servo popperServo;
    private DcMotor popperMotor;

    public Popper(HardwareMap hardwareMap) {
        popperServo = hardwareMap.get(Servo.class, Constants.HMServoPopper);
        popperMotor = hardwareMap.get(DcMotor.class, Constants.HMMotorPopper);

        popperMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void pushInPopper() {
        popperServo.setPosition(0.007);
    }

    public void pushOutPopper() {
        popperServo.setPosition(0.0);
    }

    public void spinPopper() {
        popperMotor.setPower(1);
    }

    public void stopPopper() {
        popperMotor.setPower(0);
    }

    public void deactivatePopper() {
        pushOutPopper();
        stopPopper();
    }
}