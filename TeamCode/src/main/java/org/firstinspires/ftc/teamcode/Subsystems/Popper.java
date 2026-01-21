package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;
@Config
public class Popper {
    private Servo popperServo;
    private DcMotorEx popperMotor;
    public static double popperPos = 0.009;

    private int targetVelocity;

    public Popper(HardwareMap hardwareMap) {
        popperMotor = hardwareMap.get(DcMotorEx.class, Constants.HMMotorPopper);
        popperMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        popperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        popperMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        popperServo = hardwareMap.get(Servo.class, Constants.HMServoPopper);
        targetVelocity = 0;
    }

    public void setTargetVelocity(int targetVelocity) {
        this.targetVelocity = targetVelocity;
        popperMotor.setVelocity(targetVelocity);
    }

    public int getTargetVelocity() {
        return targetVelocity;
    }

    public double getPopperVelocity() {
        return popperMotor.getVelocity();
    }

    public boolean atTargetVelocity(int threshold) {
        return Math.abs(getPopperVelocity() - getTargetVelocity()) < threshold;
    }

    public void deactivatePopper() {
        pullOutPopper();
        popperMotor.setVelocity(0);
    }

    public void pushInPopper() {
        popperServo.setPosition(popperPos);
    }

    public void pullOutPopper() {
        popperServo.setPosition(0);
    }
}