package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Subsystems.Popper;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;

@Config
@TeleOp(name="PopperTest", group="Test")
public class PopperTest extends OpMode {
    Servo popper;
    DcMotorEx popperMotor;
    Launcher launcher;
    Spindexer spindexer;

    public static double popperPos = 0;
    public static double spindexerSpeed = 0;
    public static double popperSpeed = 0;
    public static double launcherSpeed;

    @Override
    public void init() {
        popper = hardwareMap.get(Servo.class, Constants.HMServoPopper);
        popperMotor = hardwareMap.get(DcMotorEx.class, Constants.HMMotorPopper);
        popperMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        popperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        popperMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        spindexer = new Spindexer(hardwareMap);
    }

    @Override
    public void loop() {
        popper.setPosition(popperPos);
        popperMotor.setVelocity(popperSpeed);
        spindexer.setPower(spindexerSpeed);
    }
}
