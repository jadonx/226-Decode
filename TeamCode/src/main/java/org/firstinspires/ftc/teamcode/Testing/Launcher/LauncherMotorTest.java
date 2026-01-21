package org.firstinspires.ftc.teamcode.Testing.Launcher;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Constants;

@Config
@TeleOp(name="LauncherMotorTest", group="Test")
public class LauncherMotorTest extends OpMode {
    DcMotor left, right;
    public static double leftPower, rightPower;

    @Override
    public void init() {
        left = hardwareMap.get(DcMotor.class, Constants.HMMotorShooter1);
        right = hardwareMap.get(DcMotor.class, Constants.HMMotorShooter2);
    }

    @Override
    public void loop() {
        left.setPower(leftPower);
        right.setPower(rightPower);
    }
}
