package org.firstinspires.ftc.teamcode.Testing.Turret;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;

@Config
@TeleOp(name="TurretManualTest", group="Test")
public class TurretManualTest extends OpMode {
    CRServo turretLeft;
    CRServo turretRight;
    IMU turretEncoder;

    public static double turretLeftPower;
    public static double turretRightPower;

    @Override
    public void init() {
        turretLeft = hardwareMap.get(CRServo.class, Constants.HMServoTurretLeft);
        turretLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        turretRight = hardwareMap.get(CRServo.class, Constants.HMServoTurretRight);
        turretRight.setDirection(DcMotorSimple.Direction.REVERSE);

        turretEncoder = hardwareMap.get(IMU.class, Constants.HMTurretEncoder);
        IMU.Parameters turretParameters = new IMU.Parameters(new RevHubOrientationOnRobot( RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        turretEncoder.initialize(turretParameters);
    }

    @Override
    public void loop() {
        turretLeft.setPower(turretLeftPower);
        turretRight.setPower(turretRightPower);

        telemetry.addData("turret imu ", turretEncoder.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.update();
    }
}
