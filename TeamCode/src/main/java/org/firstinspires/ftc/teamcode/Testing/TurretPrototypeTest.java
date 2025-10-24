package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name="Turret Prototype Test")
public class TurretPrototypeTest extends OpMode {
    public static double shooterSpeed;
    public static double spinSpeed;

    public static double turretSpeed;

    CRServo leftServo, rightServo;

    DcMotor shooter1, shooter2, spinner;

    FtcDashboard dashboard;
    TelemetryPacket packet;

    @Override
    public void init() {
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");

        spinner = hardwareMap.get(DcMotorEx.class, "spinner");
        spinner.setDirection(DcMotorSimple.Direction.REVERSE);

        leftServo = hardwareMap.get(CRServo.class, "leftCRServo");
        rightServo = hardwareMap.get(CRServo.class, "rightCRServo");

        shooterSpeed = 0;
        spinSpeed = 0;
        turretSpeed = 0;

        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
    }

    @Override
    public void loop() {
        shooter1.setPower(shooterSpeed);
        shooter2.setPower(shooterSpeed);

        spinner.setPower(spinSpeed);

        leftServo.setPower(turretSpeed);
        rightServo.setPower(turretSpeed);
    }
}
