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
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@Config
@TeleOp(name="Spindexer Turret Prototype")
public class SpindexerTurretPrototype extends OpMode {
    public static double shooterSpeed;
    public static double spinSpeed;

    public static double turretSpeed;

    public static double spindexerSpeed;

    public static double intakeSpeed;

    // Turret/Spindexer Servos
    CRServo leftServo, rightServo;
    CRServo spindexerServo;

    // Launcher Motors
    DcMotor shooter1, shooter2, spinner, intake;

    // Color Sensor
    NormalizedColorSensor colorSensor;

    float[] hsv = new float[3];

    FtcDashboard dashboard;
    TelemetryPacket packet;

    @Override
    public void init() {
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");

        spinner = hardwareMap.get(DcMotorEx.class, "spinner");
        spinner.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = hardwareMap.get(DcMotorEx.class, "intake");

        // leftServo = hardwareMap.get(CRServo.class, "leftCRServo");
        // rightServo = hardwareMap.get(CRServo.class, "rightCRServo");

        // spindexerServo = hardwareMap.get(CRServo.class, "spindexerServo");

        shooterSpeed = 0;
        spinSpeed = 0;
        turretSpeed = 0;
        spindexerSpeed = 0;
        intakeSpeed = 0;

        // colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
    }

    @Override
    public void loop() {
        shooter1.setPower(shooterSpeed);
        shooter2.setPower(shooterSpeed);

        spinner.setPower(spinSpeed);

        intake.setPower(intakeSpeed);
    }
}
