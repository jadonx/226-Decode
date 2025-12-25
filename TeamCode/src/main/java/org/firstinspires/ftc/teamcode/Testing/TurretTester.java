package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Subsystems.PinPoint;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

@Config
@TeleOp(name="TurretTester", group = "Test")
public class TurretTester extends OpMode {
    public static double turretAngle;
    FtcDashboard dashboard;
    TelemetryPacket packet;
    Turret turret;
    PinPoint pinpoint;


    @Override
    public void init() {
        turret = new Turret(hardwareMap);
        pinpoint = new PinPoint(hardwareMap, PinPoint.AllianceColor.RED);
        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
    }

    @Override
    public void loop() {
//        turret.goToAngle(turretAngle);
        turret.setPower(1);

        packet.put("Distance from bot to goal", pinpoint.getDistanceToGoal());
        packet.put("Angle from bot to goal", pinpoint.getAngleToGoal());

        packet.put("Turret Current Angle: ", turret.getTurretAngle());

        dashboard.sendTelemetryPacket(packet);
    }
}