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
    FtcDashboard dashboard;
    TelemetryPacket packet;
    Turret turret;
    PinPoint pinpoint;
    Launcher launcher;

    public static double coverPos;
    public static double turretAngle;

    @Override
    public void init() {
        turret = new Turret(hardwareMap);
        pinpoint = new PinPoint(hardwareMap, PinPoint.AllianceColor.RED, 0, 0, 0);
        dashboard = FtcDashboard.getInstance();
        launcher = new Launcher(hardwareMap);
        packet = new TelemetryPacket();
    }

    @Override
    public void loop() {
        pinpoint.updatePose();
        // turret.goToAngle(turretAngle);

        launcher.setTargetCoverAngle(coverPos);
        packet.put("bot heading - angle to goal ", pinpoint.getHeading() - pinpoint.getAngleToGoal());
        packet.put("robot heading: ", pinpoint.getHeading());
        packet.put("Desired Angle", pinpoint.getAngleToGoal());
        packet.put("Turret Current Angle: ", turret.getTurretAngle());

        dashboard.sendTelemetryPacket(packet);
    }
}