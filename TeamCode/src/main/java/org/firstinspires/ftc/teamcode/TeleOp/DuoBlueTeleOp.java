package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.RoadRunnerPinPoint;

@TeleOp(name="DuoBlueTeleOp", group="!TeleOp")
public class DuoBlueTeleOp extends OpMode {
    Robot robot;
    FtcDashboard dashboard;
    TelemetryPacket packet;

    @Override
    public void init() {
        packet = new TelemetryPacket();
        dashboard = FtcDashboard.getInstance();
        robot = new Robot(hardwareMap, RoadRunnerPinPoint.AllianceColor.BLUE, gamepad1, gamepad2, telemetry, packet, dashboard);
        robot.start();
    }

    @Override
    public void loop() {
        robot.update();
    }
}
