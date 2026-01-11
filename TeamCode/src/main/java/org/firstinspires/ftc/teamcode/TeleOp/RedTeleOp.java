package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.PinPoint;
import org.firstinspires.ftc.teamcode.Subsystems.RoadRunnerPinPoint;
import org.firstinspires.ftc.teamcode.Subsystems.Supporters.PoseStorage;

@Config
@TeleOp(name="RedTeleOp", group="!TeleOp")
public class RedTeleOp extends OpMode {
    Robot robot;
    FtcDashboard dashboard;
    TelemetryPacket packet;

    @Override
    public void init() {
        packet = new TelemetryPacket();
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        robot = new Robot(hardwareMap, RoadRunnerPinPoint.AllianceColor.RED, gamepad1, telemetry);
        robot.start();
    }

    @Override
    public void loop() {
        robot.update();

        packet.put("launcher vel ", robot.getLauncherVel());
        packet.put("launcher target vel ", robot.getLauncherTargetVel());

        dashboard.sendTelemetryPacket(packet);
    }
}
