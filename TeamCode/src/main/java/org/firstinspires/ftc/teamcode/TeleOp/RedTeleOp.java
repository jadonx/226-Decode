package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.PinPoint;
import org.firstinspires.ftc.teamcode.Subsystems.Supporters.PoseStorage;

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
        robot = new Robot(hardwareMap, PinPoint.AllianceColor.RED, gamepad1, telemetry, PoseStorage.getX(), PoseStorage.getY(), PoseStorage.getHeading());
        robot.start();
    }

    @Override
    public void loop() {
        robot.update();
    }
}
