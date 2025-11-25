package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

@Config
@TeleOp(name="ServoLimit_Tester", group="Tester")
public class ServoLimitTest extends OpMode {
    Servo popperServo;

    public static double popperServoPosition = 0;

    TelemetryPacket packet;
    FtcDashboard dashboard;

    @Override
    public void init() {
        popperServo = hardwareMap.get(Servo.class, Constants.HMServoPopper);

        packet = new TelemetryPacket();
        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void loop() {
        popperServo.setPosition(popperServoPosition);

        packet.put("popper servo pos ", popperServoPosition);
        dashboard.sendTelemetryPacket(packet);
    }
}
