package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;

@Config
@TeleOp(name="SpindexerPIDTest")
public class SpindexerPIDTest extends OpMode {
    Spindexer spindexer;

    TelemetryPacket packet;
    FtcDashboard dashboard;

    public static int targetAngle;
    public static double kP, kI, kD;

    @Override
    public void init() {
        spindexer = new Spindexer(hardwareMap);

        packet = new TelemetryPacket();
        dashboard = FtcDashboard.getInstance();

        kP = 0; kI = 0; kD = 0;
    }

    @Override
    public void loop() {
        spindexer.goToAngle(targetAngle);
        spindexer.updatePID(kP, kI, kD);

        packet.put("target ", targetAngle);
        packet.put("current ", spindexer.getAngle());

        dashboard.sendTelemetryPacket(packet);
    }
}
