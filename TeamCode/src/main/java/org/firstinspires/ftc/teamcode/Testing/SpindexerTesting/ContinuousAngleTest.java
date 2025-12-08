package org.firstinspires.ftc.teamcode.Testing.SpindexerTesting;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;

@Config
@TeleOp(name="ContinuousAngle_Tester", group = "Tester")
public class ContinuousAngleTest extends OpMode {
    Spindexer spindexer;

    TelemetryPacket packet;
    FtcDashboard dashboard;

    @Override
    public void init() {
        spindexer = new Spindexer(hardwareMap);

        packet = new TelemetryPacket();
        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void loop() {
        packet.put("angle ", spindexer.getWrappedAngle());
        packet.put("error ", spindexer.getError(10));
        dashboard.sendTelemetryPacket(packet);
    }
}
