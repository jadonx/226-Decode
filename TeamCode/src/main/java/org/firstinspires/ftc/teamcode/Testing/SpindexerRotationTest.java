package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;

@Config
@TeleOp(name="SpindexerRotationTest")
public class SpindexerRotationTest extends OpMode {
    Spindexer spindexer;
    public static double target;
    public static Spindexer.SpindexerMode spindexerMode;

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
        spindexer.setTargetAngle(target);
        spindexer.update();

        spindexer.setMode(spindexerMode);

        if (gamepad1.aWasPressed()) {
            if (spindexerMode == Spindexer.SpindexerMode.SHORTEST_PATH) {
                spindexerMode = Spindexer.SpindexerMode.FULL_ROTATION;
            }
            else {
                spindexerMode = Spindexer.SpindexerMode.SHORTEST_PATH;
            }
        }

        packet.put("wrapped angle ", spindexer.getWrappedAngle());
        packet.put("unwrapped angle ", spindexer.getUnwrappedAngle());
        packet.put("mode ", spindexerMode);
        dashboard.sendTelemetryPacket(packet);
    }
}
