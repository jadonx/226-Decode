package org.firstinspires.ftc.teamcode.Subsystems.Spindexer;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name="Spindexer Test")
public class SpindexerTest extends OpMode {
    Spindexer spindexer;

    public static double kP;
    public static double targetAngle;
    public static double power;

    FtcDashboard dashboard;
    TelemetryPacket packet;

    @Override
    public void init() {
        spindexer = new Spindexer(hardwareMap);

        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
    }

    @Override
    public void loop() {
        if (gamepad1.b) {
            spindexer.constantSpin(power);
        }
        else if (gamepad1.a) {
            spindexer.goToAngle(targetAngle);
            spindexer.updatePID(kP);
        }

        packet.put("Target Angle ", targetAngle);
        packet.put("Current Angle ", spindexer.getAngle());

        dashboard.sendTelemetryPacket(packet);
    }
}
