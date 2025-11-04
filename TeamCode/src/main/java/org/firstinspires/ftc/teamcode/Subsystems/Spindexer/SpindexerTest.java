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

    public static double kP, kD;
    public static double targetAngle;
    public static double power;

    FtcDashboard dashboard;
    TelemetryPacket packet;

    public static boolean continuousRotationMode;

    @Override
    public void init() {
        spindexer = new Spindexer(hardwareMap);

        kP = 0.005;
        kD = 0.0001;

        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();

        continuousRotationMode = true;
    }

    @Override
    public void loop() {
        if (continuousRotationMode) {
            spindexer.constantSpin(power);
        }
        else {
            spindexer.goToAngle(targetAngle);
            spindexer.updatePID(kP, kD);
        }

        packet.put("Hue ", spindexer.getColorAtAngle(0, 0));

        packet.put("Target Angle ", targetAngle);
        packet.put("Current Angle ", spindexer.getAngle());

        dashboard.sendTelemetryPacket(packet);
    }
}
