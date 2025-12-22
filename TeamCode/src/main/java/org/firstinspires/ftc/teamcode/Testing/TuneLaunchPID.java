package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Launcher;

@Config
@TeleOp(name = "TuneLaunchPID", group = "Test")
public class TuneLaunchPID extends OpMode {
    Launcher launcher;

    public static double kS, kV, kP;
    public static int targetVelocity;

    TelemetryPacket packet;
    FtcDashboard dashboard;

    @Override
    public void init() {
        launcher = new Launcher(hardwareMap);

        targetVelocity = 0;

        packet = new TelemetryPacket();
        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void loop() {
        launcher.setTargetVelocity(targetVelocity);

        launcher.updatePIDValues(kS, kV, kP);
        launcher.update();

        packet.put("Launcher1 Velocity ", launcher.getVelocity1());
        packet.put("Launcher2 Velocity ", launcher.getVelocity2());
        packet.put("Average Velocity ", launcher.getVelocity());
        packet.put("Target Velocity ", targetVelocity);

        dashboard.sendTelemetryPacket(packet);
    }
}
