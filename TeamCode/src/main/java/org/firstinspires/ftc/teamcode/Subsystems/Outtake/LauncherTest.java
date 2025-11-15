package org.firstinspires.ftc.teamcode.Subsystems.Outtake;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name = "Turret Test")
public class LauncherTest extends OpMode {
    Launcher launcher;

    public static double power;
    public static double kP, kD;
    public static int targetRPM;

    FtcDashboard dashboard;
    TelemetryPacket packet;

    @Override
    public void init() {
        launcher = new Launcher(hardwareMap);

        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
    }

    @Override
    public void loop() {
        launcher.setPower(power);

        packet.put("Launcher RPM ", launcher.currentRPM());
        dashboard.sendTelemetryPacket(packet);
    }
}
