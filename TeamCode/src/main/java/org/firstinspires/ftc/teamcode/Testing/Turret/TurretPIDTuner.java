package org.firstinspires.ftc.teamcode.Testing.Turret;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Turret;

@Config
@TeleOp(name="TurretPIDTuner", group="Test")
public class TurretPIDTuner extends OpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    Turret turret;

    @Override
    public void init() {
        turret = new Turret(hardwareMap);
        turret.resetIMU();
    }

    @Override
    public void loop() {
        turret.update();

        packet.put("target ", turret.getTarget());
        packet.put("current ", turret.getTurretAngle());
        packet.put("power ", turret.getPower());
        dashboard.sendTelemetryPacket(packet);
    }
}
