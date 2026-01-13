package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Subsystems.Popper;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerEncoder;

@TeleOp(name="SpindexerTest", group="Test")
public class SpindexerTest extends OpMode {
    Popper popper;
    Launcher launcher;
    Spindexer spindexer;
    SpindexerEncoder spindexerEncoder;

    TelemetryPacket packet;
    FtcDashboard dashboard;

    @Override
    public void init() {
        spindexer = new Spindexer(hardwareMap);
        spindexerEncoder = hardwareMap.get(SpindexerEncoder.class, Constants.HMSpindexerEncoder);

        popper = new Popper(hardwareMap);
        launcher = new Launcher(hardwareMap);

        packet = new TelemetryPacket();
        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void loop() {
        spindexer.setPower(gamepad1.left_stick_x);

        packet.put("wrapped angle ", spindexerEncoder.getWrappedAngle());
        dashboard.sendTelemetryPacket(packet);
    }
}
