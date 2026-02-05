package org.firstinspires.ftc.teamcode.Testing.Spindexer;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;
@Config
@TeleOp(name="SpindexerPIDFTest", group = "Test")
public class SpindexerPIDFTest extends OpMode {
    Spindexer spindexer;
    public static double targetAngle;

    int numLoops;
    ElapsedTime loopTimer;

    TelemetryPacket packet;
    FtcDashboard dashboard;
    @Override
    public void init() {
        spindexer = new Spindexer(hardwareMap);
        spindexer.setMode(Spindexer.SpindexerMode.INTAKE_MODE);

        packet = new TelemetryPacket();
        dashboard = FtcDashboard.getInstance();

        numLoops = 0;
        loopTimer = new ElapsedTime();
        loopTimer.reset();
    }

    @Override
    public void loop() {
        spindexer.update();
        spindexer.setTargetAngle(targetAngle);

        numLoops++;
        packet.put("Average Loop Times", ((double) loopTimer.milliseconds())/numLoops);

        packet.put("current ", spindexer.getWrappedAngle());
        packet.put("target ", spindexer.getTargetAngle());
        packet.put("profiler ref ", spindexer.reference);
        dashboard.sendTelemetryPacket(packet);
    }
}
