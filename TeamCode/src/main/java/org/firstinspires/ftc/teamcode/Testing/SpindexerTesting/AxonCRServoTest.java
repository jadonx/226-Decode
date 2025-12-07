package org.firstinspires.ftc.teamcode.Testing.SpindexerTesting;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.AS5600Encoder;

import java.nio.file.attribute.FileTime;

// TESTING SPEED OF AXON MAX SERVO
// kS = 0.095
// Comes to an immediate stop from 1 -> 0 power
@Config
@TeleOp(name="AxonCRServo_Tester", group = "Tester")
public class AxonCRServoTest extends OpMode {
    CRServo spindexer;
    AS5600Encoder encoder;
    public static double power;

    TelemetryPacket packet;
    FtcDashboard dashboard;

    @Override
    public void init() {
        spindexer = hardwareMap.get(CRServo.class, Constants.HMServospinDexer);
        encoder = hardwareMap.get(AS5600Encoder.class, Constants.HMSpindexerEncoder);

        packet = new TelemetryPacket();
        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void loop() {
        spindexer.setPower(power);
        packet.put("current ", encoder.getWrappedAngle());

        dashboard.sendTelemetryPacket(packet);
    }
}
