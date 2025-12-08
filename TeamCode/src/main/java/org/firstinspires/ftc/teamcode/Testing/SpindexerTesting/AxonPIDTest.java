package org.firstinspires.ftc.teamcode.Testing.SpindexerTesting;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.AS5600Encoder;

@Config
@TeleOp(name = "AxonPID_Test", group = "Tester")
public class AxonPIDTest extends OpMode {
    public static double kP, kD, kS;
    public static double target;

    CRServo servo;
    AS5600Encoder encoder;

    TelemetryPacket packet;
    FtcDashboard dashboard;

    ElapsedTime timer;
    double lastError;

    @Override
    public void init() {
        servo = hardwareMap.get(CRServo.class, Constants.HMServospinDexer);
        servo.setDirection(DcMotorSimple.Direction.REVERSE);
        encoder = hardwareMap.get(AS5600Encoder.class, Constants.HMSpindexerEncoder);

        timer = new ElapsedTime();

        packet = new TelemetryPacket();
        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void loop() {
        double current = encoder.getWrappedAngle();
        double error = target - current;

        double derivative = (error - lastError) / timer.seconds();

        double ff = kS * Math.signum(error);

        double output = (kP * error) + (kD * derivative) + ff;

        output = Range.clip(output, -0.4, 0.4);

        servo.setPower(output);

        lastError = error;
        timer.reset();

        packet.put("current ", encoder.getWrappedAngle());
        packet.put("target ", target);
        packet.put("error ", error);
        packet.put("output ", output);
        packet.put("ff ", ff);
        dashboard.sendTelemetryPacket(packet);
    }
}
