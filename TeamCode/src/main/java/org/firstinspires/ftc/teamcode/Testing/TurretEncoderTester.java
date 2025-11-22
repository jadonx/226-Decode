package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Encoder.AS5600Encoder;

@TeleOp (name = "TurretEncoder Tester", group = "Testing")
public class TurretEncoderTester extends OpMode {
    FtcDashboard ftcdashboard;
    IMU turretEncoder;
    @Override
    public void init() {
        turretEncoder = hardwareMap.get(IMU.class, Constants.HMTurretEncoder);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        IMU.Parameters turretParameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));

        turretEncoder.initialize(turretParameters);
        turretEncoder.resetYaw();
    }

    @Override
    public void loop() {
        telemetry.addData("Turret Angle (Degrees): ", turretEncoder.getRobotYawPitchRollAngles().getYaw());
        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#3F51B5");
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
