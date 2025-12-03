package org.firstinspires.ftc.teamcode.Testing.SpindexerTesting;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;

@TeleOp(name="AngleEncoderDrivingTest")
public class AngleEncoderDrivingTest extends OpMode {
    FieldCentricDrive drive;
    Spindexer spindexer;

    TelemetryPacket packet;
    FtcDashboard dashboard;

    @Override
    public void init() {
        drive = new FieldCentricDrive(hardwareMap);
        spindexer = new Spindexer(hardwareMap);

        packet = new TelemetryPacket();
        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void loop() {
        // DRIVE LOGIC
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        drive.drive(y, x, rx);

        if (gamepad1.x) {
            drive.resetIMU();
        }

        spindexer.runSpindexer();

        packet.put("imu ", drive.getYaw());
        packet.put("spindexer angle ", spindexer.getAngle());
        dashboard.sendTelemetryPacket(packet);
    }
}
