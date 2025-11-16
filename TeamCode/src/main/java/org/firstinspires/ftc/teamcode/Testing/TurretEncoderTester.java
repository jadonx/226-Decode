package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Encoder.AS5600Encoder;

@TeleOp (name = "TurretEncoder Tester", group = "Testing")
public class TurretEncoderTester extends OpMode {
    AS5600Encoder turretEncoder;

    @Override
    public void init() {
        turretEncoder = hardwareMap.get(AS5600Encoder.class, Constants.HMTurretEncoder);
    }

    @Override
    public void loop() {
        telemetry.addData("Turret Angle (Degrees): ", turretEncoder.getAngleDegreesTurret());
        telemetry.addData("Turret Angle (Degrees): ", turretEncoder.getAngleDegrees());
        telemetry.update();
    }
}
