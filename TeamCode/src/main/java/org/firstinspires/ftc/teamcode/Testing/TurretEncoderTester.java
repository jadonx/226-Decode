package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Turret;

@TeleOp(name="TurretEncoderTester", group="Test")
public class TurretEncoderTester extends OpMode {
    Turret turret;

    @Override
    public void init() {
        turret = new Turret(hardwareMap);
    }

    @Override
    public void loop() {
        turret.setPower(gamepad1.left_stick_x);

        telemetry.addData("turret angle ", turret.getTurretAngle());

        telemetry.update();
    }
}
