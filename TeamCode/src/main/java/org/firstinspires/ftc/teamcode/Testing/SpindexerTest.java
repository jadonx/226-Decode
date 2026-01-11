package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;

@TeleOp(name="SpindexerTest", group="Test")
public class SpindexerTest extends OpMode {
    Spindexer spindexer;

    @Override
    public void init() {
        spindexer = new Spindexer(hardwareMap);
    }

    @Override
    public void loop() {
        spindexer.setPower(gamepad1.left_stick_x);

        telemetry.addData("wrapped angle ", spindexer.getWrappedAngle());
        telemetry.addData("unwrapped angle ", spindexer.getUnwrappedAngle());
        telemetry.update();
    }
}
