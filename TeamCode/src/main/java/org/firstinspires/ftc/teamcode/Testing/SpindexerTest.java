package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerEncoder;

@TeleOp(name="SpindexerTest", group="Test")
public class SpindexerTest extends OpMode {
    Spindexer spindexer;
    SpindexerEncoder spindexerEncoder;

    @Override
    public void init() {
        spindexer = new Spindexer(hardwareMap);
        spindexerEncoder = hardwareMap.get(SpindexerEncoder.class, Constants.HMSpindexerEncoder);
    }

    @Override
    public void loop() {
        spindexer.setPower(gamepad1.left_stick_x);

        telemetry.addData("wrapped angle ", spindexerEncoder.getWrappedAngle());
        telemetry.update();
    }
}
