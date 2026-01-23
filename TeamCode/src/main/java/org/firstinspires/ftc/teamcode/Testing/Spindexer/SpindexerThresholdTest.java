package org.firstinspires.ftc.teamcode.Testing.Spindexer;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;

@Config
@TeleOp(name="SpindexerThresholdTest", group="Test")
public class SpindexerThresholdTest extends OpMode {
    Spindexer spindexer;

    public static double targetAngle = 0;

    @Override
    public void init() {
        spindexer = new Spindexer(hardwareMap);
        spindexer.setMode(Spindexer.SpindexerMode.INTAKE_MODE);
    }

    @Override
    public void loop() {
        spindexer.setTargetAngle(targetAngle);
        spindexer.update();
        telemetry.addData("Current ", spindexer.getWrappedAngle());
        telemetry.addData("Target ", spindexer.getTargetAngle());
        telemetry.addData("Within angle ", spindexer.atTargetAngle(12));

        telemetry.update();
    }
}
