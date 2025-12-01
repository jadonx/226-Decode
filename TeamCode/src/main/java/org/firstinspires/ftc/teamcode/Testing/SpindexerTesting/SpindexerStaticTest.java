package org.firstinspires.ftc.teamcode.Testing.SpindexerTesting;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;

@Config
@TeleOp(name = "SpindexerStaticTest")
public class SpindexerStaticTest extends OpMode {
    Spindexer spindexer;
    public static double kS; // 0.12

    @Override
    public void init() {
        spindexer = new Spindexer(hardwareMap);
    }

    @Override
    public void loop() {
        spindexer.runSpindexer(kS);
    }
}
