package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name = "LL tester", group = "Test")
public class LLTest extends OpMode {
    private Limelight3A limelight;
    private LLResult result;
    private double tX;



    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "LL");
        limelight.pipelineSwitch(1);
        limelight.start();

        telemetry.addLine("Turret Initialized");
        telemetry.addLine("Press [X] to toggle auto-tracking mode");
        telemetry.update();
    }

    @Override
    public void loop() {
        result = limelight.getLatestResult();

        if (result != null) {
            tX = result.getTx();
        }

        telemetry.addData("TX", tX);
        telemetry.update();
    }
}
