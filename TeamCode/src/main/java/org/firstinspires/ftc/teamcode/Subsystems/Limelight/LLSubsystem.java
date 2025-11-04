package org.firstinspires.ftc.teamcode.Subsystems.Limelight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class LLSubsystem {
    private Limelight3A limelight;
    private LLResult result;
    private OpMode opMode;
    private double targetX;


    public LLSubsystem(OpMode _opMode){
        opMode = _opMode;
    }

    public void init() {
        limelight = opMode.hardwareMap.get(Limelight3A.class, LLConstant.HMLimeLight);
        limelight.pipelineSwitch(1);
        limelight.start();
    }

    public void update() {
        result = limelight.getLatestResult();
    }

    public boolean hasTarget() {
        return result != null && result.isValid();
    }

    public double getTargetX() {
        return result.getTx();
    }
}
