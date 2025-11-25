package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

public class LimeLight {
    Limelight3A limelight;
    LLResult result;
    double tX;
    double tY;
    double distance;
    double CAM_DEG = 29.78;
    double CAM_H = 12;
    double TARGET_H = 29;

    public LimeLight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, Constants.HMLimelight);
        limelight.pipelineSwitch(1);
        limelight.start();
    }

    public void getResult() {
        result = limelight.getLatestResult();
    }

    public void getMotifPattern() {

    }

    public double getTX() {
        if (result != null) {
            tX = result.getTx();
        } else {
            tX = 0;
        }
        return tX;
    }

    public double getTY() {
        if (result != null) {
            tY = result.getTy();
        } else {
            tY = 0;
        }
        return tY;
    }

    public double getDistance() {
        distance = getDistanceInches(getTY());
        return distance;
    }

    private double getDistanceInches(double tyDegrees) {
        double totalAngleDeg = CAM_DEG + tyDegrees;
        double totalAngleRad = Math.toRadians(totalAngleDeg);

        return (TARGET_H - CAM_H) / Math.tan(totalAngleRad);
    }
}
