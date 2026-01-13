package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Constants;

import java.util.List;

public class LimeLight {
    Limelight3A limelight;
    LLResult result;

    double tX;
    double tY;
    double distance;
    double CAM_DEG = 27.3;//25.85
    double CAM_H = 11.578;
    double TARGET_H = 29.5;
    int id = -1;
    int motifID = -1;

    int goalID = -1;

    public LimeLight(HardwareMap hardwareMap, RoadRunnerPinPoint.AllianceColor allianceColor) {
        limelight = hardwareMap.get(Limelight3A.class, Constants.HMLimelight);
        limelight.pipelineSwitch(1);
        limelight.start();

        if (allianceColor == RoadRunnerPinPoint.AllianceColor.RED) {
            goalID = 24;
        }
        else if (allianceColor == RoadRunnerPinPoint.AllianceColor.BLUE) {
            goalID = 20;
        }
    }

    public void getResult() {
        result = limelight.getLatestResult();
    }

    public void getAprilTagID() {
        if (result != null) {
            List<LLResultTypes.FiducialResult> fiducialResult = result.getFiducialResults();
            for(LLResultTypes.FiducialResult fid : fiducialResult){
                id = fid.getFiducialId();
                if (id == 21 || id == 22 || id == 23) {
                    motifID = id;
                }
            }
        } else {
            motifID = -1;
        }
    }

    public boolean isGoalTargeted() {
        getAprilTagID();
        return id == goalID;
    }

    public Pose2D getEstimatedPose() {
        if (result != null) {
            Pose3D pose = result.getBotpose_MT2();

            return new Pose2D(DistanceUnit.INCH, DistanceUnit.INCH.fromMeters(pose.getPosition().x), DistanceUnit.INCH.fromMeters(pose.getPosition().y), AngleUnit.DEGREES, pose.getOrientation().getYaw());
        }
        return null;
    }

    public boolean hasMotif() {
        return motifID != -1;
    }

    public int getMotifID() {
        return motifID;
    }

    public void updateRobotOrientation(double degrees) {
        limelight.updateRobotOrientation(degrees);
    }

    public Spindexer.HolderStatus[] getMotif() {
        if (motifID == 21) return new Spindexer.HolderStatus[]{Spindexer.HolderStatus.GREEN, Spindexer.HolderStatus.PURPLE, Spindexer.HolderStatus.PURPLE};
        else if (motifID == 22) return new Spindexer.HolderStatus[]{Spindexer.HolderStatus.PURPLE, Spindexer.HolderStatus.GREEN, Spindexer.HolderStatus.PURPLE};
        else if (motifID == 23) return new Spindexer.HolderStatus[]{Spindexer.HolderStatus.PURPLE, Spindexer.HolderStatus.PURPLE, Spindexer.HolderStatus.GREEN};
        else return new Spindexer.HolderStatus[]{Spindexer.HolderStatus.NONE, Spindexer.HolderStatus.NONE, Spindexer.HolderStatus.NONE};
    }

    public void stopLimeLight() {
        limelight.stop();
        limelight.shutdown();
    }

    public boolean isResulted() {
        return result.isValid();
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