package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

@Config
@TeleOp(name = "April Stuff")

public class bigTest extends OpMode {
    Limelight3A limelight;
    public Servo base;
    double tx = 0;
    double ty = 0;
    double id = -1;

    double camDeg = 19.47883;
    double camH = 4.75;
    double targetH = 18.75;//29.5


    public static double kP = 0.0001;

    double basePos = 0.7;

    public void init(){
        limelight = hardwareMap.get(Limelight3A.class, "LimeLight");
        limelight.pipelineSwitch(1);
        limelight.start();

        base = hardwareMap.get(Servo.class, "base");
    }
    public void loop(){
        LLResult result = limelight.getLatestResult();

        if(result != null){

            telemetry.addData("tx", result.getTx());
            telemetry.addData("ty", result.getTy());
            telemetry.addData("Distance", getDistanceInches(result.getTy()));
            tx = result.getTx();

            ty = result.getTy();

            List<LLResultTypes.FiducialResult> fiducialResult = result.getFiducialResults();
            for(LLResultTypes.FiducialResult fr : fiducialResult){
                id = fr.getFiducialId();
                telemetry.addData("id", fr.getFiducialId());
            }


        }
        /*
        if(tx < -5) {
            basePos += (0.001);
        }

        if(tx > 5){
            basePos -= (0.001);

        }

        base.setPosition(basePos);

         */

    }


    private double getDistanceInches(double tyDegrees) {
        double totalAngleDeg = camDeg - tyDegrees;
        double totalAngleRad = Math.toRadians(totalAngleDeg);

        return (targetH - camH) / Math.tan(totalAngleRad);
    }
}

