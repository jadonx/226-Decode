package org.firstinspires.ftc.teamcode.testStuff;

import android.graphics.Color;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;


@TeleOp (name = "Shooter Aiming")
public class shooterAiming extends OpMode {
    DcMotorEx shooter1, shooter2, spinner, pusher;
    Servo angleAdjust;
    Limelight3A limelight;
    NormalizedColorSensor colorSensor;
    double anglePos, angleDegree;
    double tx = 0;
    double ty = 0;
    double id = -1;

    double camDeg = 19.47883;
    double camH = 4.75;
    double targetH = 18.75;

    double initialVelocity;
    double gravity = 386.09;



    public void init(){
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        spinner = hardwareMap.get(DcMotorEx.class, "spinner");
        pusher = hardwareMap.get(DcMotorEx.class, "puhser");
        angleAdjust = hardwareMap.get(Servo.class, "angleAdjust");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        limelight = hardwareMap.get(Limelight3A.class, "LimeLight");
        limelight.pipelineSwitch(1);
        limelight.start();

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

            NormalizedRGBA colors = colorSensor.getNormalizedColors();

            float[] hsv = new float[3];
            Color.colorToHSV(colors.toColor(), hsv);
            telemetry.addData("Hue", hsv[0]);
            telemetry.addData("Saturation", hsv[1]);
            telemetry.addData("Light", hsv[2]);

            if(gamepad1.a){
                spinner.setPower(1);
                pusher.setPower(1);
                shooter1.setPower(1);
                shooter2.setPower(1);
            }

            if(gamepad1.b){
                spinner.setPower(0);
                pusher.setPower(0);
                shooter1.setPower(0);
                shooter2.setPower(0);
            }
        }


    }
    private double getDistanceInches(double tyDegrees) {
        double totalAngleDeg = camDeg - tyDegrees;
        double totalAngleRad = Math.toRadians(totalAngleDeg);

        return (targetH - camH) / Math.tan(totalAngleRad);
    }

    private double getNeededAngle(double distance){
        double degree = 0;

        degree = Math.atan((Math.pow(initialVelocity,2)+ Math.sqrt(Math.pow(initialVelocity,4) -
                (2*gravity*targetH*Math.pow(initialVelocity,2)) - (Math.pow(gravity,2)*Math.pow(distance,2))))
                /gravity*distance);


        return Math.toDegrees(degree);
    }

    private double degreeToPos(double degree){
        return degree;
    }
}
