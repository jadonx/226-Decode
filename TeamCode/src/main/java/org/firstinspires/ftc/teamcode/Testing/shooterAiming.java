package org.firstinspires.ftc.teamcode.Testing;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
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

@Config
@TeleOp (name = "Shooter Aiming")
public class shooterAiming extends OpMode {
    DcMotorEx shooter1, shooter2, spinner, spinDexer;
    Servo cover;
    Limelight3A limelight;
    NormalizedColorSensor colorSensor;
    double anglePos, angleDegree;
    double tx = 0;
    double ty = 0;
    double id = -1;
    double shooterSpeed = 1;

    double camDeg = 19.47883;
    double camH = 12;
    double targetH = 39;
    public static double distance = 140;
    double initialVelocity = 246.81;
    double gravity = 386.09;



    public void init(){
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        spinner = hardwareMap.get(DcMotorEx.class, "spinner");

        cover = hardwareMap.get(Servo.class, "cover");
        //colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        //limelight = hardwareMap.get(Limelight3A.class, "LimeLight");
        //limelight.pipelineSwitch(1);
        //limelight.start();

    }

    public void loop(){
        /*
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

        }
        */
        telemetry.addData("Angle Needed", getNeededAngle(distance));
        telemetry.addData("pos Needed", degreeToPos(getNeededAngle(distance)));

        if(distance > 52){
            cover.setPosition(degreeToPos(getNeededAngle(distance)));
        } else{
            shooterSpeed = getNeededPower(distance);
        }


        if(gamepad1.a){

            shooter1.setPower(shooterSpeed);
            shooter2.setPower(shooterSpeed);
        }

        if(gamepad1.b){
            spinner.setPower(0);
            spinDexer.setPower(0);
            shooter1.setPower(0);
            shooter2.setPower(0);
        }


    }
    private double getDistanceInches(double tyDegrees) {
        double totalAngleDeg = camDeg - tyDegrees;
        double totalAngleRad = Math.toRadians(totalAngleDeg);

        return (targetH - camH) / Math.tan(totalAngleRad);
    }
    /*
    private double getNeededAngle(double distance){
        double degree = 0;

        degree = Math.atan((Math.pow(initialVelocity,2)+ Math.sqrt(Math.pow(initialVelocity,4) -
                (2*gravity*targetH*Math.pow(initialVelocity,2)) - (Math.pow(gravity,2)*Math.pow(distance,2))))
                /gravity*distance);


        return Math.toDegrees(degree);
    }

     */
    private double getNeededAngle(double distance) {
        double v = initialVelocity;
        double g = gravity;
        double y = targetH - camH;

        double term = v*v*v*v - g * (g * distance * distance + 2 * y * v * v);
        if (term < 0) {
            return 52;
        }


        double tanThetaHigh = (v*v + Math.sqrt(term)) / (g * distance);
        double thetaRad = Math.atan(tanThetaHigh);
        double thetaDeg = Math.toDegrees(thetaRad);

        return thetaDeg;
    }

    private double getNeededPower(double distance) {

        return (0.00581395*distance)+0.697674;
    }


    private double degreeToPos(double angle){
        double minAngle = 52.0;
        double maxAngle = 74.5;
        double servoPos = (angle - minAngle) / (maxAngle - minAngle);


        return Math.max(0.0, Math.min(1.0, servoPos));
    }
}
