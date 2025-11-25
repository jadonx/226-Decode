package org.firstinspires.ftc.teamcode.Testing;

import static org.firstinspires.ftc.teamcode.Constants.HMColorSensor;
import static org.firstinspires.ftc.teamcode.Constants.HMMotorIntake;
import static org.firstinspires.ftc.teamcode.Constants.HMServospinDexer;
import static org.firstinspires.ftc.teamcode.Constants.HMSpindexerEncoder;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Encoder.AS5600Encoder;


@TeleOp(name = "New Spindexer Test")
public class NewSpindexer extends OpMode {
    DcMotorEx intake;
    CRServo spinner;
    NormalizedColorSensor colorSensor1;
    AS5600Encoder spinEncoder;

    public double pos1A = 50; //For launch
    public double pos2A = 192;
    public double pos3A = 280;
    //a
    public double pos1B = 230; //For Intake
    public double pos2B = 12;
    public double pos3B = 100;

    boolean hold1 = false;
    boolean hold2 = false;
    boolean hold3 = false;

    private double lastError = 0;
    ElapsedTime pidTimer;
    private double kP = 0.015, kI = 0, kD = -0.001;



    public void init(){
        colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, HMColorSensor);
        spinner = hardwareMap.get(CRServo.class, HMServospinDexer);
        intake = hardwareMap.get(DcMotorEx.class, HMMotorIntake);
        spinEncoder = hardwareMap.get(AS5600Encoder.class, HMSpindexerEncoder );
        pidTimer = new ElapsedTime();
    }

    public void loop(){
        NormalizedRGBA colors = colorSensor1.getNormalizedColors();
        float[] hsv = new float[3];
        Color.colorToHSV(colors.toColor(), hsv);


        if(gamepad1.a){
            intake.setPower(0.75);
        }

        if(hold1 && hold2 && hold3){
            gamepad1.rumble(1000);
        }

        if(spinEncoder.getAngleDegrees() < pos1B+1 && spinEncoder.getAngleDegrees() > pos1B-1){
            if(hsv[0] > 100){
                hold1 = true;
            } else{
                hold1= false;
            }
        }

        if(spinEncoder.getAngleDegrees() < pos2B+1 && spinEncoder.getAngleDegrees() > pos2B-1){
            if(hsv[0] > 100){
                hold2 = true;
            } else{
                hold2= false;
            }
        }

        if(spinEncoder.getAngleDegrees() < pos3B+1 && spinEncoder.getAngleDegrees() > pos3B-1){
            if(hsv[0] > 100){
                hold3 = true;
            } else{
                hold3= false;
            }
        }

        if(!hold1){
            goToAngle(pos1B);
        } else if(!hold2){
            goToAngle(pos2B);
        } else if(!hold3){
            goToAngle(pos3B);
        }



        telemetry.addData("hold1", hold1);
        telemetry.addData("hold2", hold2);
        telemetry.addData("hold3", hold3);
        telemetry.addData("hue", hsv[0]);
        telemetry.addData("Encoder Pos", spinEncoder.getAngleDegrees());
    }

    public void goToAngle(double target) {
        double error = getAngleError(target, spinEncoder.getAngleDegrees());

        double derivative = (error - lastError) / pidTimer.seconds();

        double output = (kP * error) + (-kD * derivative);

        spinner.setPower(output);

        lastError = error;
        pidTimer.reset();
    }

    private double getAngleError(double targetAngle, double currentAngle) {
        double error = targetAngle - currentAngle;

        // Normalize error to the range (-180, 180]
        error = (error + 180) % 360;
        if (error < 0) error += 360;
        return error - 180;
    }
}

