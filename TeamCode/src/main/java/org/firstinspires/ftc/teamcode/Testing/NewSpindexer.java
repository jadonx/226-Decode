package org.firstinspires.ftc.teamcode.Testing;


import static org.firstinspires.ftc.teamcode.Constants.HMFrontColorSensor;
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

    public double pos1A = 49; //For launch
    public double pos2A = 183;
    public double pos3A = 284;
    //a
    public double pos1B = 229; //For Intake
    public double pos2B = 3.69;
    public double pos3B = 106;

    ball hold1 = ball.empty;
    ball hold2 = ball.empty;
    ball hold3 = ball.empty;

    private double lastError = 0;
    ElapsedTime pidTimer;
    private double kP = 0.015, kI = 0, kD = -0.001;



    public void init(){
        colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, HMFrontColorSensor);
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

        if(gamepad1.b){
            hold1 = ball.empty;
            hold2 = ball.empty;
            hold3 = ball.empty;
        }

        if(hold1 != ball.empty && hold2!= ball.empty && hold3!= ball.empty){
            gamepad1.rumble(1000);
        }

        if(spinEncoder.getAngleDegrees() < pos1B+3 && spinEncoder.getAngleDegrees() > pos1B-3){
            if(colors.green*255 > 5.5 && hsv[1] > 0.61){
                hold1 = ball.green;
            } else if(colors.blue*255 > 6){
                hold1= ball.purple;
            } else{
                hold1 = ball.empty;
            }
        }

        if(spinEncoder.getAngleDegrees() < pos2B+3 && spinEncoder.getAngleDegrees() > pos2B-3){
            if(colors.green*255 > 5 && hsv[1] > 0.61){
                hold2 = ball.green;
            } else if(colors.blue*255 > 6){
                hold2= ball.purple;
            } else{
                hold2 = ball.empty;
            }
        }

        if(spinEncoder.getAngleDegrees() < pos3B+3 && spinEncoder.getAngleDegrees() > pos3B-3){
            if(colors.green*255 > 5 && hsv[1] > 0.61){
                hold3 = ball.green;
            } else if(colors.blue*255 > 6){
                hold3= ball.purple;
            } else{
                hold3 = ball.empty;
            }
        }

        if(hold1 == ball.empty){
            goToAngle(pos1B);
        } else if(hold2 == ball.empty){
            goToAngle(pos2B);
        } else if(hold3 == ball.empty){
            goToAngle(pos3B);
        } else{
            goToAngle(pos1A);
        }



        telemetry.addData("hold1", hold1);
        telemetry.addData("hold2", hold2);
        telemetry.addData("hold3", hold3);
        telemetry.addData("Green", colors.green*255);
        telemetry.addData("Blue", colors.blue*255);
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

    public enum ball{
        empty,
        purple,
        green
    }
}

