package org.firstinspires.ftc.teamcode.Testing;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.Constants.HMMotorShooter1;
import static org.firstinspires.ftc.teamcode.Constants.HMMotorShooter2;

import android.sax.StartElementListener;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class shooterPID extends OpMode {
    DcMotorEx shooter1, shooter2;


    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;

    double integralSum = 0;
    double lastError = 0;
    double lastTime = 0;

    boolean run = false;



    public static double target = 2000;



    public void init(){
        shooter1 = hardwareMap.get(DcMotorEx.class, HMMotorShooter1);
        shooter2 = hardwareMap.get(DcMotorEx.class, HMMotorShooter2 );

    }

    public void loop(){

        if(gamepad1.a){
            run = !run;
        }

        if(run){
            shooter1.setPower(update(2540,shooter1.getVelocity()));
            shooter2.setPower(update(2540,shooter1.getVelocity()));
        } else{
            shooter1.setPower(0);
            shooter2.setPower(0);
        }


        telemetry.addData("Power", shooter1.getPower());
        telemetry.addData("velocity", shooter1.getVelocity());
    }

    public double update(double target, double current) {



        double error = target - current;

        double now = System.nanoTime() / 1e9;
        double dt = now - lastTime;
        lastTime = now;


        if (dt <= 0) dt = 1e-3;


        integralSum += error * dt;
        double derivative = (error - lastError) / dt;

        lastError = error;

        return (kP * error) + (kI * integralSum) + (kD * derivative);
    }
}
