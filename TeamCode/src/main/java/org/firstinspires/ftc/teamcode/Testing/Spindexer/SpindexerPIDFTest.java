package org.firstinspires.ftc.teamcode.Testing.Spindexer;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;

@TeleOp(name="SpindexerPIDFTest", group = "Test")
public class SpindexerPIDFTest extends OpMode {
    CRServo spindexerServo;

    int numLoops;
    ElapsedTime loopTimer;

    PIDFController pid;

    public static double kP, kI, kD, kF;
    public static double targetAngle;
    @Override
    public void init() {
        spindexerServo = hardwareMap.get(CRServo.class, Constants.HMServospinDexer);
        spindexerServo.setDirection(DcMotorSimple.Direction.REVERSE);

        numLoops = 0;
        loopTimer = new ElapsedTime();
        loopTimer.reset();

        pid = new PIDFController(kP, kI, kD, kF);
    }

    @Override
    public void loop() {
        pid.setPIDF(kP, kI, kD, kF);

        numLoops++;
        telemetry.addData("Average Loop Times", ((double) loopTimer.milliseconds())/numLoops);
        telemetry.update();
    }
}
