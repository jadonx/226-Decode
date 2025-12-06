package org.firstinspires.ftc.teamcode.Testing.SpindexerTesting;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Constants;

// TESTING SPEED OF AXON MAX SERVO
// kS = 0.095
// Comes to an immediate stop from 1 -> 0 powercm
@Config
@TeleOp(name="AxonCRServo_Tester", group = "Tester")
public class AxonCRServoTest extends OpMode {
    CRServo spindexer;
    public static double power;

    @Override
    public void init() {
        spindexer = hardwareMap.get(CRServo.class, Constants.HMServospinDexer);
    }

    @Override
    public void loop() {
        spindexer.setPower(power);
    }
}
