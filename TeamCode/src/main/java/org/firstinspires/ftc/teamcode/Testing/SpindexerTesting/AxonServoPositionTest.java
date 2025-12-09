package org.firstinspires.ftc.teamcode.Testing.SpindexerTesting;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

// DETERMINE THE RANGE OF THE AXON MAX
// SEEMS TO ONLY MOVE SPINDEXER 180 DEGREES
@Config
@TeleOp(name="AxonServoPosition_Tester", group = "Tester")
public class AxonServoPositionTest extends OpMode {
    public static double position;

    Servo spindexerServo;

    @Override
    public void init() {
        spindexerServo = hardwareMap.get(Servo.class, Constants.HMServospinDexer);
    }

    @Override
    public void loop() {
        spindexerServo.setPosition(position);
    }
}
