package org.firstinspires.ftc.teamcode.Subsystems.Spindexer;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Encoder.AS5600Encoder;

public class SimpleSpindexer {
    // Spindexer
    private CRServo spindexerServo;
    private AS5600Encoder spindexerEncoder;

    private boolean isSpinning;
    private double lastAngle;

    private ElapsedTime timer;

    public SimpleSpindexer(HardwareMap hardwareMap) {
        spindexerServo = hardwareMap.get(CRServo.class, "leftCRServo");
        spindexerEncoder = hardwareMap.get(AS5600Encoder.class, "spinEncoder");

        isSpinning = false;
        lastAngle = 0;

        timer = new ElapsedTime();
    }

    public void runSpindexer() {
        spindexerServo.setPower(1);
        isSpinning = true;
    }

    public void stopSpindexer() {
        spindexerServo.setPower(0);
        isSpinning = false;
    }

    public double getAngle() {
        return spindexerEncoder.getAngleDegrees();
    }
}
