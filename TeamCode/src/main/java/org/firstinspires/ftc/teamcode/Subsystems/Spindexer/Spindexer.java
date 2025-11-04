package org.firstinspires.ftc.teamcode.Subsystems.Spindexer;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.Encoder.AS5600Encoder;

public class Spindexer {
    // Spindexer
    private CRServo spindexerServo;
    private AS5600Encoder spindexerEncoder;
    private NormalizedColorSensor colorSensor;

    // Spindexer Popper
    private DcMotor popper;
    private Servo popperServo;

    // Spindexer PID Values
    private double kP;

    public Spindexer(HardwareMap hardwareMap) {
        spindexerServo = hardwareMap.get(CRServo.class, "leftCRServo");
        spindexerEncoder = hardwareMap.get(AS5600Encoder.class, "spinEncoder");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        popper = hardwareMap.get(DcMotor.class, "spinner");
        popperServo = hardwareMap.get(Servo.class, "spinDexerServo");

        kP = 0;
    }

    public void goToAngle(double target) {
        double error = getAngle() - target;

        double output = kP * error;

        spindexerServo.setPower(output);
    }

    public void constantSpin(double power) {
        spindexerServo.setPower(power);
    }

    public double getAngle() {
        return spindexerEncoder.getAngleDegrees();
    }

    public void updatePID(double kP) {
        this.kP = kP;
    }
}
