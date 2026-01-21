package org.firstinspires.ftc.teamcode.Testing.Launcher;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;

@Config
@TeleOp(name = "Launcher Recovery Test", group = "Test")
public class LauncherRecoveryTest extends OpMode {

    private DcMotorEx launcher1;
    private DcMotorEx launcher2;

    // ===== TUNABLE VALUES =====
    public static double targetVelocity = 0; // ticks/sec
    public static double spinSpeed = 0;
    public static double popperPos = 0;

    private final ElapsedTime recoveryTimer = new ElapsedTime();

    private boolean waitingForRecovery = false;
    private boolean shotTriggered = false;
    private double lastRecoveryTimeMs = 0;

    DcMotor spinner;
    Servo popperServo;

    TelemetryPacket packet;
    FtcDashboard dashboard;

    @Override
    public void init() {
        launcher1 = hardwareMap.get(DcMotorEx.class, Constants.HMMotorShooter1);
        launcher2 = hardwareMap.get(DcMotorEx.class, Constants.HMMotorShooter2);

        launcher1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcher1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcher2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        spinner = hardwareMap.get(DcMotorEx.class, Constants.HMMotorPopper);
        spinner.setDirection(DcMotorSimple.Direction.REVERSE);
        popperServo = hardwareMap.get(Servo.class, Constants.HMServoPopper);

        packet = new TelemetryPacket();
        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void loop() {
        launcher1.setVelocity(targetVelocity);
        launcher2.setVelocity(targetVelocity);

        spinner.setPower(spinSpeed);
        popperServo.setPosition(popperPos);

        double leftVel = launcher1.getVelocity();
        double rightVel = launcher2.getVelocity();
        double avgVel = (leftVel + rightVel) / 2.0;

        // Press A to indicate a shot occurred
        if (gamepad1.a && !shotTriggered) {
            shotTriggered = true;
            waitingForRecovery = true;
            recoveryTimer.reset();
        }

        // Detect recovery
        if (waitingForRecovery) {
            if (Math.abs(avgVel - targetVelocity) < 50) {
                lastRecoveryTimeMs = recoveryTimer.milliseconds();
                waitingForRecovery = false;
            }
        }

        // Reset trigger when button released
        if (!gamepad1.a) {
            shotTriggered = false;
        }

        packet.put("Target Velocity", targetVelocity);
        packet.put("Left Velocity", leftVel);
        packet.put("Right Velocity", rightVel);
        packet.put("Average Velocity", avgVel);

        if (waitingForRecovery) {
            packet.put("Status", "Recovering...");
        } else {
            packet.put("Status", "At Speed");
        }

        packet.put("Last Recovery Time (ms)", lastRecoveryTimeMs);

        dashboard.sendTelemetryPacket(packet);
    }
}
