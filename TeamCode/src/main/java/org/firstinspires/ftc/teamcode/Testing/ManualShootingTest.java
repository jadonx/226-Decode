package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Subsystems.Popper;

@Config
@TeleOp(name="ManualShootingTest")
public class ManualShootingTest extends OpMode {
    CRServo spindexerServo;
    Servo cover;
    Popper popper;
    Launcher launcher;

    public static int targetVelocity;
    public static double spindexerSpeed;
    public static double coverAngle;

    private boolean popperIsEngaged = false;

    TelemetryPacket packet;
    FtcDashboard dashboard;

    @Override
    public void init() {
        spindexerServo = hardwareMap.get(CRServo.class, Constants.HMServospinDexer);
        spindexerServo.setDirection(DcMotorSimple.Direction.REVERSE);
        cover = hardwareMap.get(Servo.class, Constants.HMServobackSpin);
        popper = new Popper(hardwareMap);
        launcher = new Launcher(hardwareMap);

        packet = new TelemetryPacket();
        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void loop() {
        spindexerServo.setPower(spindexerSpeed);

        cover.setPosition(coverAngle);

        launcher.setTargetVelocity(targetVelocity);
        launcher.update();

        if (gamepad1.aWasPressed()) {
            popperIsEngaged = !popperIsEngaged;

            if (popperIsEngaged) {
                popper.setTargetVelocity(1800);
                popper.pushInPopper();
            }
            else {
                popper.deactivatePopper();
            }
        }
        
        packet.put("Launcher current vel ", launcher.getVelocity());
        packet.put("Launcher target vel ", launcher.getTargetVelocity());
        packet.put("Popper current vel ", popper.getPopperVelocity());

        dashboard.sendTelemetryPacket(packet);
    }
}
