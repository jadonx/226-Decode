package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;

@Config
@TeleOp(name = "Turret_Tester", group = "Tester")
public class TurretLL extends OpMode {
    private TurretTester turret;
    private Limelight3A limelight;
    private LLResult result;
    private double tX;

    private boolean trackingEnabled = false;
    private boolean togglePressed = false;

    public static double targetAngle = 0.0; // Target angle for manual aiming

    @Override
    public void init() {
        turret = new TurretTester(this);
        turret.init();

        limelight = hardwareMap.get(Limelight3A.class, Constants.HMLimelight);
        limelight.pipelineSwitch(1);
        limelight.start();

        telemetry.addLine("Turret Initialized");
        telemetry.addLine("Press [X] to toggle auto-tracking mode");
        telemetry.update();
    }

    @Override
    public void loop() {
        result = limelight.getLatestResult();
        if(result != null) {
            tX = result.getTx();
        } else {
            tX = 0;
        }

        if (gamepad1.x && !togglePressed) {
            trackingEnabled = !trackingEnabled;
            togglePressed = true;
        } else if (!gamepad1.x) {
            togglePressed = false;
        }

        if (trackingEnabled) {
            turret.trackTarget(tX);
            telemetry.addLine("Mode: Tracking Apriltag");
        } else {
            turret.AimToAngle(targetAngle);
            double manualPower = gamepad1.right_stick_x * 0.5; // reduce sensitivity
            turret.manualPower(manualPower);
            telemetry.addLine("Mode: Manual Control");
        }

        telemetry.addData("Tracking Enabled", trackingEnabled);
        telemetry.update();
    }
}
