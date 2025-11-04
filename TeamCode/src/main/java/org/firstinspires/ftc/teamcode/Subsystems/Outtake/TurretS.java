package org.firstinspires.ftc.teamcode.Subsystems.Outtake;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Limelight.LLSubsystem;

@TeleOp(name = "Turret", group = "Testing")
public class TurretS extends OpMode {
    private TurretSubsystem turret;
    private LLSubsystem limelight;
    private FtcDashboard dashboard;


    private boolean trackingEnabled = false;
    private boolean togglePressed = false;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        limelight = new LLSubsystem(this);
        limelight.init();

        turret = new TurretSubsystem(this, limelight);
        turret.init();

        telemetry.addLine("Turret Initialized");
        telemetry.addLine("Press [X] to toggle auto-tracking mode");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.x && !togglePressed) {
            trackingEnabled = !trackingEnabled;
            togglePressed = true;
        } else if (!gamepad1.x) {
            togglePressed = false;
        }

        if (trackingEnabled) {
            turret.trackTarget();
            telemetry.addLine("Mode: Tracking Apriltag");
        } else {
            double manualPower = gamepad1.right_stick_x * 0.5; // reduce sensitivity
            turret.manualPower(manualPower);
            telemetry.addLine("Mode: Manual Control");
        }

        telemetry.addData("Tracking Enabled", trackingEnabled);
        telemetry.addData("Target X", limelight.getTargetX());
        telemetry.addData("kP", TurretConstant.kP);
        telemetry.addData("kI", TurretConstant.kI);
        telemetry.addData("kD", TurretConstant.kD);
        telemetry.update();
    }
}
