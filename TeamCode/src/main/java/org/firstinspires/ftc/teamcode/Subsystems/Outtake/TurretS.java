package org.firstinspires.ftc.teamcode.Subsystems.Outtake;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name = "Turret", group = "Testing")
public class TurretS extends OpMode {
    private TurretSubsystem turret;
    private FtcDashboard dashboard;
    private Limelight3A limelight;
    private LLResult result;
    private double tX;

    private boolean trackingEnabled = false;
    private boolean togglePressed = false;

    @Override
    public void init() {

        turret = new TurretSubsystem(this);
        turret.init();

        limelight = hardwareMap.get(Limelight3A.class, "LL");
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
            double manualPower = gamepad1.right_stick_x * 0.5; // reduce sensitivity
            turret.manualPower(manualPower);
            telemetry.addLine("Mode: Manual Control");
        }

        telemetry.addData("Tracking Enabled", trackingEnabled);
        telemetry.update();
    }
}
