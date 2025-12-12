package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Commands.LaunchArtifactCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Commands.SpindexerColorIntakeCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Subsystems.LimeLight;
import org.firstinspires.ftc.teamcode.Subsystems.Popper;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

@TeleOp(name="TeleOp_Tester", group="!TeleOp")
@Config
public class TeleOpTesting extends OpMode {
    Spindexer spindexer;
    Popper popper;
    Launcher launcher;
    LimeLight limelight;
    public double motifID = -1;
    public double aprilTagID = -1;
    Turret turret;
    ElapsedTime offSetTurretTime = new ElapsedTime();
    boolean isUsingTurret = false;
    boolean prevDpadUp = false;

    LaunchArtifactCommand launchArtifactCommand;
    SpindexerColorIntakeCommand spindexerColorSensorIntakeCommand;

    MecanumDrive drive_roadrunner;
    public static double xValueGoal = -60;
    public static double yValueGoal = 60;
    public static double xValueBot = -2;
    public static double yValueBot = 40;
    Pose2d initialPose = new Pose2d(xValueBot, yValueBot, Math.toRadians(90));
    Pose2d BLUE_GOALPose = new Pose2d(xValueGoal, yValueGoal, 0);


    TelemetryPacket packet;
    FtcDashboard dashboard;

    public static double targetVel = 0;
    public static double targetAngle = 0;
    @Override
    public void init(){
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        limelight = new LimeLight(hardwareMap);

        spindexer = new Spindexer(hardwareMap);

        popper = new Popper(hardwareMap);
        launcher = new Launcher(hardwareMap);
        turret = new Turret(hardwareMap);
        drive_roadrunner = new MecanumDrive(hardwareMap, initialPose);
        offSetTurretTime.reset();

        spindexerColorSensorIntakeCommand = new SpindexerColorIntakeCommand(spindexer);
        spindexerColorSensorIntakeCommand.start();

        packet = new TelemetryPacket();
        dashboard = FtcDashboard.getInstance();


        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {

        launcher.setVelocity(targetVel);
        launcher.setCoverAngle(targetAngle);

        limelight();

        colorSensorIntake();

        launchCommand();

        turret();


        telemetry.update();
        dashboard.sendTelemetryPacket(packet);
    }

    public void launchCommand() {
        if (gamepad1.a) {
            launchArtifactCommand = new LaunchArtifactCommand(spindexer, popper, launcher, drive_roadrunner);
            launchArtifactCommand.startAuto(targetVel, targetAngle);
        }

        if (launchArtifactCommand != null && !launchArtifactCommand.isFinished()) {
            launchArtifactCommand.update(packet);
        }

        if (launchArtifactCommand != null && launchArtifactCommand.isFinished()) {
            launchArtifactCommand = null;
            launcher.stopLauncher();
            popper.deactivatePopper();
        }
    }

    public void colorSensorIntake() {
        if (launchArtifactCommand == null) {
            spindexerColorSensorIntakeCommand.update(telemetry);
        }
    }

    public void turret() {
        // TURRET LOGIC
        drive_roadrunner.updatePoseEstimate();
        turret.setRobotHeading(Math.toDegrees(drive_roadrunner.localizer.getPose().heading.log()));
        double targetangle = turret.angleBotToGoal(
                BLUE_GOALPose.position.y - drive_roadrunner.localizer.getPose().position.y,
                BLUE_GOALPose.position.x - drive_roadrunner.localizer.getPose().position.x,
                Math.toDegrees(drive_roadrunner.localizer.getPose().heading.log()));

        if (offSetTurretTime.seconds() < 2) {
            turret.zeroTurretRelativeToRobot();
        }
        boolean current = gamepad1.dpad_up;
        if (current && !prevDpadUp) {
            isUsingTurret = !isUsingTurret;
        }
        prevDpadUp = current;
        if(turret.getTurretZeroOffsetField() != 0.0) {
            if (isUsingTurret) {
                if (limelight.isResulted()) {
                    if (limelight.getAprilTagID() == 24) {
                        turret.trackAprilTag(limelight.getTX());
                        packet.put("Turret with: ", "LimeLight");
                    }
                } else {
                    packet.put("Turret with: ", "PinPoint");
                    turret.trackTargetAngle(targetangle);
                }
                packet.put("Turret Mode: ", "ON");
            } else {
                turret.setPower(0);
                packet.put("Turret Mode: ", "OFF");
            }
        }
        packet.put("Desired Turret Angle: ", targetangle);
    }

    public void limelight() {
        limelight.getResult();
        aprilTagID = limelight.getAprilTagID();
        motifID = limelight.getMotifID();
        packet.put("AprilTag ID: ", limelight.getAprilTagID());
        packet.put("Motif ID: ", limelight.getMotifID());
    }

}
