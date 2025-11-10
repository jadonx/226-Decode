package org.firstinspires.ftc.teamcode.Subsystems.Outtake;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Core.Controller.PIDFController;
import org.firstinspires.ftc.teamcode.Subsystems.Encoder.AS5600Encoder;

public class TurretIMU {
    private CRServo turretLeft;
    private CRServo turretRight;
    private IMU imu;
    private AS5600Encoder turretEncoder;
    private int setPoint = 0;
    private Telemetry telemetry;

    private double turretOffsetInches = 3;


    Pose2d poseBot = new Pose2d(0.0, 0.0, 0.0);
    PIDFController pidf = new PIDFController(
            TurretConstant.kP,
            TurretConstant.kI,
            TurretConstant.kD,
            TurretConstant.kF
    );

    public void Init(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;
        imu = hardwareMap.get(IMU.class, TurretConstant.HMIMU);

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        turretLeft = hardwareMap.get(CRServo.class, TurretConstant.HMServoTurretLeft);
        turretRight = hardwareMap.get(CRServo.class, TurretConstant.HMServoTurretRight);
        turretEncoder = hardwareMap.get(AS5600Encoder.class, TurretConstant.HMEncoder);
    }

    public enum GOALS {
        RED_GOAL,
        BLUE_GOAL
    }

    Pose2d poseRedGoal = new Pose2d(155, 155,0);
    Pose2d poseBlueGoal = new Pose2d(0, 135,0);


    double distanceToGoal = 0;
    public void AimTo(GOALS goal, Pose2d poseBot) {
        this.poseBot = poseBot;
        Pose2d target;
        switch (goal) {
            case RED_GOAL:
                target = poseRedGoal;
                break;
            case BLUE_GOAL:
                target = poseBlueGoal;
                break;
            default:
                throw new IllegalStateException("Unexpected value: " + goal);
        }

        Pose2d turretPose = getTurretPose(poseBot);

        double deltax = target.position.x - turretPose.position.x;
        double deltay = target.position.y - turretPose.position.y;


        distanceToGoal = Math.sqrt(Math.pow(deltax, 2) + Math.pow(deltay, 2));
        double angle = Math.toDegrees(Math.atan2(deltay, deltax));

        SetAngleFC(angle);
    }

    private Pose2d getTurretPose(Pose2d botPose) {
        double heading = botPose.heading.toDouble(); // radianos
        double turretX = botPose.position.x + turretOffsetInches * Math.cos(heading);
        double turretY = botPose.position.y + turretOffsetInches * Math.sin(heading);
        return new Pose2d(turretX, turretY, botPose.heading.toDouble());
    }

    public double GetDistanceToGoal() {
        return distanceToGoal;
    }

    public void Reset(){
        imu.resetYaw();
        turretEncoder.resetTurretAngle();
    }

    public double getAngle() {
        return turretEncoder.getTurretAngleDegrees();
    }

    public void SetSetPoint(int sp) {
        setPoint = sp;
    }

    public void SetAngleFC(double desiredDegrees) {
        double robotHeading = Math.toDegrees(poseBot.heading.toDouble());
        double turretTargetRel = desiredDegrees - robotHeading + 180;
        turretTargetRel = AngleUnit.normalizeDegrees(turretTargetRel);
        SetAngle(-turretTargetRel);
    }
    public void SetAngle(double desiredDegrees) {
        setPoint = (int) desiredDegrees; // direct degrees, not ticks
    }

    public void Periodic() {
        double currentAngle = turretEncoder.getTurretAngleDegrees();
        pidf.setSetpoint(setPoint);

        double output = pidf.calculate(currentAngle);

        turretLeft.setPower(output);
        turretRight.setPower(output);
    }

    public boolean stabilized() {
        pidf.setTolerance(TurretConstant.stabilizedTolerance);
        return pidf.atSetpoint(turretEncoder.getTurretAngleDegrees());
    }
}
