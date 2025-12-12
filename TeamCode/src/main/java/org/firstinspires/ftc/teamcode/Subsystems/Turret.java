package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;

@Config
public class Turret {
    private CRServo turretLeft, turretRight;
    private PIDController pid;
    private IMU turretEncoder;

    public static double kP_Turret = 0.02;
    public static double kI_Turret = 0.0;
    public static double kD_Turret = 0.0008;
    public static double tolerance_Turret = 0.8;
    private double turretZeroOffsetField = 0;
    private double robotHeadingDeg = 0;

    public Turret(HardwareMap hardwareMap) {
        turretRight = hardwareMap.get(CRServo.class, Constants.HMServoTurretRight);
        turretLeft = hardwareMap.get(CRServo.class, Constants.HMServoTurretLeft);

        turretEncoder = hardwareMap.get(IMU.class, Constants.HMTurretEncoder);

        IMU.Parameters turretParameters = new IMU.Parameters(new RevHubOrientationOnRobot( RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP));

        turretEncoder.initialize(turretParameters);

        turretEncoder.resetYaw();

        pid = new PIDController(kP_Turret, kI_Turret, kD_Turret);
        pid.setTolerance(tolerance_Turret);
    }

    public void trackAprilTag(double targetX) {
        pid.setPID(kP_Turret, kI_Turret, kD_Turret);

        double correction = pid.calculate(targetX, 0);

        correction = Math.max(Math.min(correction, 0.5), -0.5);

        if (pid.atSetPoint()) {
            setPower(0);
            return;
        }

        setPower(correction);
    }

    public void trackTargetAngle(double targetA){
        pid.setPID(kP_Turret, kI_Turret, kD_Turret);

        double currentAngle = getTurretAngle();

        double correction = pid.calculate(currentAngle, targetA);

        correction = Math.max(Math.min(correction, 0.5), -0.5);

        if (pid.atSetPoint()) {
            setPower(0);
            return;
        }

        setPower(correction);
    }

    public void setPower(double power) {
        turretLeft.setPower(power);
        turretRight.setPower(power);
    }

    public double angleBotToGoal(double y, double x, double h) {
        if (x == 0.0 && y == 0.0) return 0;
        double angleToGoalDeg = Math.toDegrees(Math.atan2(y, x));
        double turretAngle = angleToGoalDeg - h;
        turretAngle = normalize(turretAngle);
        return turretAngle;
    }

    public void setRobotHeading(double headingDeg) {
        robotHeadingDeg = headingDeg;
    }

    public void zeroTurretRelativeToRobot() {
        double turretField = turretEncoder.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        turretZeroOffsetField = normalize(turretField - robotHeadingDeg);
    }

    public void stopTurret() {
        setPower(0);
        turretEncoder.resetYaw();
    }

    public void turretResetEncoder() {
        turretEncoder.resetYaw();
    }

    private double normalize(double angle) {
        angle = angle % 360.0;
        if (angle > 180.0) angle -= 360.0;
        if (angle <= -180.0) angle += 360.0;
        return angle;
    }

    public double getTurretAngle() {
        double turretField = turretEncoder.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        return normalize(turretField - robotHeadingDeg - turretZeroOffsetField);
    }

    public double getTurretAngleRaw() {
        return turretEncoder.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public double getTurretZeroOffsetField() {
        return turretZeroOffsetField;
    }

    public double getRobotHeadingDeg() {
        return robotHeadingDeg;
    }


    /*
    AUTO INITIALIZATIONS
    **/
    public void trackTargetAngleAuto(double targetA){
        pid.setPID(kP_Turret, kI_Turret, kD_Turret);

        double currentAngle = getTurretAngleRaw();

        double correction = pid.calculate(currentAngle, targetA);

        correction = Math.max(Math.min(correction, 0.5), -0.5);

        if (pid.atSetPoint()) {
            setPower(0);
            return;
        }

        setPower(correction);
    }

    public boolean isAtTarget() {
        return pid.atSetPoint();
    }

}
