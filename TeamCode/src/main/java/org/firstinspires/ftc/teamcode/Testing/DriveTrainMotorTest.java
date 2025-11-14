package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "DriveTrain Motor Test")
public class DriveTrainMotorTest extends OpMode {
    DcMotor frontLeft, frontRight, backLeft, backRight;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotorEx.class, "");
        frontRight = hardwareMap.get(DcMotorEx.class, "");
        backLeft = hardwareMap.get(DcMotorEx.class, "");
        backRight = hardwareMap.get(DcMotorEx.class, "");
    }

    @Override
    public void loop() {
        if (gamepad1.x) {
            frontLeft.setPower(1);
        }
        else if (gamepad1.y) {
            frontRight.setPower(1);
        }
        else if (gamepad1.b) {
            backLeft.setPower(1);
        }
        else if (gamepad1.a) {
            backRight.setPower(1);
        }
        else {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }
    }
}
