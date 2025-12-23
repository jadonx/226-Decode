package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;

@TeleOp(name="BlueTeleOp", group="!TeleOp")
public class BlueTeleOp extends OpMode {
    FieldCentricDrive drive;
    Intake intake;

    @Override
    public void init() {
        drive = new FieldCentricDrive(hardwareMap);
        intake = new Intake(hardwareMap);
    }

    @Override
    public void loop() {
        drive.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        if (gamepad1.x) {
            drive.resetIMU();
        }

        intake.runIntake(gamepad1.right_trigger);
        intake.reverseIntake(gamepad1.left_trigger);
    }
}
