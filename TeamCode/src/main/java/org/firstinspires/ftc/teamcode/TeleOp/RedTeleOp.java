package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.PinPoint;

@TeleOp(name="RedTeleOp", group="!TeleOp")
public class RedTeleOp extends OpMode {
    Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, PinPoint.AllianceColor.RED, gamepad1, telemetry);
        robot.start();
    }

    @Override
    public void loop() {
        robot.update();
    }
}
