package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Supporters.PoseStorage;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

@TeleOp(name="TurretEncoderTester", group="Test")
public class TurretEncoderTester extends OpMode {
    Turret turret;
    boolean isFollowingBotHeading = false;

    @Override
    public void init() {
        turret = new Turret(hardwareMap);
    }

    @Override
    public void loop() {
        // turret.setPower(gamepad1.left_stick_x);
        if (gamepad1.aWasPressed()) {
            isFollowingBotHeading = !isFollowingBotHeading;
        }

        if (isFollowingBotHeading) {
            turret.goToAngle(PoseStorage.getHeading());
        }
        else {
            turret.setPower(gamepad1.left_stick_x);
        }

        telemetry.addData("current status ", isFollowingBotHeading);
        telemetry.addData("target angle ", PoseStorage.getHeading());
        telemetry.addData("turret angle ", turret.getTurretAngle());
        telemetry.addData("error ", turret.getError());

        telemetry.update();
    }
}
