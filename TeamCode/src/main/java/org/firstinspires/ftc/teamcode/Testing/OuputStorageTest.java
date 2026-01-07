package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Supporters.PoseStorage;

@TeleOp(name="OutputStorageTest", group="Test")
public class OuputStorageTest extends OpMode {
    @Override
    public void init() {

    }

    @Override
    public void loop() {
        telemetry.addData("Storage x ", PoseStorage.getX());
        telemetry.addData("Storage y ", PoseStorage.getY());
        telemetry.addData("Storage heading ", PoseStorage.getHeading());

        telemetry.update();
    }
}
