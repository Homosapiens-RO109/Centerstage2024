package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp
public class magnetic extends LinearOpMode {

    @Override
    public void runOpMode() {
        DigitalChannel mag;
        mag = hardwareMap.get(DigitalChannel.class, "magnetic");
        mag.setMode(DigitalChannel.Mode.INPUT);
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Magnetic sensor ", mag.getState());
            telemetry.update();
        }
    }
}
