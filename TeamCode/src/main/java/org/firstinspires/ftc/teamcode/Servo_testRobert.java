package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
//nu ma intreba Dragos de ce am facut asa, mi-a murit creierul, nu am gandit mai eficient, n-am putut
@TeleOp
public class Servo_testRobert extends LinearOpMode {
    Servo servo;
    double position = 0, modifier = 0.00001;
    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(Servo.class, "Cleste");
        servo.setPosition(0);

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                position += modifier;
            }

            if(gamepad1.b) {
                position -= modifier;
            }

            if(gamepad1.x) {
                servo.setPosition(position);
            }


            telemetry.addData("Position: ", position);
            telemetry.addData("Pozitie servo: ", servo.getPosition());
            telemetry.update();
        }
    }
}
