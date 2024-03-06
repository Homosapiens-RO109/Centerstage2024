package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.CRServo;


@TeleOp(name = "DriveTeleOP", group = "TeleOp")
public class DriveTeleOP extends LinearOpMode {
 //   private DcMotor motor1;
 //   private DcMotor motor2;
//    private DcMotor motor3;
   // private DcMotor motor4;
   // private ColorSensor Colorsensor;
    private CRServo Servo;


    @Override
    public void runOpMode() {
        //motor1 = hardwareMap.get(DcMotor.class, "stsus");
       // motor2 = hardwareMap.get(DcMotor.class, "drsus");
        //motor3 = hardwareMap.get(DcMotor.class, "drjos");
       // motor4 = hardwareMap.get(DcMotor.class, "stjos");
       // Colorsensor = hardwareMap.get(ColorSensor.class, "ColorSensor");
        Servo = hardwareMap.get(CRServo.class, "Zoom");

        waitForStart();

        while (opModeIsActive()) {
            double leftStickY = -gamepad1.left_stick_y;
            double leftStickX = gamepad1.left_stick_x;
            double LTrigger = gamepad1.left_trigger;
            double RTrigger = gamepad1.right_trigger;

            // Puterea care o primesc motoarele sub forma de double inregistrate de controller

            double power1 = Range.clip(-leftStickY - leftStickX, -1, 1);
            double power2 = Range.clip(leftStickY - leftStickX, -1, 1);
            double power3 = Range.clip(-leftStickY - leftStickX, -1, 1);
            double power4 = Range.clip(-leftStickY + leftStickX, -1, 1);

            // Culorile vazute de color sensor sub forma de int

          //  double colorBlue = Colorsensor.blue();
           // double colorGreen = Colorsensor.green();
            //double colorRed = Colorsensor.red();
           // Colorsensor.enableLed(true);

        if (LTrigger > 0) {
         //   motor1.setPower(LTrigger);
          //  motor4.setPower(LTrigger);
          //  motor3.setPower(-LTrigger);
          //  motor2.setPower(-LTrigger);
        }
        
        if (RTrigger > 0) {
         //   motor1.setPower(-RTrigger);
         //   motor4.setPower(-RTrigger);
          //  motor3.setPower(RTrigger);
          //  motor2.setPower(RTrigger);
        }



          //  motor1.setPower(power1);
          //  motor2.setPower(power2);
          //  motor3.setPower(power3);
          //  motor4.setPower(power4);

          //  telemetry.addData("Motor Powers", "Motor1: %.2f, Motor2: %.2f, Motor3: %.2f, Motor4: %.2f",
           //         power1, power2, power3, power4);
          //  telemetry.addData("Colors", "colorBlue: %.0f, colorGreen: %.0f, colorRed: %.0f", colorBlue, colorGreen, colorRed);
            telemetry.update();


            sleep(10);
        }
    }
}
