package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.sql.Time;
import java.util.concurrent.TimeUnit;
@TeleOp
public class Test extends LinearOpMode {
    private DcMotor MotorIn;
    private CRServo servoras;
    ElapsedTime timerIntake = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    public int p00la = 0;

    public static double niv1 = 0.25, niv2 = 0.5, niv3 = 0.75, niv4 = 1;


    @Override
    public void runOpMode() throws InterruptedException {
//        MotorIn = hardwareMap.get(DcMotor.class, "motor");
        servoras = hardwareMap.get(CRServo.class, "sirbulici");

        servoras.resetDeviceConfigurationForOpMode();


        waitForStart();

        if(opModeIsActive())
            timerIntake.startTime();

        while(opModeIsActive()) {
            if(gamepad1.a)
                servoras.setPower(1);

            if(gamepad1.b)
                servoras.setPower(-1);

//            if(gamepad1.a)
//            {
//                if(MotorIn.getPower() == 0 && timerIntake.time(TimeUnit.SECONDS) >=+ 1) {
//                    MotorIn.setPower(1);
//                    timerIntake.reset();
//                }
//
//                if(MotorIn.getPower() != 0 && timerIntake.time(TimeUnit.SECONDS) >= 1) {
//                    MotorIn.setPower(0);
//                    timerIntake.reset();
//                }
//
//            }
//            p00la += gamepad1.right_stick_y*2;

            telemetry.addData("Timer: ",timerIntake.time(TimeUnit.SECONDS));
            telemetry.addData("Coaca: ", p00la);
            telemetry.update();
        }
    }
}
