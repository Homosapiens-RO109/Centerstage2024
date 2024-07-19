package org.firstinspires.ftc.teamcode.arghh;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.concurrent.TimeUnit;

@TeleOp
@Config
public class teleop extends LinearOpMode {
    FtcDashboard dashboard;
    public Outtake outtake;
    public Intake intake;
    public DDrive drive;
    @Override
    public void runOpMode() throws InterruptedException {
        intake = new Intake(hardwareMap);
        drive = new DDrive(hardwareMap, gamepad1);
        outtake = new Outtake(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        waitForStart();

        while (opModeIsActive()) {
            intake.timer.startTime();
            intakecontrol();
            drive.movement();
            outtake.PID();
            outtakecontrol();
            telemetry.addData("Target", outtake.target);
            telemetry.addData("Pozitie Slidere", outtake.motordr.getCurrentPosition());
            telemetry.addData("Timer", intake.timer.time(TimeUnit.MILLISECONDS));
            telemetry.update();
        }
    }
    public void intakecontrol() {
        if(gamepad1.right_bumper) {
            if(intake.motorin.getPower() == 0  && intake.timer.time(TimeUnit.MILLISECONDS) > 500) {
                intake.start_in();
            }
            if(intake.motorin.getPower() != 0  && intake.timer.time(TimeUnit.MILLISECONDS) > 500) {
                intake.stop_in();
            }
        }
        if(gamepad1.left_bumper) {
            if(intake.motorin.getPower() == 0 && intake.timer.time(TimeUnit.MILLISECONDS) > 500) {
                intake.reverse_in();
            }
            if(intake.motorin.getPower() != 0 && intake.timer.time(TimeUnit.MILLISECONDS) > 500) {
                intake.stop_in();
            }
        }
    }
    public void outtakecontrol() {
        if(outtake.slider_peste_lim())
            outtake.brat_out();
        if(!outtake.slider_peste_lim())
            outtake.brat_in();
        if(!outtake.motordr.isBusy()) {
            if(gamepad1.dpad_down)
                outtake.poz_1();
            if(gamepad1.dpad_right)
                outtake.poz_2();
            if(gamepad1.dpad_up)
                outtake.poz_3();
            if(gamepad1.dpad_left)
                outtake.poz_4();
        }
        if(gamepad1.a)
            outtake.servoDegetica.setPosition(0.5);
        if(gamepad1.b)
            outtake.servoDegetica.setPosition(0);
    }
}