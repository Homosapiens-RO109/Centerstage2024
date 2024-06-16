package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class robot_TeleOp extends LinearOpMode {
    FtcDashboard dashboard;
    @Override
    public void runOpMode() {
        robot_drive drive = new robot_drive(hardwareMap, gamepad1);
        robot_intake intake = new robot_intake(hardwareMap, gamepad1);
        robot_outtake outtake = new robot_outtake(hardwareMap, gamepad1);
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        waitForStart();
        while(opModeIsActive()) {
            drive.movement();
            intake.timerIntake.startTime();
            intake.ControlIntake();
            outtake.SlidersPID();
            outtake.MoveSliders(-gamepad1.right_stick_y * 50);
            outtake.UseServos();
            telemetry.addData("Target: ", robot_outtake.target);
            telemetry.addData("MotorGL: ", outtake.MotorLeftSlider.getCurrentPosition());
            telemetry.addData("ServoSt: ", outtake.ServoLeft.getPosition());
            telemetry.addData("ServoDr: ", outtake.ServoRight.getPosition());
            telemetry.addData("ServoWrist: ", outtake.ServoWrist.getPosition());
            telemetry.update();
        }
    }
}