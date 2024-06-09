package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class robot_TeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        robot_drive drive = new robot_drive(hardwareMap, gamepad1);
        robot_intake intake = new robot_intake(hardwareMap, gamepad1);
        robot_outtake outtake = new robot_outtake(hardwareMap, gamepad1);
        waitForStart();
        while(opModeIsActive()) {
            drive.movement();
            intake.power();
            outtake.SlidersPID();
            outtake.UseServos();
            outtake.ExtendSliders(-gamepad1.right_stick_y * 50);
        }
    }
}
