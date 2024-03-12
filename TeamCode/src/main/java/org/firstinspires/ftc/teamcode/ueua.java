package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Autonomie.Pose;

@TeleOp
public class ueua extends LinearOpMode {
    double c;
    @Override
    public void runOpMode() throws InterruptedException {
        while(opModeIsActive())
        {
            c = Pose.pozo;
            telemetry.addData("" ,c);
            telemetry.update();
        }
    }
}
