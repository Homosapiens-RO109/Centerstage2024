package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Autonomie.Pose;

@TeleOp
public class Testpoz extends LinearOpMode {
    int a ;
    public void runOpMode() throws InterruptedException
    {
        while(opModeIsActive())
        {
            a++;
            Pose.pozo = a;
            telemetry.addData("",a);
        }
    }
}
