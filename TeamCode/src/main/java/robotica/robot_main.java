package robotica;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class robot_main extends LinearOpMode {
    FtcDashboard dashboard;


    @Override
    public void runOpMode() throws InterruptedException {
        inteik intake = new inteik(hardwareMap, gamepad1);
        outeik outake = new outeik(hardwareMap, gamepad1);
        draiv drive = new draiv(hardwareMap, gamepad1);
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());


        waitForStart();
        while (opModeIsActive()) {
            intake.timerIntake.startTime();

            drive.movement();

            intake.ControlIn();

            outake.miscaSlider(-gamepad1.right_stick_y * 50);

            outake.miscaPID();

            outake.ControlBrat();

        }
    }
}
