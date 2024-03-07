package org.firstinspires.ftc.teamcode.Autonomie;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "Blue_Detection_Frontstage")
public class Camera_Blue_Frontstage extends LinearOpMode{
    Pipeline_Blue detection = new Pipeline_Blue();
    OpenCvWebcam webcam;
    final int ticks_motorFL = 536, ticks_motorFR = 529, ticks_motorRL = 538, ticks_motorRR = 525;
    DcMotor motorFL,motorFR, motorRL, motorRR, motorRoata, motorGL, motorGR;
    Servo servoBD, servoBS, servowr;
    int pas = 0,timere=0;
    @Override
    public void runOpMode() throws InterruptedException {
        motorFL = hardwareMap.get(DcMotor.class, "stsus");
        motorFR = hardwareMap.get(DcMotor.class, "drsus");
        motorRL = hardwareMap.get(DcMotor.class, "stjos");
        motorRR = hardwareMap.get(DcMotor.class, "drjos");
        motorGL = hardwareMap.get(DcMotor.class, "glis1");
        motorGR = hardwareMap.get(DcMotor.class, "glis2");
        motorRoata = hardwareMap.get(DcMotor.class, "motorin");

        servoBD = hardwareMap.get(Servo.class, "servodr");
        servoBS = hardwareMap.get(Servo.class, "servost");
        servowr = hardwareMap.get(Servo.class, "servoin");

        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorRL.setDirection(DcMotor.Direction.REVERSE);
        motorRR.setDirection(DcMotor.Direction.FORWARD);
        motorRoata.setDirection(DcMotor.Direction.REVERSE);

        servoBD.setDirection(Servo.Direction.REVERSE);
        motorGL.setDirection(DcMotorSimple.Direction.REVERSE);

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int webcamID = hardwareMap.appContext.getResources().getIdentifier("webcamID", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, webcamID);
        webcam.setPipeline(detection);

        webcam.openCameraDeviceAsync(new OpenCvWebcam.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        Pipeline_Blue.SkystonePosition pozitie = detection.getAnalysis();

        while(!isStarted())
        {
            pozitie = detection.getAnalysis();
            telemetry.addData("Position: ", pozitie);
            telemetry.update();
        }

        waitForStart();

        while(opModeIsActive())
        {
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
            if(pozitie == Pipeline_Blue.SkystonePosition.LEFT)
            {
                {
                    if(pas == 0){
                        DriveBackwards(1.8);
                        pas++;
                    }

                    if(pas == 1 && !motorFL.isBusy())
                    {
                        DriveRight(1);
                        pas++;
                    }

                    if(pas == 2 && !motorFL.isBusy())
                    {
                        DriveForward(0.4);
                        motorRoata.setPower(-0.8);
                        sleep(500);
                        motorRoata.setPower(0);
                        pas++;
                    }

                    if(pas == 3)
                    {
                        stop_motors();
                        pas++;
                    }
//                if(pas == 3)
//                {
//                    RotateLeft(2);
//                    pas++;
//                }
                }
                // to do: pus pixel
            }
            else
            if(pozitie == Pipeline_Blue.SkystonePosition.CENTER && pas == 0)
            {
                TrajectorySequence myTraj = drive.trajectorySequenceBuilder(new Pose2d())
                        .splineToConstantHeading(new Vector2d(28, 0), 0)
                        .waitSeconds(0.5)
                        .splineToConstantHeading(new Vector2d(26, 0), 0)
                        .addDisplacementMarker(() -> {
                            motorRoata.setPower(-0.5);
                        })
                        .waitSeconds(1)
                        .splineToConstantHeading(new Vector2d(23, 0), 0)
                        .addDisplacementMarker(() -> {
                            motorRoata.setPower(0);
                        })
//                        .turn(Math.toRadians(-90))
//                        .addDisplacementMarker(() -> {
//                            motorGL.setPower(1);
//                            motorGR.setPower(1);
//                        }) -> Pid mi e prea somn sa fac :3
                        .waitSeconds(1)
                        .splineToConstantHeading(new Vector2d(-20, 0), 0)
                        .waitSeconds(1)
                        .splineToConstantHeading(new Vector2d(-20, 10), 0)
                        .waitSeconds(1)
                        .splineToConstantHeading(new Vector2d(-30, 10), 0)
                        .build();
                drive.followTrajectorySequence(myTraj);
                Pose2d poseEstimate = drive.getPoseEstimate();
                telemetry.addData("x", poseEstimate.getX());
                telemetry.addData("y", poseEstimate.getY());
                telemetry.addData("heading", poseEstimate.getHeading());
                telemetry.update();
                pas++;
            }
            else
            if(pozitie == Pipeline_Blue.SkystonePosition.RIGHT)
            {
                if(pas == 0){
                    DriveBackwards(2.3);
                    pas++;
                }

                if(pas == 1 && !motorFL.isBusy())
                {
                    pas++;
                    DriveRight(0.4);
                }

                if(pas == 2 && !motorFL.isBusy())
                {
                    RotateRight(2);
                    pas++;
                }

                if(pas == 3 && !motorFL.isBusy())
                {
                    DriveBackwards(0.2);
                    pas++;
                }

                if(pas == 4 && !motorFL.isBusy()) {
                    motorRoata.setPower(-1);
                    sleep(500);
                    motorRoata.setPower(0);
                    pas++;
                }
            }

        }

    }
    public void stop_motors()
    {
        motorRL.setPower(0);
        motorRR.setPower(0);
        motorFL.setPower(0);
        motorFR.setPower(0);
    }
    public void DriveForward(double times)
    {
        motorFL.setTargetPosition((int)(ticks_motorFL * times + motorFL.getCurrentPosition()));
        motorRL.setTargetPosition((int)(ticks_motorRL * times + motorRL.getCurrentPosition()));
        motorRR.setTargetPosition((int)(ticks_motorRR * times + motorRR.getCurrentPosition()));
        motorFL.setTargetPosition((int)(ticks_motorFL * times + motorFL.getCurrentPosition()));
        motorFR.setTargetPosition((int)(ticks_motorFR * times + motorFR.getCurrentPosition()));

        motorRL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorRL.setPower(0.5);
        motorRR.setPower(0.5);
        motorFL.setPower(0.5);
        motorFR.setPower(0.5);
    }

    public void DriveBackwards(double times)
    {
        motorRL.setTargetPosition((int)(-ticks_motorRL * times + motorRL.getCurrentPosition()));
        motorRR.setTargetPosition((int)(-ticks_motorRR * times + motorRR.getCurrentPosition()));
        motorFL.setTargetPosition((int)(-ticks_motorFL * times + motorFL.getCurrentPosition()));
        motorFR.setTargetPosition((int)(-ticks_motorFR * times + motorFR.getCurrentPosition()));

        motorRL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorRL.setPower(-0.5);
        motorRR.setPower(-0.5);
        motorFL.setPower(-0.5);
        motorFR.setPower(-0.5);
    }

    public void DriveLeft(double times)
    {
        motorRL.setTargetPosition((int)(ticks_motorRL * times + motorRL.getCurrentPosition()));
        motorRR.setTargetPosition((int)(-ticks_motorRR * times + motorRR.getCurrentPosition()));
        motorFL.setTargetPosition((int)(-ticks_motorFL * times + motorFL.getCurrentPosition()));
        motorFR.setTargetPosition((int) (ticks_motorFR * times + motorFR.getCurrentPosition()));

        motorRL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorRL.setPower(0.5);
        motorRR.setPower(-0.5);
        motorFL.setPower(-0.5);
        motorFR.setPower(0.5);
    }
    public void DriveRight(double times)
    {
        motorRL.setTargetPosition((int)(-ticks_motorRL * times + motorRL.getCurrentPosition()));
        motorRR.setTargetPosition((int)(ticks_motorRR * times + motorRR.getCurrentPosition()));
        motorFL.setTargetPosition((int)(ticks_motorFL * times + motorFL.getCurrentPosition()));
        motorFR.setTargetPosition((int)(-ticks_motorFR * times + motorFR.getCurrentPosition()));

        motorRL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorRL.setPower(-0.5);
        motorRR.setPower(0.5);
        motorFL.setPower(0.5);
        motorFR.setPower(-0.5);
    }

    public void RotateLeft(double times)
    {
        motorRL.setTargetPosition((int) (-ticks_motorRL * times + motorRL.getCurrentPosition()));
        motorRR.setTargetPosition((int) (ticks_motorRR * times + motorRR.getCurrentPosition()));
        motorFL.setTargetPosition((int) (-ticks_motorFL * times + motorFL.getCurrentPosition()));
        motorFR.setTargetPosition((int) (ticks_motorFR * times + motorFR.getCurrentPosition()));

        motorRL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorRL.setPower(-1);
        motorRR.setPower(1);
        motorFL.setPower(-1);
        motorFR.setPower(1);
    }

    public void RotateRight(double times)
    {
        motorRL.setTargetPosition((int)(ticks_motorRL * times + motorRL.getCurrentPosition()));
        motorRR.setTargetPosition((int)(-ticks_motorRR * times + motorRR.getCurrentPosition()));
        motorFL.setTargetPosition((int)(ticks_motorFL * times + motorFL.getCurrentPosition()));
        motorFR.setTargetPosition((int)(-ticks_motorFR * times + motorFR.getCurrentPosition()));

        motorRL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorRL.setPower(1);
        motorRR.setPower(-1);
        motorFL.setPower(1);
        motorFR.setPower(-1);
    }
}