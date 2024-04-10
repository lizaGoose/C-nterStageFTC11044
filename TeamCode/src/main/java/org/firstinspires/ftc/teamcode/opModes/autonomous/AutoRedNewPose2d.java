package org.firstinspires.ftc.teamcode.opModes.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Camera.PropDetection;
import org.firstinspires.ftc.teamcode.Camera.PropDetectionRed;
import org.firstinspires.ftc.teamcode.Modules.IntakeSecondVersion;
import org.firstinspires.ftc.teamcode.Modules.LiftAuto;
import org.firstinspires.ftc.teamcode.Modules.SpikeScorer;

import org.firstinspires.ftc.teamcode.Robot1;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Config
@Autonomous(group = "drive")
public class AutoRedNewPose2d extends LinearOpMode {
    Robot1 R;
    IntakeSecondVersion intake;
    SpikeScorer scorer;
    Servo hook;
    OpenCvCamera cam;

    LiftAuto lift;
    DcMotor zahvat, vidvizh, lift1, lift2;

    public  static double xSplineOutOfWallCenter = -43,ySplineOutOfWallCenter =  -56,
            xSpikeCenter = -38, ySpikeCenter =  -23,
            xSteakRunning0Center = -55, ySteakRunning0Center = -25,
            xSteakRunning1Cebter = -60, ySteakRunning1Cebter = -11,
            xBackdropRunning1Center = 38 ,yBackdropRunning1Center = -10,
            xBackdropRunning2Center =  50, yBackdropRunning2Center = -33,
            xSteakRunning2Cebter=38, ySteakRunning2Cebter=-10,
            xSteakRunning3Cebter = -30,ySteakRunning3Cebter = -5,
            xBackdropRunning3Center = 38, yBackdropRunning3Center = -10,
            xBackdropRunning4Center = 50, yBackdropRunning4Center = -33,
            xParking = 45, yParking = -10;

    public  static double xSplineOutOfWallLeft = -43,ySplineOutOfWallLeft =  -56,
            xSpikeLeft = -30, ySpikeLeft =  -33,
            xSteakRunning0Left = -55, ySteakRunning0Left = -25,
            xSteakRunning1Left = -60, ySteakRunning1Left = -11,
            xBackdropRunning1Left = 38 ,yBackdropRunning1Left = -10,
            xBackdropRunning2Left =  50, yBackdropRunning2Left = -43,
            xSteakRunning2Left=38, ySteakRunning2Left=-10,
            xSteakRunning3Left = -30,ySteakRunning3Left = -5,
            xBackdropRunning3Left = 38, yBackdropRunning3Left = -10,
            xBackdropRunning4Left = 50, yBackdropRunning4Left = -33,
            xParkingLeft = 45, yParkingLeft = -10;
    public  static double xSplineOutOfWallRight = -43,ySplineOutOfWallRight =  -56,
            xSpikeRight = -55, ySpikeRight =  -33,
            xSteakRunning0Right = -55, ySteakRunning0Right = -25,
            xSteakRunning1Right = -60, ySteakRunning1Right = -11,
            xBackdropRunning1Right = 38 ,yBackdropRunning1Right = -10,
            xBackdropRunning2Right =  50, yBackdropRunning2Right = -28,
            xSteakRunning2Right=38, ySteakRunning2Right=-10,
            xSteakRunning3Right = -30,ySteakRunning3Right = -5,
            xBackdropRunning3Right = 38, yBackdropRunning3Right = -10,
            xBackdropRunning4Right = 50, yBackdropRunning4Right = -33,
            xParkingRight = 45, yParkingRight = -10;

    @Override
    public void runOpMode() throws InterruptedException {

        R = new Robot1(this);
        lift = new LiftAuto(this);
        scorer = new SpikeScorer(this);
        intake = new IntakeSecondVersion(this);
        hook = hardwareMap.get(Servo.class, "servoHook");
        zahvat = hardwareMap.get(DcMotor.class, "zahvat");
        zahvat.setDirection(DcMotorSimple.Direction.FORWARD);
        zahvat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        zahvat.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        vidvizh = hardwareMap.get(DcMotor.class, "vidvizhenie_zahvata");
        vidvizh.setDirection(DcMotorSimple.Direction.FORWARD);
        vidvizh.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vidvizh.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        vidvizh.setTargetPosition(0);
        lift1 = hardwareMap.get(DcMotor.class, "lift1");
        lift1.setDirection(DcMotorSimple.Direction.FORWARD);
        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //lift

        lift2 = hardwareMap.get(DcMotor.class, "lift2");
        lift2.setDirection(DcMotorSimple.Direction.REVERSE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);




        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-38, -61.5, 0);

        drive.setPoseEstimate(startPose);

        TrajectorySequence firstCenter = R.drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(xSplineOutOfWallCenter, ySplineOutOfWallCenter, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(xSpikeCenter, ySpikeCenter, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(0.3)
                .splineToLinearHeading(new Pose2d(xSteakRunning0Center, ySteakRunning0Center, Math.toRadians(0)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(xSteakRunning1Cebter, ySteakRunning1Cebter, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(xBackdropRunning1Center, yBackdropRunning1Center, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(xBackdropRunning2Center, yBackdropRunning2Center, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(0.5)
                .splineToLinearHeading(new Pose2d(xSteakRunning2Cebter, ySteakRunning2Cebter, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(xSteakRunning3Cebter, ySteakRunning3Cebter, Math.toRadians(-10)), Math.toRadians(180))
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(xBackdropRunning3Center, yBackdropRunning3Center, Math.toRadians(0)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(xBackdropRunning4Center, yBackdropRunning4Center, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(0.5)
                .splineToLinearHeading(new Pose2d(xParking, yParking, Math.toRadians(0)), Math.toRadians(0))
                .build();
        TrajectorySequence firstLeft = R.drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(xSplineOutOfWallLeft, ySplineOutOfWallLeft, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(xSpikeLeft, ySpikeLeft, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(0.3)
                .splineToLinearHeading(new Pose2d(xSteakRunning0Left, ySteakRunning0Left, Math.toRadians(0)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(xSteakRunning1Left, ySteakRunning1Left, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(xBackdropRunning1Left, yBackdropRunning1Left, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(xBackdropRunning2Left, yBackdropRunning2Left, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(0.5)
                .splineToLinearHeading(new Pose2d(xSteakRunning2Left, ySteakRunning2Left, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(xSteakRunning3Left, ySteakRunning3Left, Math.toRadians(10)), Math.toRadians(180))
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(xBackdropRunning3Left, yBackdropRunning3Left, Math.toRadians(0)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(xBackdropRunning4Left, yBackdropRunning4Left, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(0.5)
                .splineToLinearHeading(new Pose2d(xParkingLeft, yParkingLeft, Math.toRadians(0)), Math.toRadians(0))
                .build();
        TrajectorySequence firstRight = R.drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(xSplineOutOfWallRight, ySplineOutOfWallRight, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(xSpikeRight, ySpikeRight, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(0.3)
                .splineToLinearHeading(new Pose2d(xSteakRunning0Right, ySteakRunning0Right, Math.toRadians(0)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(xSteakRunning1Right, ySteakRunning1Right, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(xBackdropRunning1Right, yBackdropRunning1Right, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(xBackdropRunning2Right, yBackdropRunning2Right, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(0.5)
                .splineToLinearHeading(new Pose2d(xSteakRunning2Right, ySteakRunning2Right, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(xSteakRunning3Right, ySteakRunning3Right, Math.toRadians(10)), Math.toRadians(180))
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(xBackdropRunning3Right, yBackdropRunning3Right, Math.toRadians(0)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(xBackdropRunning4Right, yBackdropRunning4Right, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(0.5)
                .splineToLinearHeading(new Pose2d(xParkingRight, yParkingRight, Math.toRadians(0)), Math.toRadians(0))
                .build();
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());
        cam = OpenCvCameraFactory.getInstance()
                .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        cam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        PropDetection detector = new PropDetection(telemetry);
        cam.setPipeline(detector);
        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                cam.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        waitForStart();



        switch (detector.getLocation()) {
            case RIGHT:
                cam.stopStreaming();
                intake.Autonomous6();
                intake.Autonomous6();
                R.drive.followTrajectorySequenceAsync(firstLeft);
                break;

            case LEFT:
                cam.stopStreaming();
                intake.Autonomous6();
                intake.Autonomous6();
                R.drive.followTrajectorySequenceAsync(firstRight);
                break;
            case CENTER:
                cam.stopStreaming();
                intake.Autonomous6();
                R.drive.followTrajectorySequenceAsync(firstCenter);
                break;
            case NOT_FOUND:
                break;
        }
        while (opModeIsActive()){
            R.drive.update();
            lift.Auto();
        }

        //  cam.stopStreaming();


        while (!isStopRequested() && opModeIsActive());
    }

}