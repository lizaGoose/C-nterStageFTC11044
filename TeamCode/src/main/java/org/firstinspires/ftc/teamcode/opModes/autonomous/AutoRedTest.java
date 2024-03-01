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
public class AutoRedTest extends LinearOpMode {
    Robot1 R;
    IntakeSecondVersion intake;
    SpikeScorer scorer;
    Servo hook;
    OpenCvCamera cam;
    DcMotor zahvat, vidvizh, lift1, lift2;

    @Override
    public void runOpMode() throws InterruptedException {

        R = new Robot1(this);
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

        TrajectorySequence firstCenter = R.drive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(-44, -27, Math.toRadians(0))) // senter
                .waitSeconds(0.3)
                .lineToLinearHeading(new Pose2d(-60, -12, 0))
                .waitSeconds(0.9)

                .waitSeconds(0.2)


                .lineToLinearHeading(new Pose2d(49, -34, 0))
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(36, -12, 0))
                .lineToLinearHeading(new Pose2d(-50, -12, 0))
                .splineToLinearHeading(new Pose2d(-60, -12, 0), Math.toRadians(-90))
                .waitSeconds(0.6)
                .lineToLinearHeading(new Pose2d(36, -12, 0))
                .lineToLinearHeading(new Pose2d(49, -34, 0))
                .waitSeconds(0.5)

                .lineToLinearHeading(new Pose2d(42, -14, 0))

                .build();


        TrajectorySequence firstLeft = R.drive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(-33, -32, 0)) // left
                .waitSeconds(0.3)
                .lineToLinearHeading(new Pose2d(-60, -12, 0))
                .waitSeconds(0.9)
                .lineToLinearHeading(new Pose2d(36, -12, 0))
                .splineToLinearHeading(new Pose2d(49, -41, 0), Math.toRadians(0)) // left
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(36, -12, 0))
                .lineToLinearHeading(new Pose2d(-50, -12, 0))
                .splineToLinearHeading(new Pose2d(-60, -12, 0), Math.toRadians(-90))
                .waitSeconds(0.6)
                .lineToLinearHeading(new Pose2d(36, -12, 0))
                .lineToLinearHeading(new Pose2d(49, -34, 0))
                .waitSeconds(0.5)

                .lineToLinearHeading(new Pose2d(42, -14, 0))
                .build();

        TrajectorySequence firstRight = R.drive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(-56.4, -32, 0)) // right
                .waitSeconds(0.3)
                .lineToLinearHeading(new Pose2d(-60, -12, 0))
                .waitSeconds(0.9)

                .waitSeconds(0.2)


                .lineToLinearHeading(new Pose2d(36, -12, 0))
                .splineToLinearHeading(new Pose2d(49, -30, 0), Math.toRadians(0)) // right
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(36, -12, 0))
                .lineToLinearHeading(new Pose2d(-50, -12, 0))
                .splineToLinearHeading(new Pose2d(-60, -12, 0), Math.toRadians(-90))
                .waitSeconds(0.6)
                .lineToLinearHeading(new Pose2d(36, -12, 0))
                .lineToLinearHeading(new Pose2d(49, -34, 0))
                .waitSeconds(0.5)

                .lineToLinearHeading(new Pose2d(42, -14, 0))

                .build();


        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());
        cam = OpenCvCameraFactory.getInstance()
                .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        cam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        PropDetectionRed detector = new PropDetectionRed(telemetry);
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

        if (isStarted()) {

            switch (detector.getLocation()) {
                case LEFT:

                    R.drive.followTrajectorySequence(firstRight);
                    break;

                case RIGHT:

                    R.drive.followTrajectorySequence(firstLeft);
                    break;
                case CENTER:
                    R.drive.followTrajectorySequence(firstCenter);
                    break;
                case NOT_FOUND:
                    break;
            }


        }
        cam.stopStreaming();


        while (!isStopRequested() && opModeIsActive());
    }
}
