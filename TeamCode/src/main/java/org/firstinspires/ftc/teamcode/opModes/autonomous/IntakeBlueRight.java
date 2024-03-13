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
public class IntakeBlueRight extends LinearOpMode {
    Robot1 R;
    IntakeSecondVersion intake;
    SpikeScorer scorer;
    Servo hook;
    OpenCvCamera cam;

    LiftAuto lift;
    DcMotor zahvat, vidvizh, lift1, lift2;

    public  static double x1 = -5, y1 = -5, x2 = -10, y2 = -35, x3 = -23, y3 = -29.6, x4= -23.1, y4 = -50.5,
            x5 = 60, y5 = -52, x6 = 75, y6 = -36, x7 = 89.7, y7 = -24.5, x8 = 77, y8 = -51, x9 = 70, y9 = -49,
            x10 = -23.5, y10 = -49, x11 = 60, y11 = -53, x12 = 75, y12 = -36, x13 = 89.7, y13 = -28.7, x14 = 89.7, y14 = -28.77,
            x15 = 70, y15 = -54, xx = -13.5, yy = -50;

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

        TrajectorySequence firstCenter = R.drive.trajectorySequenceBuilder(new Pose2d())
                .splineToLinearHeading(new Pose2d(x1, y1, Math.toRadians(0)), Math.toRadians(0))

                .splineToLinearHeading(new Pose2d(x2-3, y2-5, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(0.3)

                .addDisplacementMarker(() -> {
                    scorer.scor();

                    //zahvat.setPower(1);
                })
                .splineToLinearHeading(new Pose2d(x3, y3, 0), Math.toRadians(-90))
                .addTemporalMarker(4.3, () -> {
                    zahvat.setPower(1);
                })
                .lineToLinearHeading(new Pose2d(x4, y4, 0))
                .waitSeconds(0.9)
                .addDisplacementMarker(() -> {
                    intake.CloseScor();
                })
                .addDisplacementMarker(() -> {
                    zahvat.setPower(-1);
                })

                .splineToConstantHeading(new Vector2d(x5, y5), 0)
                .addDisplacementMarker(()-> {
                    zahvat.setPower(0);
                    lift.GetPose(400);
                    intake.Autonomous2();

                })

                .splineToLinearHeading(new Pose2d(x6, y6, 0), Math.toRadians(0))
                .addDisplacementMarker(()-> {
                    intake.Autonomoys7();
                })

                .waitSeconds(0.64)
                .splineToLinearHeading(new Pose2d(x7+1.5, y7-2, 0), Math.toRadians(0))
                .waitSeconds(0.74)
                .addDisplacementMarker(()->{
                    intake.OpenScor();
//                    sleep(500);
                })

                .lineToLinearHeading(new Pose2d(x7+1.51, y7-2, 0))
                .waitSeconds(0.4)
                .splineToLinearHeading(new Pose2d(x8, y8, 0), Math.toRadians(-90))
                .addDisplacementMarker(() -> {

                    intake.CloseScor();
                    intake.MovSetCenter();

                })
                .splineToLinearHeading(new Pose2d(82, -50, Math.toRadians(0)), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    lift.GetPose(0);
                    intake.OpenScor();
                    intake.perekidSETsenter();
                })

                .build();
        TrajectorySequence firstright = R.drive.trajectorySequenceBuilder(new Pose2d())
                .splineToLinearHeading(new Pose2d(-5, -5, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-16.3, -30, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(0.3)
                .addDisplacementMarker(() -> {
                    scorer.scor();

                    //zahvat.setPower(1);
                })
                .splineToLinearHeading(new Pose2d(x3+2, y3, 0), Math.toRadians(-90))
                .addTemporalMarker(4.3, () -> {
                    zahvat.setPower(1);
                })
                .lineToLinearHeading(new Pose2d(x4+1.8, y4, 0))
                .waitSeconds(0.9)
                .addDisplacementMarker(() -> {
                    intake.CloseScor();
                })
                .addDisplacementMarker(() -> {
                    zahvat.setPower(-1);
                })

                .splineToConstantHeading(new Vector2d(x5, y5), 0)
                .addDisplacementMarker(()-> {
                    zahvat.setPower(0);
                    lift.GetPose(400);
                    intake.Autonomous2();

                })
                .splineToLinearHeading(new Pose2d(x6, y6, 0), Math.toRadians(0))
                .addDisplacementMarker(()-> {
                    intake.Autonomoys7();
                })
                .waitSeconds(0.64)

                .splineToLinearHeading(new Pose2d(86.5, -32, 0), Math.toRadians(0))
                .waitSeconds(0.74)
                .addDisplacementMarker(()->{
                    intake.OpenScor();
//                    sleep(500);
                })

                .lineToLinearHeading(new Pose2d(86.5 - 0.01, -32, 0))
                .waitSeconds(0.6)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(x8, y8, 0), Math.toRadians(-90))
                .addDisplacementMarker(() -> {

                    intake.CloseScor();
                    intake.MovSetCenter();

                })
                .splineToLinearHeading(new Pose2d(82, -50, Math.toRadians(0)), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    lift.GetPose(0);
                    intake.OpenScor();
                    intake.perekidSETsenter();
               })
               .build();
        TrajectorySequence firstleft = R.drive.trajectorySequenceBuilder(new Pose2d())
                .splineToLinearHeading(new Pose2d(-5, -5, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(1, -30, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(0.3)
                .addDisplacementMarker(() -> {
                    scorer.scor();

                    //zahvat.setPower(1);
                })
                .splineToLinearHeading(new Pose2d(x3 + 1, y3, 0), Math.toRadians(-90))
                .addTemporalMarker(4.2, () -> {
                    zahvat.setPower(1);
                })
                .lineToLinearHeading(new Pose2d(x4 + 1.8, y4-1.5, 0))
                .waitSeconds(0.9)
                .addDisplacementMarker(() -> {
                    intake.CloseScor();
                })
                .addDisplacementMarker(() -> {
                    zahvat.setPower(-1);
                })

                .splineToConstantHeading(new Vector2d(x5, y5), 0)
                .addDisplacementMarker(()-> {
                    zahvat.setPower(0);
                    lift.GetPose(400);
                    intake.Autonomous2();

                })
                .splineToLinearHeading(new Pose2d(x6+2, y6, 0), Math.toRadians(0))
                .addDisplacementMarker(()-> {
                    intake.Autonomoys7();
                })
                .waitSeconds(0.64)
                .splineToLinearHeading(new Pose2d(87, -26, 0), Math.toRadians(0))
                .waitSeconds(0.64)
                .addDisplacementMarker(()->{
                    intake.OpenScor();
//                    sleep(500);
                })
                .lineToLinearHeading(new Pose2d(87-0.01, -26, 0))
                .splineToLinearHeading(new Pose2d(x8, y8, 0), Math.toRadians(-90))
                .addDisplacementMarker(() -> {

                    intake.CloseScor();
                    intake.MovSetCenter();

                })
                .splineToLinearHeading(new Pose2d(82, -50, Math.toRadians(0)), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    lift.GetPose(0);
                    intake.OpenScor();
                    intake.perekidSETsenter();
                })

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
                R.drive.followTrajectorySequenceAsync(firstleft);
                break;

            case LEFT:
                cam.stopStreaming();
                intake.Autonomous6();
                intake.Autonomous6();
                R.drive.followTrajectorySequenceAsync(firstright);
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