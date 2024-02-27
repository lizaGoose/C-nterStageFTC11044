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
public class IntakeRedRight extends LinearOpMode {
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
                .splineToLinearHeading(new Pose2d(-5, 5, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-10, 35, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(0.3)
                .addDisplacementMarker(() -> {
                    scorer.scor();

                    zahvat.setPower(1);
                })
                .splineToLinearHeading(new Pose2d(-23.5, 52, 0), Math.toRadians(90))
                .waitSeconds(0.7)
                .addDisplacementMarker(() -> {
                    intake.CloseScor();
                })
                .waitSeconds(0.2)
                .addDisplacementMarker(() -> {
                    zahvat.setPower(-1);
                })

                //.splineToConstantHeading(new Vector2d(48, -50), 0)
                .splineToLinearHeading(new Pose2d(87.9, 24.7, 0), Math.toRadians(-60))
                .waitSeconds(0.5)
                .addDisplacementMarker(() -> {
                    zahvat.setPower(0);
                    intake.Autonomous2();
                    sleep(100);
                    intake.liftF();
                    sleep(400);
                    intake.Autonomoys3();
                    sleep(400);
                    intake.OpenScor();
                    sleep(200);
                    intake.CloseScor();
                    sleep(400);
                    intake.MovSetCenter();
                    intake.perekidSETsenter();
                    intake.lift2();
                    sleep(600);
                    intake.Autonomous6();
                })
                .build();
        TrajectorySequence secondSenter = R.drive.trajectorySequenceBuilder(firstCenter.end())
                .splineToLinearHeading(new Pose2d(70, 53, 0), Math.toRadians(180))
                .addTemporalMarker(2.3, () -> {
                    hook.setPosition(0.15);
                    vidvizh.setTargetPosition(-2000);
                    vidvizh.setPower(1);
                    zahvat.setPower(1);
                    intake.AutoWallClose();
                })
                .splineToLinearHeading(new Pose2d(0, 53, 0), Math.toRadians(180))
                .waitSeconds(0.4)
                .addDisplacementMarker(() -> {

                    vidvizh.setTargetPosition(17);
                    vidvizh.setPower(-1);
                    zahvat.setPower(-0.2);
                })


                //splineToConstantHeading(new Vector2d(48, -50), 0)

                .splineToLinearHeading(new Pose2d(87.3, 28.5, 0), Math.toRadians(-60))
                .waitSeconds(0.3)
                .addDisplacementMarker(()->{
                    intake.Autonomous6();
                    intake.AutoWallOpen();
                    zahvat.setPower(1);
                    hook.setPosition(0.35);
                    vidvizh.setPower(0);
                    sleep(900);
                    intake.CloseScor();
                    zahvat.setPower(0);
                    sleep(200);
                    intake.Autonomous2();
                    sleep(100);
                    intake.lift();
                    sleep(400);
                    intake.Autonomoys7();
                    sleep(300);
                    intake.OpenScor();
                    sleep(200);
                    intake.CloseScor();
                    sleep(400);
                    intake.MovSetCenter();
                    sleep(100);
                    intake.Autonomous6();
                    intake.lift2();
                    sleep(600);

                })

                .build();


        TrajectorySequence firstLeft = R.drive.trajectorySequenceBuilder(new Pose2d())
                .splineToLinearHeading(new Pose2d(-5, 5, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-2, 29, Math.toRadians(0)), Math.toRadians(0))
                //.lineToLinearHeading(new Pose2d(3, -29, 0))

                .waitSeconds(0.4)
                .addDisplacementMarker(() -> {
                    scorer.scor();

                    zahvat.setPower(1);
                })
                .splineToLinearHeading(new Pose2d(-23, 51, 0), Math.toRadians(90))
                .addDisplacementMarker(() -> {
                    zahvat.setPower(1);
                })
                .waitSeconds(0.6)
                .addDisplacementMarker(() -> {
                    intake.CloseScor();
                })
                .addDisplacementMarker(() -> {
                    zahvat.setPower(-1);
                })



                //.splineToConstantHeading(new Vector2d(48, -50), 0)
                .splineToLinearHeading(new Pose2d(88, 24.5, 0), Math.toRadians(-55))
                .waitSeconds(0.5)
                .addDisplacementMarker(()-> {
//                    zahvat.setPower(0);
                    intake.Autonomous2();
                    sleep(100);
                    intake.liftF();
                    sleep(400);
                    intake.Autonomoys3();
                    sleep(400);
                    intake.OpenScor();
                    sleep(200);
                    intake.CloseScor();
                    sleep(400);
                    intake.MovSetCenter();
                    intake.perekidSETsenter();
                    intake.lift2();
                    sleep(600);
                    intake.Autonomous6();
                })
                .build();

        TrajectorySequence secondLeft = R.drive.trajectorySequenceBuilder(firstLeft.end())
                .splineToLinearHeading(new Pose2d(70, 52.5, 0), Math.toRadians(180))
                .addTemporalMarker(2.3, () -> {
                    hook.setPosition(0.15);
                    vidvizh.setTargetPosition(-2000);
                    vidvizh.setPower(1);
                    zahvat.setPower(1);
                    intake.AutoWallClose();
                })
                .splineToLinearHeading(new Pose2d(0, 52.5, 0), Math.toRadians(180))
                .waitSeconds(0.4)
                .addDisplacementMarker(() -> {

                    vidvizh.setTargetPosition(17);
                    vidvizh.setPower(-1);
                    zahvat.setPower(-0.2);
                })


                //splineToConstantHeading(new Vector2d(48, -50), 0)

                .splineToLinearHeading(new Pose2d(87.7, 28.5, 0), Math.toRadians(-60))
                .waitSeconds(0.3)
                .addDisplacementMarker(()->{
                    intake.Autonomous6();
                    intake.AutoWallOpen();
                    zahvat.setPower(1);
                    hook.setPosition(0.35);
                    vidvizh.setPower(0);
                    sleep(900);
                    intake.CloseScor();
                    zahvat.setPower(0);
                    sleep(200);
                    intake.Autonomous2();
                    sleep(100);
                    intake.lift();
                    sleep(400);
                    intake.Autonomoys7();
                    sleep(300);
                    intake.OpenScor();
                    sleep(200);
                    intake.CloseScor();
                    sleep(400);
                    intake.MovSetCenter();
                    sleep(100);
                    intake.Autonomous6();
                    intake.lift2();
                    sleep(600);

                })

                .build();
        TrajectorySequence firstRight = R.drive.trajectorySequenceBuilder(new Pose2d())
                .splineToLinearHeading(new Pose2d(-5, 5, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-20, 30, Math.toRadians(0)), Math.toRadians(0))
                //.lineToLinearHeading(new Pose2d(-18.5, -30, 0))
                .waitSeconds(0.3)
                .addDisplacementMarker(() -> {
                    scorer.scor();

                    zahvat.setPower(1);
                })
//                .splineToLinearHeading(new Pose2d(-22, -52, 0), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(-23, 51, 0))
                .addDisplacementMarker(() -> {
                    zahvat.setPower(1);
                })
                .waitSeconds(0.6)
                .addDisplacementMarker(() -> {
                    intake.CloseScor();
                })
                .addDisplacementMarker(() -> {
                    zahvat.setPower(-1);
                })



                //.splineToConstantHeading(new Vector2d(48, -50), 0)
                .splineToLinearHeading(new Pose2d(88, 34.8, 0), Math.toRadians(-55))
                .waitSeconds(0.5)
                .addDisplacementMarker(()-> {
                    zahvat.setPower(0);
                    intake.Autonomous2();
                    sleep(100);
                    intake.liftF();
                    sleep(400);
                    intake.Autonomoys7();
                    sleep(400);
                    intake.OpenScor();
                    sleep(200);
                    intake.CloseScor();
                    sleep(400);
                    intake.MovSetCenter();
                    intake.perekidSETsenter();
                    intake.lift2();
                    sleep(600);
                    intake.Autonomous6();
                })
                .build();

        TrajectorySequence secondRight = R.drive.trajectorySequenceBuilder(firstRight.end())
                .splineToLinearHeading(new Pose2d(70, 49.5, 0), Math.toRadians(180))
                .addTemporalMarker(2.3, () -> {
                    hook.setPosition(0.15);
                    vidvizh.setTargetPosition(-2000);
                    vidvizh.setPower(1);
                    zahvat.setPower(1);
                    intake.AutoWallClose();
                })
                .splineToLinearHeading(new Pose2d(0, 50, 0), Math.toRadians(180))
                .waitSeconds(0.4)
                .addDisplacementMarker(() -> {

                    vidvizh.setTargetPosition(17);
                    vidvizh.setPower(-1);
                    zahvat.setPower(-0.2);
                })


                //splineToConstantHeading(new Vector2d(48, -50), 0)

                .splineToLinearHeading(new Pose2d(88, 20.5, 0), Math.toRadians(-60))
                .waitSeconds(0.3)
                .addDisplacementMarker(()->{
                    intake.Autonomous6();
                    intake.AutoWallOpen();
                    zahvat.setPower(1);
                    hook.setPosition(0.35);
                    vidvizh.setPower(0);
                    sleep(900);
                    intake.CloseScor();
                    zahvat.setPower(0);
                    sleep(200);
                    intake.Autonomous2();
                    sleep(100);
                    intake.lift();
                    sleep(400);
                    intake.Autonomoys7();
                    sleep(300);
                    intake.OpenScor();
                    sleep(200);
                    intake.CloseScor();
                    sleep(400);
                    intake.MovSetCenter();
                    sleep(100);
                    intake.Autonomous6();
                    intake.lift2();
                    sleep(600);

                })

                .build();

        TrajectorySequence secondCenter2 = R.drive.trajectorySequenceBuilder(firstRight.end())
                .splineToLinearHeading(new Pose2d(82, 50, Math.toRadians(0)), Math.toRadians(0))
                .build();

        TrajectorySequence secondRight2 = R.drive.trajectorySequenceBuilder(firstRight.end())
                .splineToLinearHeading(new Pose2d(82, 50, Math.toRadians(0)), Math.toRadians(0))
                .build();

        TrajectorySequence secondLeft2 = R.drive.trajectorySequenceBuilder(firstLeft.end())
                .splineToLinearHeading(new Pose2d(82, 50, Math.toRadians(0)), Math.toRadians(0))
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

                    intake.Autonomous6();
                    R.drive.followTrajectorySequence(firstRight);
                    R.drive.followTrajectorySequence(secondRight2);
                    //scorer.scor();
                    /*R.drive.followTrajectorySequence(UnderFermRunningRight);
                    R.drive.followTrajectorySequence(BackDropRunningRight);*/
                    /*intake.Autonomous2();
                    sleep(1000);
                    intake.Autonomoys7();
                    sleep(1000);
                    intake.Autonomous4();
                    sleep(500);
                    intake.Autonomous5();;
                    sleep(500);
                    intake.Autonomous6();*/
                    // R.drive.followTrajectorySequence(ParkingRight);
                    break;

                case RIGHT:

                    intake.Autonomous6();
                    R.drive.followTrajectorySequence(firstLeft);
                    R.drive.followTrajectorySequence(secondLeft2);
                    //scorer.scor();
                    //R.drive.followTrajectorySequence(UnderFermRunningLeft);
                    /*R.drive.followTrajectorySequence(BackDropRunningLeft);*/
                    /*intake.Autonomous2();
                    sleep(1000);
                    intake.Autonomoys3();
                    sleep(1000);
                    intake.Autonomous4();
                    sleep(500);
                    intake.Autonomous5();;
                    sleep(500);
                    intake.Autonomous6();*/
                    // R.drive.followTrajectorySequence(ParkingLeft);

                    break;
                case CENTER:
                    intake.Autonomous6();

//                    intake.Autonomous6();
//                    intake.AutoWallOpen();
//                    zahvat.setPower(1);
//                    hook.setPosition(0.35);
//                    sleep(700);
//                    intake.CloseScor();
//                    zahvat.setPower(0);
//                    sleep(200);
//                    intake.Autonomous2();
//                    sleep(100);
//                    intake.lift();
//                    sleep(400);
//                    intake.Autonomoys3();
//                    sleep(300);
//                    intake.OpenScor();
//                    sleep(600);
//                    intake.CloseScor();
//                    intake.MovSetCenter();
//                    sleep(100);
//                    intake.Autonomous6();
//                    intake.lift2();
//                    sleep(600);

                    R.drive.followTrajectorySequence(firstCenter);
                    R.drive.followTrajectorySequence(secondCenter2);
                    //scorer.scor();
                    //R.drive.followTrajectorySequence(UnderFermRunning);
                    //  R.drive.followTrajectorySequence(BackDropRunning);
                    /*intake.Autonomous2();
                    sleep(1000);
                    intake.Autonomoys3();
                    sleep(1000);
                    intake.Autonomous4();
                    sleep(500);
                    intake.Autonomous5();;
                    sleep(500);
                    intake.Autonomous6();*/
                    //  R.drive.followTrajectorySequence(Parking);
                    break;
                case NOT_FOUND:
                    break;
            }


        }
        cam.stopStreaming();


        while (!isStopRequested() && opModeIsActive());
    }
}