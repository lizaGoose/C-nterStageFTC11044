package org.firstinspires.ftc.teamcode.opModes.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
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
public class IntakeRedRight extends LinearOpMode {
    Robot1 R;
    IntakeSecondVersion intake;
    SpikeScorer scorer;
    Servo hook;
    OpenCvCamera cam;

    LiftAuto lift;
    DcMotor zahvat, vidvizh, lift1, lift2;

    public  static double x1 = -5, y1 = 5, x2 = -10, y2 = 35, x3 = -23, y3 = 29.6, x4= -23.1, y4 = 50.5,
            x5 = 60, y5 = 52, x6 = 75, y6 = 36, x7 = 89.7, y7 = 27.5, x8 = 77, y8 = 51, x9 = 70, y9 = 49,
            x10 = -23.5, y10 = 49, x11 = 60, y11 = 53, x12 = 75, y12 = 36, x13 = 89.7, y13 = 28.7, x14 = 89.7, y14 = 28.77,
            x15 = 70, y15 = 54, xx = -13.5, yy = 50;

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
                .splineToLinearHeading(new Pose2d(x2, y2, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(0.3)
                .addDisplacementMarker(() -> {
                    scorer.scor();

                    //zahvat.setPower(1);
                })
                .splineToLinearHeading(new Pose2d(x3, y3, 0), Math.toRadians(90))
                .addTemporalMarker(4.7, () -> {
                    zahvat.setPower(1);
                })
                .lineToLinearHeading(new Pose2d(x4, y4, 0))
                .waitSeconds(0.4)
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

                .splineToLinearHeading(new Pose2d(x7, y7-3, 0), Math.toRadians(0))
                .waitSeconds(0.24)
                .addDisplacementMarker(()->{
                    intake.OpenScor();
//                    sleep(500);
                })

                .lineToLinearHeading(new Pose2d(x7-0.01, y7-3, 0))
                .waitSeconds(0.3)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(x8, y8+2, 0), Math.toRadians(90))
                .setReversed(false)
                .addDisplacementMarker(() -> {
                    lift.GetPose(0);
                    intake.CloseScor();
                    intake.MovSetCenter();

                })
                .splineToLinearHeading(new Pose2d(x9, y9+2, 0), Math.toRadians(180))
                .addDisplacementMarker(() -> {
                    zahvat.setPower(1);
                    intake.perekidSETsenter();
                    intake.OpenScor();

                })
//                .setReversed(true)
                .splineToLinearHeading(new Pose2d(x10 + 2.2, y10+2, 0), Math.toRadians(180))
//                .splineToLinearHeading(new Pose2d(x10 + 2.5, y10, 0), Math.toRadians(-110))
                .lineToLinearHeading(new Pose2d(x10 + 1.5, y10 - 20, 0))
                .splineToLinearHeading(new Pose2d(xx, yy, 0), Math.toRadians(0))
//                .setReversed(false)
                .waitSeconds(0.1)
                .addDisplacementMarker(() -> {
                    intake.CloseScor();
                })
                .addDisplacementMarker(() -> {
                    zahvat.setPower(-1);
                })
                .splineToConstantHeading(new Vector2d(x11, y11), 0)
                .addDisplacementMarker(()-> {
                    zahvat.setPower(0);
                    lift.GetPose(500);
                    intake.Autonomous2();

                })
                .splineToLinearHeading(new Pose2d(x12, y12, 0), Math.toRadians(0))
                .addDisplacementMarker(()-> {
                    intake.Autonomoys3();
                })
                .splineToLinearHeading(new Pose2d(x13, y13, 0), Math.toRadians(0))
                .waitSeconds(0.24)
                .addDisplacementMarker(()->{
                    intake.OpenScor();
                })
                .splineToLinearHeading(new Pose2d(x7-0.01, y7, 0), Math.toRadians(0))
                .waitSeconds(0.3)
                .splineToLinearHeading(new Pose2d(x8, y8, 0), Math.toRadians(90))
                .addDisplacementMarker(() -> {

                    intake.CloseScor();
                    intake.MovSetCenter();

                })
                .splineToLinearHeading(new Pose2d(82, 50, Math.toRadians(0)), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    lift.GetPose(0);
                    intake.OpenScor();
                    intake.perekidSETsenter();
//                    zahvat.setPower(1);
                })
//                .splineToLinearHeading(new Pose2d(-23.5, 55, 0), Math.toRadians(180))
//                .waitSeconds(0.5)
//                .addDisplacementMarker(() -> {
//                    intake.CloseScor();
//                })
//                .addDisplacementMarker(() -> {
//                    zahvat.setPower(-1);
//                })
//                .splineToConstantHeading(new Vector2d(60, 52), 0)
//                .addDisplacementMarker(()-> {
//                    lift.GetPose(300);
//                    intake.Autonomous2();
//                    intake.Autonomoys7();
//                })
//                .splineToLinearHeading(new Pose2d(87.9, 25.7, 0), Math.toRadians(0))
//                .addDisplacementMarker(()->{
//                    intake.OpenScor();
//                })
//                .waitSeconds(0.2)
//                .splineToLinearHeading(new Pose2d(82, 50, 0), Math.toRadians(180))
//                .addDisplacementMarker(() -> {
//                    lift.GetPose(0);
//                    intake.CloseScor();
//                    intake.MovSetCenter();
//                    intake.perekidSETsenter();
//                    zahvat.setPower(0);
//                })

                .build();
        TrajectorySequence firstright = R.drive.trajectorySequenceBuilder(new Pose2d())
                .splineToLinearHeading(new Pose2d(-5, 5, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-17.1, 30, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(0.3)
                .addDisplacementMarker(() -> {
                    scorer.scor();

                    //zahvat.setPower(1);
                })
                .splineToLinearHeading(new Pose2d(x3+2, y3, 0), Math.toRadians(90))
                .addTemporalMarker(4.7, () -> {
                    zahvat.setPower(1);
                })
                //.setVelConstraint(SampleMecanumDrive.getVelocityConstraint(48.0, 3.5, 6.93))
                .lineToLinearHeading(new Pose2d(x4+1.8, y4, 0))
                .waitSeconds(0.4)
                .addDisplacementMarker(() -> {
                    intake.CloseScor();
                })
                .addDisplacementMarker(() -> {
                    zahvat.setPower(-1);
                })
                //.setVelConstraint(SampleMecanumDrive.getVelocityConstraint(32.0, 3.5, 6.93))

                .splineToConstantHeading(new Vector2d(x5, y5), 0)
                .addDisplacementMarker(()-> {
                    zahvat.setPower(0);
                    lift.GetPose(400);
                    intake.Autonomous2();

                })
                .splineToLinearHeading(new Pose2d(x6, y6, 0), Math.toRadians(0))
                .addDisplacementMarker(()-> {
                    intake.Autonomoys3();
                })

                .splineToLinearHeading(new Pose2d(90, 36, 0), Math.toRadians(0))
                .waitSeconds(0.24)
                .addDisplacementMarker(()->{
                    intake.OpenScor();
//                    sleep(500);
                })

                .lineToLinearHeading(new Pose2d(90 - 0.01, 36, 0))
                .waitSeconds(0.3)
                .setReversed(true)
                //.setVelConstraint(SampleMecanumDrive.getVelocityConstraint(48.0, 3.5, 6.93))

                .splineToLinearHeading(new Pose2d(x8, y8+2, 0), Math.toRadians(90))
                .setReversed(false)
                .addDisplacementMarker(() -> {
                    lift.GetPose(0);
                    intake.CloseScor();
                    intake.MovSetCenter();

                })

                .splineToLinearHeading(new Pose2d(x9, y9+2, 0), Math.toRadians(180))
                .addDisplacementMarker(() -> {
                    zahvat.setPower(1);
                    intake.perekidSETsenter();
                    intake.OpenScor();

                })
//                .setReversed(true)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(32.0, 3.5, 6.93))

                .splineToLinearHeading(new Pose2d(x10 + 2.2, y10+2, 0), Math.toRadians(180))
//                .splineToLinearHeading(new Pose2d(x10 + 2.5, y10, 0), Math.toRadians(-110))
                .lineToLinearHeading(new Pose2d(x10 + 1.5, y10 - 13, 0))
                //.setVelConstraint(SampleMecanumDrive.getVelocityConstraint(48.0, 3.5, 6.93))
                .splineToLinearHeading(new Pose2d(xx, yy, 0), Math.toRadians(0))
                //.setVelConstraint(SampleMecanumDrive.getVelocityConstraint(32.0, 3.5, 6.93))
                .waitSeconds(0.1)
                .addDisplacementMarker(() -> {
                    intake.CloseScor();
                })
                .addDisplacementMarker(() -> {
                    zahvat.setPower(-1);
                })
                .splineToConstantHeading(new Vector2d(x11, y11), 0)
                .addDisplacementMarker(()-> {
                    zahvat.setPower(0);
                    lift.GetPose(500);
                    intake.Autonomous2();

                })
                .splineToLinearHeading(new Pose2d(x12, y12, 0), Math.toRadians(0))
                .addDisplacementMarker(()-> {
                    intake.Autonomoys3();
                })
                .splineToLinearHeading(new Pose2d(x13, y13, 0), Math.toRadians(0))
                .waitSeconds(0.24)
                .addDisplacementMarker(()->{
                    lift.GetPose(570);
                    intake.OpenScor();
                })
                .splineToLinearHeading(new Pose2d(x7-0.01, y7, 0), Math.toRadians(0))
                .waitSeconds(0.3)
                .splineToLinearHeading(new Pose2d(x8, y8, 0), Math.toRadians(90))
                .addDisplacementMarker(() -> {

                    intake.CloseScor();
                    intake.MovSetCenter();

                })
                .splineToLinearHeading(new Pose2d(82, 50, Math.toRadians(0)), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    lift.GetPose(0);
                    intake.OpenScor();
                    intake.perekidSETsenter();
//                    zahvat.setPower(1);
                })
//                .splineToLinearHeading(new Pose2d(-23.5, 55, 0), Math.toRadians(180))
//                .waitSeconds(0.5)
//                .addDisplacementMarker(() -> {
//                    intake.CloseScor();
//                })
//                .addDisplacementMarker(() -> {
//                    zahvat.setPower(-1);
//                })
//                .splineToConstantHeading(new Vector2d(60, 52), 0)
//                .addDisplacementMarker(()-> {
//                    lift.GetPose(300);
//                    intake.Autonomous2();
//                    intake.Autonomoys7();
//                })
//                .splineToLinearHeading(new Pose2d(87.9, 25.7, 0), Math.toRadians(0))
//                .addDisplacementMarker(()->{
//                    intake.OpenScor();
//                })
//                .waitSeconds(0.2)
//                .splineToLinearHeading(new Pose2d(82, 50, 0), Math.toRadians(180))
//                .addDisplacementMarker(() -> {
//                    lift.GetPose(0);
//                    intake.CloseScor();
//                    intake.MovSetCenter();
//                    intake.perekidSETsenter();
//                    zahvat.setPower(0);
//                })

                .build();
        TrajectorySequence firstleft = R.drive.trajectorySequenceBuilder(new Pose2d())
                .splineToLinearHeading(new Pose2d(-5, 5, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(1.6, 25, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(0.3)
                .addDisplacementMarker(() -> {
                    scorer.scor();

                    //zahvat.setPower(1);
                })
                .splineToLinearHeading(new Pose2d(x3 + 1.5, y3, 0), Math.toRadians(90))
                .addTemporalMarker(4.7, () -> {
                    zahvat.setPower(1);
                })
                .lineToLinearHeading(new Pose2d(x4 + 1.5, y4, 0))
                .waitSeconds(0.4)
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

                .splineToLinearHeading(new Pose2d(88, 17, 0), Math.toRadians(0))
                .waitSeconds(0.24)
                .addDisplacementMarker(()->{
                    intake.OpenScor();
//                    sleep(500);
                })
                .lineToLinearHeading(new Pose2d(88-0.01, 17, 0))
                .waitSeconds(0.3)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(x8, y8+2, 0), Math.toRadians(90))
                .setReversed(false)
                .addDisplacementMarker(() -> {
                    lift.GetPose(0);
                    intake.CloseScor();
                    intake.MovSetCenter();

                })
                .splineToLinearHeading(new Pose2d(x9, y9+2, 0), Math.toRadians(180))
                .addDisplacementMarker(() -> {
                    zahvat.setPower(1);
                    intake.perekidSETsenter();
                    intake.OpenScor();

                })

//                .setReversed(true)
                .splineToLinearHeading(new Pose2d(x10 + 2.2, y10+2, 0), Math.toRadians(180))
//                .splineToLinearHeading(new Pose2d(x10 + 2.5, y10, 0), Math.toRadians(-110))
                .lineToLinearHeading(new Pose2d(x10 + 1.5, y10 - 20, 0))
                .splineToLinearHeading(new Pose2d(xx, yy, 0), Math.toRadians(0))
//                .setReversed(false)
                .waitSeconds(0.1)
                .addDisplacementMarker(() -> {
                    intake.CloseScor();
                })
                .addDisplacementMarker(() -> {
                    zahvat.setPower(-1);
                })
                .splineToConstantHeading(new Vector2d(x11, y11), 0)
                .addDisplacementMarker(()-> {
                    zahvat.setPower(0);
                    lift.GetPose(500);
                    intake.Autonomous2();

                })
                .splineToLinearHeading(new Pose2d(x12, y12, 0), Math.toRadians(0))
                .addDisplacementMarker(()-> {
                    intake.Autonomoys3();
                })
                .splineToLinearHeading(new Pose2d(x13, y13, 0), Math.toRadians(0))
                .waitSeconds(0.24)
                .addDisplacementMarker(()->{
                    intake.OpenScor();
                })
                .splineToLinearHeading(new Pose2d(x7-0.01, y7, 0), Math.toRadians(0))
                .waitSeconds(0.3)
                .splineToLinearHeading(new Pose2d(x8, y8, 0), Math.toRadians(90))
                .addDisplacementMarker(() -> {

                    intake.CloseScor();
                    intake.MovSetCenter();

                })
                .splineToLinearHeading(new Pose2d(82, 50, Math.toRadians(0)), Math.toRadians(0))
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
        PropDetectionRed detector = new PropDetectionRed(telemetry, this);
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
                case LEFT:
                    cam.stopStreaming();
                    intake.Autonomous6();
                    intake.Autonomous6();
                    R.drive.followTrajectorySequenceAsync(firstright);
                    break;

                case RIGHT:
                    cam.stopStreaming();
                    intake.Autonomous6();
                    intake.Autonomous6();
                    R.drive.followTrajectorySequenceAsync(firstleft);
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