package org.firstinspires.ftc.teamcode.opModes.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Camera.PropDetection;
import org.firstinspires.ftc.teamcode.Camera.PropDetectionRed;
import org.firstinspires.ftc.teamcode.Modules.IntakeSecondVersion;
import org.firstinspires.ftc.teamcode.Modules.SpikeScorer;
import org.firstinspires.ftc.teamcode.Robot1;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Config
@Autonomous(group = "drive")
public class IntakeSecondPixelRed extends LinearOpMode {
    Robot1 R;
    SpikeScorer scorer;
    IntakeSecondVersion intake;
    DcMotor zahvat;
    OpenCvCamera cam;

    @Override
    public void runOpMode() throws InterruptedException {

        zahvat = hardwareMap.get(DcMotor.class, "zahvat");
        zahvat.setDirection(DcMotorSimple.Direction.FORWARD);
        zahvat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        zahvat.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        R = new Robot1(this);
        scorer = new SpikeScorer(this);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        intake = new IntakeSecondVersion(this);
        TrajectorySequence firstCenter = R.drive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(-8, 37.5, 0))
                //.splineToLinearHeading(new Pose2d(0,-2,Math.toRadians(0)),Math.toRadians(0))
                //  .strafeRight(20)
                .build();
        TrajectorySequence firstLeft = R.drive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(3.35, 29, 0))
                .build();
        TrajectorySequence firstRight = R.drive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(-18.25, 28, 0))
                .build();
        TrajectorySequence SteakRunning = R.drive.trajectorySequenceBuilder(firstCenter.end())
                .lineToLinearHeading(new Pose2d(-23, 39, 0))
                .build();
        TrajectorySequence SteakRunning2 = R.drive.trajectorySequenceBuilder(SteakRunning.end())
                .lineToLinearHeading(new Pose2d(-25, 39, 0))
                .build();
        TrajectorySequence UnderFermRunning = R.drive.trajectorySequenceBuilder(firstCenter.end())
                .lineToLinearHeading(new Pose2d(-24, 50, 0))
                .lineToLinearHeading(new Pose2d(53, 52, 0))
                .build();
        TrajectorySequence BackDropRunning = R.drive.trajectorySequenceBuilder(UnderFermRunning.end())
                .lineToLinearHeading(new Pose2d(92, 28.5, 0))
                .build();
        TrajectorySequence Parking = R.drive.trajectorySequenceBuilder(BackDropRunning.end())
                .lineToLinearHeading(new Pose2d(80, 45, 0))
                .build();
        TrajectorySequence UnderFermRunningLeft = R.drive.trajectorySequenceBuilder(SteakRunning2.end())
                .lineToLinearHeading(new Pose2d(-22, 28, 0))
                .addTemporalMarker(() -> {zahvat.setPower(-1);})
                .lineToLinearHeading(new Pose2d(-22, 52))
                .lineToLinearHeading(new Pose2d(48, 54, 0))
                .build();
        TrajectorySequence BackDropRunningLeft = R.drive.trajectorySequenceBuilder(UnderFermRunningLeft.end())
                .lineToLinearHeading(new Pose2d(92, 26.5, 0))
                .build();
        TrajectorySequence ParkingLeft = R.drive.trajectorySequenceBuilder(BackDropRunningLeft.end())
                .lineToLinearHeading(new Pose2d(80, 15, 0))
                .lineToLinearHeading(new Pose2d(80, 45, 0))
                .build();
        TrajectorySequence UnderFermRunningRight = R.drive.trajectorySequenceBuilder(firstLeft.end())
                .lineToLinearHeading(new Pose2d(-25, 48, 0))
                .lineToLinearHeading(new Pose2d(50, 56, 0))
                .build();
        TrajectorySequence BackDropRunningRight = R.drive.trajectorySequenceBuilder(UnderFermRunningRight.end())
                .lineToLinearHeading(new Pose2d(92, 22, 0))
                .build();
        TrajectorySequence ParkingRight = R.drive.trajectorySequenceBuilder(BackDropRunningRight.end())
                .lineToLinearHeading(new Pose2d(80, 15, 0))
                .lineToLinearHeading(new Pose2d(80, 45, 0))
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
                    scorer.scor();
                    R.drive.followTrajectorySequence(UnderFermRunningLeft);
                    sleep(5000);
                    R.drive.followTrajectorySequence(BackDropRunningLeft);
                    intake.Autonomous2();
                    sleep(1000);
                    intake.Autonomoys3();
                    sleep(1000);
                    intake.Autonomous4();
                    sleep(500);
                    intake.Autonomous5();;
                    sleep(500);
                    intake.Autonomous6();
                    R.drive.followTrajectorySequence(ParkingLeft);
                    break;
                case RIGHT:
                    R.drive.followTrajectorySequence(firstLeft);
                    scorer.scor();
                    R.drive.followTrajectorySequence(UnderFermRunningRight);
                    sleep(5000);
                    R.drive.followTrajectorySequence(BackDropRunningRight);
                    intake.Autonomous2();
                    sleep(1000);
                    intake.Autonomoys7();
                    sleep(1000);
                    intake.Autonomous4();
                    sleep(500);
                    intake.Autonomous5();;
                    sleep(500);
                    intake.Autonomous6();
                    R.drive.followTrajectorySequence(ParkingRight);
                    break;
                case CENTER:
                    R.drive.followTrajectorySequence(firstCenter);
                    scorer.scor();
                    R.drive.followTrajectorySequence(SteakRunning);
                    intake.IntakeFromSteak();
                    sleep(100);
                    R.drive.followTrajectorySequence(SteakRunning2);
                    intake.Steak();
                    /*R.drive.followTrajectorySequence(UnderFermRunning);
                    sleep(5000);
                    R.drive.followTrajectorySequence(BackDropRunning);
                    intake.Autonomous2();
                    sleep(1000);
                    intake.Autonomoys7();
                    sleep(1000);
                    intake.Autonomous4();
                    sleep(500);
                    intake.Autonomous5();;
                    sleep(500);
                    intake.Autonomous6();
                    R.drive.followTrajectorySequence(Parking);*/
                    break;
                case NOT_FOUND:
            }
        }
        cam.stopStreaming();


        while (!isStopRequested() && opModeIsActive());
    }
}