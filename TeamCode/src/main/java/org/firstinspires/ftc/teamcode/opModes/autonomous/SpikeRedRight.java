package org.firstinspires.ftc.teamcode.opModes.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
public class SpikeRedRight extends LinearOpMode {
    Robot1 R;
    SpikeScorer scorer;
    IntakeSecondVersion intake;
    OpenCvCamera cam;

    @Override
    public void runOpMode() throws InterruptedException {

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
                .lineToLinearHeading(new Pose2d(-21, 34, 0))
                .build();
        TrajectorySequence UnderFermRunning = R.drive.trajectorySequenceBuilder(firstCenter.end())
                .lineToLinearHeading(new Pose2d(-24, 50, 0))
                .lineToLinearHeading(new Pose2d(53, 48, 0))
                .build();
        TrajectorySequence BackDropRunning = R.drive.trajectorySequenceBuilder(UnderFermRunning.end())
                .lineToLinearHeading(new Pose2d(92, 28.5, 0))
                .build();
        TrajectorySequence Parking = R.drive.trajectorySequenceBuilder(BackDropRunning.end())
                .lineToLinearHeading(new Pose2d(80, 45, 0))
                .build();
        TrajectorySequence UnderFermRunningLeft = R.drive.trajectorySequenceBuilder(firstRight.end())
                .lineToLinearHeading(new Pose2d(-22, 28, 0))
                .lineToLinearHeading(new Pose2d(-22, 52))
                .lineToLinearHeading(new Pose2d(48, 50, 0))
                .build();
        TrajectorySequence BackDropRunningLeft = R.drive.trajectorySequenceBuilder(UnderFermRunningLeft.end())
                .lineToLinearHeading(new Pose2d(91, 24.5, 0))
                .build();
        TrajectorySequence ParkingLeft = R.drive.trajectorySequenceBuilder(BackDropRunningLeft.end())
                .lineToLinearHeading(new Pose2d(80, 15, 0))
                .lineToLinearHeading(new Pose2d(80, 45, 0))
                .build();
        TrajectorySequence UnderFermRunningRight = R.drive.trajectorySequenceBuilder(firstLeft.end())
                .lineToLinearHeading(new Pose2d(-25, 48, 0))
                .lineToLinearHeading(new Pose2d(50, 48, 0))
                .build();
        TrajectorySequence BackDropRunningRight = R.drive.trajectorySequenceBuilder(UnderFermRunningRight.end())
                .lineToLinearHeading(new Pose2d(92, 21.5, 0))
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
                    sleep(8500);
                    R.drive.followTrajectorySequence(BackDropRunningLeft);
                    intake.lift();
                    intake.Autonomous2();
                    sleep(2500);
                    intake.Autonomoys3();
                    sleep(1000);
                    intake.Autonomous4();
                    sleep(1000);
                    intake.Autonomous5();;
                    sleep(1000);
                    intake.Autonomous6();
                    R.drive.followTrajectorySequence(ParkingLeft);
                    intake.lift2();
                    break;
                case RIGHT:
                    R.drive.followTrajectorySequence(firstLeft);
                    scorer.scor();
                    R.drive.followTrajectorySequence(UnderFermRunningRight);
                    sleep(10000);
                    R.drive.followTrajectorySequence(BackDropRunningRight);
                    intake.lift();
                    intake.Autonomous2();
                    sleep(2500);
                    intake.Autonomoys7();
                    sleep(1000);
                    intake.Autonomous4();
                    sleep(1000);
                    intake.Autonomous5();;
                    sleep(1000);
                    intake.Autonomous6();
                    R.drive.followTrajectorySequence(ParkingRight);
                    break;
                case CENTER:
                    R.drive.followTrajectorySequence(firstCenter);
                    scorer.scor();
                    R.drive.followTrajectorySequence(UnderFermRunning);
                    sleep(7500);
                    R.drive.followTrajectorySequence(BackDropRunning);
                    intake.Autonomous2();
                    sleep(2500);
                    intake.Autonomoys7();
                    sleep(1000);
                    intake.Autonomous4();
                    sleep(1000);
                    intake.Autonomous5();
                    sleep(1000);
                    intake.Autonomous6();
                    R.drive.followTrajectorySequence(Parking);
                    break;
                case NOT_FOUND:
            }
        }
        cam.stopStreaming();


        while (!isStopRequested() && opModeIsActive());
    }
}