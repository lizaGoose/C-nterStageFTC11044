package org.firstinspires.ftc.teamcode.opModes.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Camera.PropDetection;
import org.firstinspires.ftc.teamcode.Camera.PropDetectionRed;
import org.firstinspires.ftc.teamcode.Modules.Goose;
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
public class SpikeRedLeft extends LinearOpMode {
    Robot1 R;
    Goose scorer;
    OpenCvCamera cam;

    @Override
    public void runOpMode() throws InterruptedException {

        R = new Robot1(this);
        scorer = new Goose(this);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        TrajectorySequence firstCenter = R.drive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(6, 31, 0))
                //.splineToLinearHeading(new Pose2d(0,-2,Math.toRadians(0)),Math.toRadians(0))
                //  .strafeRight(20)
                .build();
        TrajectorySequence firstLeft = R.drive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(18, 26, 0))
                .build();
        TrajectorySequence firstRight = R.drive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(-5, 26, 0))
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
                    //scorer.scor();
                    break;
                case RIGHT:
                    R.drive.followTrajectorySequence(firstLeft);
                    //scorer.scor();

                    break;
                case CENTER:
                    R.drive.followTrajectorySequence(firstCenter);
                   // scorer.scor();
                    break;
                case NOT_FOUND:
            }
        }
        cam.stopStreaming();


        while (!isStopRequested() && opModeIsActive()) ;
    }
}