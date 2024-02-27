package org.firstinspires.ftc.teamcode.opModes.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Camera.PropDetection;
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
public class SpikeBlueRight extends LinearOpMode {
    Robot1 R;
    IntakeSecondVersion intake;
    SpikeScorer scorer;
    OpenCvCamera cam;

    @Override
    public void runOpMode() throws InterruptedException {

        R = new Robot1(this);
        scorer = new SpikeScorer(this);
        intake = new IntakeSecondVersion(this);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        TrajectorySequence firstCenter = R.drive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(-5, -38.8, 0))
                //.splineToLinearHeading(new Pose2d(0,-2,Math.toRadians(0)),Math.toRadians(0))
              //  .strafeRight(20)
                .build();
        TrajectorySequence firstLeft = R.drive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(3, -29, 0))
                .build();
        TrajectorySequence firstRight = R.drive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(-18.5, -30, 0))
                .build();
        TrajectorySequence SteakRunning = R.drive.trajectorySequenceBuilder(firstCenter.end())
                .lineToLinearHeading(new Pose2d(-30, -34, 0))
                .build();
        TrajectorySequence UnderFermRunning = R.drive.trajectorySequenceBuilder(firstCenter.end())
                .lineToLinearHeading(new Pose2d(-15, -48, 0))
                .lineToLinearHeading(new Pose2d(48, -56, 0))
                .build();
        TrajectorySequence BackDropRunning = R.drive.trajectorySequenceBuilder(UnderFermRunning.end())
                .lineToLinearHeading(new Pose2d(92, -28.5, 0))
                .build();
        TrajectorySequence Parking = R.drive.trajectorySequenceBuilder(BackDropRunning.end())
                .lineToLinearHeading(new Pose2d(77, -40, 0))
                .build();
        TrajectorySequence UnderFermRunningLeft = R.drive.trajectorySequenceBuilder(firstLeft.end())
                .lineToLinearHeading(new Pose2d(-15, -48, 0))
                .lineToLinearHeading(new Pose2d(56, -56, 0))
                .build();
        TrajectorySequence BackDropRunningLeft = R.drive.trajectorySequenceBuilder(UnderFermRunningLeft.end())
                .lineToLinearHeading(new Pose2d(92, -19.5, 0))
                .build();
        TrajectorySequence ParkingLeft = R.drive.trajectorySequenceBuilder(BackDropRunningLeft.end())
                .lineToLinearHeading(new Pose2d(70, -15, 0))
                .lineToLinearHeading(new Pose2d(80, -40, 0))
                .build();
        TrajectorySequence UnderFermRunningRight = R.drive.trajectorySequenceBuilder(firstRight.end())
                .lineToLinearHeading(new Pose2d(-25, -30, 0))
                .lineToLinearHeading(new Pose2d(-25, -52, 0))
                .lineToLinearHeading(new Pose2d(53, -56, 0))
                .build();
        TrajectorySequence BackDropRunningRight = R.drive.trajectorySequenceBuilder(UnderFermRunningRight.end())
                .lineToLinearHeading(new Pose2d(92, -26, 0))
                .build();
        TrajectorySequence ParkingRight = R.drive.trajectorySequenceBuilder(BackDropRunningRight.end())
                .lineToLinearHeading(new Pose2d(80, -30, 0))
                .lineToLinearHeading(new Pose2d(80, -45, 0))
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

        if (isStarted()) {

            switch (detector.getLocation()) {
                case LEFT:
                    R.drive.followTrajectorySequence(firstRight);
                    scorer.scor();

                    R.drive.followTrajectorySequence(UnderFermRunningRight);
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

                case RIGHT:
                    R.drive.followTrajectorySequence(firstLeft);
                    scorer.scor();
                    R.drive.followTrajectorySequence(UnderFermRunningLeft);
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
                case CENTER:
                    R.drive.followTrajectorySequence(firstCenter);
                    scorer.scor();

                    R.drive.followTrajectorySequence(UnderFermRunning);
                    R.drive.followTrajectorySequence(BackDropRunning);
                    intake.Autonomous2();
                    sleep(1000);
                    intake.Autonomoys3();
                    sleep(1000);
                    intake.Autonomous4();
                    sleep(500);
                    intake.Autonomous5();;
                    sleep(500);
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