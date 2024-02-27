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
public class AutoBRWithoutDelay extends LinearOpMode {
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
        TrajectorySequence violetPlusSteak = R.drive.trajectorySequenceBuilder(new Pose2d())
                .splineToLinearHeading(new Pose2d(-5, -5, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-10, -39, Math.toRadians(0)), Math.toRadians(0))
                .addDisplacementMarker(() -> scorer.scor())
                .splineToLinearHeading(new Pose2d(-16, -35.5, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-24, -48.5, Math.toRadians(0)), Math.toRadians(0))
                .build();
        TrajectorySequence BackDropRunning = R.drive.trajectorySequenceBuilder(violetPlusSteak.end())
                .splineToLinearHeading(new Pose2d(53, -56.5, Math.toRadians(0)), Math.toRadians(0))
                .addDisplacementMarker(() -> {

                    intake.Autonomous4();
                    intake.Autonomous2();
                    sleep(200);
                    intake.Autonomoys3();
                    intake.lift();
                })
               /* .addDisplacementMarker(15,() -> {
                    intake.Autonomoys3();
                })*/
                .splineToLinearHeading(new Pose2d(89, -26, Math.toRadians(0)), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    intake.OpenScor();
                    sleep(300);
                    intake.MovSetCenter();
                })
                .build();
        TrajectorySequence SecondSteakRunning = R.drive.trajectorySequenceBuilder(violetPlusSteak.end())
                .splineToLinearHeading(new Pose2d(53, -56.5, Math.toRadians(0)), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    //intake.lift2();
                    intake.Autonomous6();
                })
                //.splineToLinearHeading(new Pose2d(-24, -48.5, Math.toRadians(0)), Math.toRadians(0))
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
                    R.drive.followTrajectorySequence(violetPlusSteak);
                    R.drive.followTrajectorySequence(BackDropRunning);
                    break;
                case RIGHT:
                    R.drive.followTrajectorySequence(violetPlusSteak);
                    R.drive.followTrajectorySequence(BackDropRunning);
                    break;
                case CENTER:
                    R.drive.followTrajectorySequence(violetPlusSteak);
                   //intake.IntakePixels();
                    R.drive.followTrajectorySequence(BackDropRunning);
                    R.drive.followTrajectorySequence(SecondSteakRunning);
                    break;
            }
        }
        cam.stopStreaming();


        while (!isStopRequested() && opModeIsActive());
    }
}
