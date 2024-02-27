package org.firstinspires.ftc.teamcode.Camera;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Camera.PropDetection;
import org.firstinspires.ftc.teamcode.Robot1;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="Prop Detecotor 1", group="Auto")
public class PropDetectionOpMode extends LinearOpMode {

    OpenCvCamera cam;

    Robot1 R;

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());
        cam = OpenCvCameraFactory.getInstance()
                .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        cam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        PropDetection detector = new PropDetection(telemetry);
        cam.setPipeline(detector);
        R = new Robot1(this);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Trajectory strafe1 = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(9.850)
                .build();
        Trajectory forward1 = R.drive.trajectoryBuilder(new Pose2d())
                .forward(10)
                .build();
        Trajectory strafe2 = R.drive.trajectoryBuilder(forward1.end())
                .strafeLeft(8.5)
                .build();
        Trajectory back1 = R.drive.trajectoryBuilder(new Pose2d())
                .back(79)
                .build();
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
        while (!isStarted()) {
            switch (detector.getLocation()) {
                case LEFT:

                    break;
                case RIGHT:

                    break;
                case CENTER:

                    break;
                case NOT_FOUND:
            }
        }
        cam.stopStreaming();
    }
}
