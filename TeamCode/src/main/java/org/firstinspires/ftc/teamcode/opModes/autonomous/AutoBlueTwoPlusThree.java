package org.firstinspires.ftc.teamcode.opModes.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Camera.PropDetection;
import org.firstinspires.ftc.teamcode.Camera.PropDetectionRed;
import org.firstinspires.ftc.teamcode.Modules.ExtendAuto;
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
public class AutoBlueTwoPlusThree extends LinearOpMode {
    Robot1 R;
    IntakeSecondVersion intake;
    SpikeScorer scorer;
    Servo hook;
    OpenCvCamera cam;

    LiftAuto lift;
    DcMotor zahvat, vidvizh, lift1, lift2;

    ExtendAuto extend;

    public  static double xSplineOutOfWallCenter = -48,ySplineOutOfWallCenter =  60,
            xSpikeCenter = -41, ySpikeCenter =  23.5,
            xSteakRunning0Center = -58, ySteakRunning0Center = 37,
            xSteakRunning1Cebter = -59.5, ySteakRunning1Cebter = 8,
            xBackdropRunning1Center = 28,yBackdropRunning1Center = 5,
            xBackdropRunning1_2Center = 28 ,yBackdropRunning1_2Center = 30,
            xBackdropRunning2Center =  54.2, yBackdropRunning2Center = 36,
            xBackdropRunning2_1Center =  43.7, yBackdropRunning2_1Center = 35.5,
            xSteakRunning2Cebter=46, ySteakRunning2Cebter=10,
            xSteakRunning3Cebter = -25, ySteakRunning3Cebter = 10,
            xBackdropRunning3Center = 28, yBackdropRunning3Center = 5,
            xBackdropRunning4Center = 50, yBackdropRunning4Center = 33,
            xParking = 45, yParking = 10;

    public  static double xSplineOutOfWallLeft = -44,ySplineOutOfWallLeft =  56,
            xSpikeLeft = -31.3, ySpikeLeft =  35,
            xSteakRunning0Left = -48, ySteakRunning0Left = 34,
            xSteakRunning1Left = -58.5, ySteakRunning1Left = 3.5,
            xBackdropRunning1Left = 25 ,yBackdropRunning1Left = 5,
            xBackdropRunning1_2Left = 45 ,yBackdropRunning1_2Left = 30,
            xBackdropRunning1_3Left = 48 ,yBackdropRunning1_3Left = 42,
            xBackdropRunning2Left =  52.5, yBackdropRunning2Left = 46.2,
            xSteakRunning2Left=43, ySteakRunning2Left=15,
            xSteakRunning2_1Left=33, ySteakRunning2_1Left=46,
            xSteakRunning3Left = -15,ySteakRunning3Left = 10,
            xBackdropRunning3Left = 38, yBackdropRunning3Left = 10,
            xBackdropRunning4Left = 50, yBackdropRunning4Left = 33,
            xParkingLeft = 45, yParkingLeft = 10;

    public  static double xSplineOutOfWallRight = -33,ySplineOutOfWallRight =  56,
            xSpikeRight = -52.3, ySpikeRight =  32,
            xSteakRunning0Right = -53, ySteakRunning0Right = 34,
            xSteakRunning1Right = -60, ySteakRunning1Right = 1.5,
            xBackdropRunning1Right = 25,yBackdropRunning1Right = 5,
            xBackdropRunning1_2Right =  45, yBackdropRunning1_2Right = 30,
            xBackdropRunning1_3Right =  45, yBackdropRunning1_3Right = 25,
            xBackdropRunning2Right =  52.5, yBackdropRunning2Right = 30.2,
            xBackdropRunning2_1Right =  38.7, yBackdropRunning2_1Right = 29,
            xSteakRunning2Right=46, ySteakRunning2Right=15,
            xSteakRunning3Right = -15,ySteakRunning3Right = 10,
            xBackdropRunning3Right = 38, yBackdropRunning3Right = 10,
            xBackdropRunning4Right = 50, yBackdropRunning4Right = 31,
            xParkingRight = 45, yParkingRight = 10;

    @Override
    public void runOpMode() throws InterruptedException {

        R = new Robot1(this);
        lift = new LiftAuto(this);
        extend = new ExtendAuto(this);
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
      //  vidvizh.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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

        Pose2d startPose = new Pose2d(-38, 61.5, 0);

        R.drive.setPoseEstimate(startPose);

        TrajectorySequence firstCenter = R.drive.trajectorySequenceBuilder(startPose)
                //.splineToLinearHeading(new Pose2d(xSplineOutOfWallCenter, ySplineOutOfWallCenter, Math.toRadians(0)), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(xSpikeCenter, ySpikeCenter, Math.toRadians(0))/*, Math.toRadians(0)*/)

                .addDisplacementMarker(() -> {
                    scorer.scor();

                    //zahvat.setPower(1);
                })
                .lineToLinearHeading(new Pose2d(xSpikeCenter-2, ySpikeCenter, Math.toRadians(0))/* Math.toRadians(180)*/)
                //.waitSeconds(0.4)
                .setAccelConstraint(new TrajectoryAccelerationConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        return 18.156578905846143;
                    }
                })
                //.waitSeconds(0.3)
                .lineToLinearHeading(new Pose2d(xSteakRunning0Center, ySteakRunning0Center, Math.toRadians(0))/*, Math.toRadians(0)*/)
                .addDisplacementMarker(() -> {
                    zahvat.setPower(1);
                })


                //.waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(xSteakRunning1Cebter, ySteakRunning1Cebter, Math.toRadians(0))/*, Math.toRadians(0)*/)

                .setAccelConstraint(new TrajectoryAccelerationConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        return 18.156578905846143;
                    }
                })
                .splineToLinearHeading(new Pose2d(xSteakRunning1Cebter+0.001, ySteakRunning1Cebter+0.001, Math.toRadians(0)), Math.toRadians(0))
                .setAccelConstraint(new TrajectoryAccelerationConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        return 58.156578905846143;
                    }
                })
                //.splineToLinearHeading(new Pose2d(xSteakRunning1Cebter+2, ySteakRunning1Cebter-4, Math.toRadians(0)), Math.toRadians(0))
              /*  .splineToLinearHeading(new Pose2d(xSteakRunning1Cebter+2, ySteakRunning1Cebter+16, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(0.1)
                .splineToLinearHeading(new Pose2d(xSteakRunning1Cebter, ySteakRunning1Cebter, Math.toRadians(0)), Math.toRadians(0))*/
                .addDisplacementMarker(() -> {
                    zahvat.setPower(-1);
                })
                .waitSeconds(0.1)
                .splineToLinearHeading(new Pose2d(xBackdropRunning1Center, yBackdropRunning1Center, Math.toRadians(0)), Math.toRadians(0))

                .addDisplacementMarker(() -> {
                    intake.ScorerClose();
                    zahvat.setPower(-1);
                })

                .splineToLinearHeading(new Pose2d(xBackdropRunning1_2Center, yBackdropRunning1_2Center, Math.toRadians(0)), Math.toRadians(0))
                .setAccelConstraint(new TrajectoryAccelerationConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        return 18.156578905846143;
                    }
                })
                .addDisplacementMarker(()-> {
                    zahvat.setPower(-1);
                    intake.Autonomous2();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.05,()-> {
                    lift.GetPose(300);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1,()-> {
                    intake.SetLeftMov();
                })
                .splineToLinearHeading(new Pose2d(xBackdropRunning2Center, yBackdropRunning2Center, Math.toRadians(0)), Math.toRadians(0))
                //.setTurnConstraint(18.156578905846143, 18.156578905846143)
                .waitSeconds(0.1)
                .addDisplacementMarker(() -> {
                    zahvat.setPower(0);
                    //     intake.ScorerOpen();
                })
                .splineToLinearHeading(new Pose2d(xBackdropRunning2Center+0.001, yBackdropRunning2Center+0.001, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(0.1)
                .addDisplacementMarker(() -> {
                    zahvat.setPower(0);
                    intake.ScorerOpen();
                })

                .splineToLinearHeading(new Pose2d(xBackdropRunning2Center+0.002, yBackdropRunning2Center+0.002, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(0.3)
                .addDisplacementMarker(()-> {
                    zahvat.setPower(0);
                    lift.GetPose(500);
                    intake.SetCenterMov();
                })
                .setAccelConstraint(new TrajectoryAccelerationConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        return 58.156578905846143;
                    }
                })
                .lineToLinearHeading(new Pose2d(xBackdropRunning2_1Center, yBackdropRunning2_1Center, Math.toRadians(0))/*, Math.toRadians(180)*/)
                .lineToLinearHeading(new Pose2d(xSteakRunning2Cebter, ySteakRunning2Cebter, Math.toRadians(0))/*, Math.toRadians(180)*/)
                .UNSTABLE_addTemporalMarkerOffset(1,()-> {
                    lift.GetPose(0);
                    intake.OpenHook();
                })
                .waitSeconds(0.1)
                .lineToLinearHeading(new Pose2d(xSteakRunning2Cebter+0.001, ySteakRunning2Cebter+0.001, Math.toRadians(0))/*, Math.toRadians(180)*/)
                .addDisplacementMarker(()-> {
                    intake.CloseWall();
                    intake.AutonomousPerekid();
                    extend.GetPose(-5000);
                    zahvat.setPower(1);
                })
                .setAccelConstraint(new TrajectoryAccelerationConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        return 18.156578905846143;
                    }
                })

                //.waitSeconds(4)
                 .lineToLinearHeading(new Pose2d(xSteakRunning3Cebter, ySteakRunning3Cebter, Math.toRadians(0))/*, Math.toRadians(180)*/)
                .addDisplacementMarker(()-> {
                   zahvat.setPower(1);
                })
                .waitSeconds(0.1)

                .splineToLinearHeading(new Pose2d(xBackdropRunning3Center, yBackdropRunning3Center, Math.toRadians(0)), Math.toRadians(180))
                .addDisplacementMarker(()-> {
                    zahvat.setPower(1);
                    extend.GetPose(0);
                })
                .setAccelConstraint(new TrajectoryAccelerationConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        return 58.156578905846143;
                    }
                })
                //.waitSeconds(1)
                /*.splineToLinearHeading(new Pose2d(xBackdropRunning3Center, yBackdropRunning3Center, Math.toRadians(0)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(xBackdropRunning4Center, yBackdropRunning4Center, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(0.5)
                .splineToLinearHeading(new Pose2d(xParking, yParking, Math.toRadians(0)), Math.toRadians(0))*/
                .build();
        TrajectorySequence firstLeft = R.drive.trajectorySequenceBuilder(startPose)
                //.splineToLinearHeading(new Pose2d(xSplineOutOfWallCenter, ySplineOutOfWallCenter, Math.toRadians(0)), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(xSpikeLeft-5, ySpikeLeft, Math.toRadians(0))/*, Math.toRadians(0)*/)
                .lineToLinearHeading(new Pose2d(xSpikeLeft, ySpikeLeft, Math.toRadians(0))/*, Math.toRadians(0)*/)

                .addDisplacementMarker(() -> {
                    scorer.scor();

                    //zahvat.setPower(1);
                })
                .lineToLinearHeading(new Pose2d(xSpikeLeft-3.5, ySpikeLeft+2, Math.toRadians(0))/* Math.toRadians(180)*/)
                .waitSeconds(0.4)

                .setAccelConstraint(new TrajectoryAccelerationConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        return 14.156578905846143;
                    }
                })
                //.waitSeconds(0.3)
                .lineToLinearHeading(new Pose2d(xSteakRunning0Left, ySteakRunning0Left, Math.toRadians(0))/* Math.toRadians(180)*/)
                .addDisplacementMarker(() -> {
                    zahvat.setPower(1);
                })


                //.waitSeconds(0.5)
                .splineToLinearHeading(new Pose2d(xSteakRunning1Left, ySteakRunning1Left, Math.toRadians(0)), Math.toRadians(0))

                .setAccelConstraint(new TrajectoryAccelerationConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        return 58.156578905846143;
                    }
                })
                .splineToLinearHeading(new Pose2d(xSteakRunning1Left-2.7, ySteakRunning1Left+10, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(0.3)
                .splineToLinearHeading(new Pose2d(xBackdropRunning1Left, yBackdropRunning1Left, Math.toRadians(0)), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    intake.ScorerClose();
                    zahvat.setPower(-1);
                })
                .splineToLinearHeading(new Pose2d(xBackdropRunning1_2Left, yBackdropRunning1_2Left, Math.toRadians(0)), Math.toRadians(0))
                .setAccelConstraint(new TrajectoryAccelerationConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        return 18.156578905846143;
                    }
                })
                .addDisplacementMarker(()-> {
                    zahvat.setPower(-1);
                    intake.Autonomous2();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.05,()-> {
                    lift.GetPose(250);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1,()-> {
                    intake.SetRightMov();
                })
                .splineToLinearHeading(new Pose2d(xBackdropRunning1_3Left, yBackdropRunning1_3Left, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(xBackdropRunning2Left, yBackdropRunning2Left, Math.toRadians(0)), Math.toRadians(0))
                //.setTurnConstraint(18.156578905846143, 18.156578905846143)
                .waitSeconds(0.6)
                .addDisplacementMarker(() -> {
                    zahvat.setPower(0);
                    //     intake.ScorerOpen();
                })
                .splineToLinearHeading(new Pose2d(xBackdropRunning2Left+0.001, yBackdropRunning2Left+0.001, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(0.6)
                .addDisplacementMarker(() -> {
                    zahvat.setPower(0);
                    intake.ScorerOpen();
                    // extend.GetPose(-1500);
                })

                .splineToLinearHeading(new Pose2d(xBackdropRunning2Left+0.002, yBackdropRunning2Left+0.002, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(1.5)
                .addDisplacementMarker(()-> {
                    zahvat.setPower(0);
                    lift.GetPose(500);
                    intake.SetCenterMov();
                })
                .setAccelConstraint(new TrajectoryAccelerationConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        return 58.156578905846143;
                    }
                })
                .lineToLinearHeading(new Pose2d(xSteakRunning2_1Left, ySteakRunning2_1Left, Math.toRadians(0))/*, Math.toRadians(180)*/)
                .lineToLinearHeading(new Pose2d(xSteakRunning2Left, ySteakRunning2Left, Math.toRadians(0))/*, Math.toRadians(180)*/)
                .UNSTABLE_addTemporalMarkerOffset(1,()-> {
                    lift.GetPose(0);
                    intake.OpenHook();
                })

                .lineToLinearHeading(new Pose2d(xSteakRunning2Left+0.001, ySteakRunning2Left+0.001, Math.toRadians(0))/*, Math.toRadians(180)*/)
                .UNSTABLE_addTemporalMarkerOffset(5,()-> {
                    intake.AutonomousPerekid();
                    extend.GetPose(-2000);
                })
                //.waitSeconds(4)
                 .lineToLinearHeading(new Pose2d(xSteakRunning3Left, ySteakRunning3Left, Math.toRadians(0))/*, Math.toRadians(180)*/)
                //.waitSeconds(1)
                /*.splineToLinearHeading(new Pose2d(xBackdropRunning3Center, yBackdropRunning3Center, Math.toRadians(0)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(xBackdropRunning4Center, yBackdropRunning4Center, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(0.5)
                .splineToLinearHeading(new Pose2d(xParking, yParking, Math.toRadians(0)), Math.toRadians(0))*/
                .build();
        TrajectorySequence firstRight = R.drive.trajectorySequenceBuilder(startPose)
                //.splineToLinearHeading(new Pose2d(xSplineOutOfWallCenter, ySplineOutOfWallCenter, Math.toRadians(0)), Math.toRadians(0))
                //.lineToLinearHeading(new Pose2d(xSpikeRight-5, ySpikeRight, Math.toRadians(0))/*, Math.toRadians(0)*/)
                .lineToLinearHeading(new Pose2d(xSpikeRight, ySpikeRight, Math.toRadians(0))/*, Math.toRadians(0)*/)
                .addDisplacementMarker(() -> {
                    scorer.scor();

                    //zahvat.setPower(1);
                })
                .lineToLinearHeading(new Pose2d(xSpikeRight-7, ySpikeRight, Math.toRadians(0))/*, Math.toRadians(0)*/)
                .addDisplacementMarker(() -> {
                    zahvat.setPower(1);
                })

                .waitSeconds(0.4)

                .setAccelConstraint(new TrajectoryAccelerationConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        return 14.156578905846143;
                    }
                })
                //.waitSeconds(0.3)
                // .lineToLinearHeading(new Pose2d(xSteakRunning0Right, ySteakRunning0Right, Math.toRadians(0))/* Math.toRadians(180)*/)


                //.waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(xSteakRunning1Right, ySteakRunning1Right, Math.toRadians(0))/*, Math.toRadians(0)*/)

                .setAccelConstraint(new TrajectoryAccelerationConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        return 38.156578905846143;
                    }
                })
                .splineToLinearHeading(new Pose2d(xSteakRunning1Right-3, ySteakRunning1Right+10, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(0.3)
                .splineToLinearHeading(new Pose2d(xSteakRunning1Right, ySteakRunning1Right, Math.toRadians(0)), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    intake.ScorerClose();
                    zahvat.setPower(-1);
                })
                .splineToLinearHeading(new Pose2d(xBackdropRunning1Right, yBackdropRunning1Right, Math.toRadians(0)), Math.toRadians(0))
                /* .addDisplacementMarker(() -> {
                     intake.ScorerClose();
                     zahvat.setPower(-1);
                 })*/
                .splineToLinearHeading(new Pose2d(xBackdropRunning1_2Right, yBackdropRunning1_2Right, Math.toRadians(0)), Math.toRadians(0))
                .setAccelConstraint(new TrajectoryAccelerationConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        return 18.156578905846143;
                    }
                })
                .addDisplacementMarker(()-> {
                    zahvat.setPower(-1);
                    intake.Autonomous2();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.05,()-> {
                    lift.GetPose(280);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1,()-> {
                    intake.SetLeftMov();
                })
                .splineToLinearHeading(new Pose2d(xBackdropRunning1_3Right, yBackdropRunning1_3Right, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(xBackdropRunning2Right, yBackdropRunning2Right, Math.toRadians(0)), Math.toRadians(0))
                //.setTurnConstraint(18.156578905846143, 18.156578905846143)
                .waitSeconds(0.6)
                .addDisplacementMarker(() -> {
                    zahvat.setPower(0);
                    //     intake.ScorerOpen();
                })
                .splineToLinearHeading(new Pose2d(xBackdropRunning2Right+0.001, yBackdropRunning2Right+0.001, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(0.6)
                .addDisplacementMarker(() -> {
                    zahvat.setPower(0);
                    intake.ScorerOpen();
                    // extend.GetPose(-1500);
                })

                .splineToLinearHeading(new Pose2d(xBackdropRunning2Right+0.002, yBackdropRunning2Right+0.002, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(1.5)
                .addDisplacementMarker(()-> {
                    zahvat.setPower(0);
                    lift.GetPose(500);
                    intake.SetCenterMov();
                })
                .setAccelConstraint(new TrajectoryAccelerationConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        return 58.156578905846143;
                    }
                })
                .lineToLinearHeading(new Pose2d(xBackdropRunning2_1Right, xBackdropRunning2_1Right, Math.toRadians(0))/*, Math.toRadians(180)*/)
                .lineToLinearHeading(new Pose2d(xSteakRunning2Right, ySteakRunning2Right, Math.toRadians(0))/*, Math.toRadians(180)*/)
                .UNSTABLE_addTemporalMarkerOffset(1,()-> {
                    lift.GetPose(0);
                    intake.OpenHook();
                })
                .waitSeconds(2)
                .lineToLinearHeading(new Pose2d(xSteakRunning2Right+0.001, ySteakRunning2Right+0.001, Math.toRadians(0))/*, Math.toRadians(180)*/)
                .addDisplacementMarker(()-> {
                    intake.AutonomousPerekid();
                    extend.GetPose(-2000);
                })
                //.waitSeconds(4)
                 .lineToLinearHeading(new Pose2d(xSteakRunning3Right, ySteakRunning3Right, Math.toRadians(0))/*, Math.toRadians(180)*/)
                //.waitSeconds(1)
                /*.splineToLinearHeading(new Pose2d(xBackdropRunning3Center, yBackdropRunning3Center, Math.toRadians(0)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(xBackdropRunning4Center, yBackdropRunning4Center, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(0.5)
                .splineToLinearHeading(new Pose2d(xParking, yParking, Math.toRadians(0)), Math.toRadians(0))*/
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
                R.drive.followTrajectorySequenceAsync(firstLeft);
                break;

            case LEFT:
                cam.stopStreaming();
                intake.Autonomous6();
                intake.Autonomous6();
                R.drive.followTrajectorySequenceAsync(firstRight);
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
            extend.Auto();
        }

        //  cam.stopStreaming();


        while (!isStopRequested() && opModeIsActive());
    }

}