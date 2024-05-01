package org.firstinspires.ftc.teamcode.pedroPathing.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

/*
 * Op mode for preliminary tuning of the follower PID coefficients (located in the drive base
 * classes). The robot drives back and forth in a straight line indefinitely. Utilization of the
 * dashboard is recommended for this tuning routine. To access the dashboard, connect your computer
 * to the RC's WiFi network. In your browser, navigate to https://192.168.49.1:8080/dash if you're
 * using the RC phone or https://192.168.43.1:8080/dash if you are using the Control Hub. Once
 * you've successfully connected, start the program, and your robot will begin moving forward and
 * backward. You should observe the target position (green) and your pose estimate (blue) and adjust
 * your follower PID coefficients such that you follow the target position as accurately as possible.
 * If you are using SampleMecanumDrive, you should be tuning TRANSLATIONAL_PID and HEADING_PID.
 * If you are using SampleTankDrive, you should be tuning AXIAL_PID, CROSS_TRACK_PID, and HEADING_PID.
 * These coefficients can be tuned live in dashboard.
 *
 * This opmode is designed as a convenient, coarse tuning for the follower PID coefficients. It
 * is recommended that you use the FollowerPIDTuner opmode for further fine tuning.
 */
@Config
@Autonomous
public class StraightBackAndForth extends LinearOpMode {

    public static double DISTANCE = 10;

    Follower drive;

    private PathChain forwards, backwards;

    Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d goForward = new Pose2d(DISTANCE, 0, Math.toRadians(0));

    public PathChain directPath(Pose2d startPath, Pose2d endPath) {
        return drive.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(startPath), new Point(endPath))))
                .setLinearHeadingInterpolation(startPath.getHeading(), endPath.getHeading())
                .build();

    }
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new Follower(hardwareMap);

        forwards = directPath(startPose, goForward);
        backwards = directPath(goForward, startPose);



        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            drive.followPath(forwards);
            drive.followPath(backwards);
            drive.update();
        }
    }
}

