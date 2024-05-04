package org.firstinspires.ftc.teamcode.pedroPathing.localization;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    ^
 *    |
 *    | ( x direction)
 *    |
 *    v
 *    <----( y direction )---->

 *        (forward)
 *    /--------------\
 *    |     ____     |
 *    |     ----     |    <- Perpendicular Wheel
 *    |           || |
 *    |           || |    <- Parallel Wheel
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
public class TwoWheelTrackingLocalizer2 extends TwoTrackingWheelLocalizer {

    private IMU imu;
    public static double TICKS_PER_REV = 4096;
    public static double WHEEL_RADIUS = 19/25.4; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double PARALLEL_X = 60 / 25.4; // X is the up and down direction
    public static double PARALLEL_Y = -100 / 25.4; // Y is the strafe direction

    public static double PERPENDICULAR_X = -130 / 25.4;
    public static double PERPENDICULAR_Y = 30 / 25.4;

    private double totalHeading;

    private List<Integer> lastEncPositions, lastEncVels;

    // Parallel/Perpendicular to the forward axis
    // Parallel wheel is parallel to the forward axis
    // Perpendicular is perpendicular to the forward axis
    private Encoder parallelEncoder, perpendicularEncoder;


    public TwoWheelTrackingLocalizer2(HardwareMap hardwareMap,List<Integer> lastTrackingEncPositions, List<Integer> lastTrackingEncVels) {
        super(Arrays.asList(
                new Pose2d(PARALLEL_X, PARALLEL_Y, 0),
                new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90))
        ));

        imu = hardwareMap.get(IMU.class, "imu1");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                DriveConstants.LOGO_FACING_DIR, DriveConstants.USB_FACING_DIR));
        imu.initialize(parameters);

        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightRear"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightFront"));
        parallelEncoder.setDirection(Encoder.Direction.REVERSE);

        lastEncPositions = lastTrackingEncPositions;
        lastEncVels = lastTrackingEncVels;

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
    }




    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }


    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        lastEncPositions.clear();
        lastEncPositions.add(-parallelEncoder.getCurrentPosition());
        lastEncPositions.add(perpendicularEncoder.getCurrentPosition());

        return Arrays.asList(
                encoderTicksToInches(-parallelEncoder.getCurrentPosition()),
                encoderTicksToInches(perpendicularEncoder.getCurrentPosition())
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        lastEncVels.clear();
        lastEncVels.add((int)(-parallelEncoder.getCorrectedVelocity()));
        lastEncVels.add((int)perpendicularEncoder.getCorrectedVelocity());

        return Arrays.asList(
                encoderTicksToInches(-parallelEncoder.getCorrectedVelocity()),//getCorrectedVelocity
                encoderTicksToInches(perpendicularEncoder.getCorrectedVelocity())//getCorrectedVelocity
        );
    }

    public void resetHeading(double heading) {
        setPoseEstimate(new Pose2d(getPoseEstimate().getX(), getPoseEstimate().getY(), heading));
    }

    @Override
    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public double getTotalHeading() {
        return totalHeading;
    }
}