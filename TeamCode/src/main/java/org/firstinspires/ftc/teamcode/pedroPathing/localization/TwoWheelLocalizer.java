package org.firstinspires.ftc.teamcode.pedroPathing.localization;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;
import org.firstinspires.ftc.teamcode.pedroPathing.util.NanoTimer;

/**
 * This is the TwoWheelLocalizer class.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @version 1.0, 4/2/2024
 */
@Config
public class TwoWheelLocalizer extends Localizer { // todo: make two wheel odo work
    private HardwareMap hardwareMap;
    private IMU imu;
    private Pose startPose;
    private Pose currentPose;
    private Pose currentVelocity;
    private Matrix startRotationMatrix;
    private NanoTimer timer;
    private long deltaTimeNano;
    public Encoder forwardEncoder;
    public Encoder strafeEncoder;
    private Pose forwardEncoderPose;
    private Pose strafeEncoderPose;
    private double previousIMUOrientation;
    private double deltaRadians;
    private double totalHeading;
    public static double FORWARD_PER_REV = 4096;
    public static double STRAFE_TICKS_PER_REV = 4096 ;

    public TwoWheelLocalizer(HardwareMap map) {
        this(map, new Pose());
    }

    public TwoWheelLocalizer(HardwareMap map, Pose setStartPose) {
        // TODO: replace these with your encoder positions
        forwardEncoderPose = new Pose(60 / 25.4, -100 / 25.4, 0);
        strafeEncoderPose = new Pose(-130 / 25.4, 30 / 25.4, Math.toRadians(90));

        hardwareMap = map;

        imu = hardwareMap.get(IMU.class, "imu1");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                DriveConstants.LOGO_FACING_DIR, DriveConstants.USB_FACING_DIR));
        imu.initialize(parameters);

        // TODO: replace these with your encoder ports
        forwardEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightRear"));
        strafeEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightFront"));

        // TODO: reverse any encoders necessary
        forwardEncoder.setDirection(Encoder.REVERSE);
        strafeEncoder.setDirection(Encoder.FORWARD);


        setStartPose(setStartPose);
        timer = new NanoTimer();
        deltaTimeNano = 1;
        currentPose = startPose;
        currentVelocity = new Pose();

        previousIMUOrientation = MathFunctions.normalizeAngle(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        deltaRadians = 0;
    }

    @Override
    public Pose getPose() {
        return currentPose.copy();
    }

    @Override
    public Pose getVelocity() {
        return currentVelocity.copy();
    }

    @Override
    public Vector getVelocityVector() {
        return currentVelocity.getVector();
    }

    @Override
    public void setStartPose(Pose setStart) {
        startPose = setStart;
        setStartRotationMatrix(startPose.getHeading());
    }

    public void setStartRotationMatrix(double startHeading) {
        startRotationMatrix = new Matrix(3,3);
        startRotationMatrix.set(0, 0, Math.cos(startHeading));
        startRotationMatrix.set(0, 1, -Math.sin(startHeading));
        startRotationMatrix.set(1, 0, Math.sin(startHeading));
        startRotationMatrix.set(1, 1, -Math.cos(startHeading));
        startRotationMatrix.set(2, 2, 1.0);
    }
    public static double encoderTicksToInches(double ticks) {
        return 19/25.4 * 2 * Math.PI * 1 * ticks/ FORWARD_PER_REV;
    }

    @Override
    public void setPose(Pose setPose) {
        currentPose = setPose;
    }

    @Override
    public void update() {
        deltaTimeNano = timer.getElapsedTime();
        timer.resetTimer();

        updateEncoders();
        Matrix robotDeltas = getRobotDeltas();
        Matrix globalDeltas = new Matrix(3,1);

        Matrix transformation = new Matrix(3,3);
        if (Math.abs(robotDeltas.get(2, 0)) < 0.001) {
            transformation.set(0, 0, 1.0 - (Math.pow(robotDeltas.get(2, 0), 2) / 6.0));
            transformation.set(0, 1, -robotDeltas.get(2, 0) / 2.0);
            transformation.set(1, 0, robotDeltas.get(2, 0) / 2.0);
            transformation.set(1, 1, 1.0 - (Math.pow(robotDeltas.get(2, 0), 2) / 6.0));
            transformation.set(2, 2, 1.0);
        } else {
            transformation.set(0, 0, Math.sin(robotDeltas.get(2, 0)) / robotDeltas.get(2, 0));
            transformation.set(0, 1, (Math.cos(robotDeltas.get(2, 0)) - 1.0) / robotDeltas.get(2, 0));
            transformation.set(1, 0, (1.0 - Math.cos(robotDeltas.get(2, 0))) / robotDeltas.get(2, 0));
            transformation.set(1, 1, Math.sin(robotDeltas.get(2, 0)) / robotDeltas.get(2, 0));
            transformation.set(2, 2, 1.0);
        }

        globalDeltas = Matrix.multiply(Matrix.multiply(startRotationMatrix, transformation), robotDeltas);

        currentPose.add(new Pose(globalDeltas.get(0, 0), globalDeltas.get(1, 0), globalDeltas.get(2, 0)));
        currentVelocity = new Pose(globalDeltas.get(0, 0) / (deltaTimeNano * Math.pow(10.0, 9)), globalDeltas.get(1, 0) / (deltaTimeNano * Math.pow(10.0, 9)), globalDeltas.get(2, 0) / (deltaTimeNano * Math.pow(10.0, 9)));

        totalHeading += globalDeltas.get(2, 0);
    }

    public void updateEncoders() {
        forwardEncoder.update();
        strafeEncoder.update();

        double currentIMUOrientation =MathFunctions.normalizeAngle(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        deltaRadians = MathFunctions.getTurnDirection(previousIMUOrientation, currentIMUOrientation) * MathFunctions.getSmallestAngleDifference(currentIMUOrientation, previousIMUOrientation);
        previousIMUOrientation = currentIMUOrientation;
    }

    public Matrix getRobotDeltas() {
        Matrix returnMatrix = new Matrix(3,1);
        // x/forward movement
        returnMatrix.set(0,0, encoderTicksToInches(forwardEncoder.getDeltaPosition())/* * (forwardEncoder.getDeltaPosition() - forwardEncoderPose.getY() * deltaRadians)*/);
        //y/strafe movement
        returnMatrix.set(1,0, encoderTicksToInches(/*forwardEncoder.getDeltaPosition()) * (*/strafeEncoder.getDeltaPosition())/* - strafeEncoderPose.getX() * deltaRadians)*/);
        // theta/turning
        returnMatrix.set(2,0, deltaRadians);
        return returnMatrix;
    }

    public double getTotalHeading() {
        return totalHeading;
    }
}
