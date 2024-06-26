package org.firstinspires.ftc.teamcode.pedroPathing.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;

@TeleOp (group = "Pedro Pathing Tuning", name = "Localization Test Pedro")
public class LocalizationTest2 extends OpMode {
    private Follower follower;
    private Vector driveVector;
    private Vector headingVector;

    private Vector correctiveVector;
    private Telemetry telemetryA;

    @Override
    public void init() {
        follower = new Follower(hardwareMap, false);
        driveVector = new Vector();
        headingVector = new Vector();
        correctiveVector = new Vector();

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("This will print your robot's position to telemetry while "
                + "allowing robot control through a basic mecanum drive on gamepad 1.");
        telemetryA.update();
    }

    @Override
    public void loop() {
        driveVector.setOrthogonalComponents(gamepad1.right_stick_x, gamepad1.right_stick_y);
        driveVector.setMagnitude(MathFunctions.clamp(driveVector.getMagnitude(), 0, 1));
        driveVector.rotateVector(follower.getPose().getHeading());

        headingVector.setComponents(-gamepad1.left_stick_x, follower.getPose().getHeading());

        follower.setMovementVectors(correctiveVector, headingVector, driveVector);
        follower.update();

        telemetryA.addData("x", follower.getPose().getX());
        telemetryA.addData("y", follower.getPose().getY());
        telemetryA.addData("heading", follower.getPose().getHeading());
        telemetryA.addLine("Pedro");
        telemetryA.update();

    }
}
