package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Modules.Goose;
import org.firstinspires.ftc.teamcode.Modules.Intake;
import org.firstinspires.ftc.teamcode.Modules.Intake_mover;
import org.firstinspires.ftc.teamcode.Modules.Lift;
import org.firstinspires.ftc.teamcode.Modules.Modules;
import org.firstinspires.ftc.teamcode.Modules.Scorer;
import org.firstinspires.ftc.teamcode.Modules.V4bMover;
import org.firstinspires.ftc.teamcode.Modules.Virtual4bar;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


public class Robot1 extends Robot {
    //public DigitalChannel digitalTouch;
    public SampleMecanumDrive drive;

    public Modules modules;
    public double heading, x, y, heading2;
    double k = gamepad2.right_stick_y;

    public Robot1(LinearOpMode opMode) {
        super(opMode);
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        modules = new Modules(opMode);
    }

    public void control() {

        heading = drive.getRawExternalHeading();
        x = -gamepad1.left_stick_y;
        y = -gamepad1.left_stick_x;
       /* if (gamepad1.dpad_up) {
            drive.setWeightedDrivePower(
                    new Pose2d(0.5,
                            0, 0
                    ));
        } else if (gamepad1.dpad_down) {
            drive.setWeightedDrivePower(
                    new Pose2d(-0.5,
                            -0, -0
                    ));
        } if (gamepad1.dpad_left) {
            drive.setWeightedDrivePower(
                    new Pose2d(0,
                            0.5, -0
                    ));
        } else if (gamepad1.dpad_right) {
            drive.setWeightedDrivePower(
                    new Pose2d(0,
                            -0.5, -0
                    ));
        } if (gamepad1.left_bumper) {
            drive.setWeightedDrivePower(
                    new Pose2d(-0,
                            -0, -0.5
                    ));
        } else if (gamepad1.right_bumper) {
            drive.setWeightedDrivePower(
                    new Pose2d(-0,
                            -0, 0.5
                    ));
        } */
            drive.setWeightedDrivePower(new Pose2d(
                    /*x * Math.cos(heading) + y * Math.sin(heading),                            -1 * x * Math.sin(heading) + y * Math.cos(heading),*/-gamepad1.left_stick_y,
                    -gamepad1.left_stick_x, (-gamepad1.left_trigger + gamepad1.right_trigger)/* + (heading2 - heading) * 0.1*/
            ));
            // heading2 = heading;        }
        /*else if (gamepad1.left_stick_y < 0) {
        }*/
            if (gamepad1.left_trigger + gamepad1.right_trigger != 0) {
                heading2 = heading;
            }
            telemetry.update();
        }
    }

