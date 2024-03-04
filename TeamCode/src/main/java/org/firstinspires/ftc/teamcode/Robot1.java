package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Modules.Modules;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import com.acmerobotics.roadrunner.util.Angle;


public class Robot1 extends Robot {
    //public DigitalChannel digitalTouch;
    public SampleMecanumDrive drive;

    public Modules modules;
    public double x, y;

    double k = gamepad2.right_stick_y;

    Pose2d poseEstimate;
    double heading2 = 0, heading = 0;



    double time = 0;
    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }
    Mode currentMode = Mode.DRIVER_CONTROL;
   public double targetAngle = Math.toRadians(0);



    public Robot1(LinearOpMode opMode) {
        super(opMode);
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        modules = new Modules(opMode);
    }

    public void control() {
        time+=1;
        poseEstimate = drive.getPoseEstimate();

        x = -gamepad1.left_stick_y;
        y = -gamepad1.left_stick_x;


        switch (currentMode){
            case DRIVER_CONTROL:
                targetAngle = Math.toRadians(0);
                if ((gamepad1.left_trigger + gamepad1.right_trigger)!=0) {
                    drive.setWeightedDrivePower(new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            (-gamepad1.left_trigger + gamepad1.right_trigger)

                    ));
                    heading = drive.getPoseEstimate().getHeading();
                }
                else {
                    heading2 = drive.getPoseEstimate().getHeading();
                    drive.setWeightedDrivePower(new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
0
                            //heading2 - targetAngle + poseEstimate.getHeading()

                    ));
                }

                break;
            case AUTOMATIC_CONTROL:

          drive.turn(Angle.normDelta(targetAngle - poseEstimate.getHeading()));
          if (targetAngle - poseEstimate.getHeading() !=0){
                        drive.setWeightedDrivePower(new Pose2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x,
                                (-gamepad1.left_trigger + gamepad1.right_trigger)

                        ));
                      //targetAngle = poseEstimate.getHeading();
                        currentMode = Mode.DRIVER_CONTROL;
                    }
                break;
        }
        if (gamepad1.x){
            currentMode = Mode.AUTOMATIC_CONTROL;
        }
        else if (gamepad1.left_stick_y !=0 || gamepad1.left_stick_x!=0 || (-gamepad1.left_trigger + gamepad1.right_trigger) !=0){
           currentMode = Mode.DRIVER_CONTROL;
        }

       // telemetry.addData("heading", poseEstimate.getHeading());
       // telemetry.addData("angle2",  Math.toDegrees(targetAngle));
        telemetry.addData("angle2",  Math.toDegrees(heading - heading2));
            telemetry.update();
        }
    }

