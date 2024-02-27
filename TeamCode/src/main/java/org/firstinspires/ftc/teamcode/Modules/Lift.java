package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Lift extends Robot {
    DcMotor lift1, lift2;
    //Virtual4bar v4b;
    double pos1 = 0, pos2 = 300, a = 0, b = 0, pos3 = 300, c =0, d = 0, e = 0, f = 0, g =0, kpl = 0.005, posStatic =0, error2 = 0;
    Servo scorer,perekid1, perekid2, mover;

   // ArrayList posStatic = new ArrayList<>();

    //IntakeSecondVersion intake;

    double kp = 0.2;

    public Lift(LinearOpMode opMode) {
        super(opMode);
        //v4b = new Virtual4bar(opMode);
      //  posStatic.add(0);
        lift1 = hardwareMap.get(DcMotor.class, "lift1");
        lift1.setDirection(DcMotorSimple.Direction.FORWARD);
        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //lift

        lift2 = hardwareMap.get(DcMotor.class, "lift2");
        lift2.setDirection(DcMotorSimple.Direction.REVERSE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        scorer = hardwareMap.get(Servo.class, "skorer");
        mover = hardwareMap.get(Servo.class, "skorMover");

        perekid1 = hardwareMap.get(Servo.class, "perekid1");
        perekid2 = hardwareMap.get(Servo.class, "perekid2");

        //intake = new IntakeSecondVersion(opMode);
    }

    public void teleop() {
        if (e == 0){
            pos1 = lift1.getCurrentPosition();
            e +=1;
        }
        if (gamepad2.dpad_right) {
            pos1 = lift1.getCurrentPosition();
            d = 0;

        }
        if (gamepad1.dpad_right || gamepad1.dpad_left){
            d += 1;
            f +=1;
        }
        if (gamepad2.dpad_down) {
            d += 1;
        }

        if(f == 0) {
            if (d == 0) {
                if (lift1.getCurrentPosition() >= pos1) {
                    lift1.setPower(-gamepad2.right_stick_y * 0.8);
                    lift2.setPower(-gamepad2.right_stick_y * 0.8);
                    if (gamepad2.left_stick_y > 0) {
                        //v4b.perekid1.setPosition(v4b.pr12);
                        //v4b.perekid2.setPosition(1 - v4b.pr12);
                    }
                } else if (lift1.getCurrentPosition() < pos1 && gamepad2.right_stick_y > 0) {
                    lift1.setPower(0);
                    lift2.setPower(0);
                } else if (lift1.getCurrentPosition() < pos1 && gamepad2.right_stick_y <= 0) {
                    lift1.setPower(-gamepad2.right_stick_y * 0.8);
                    lift2.setPower(-gamepad2.right_stick_y * 0.8);
                }
            } else {
                if (gamepad2.right_stick_y == 0){
                    //g = 0;
                }
                else {
                    lift1.setPower(-gamepad2.right_stick_y * 0.8);
                    lift2.setPower(-gamepad2.right_stick_y * 0.8);
                }
            }
        }
        if (gamepad2.right_stick_y == 0 && c == 0){
            error2 = posStatic - lift1.getCurrentPosition();
            if (error2 >= 0) {
                lift1.setPower(error2 * kpl);
                lift2.setPower(error2 * kpl);
            }
            //g = 0;
        }
        if (gamepad2.right_stick_y !=0){
            posStatic  = lift1.getCurrentPosition();
        }
      /*  else {
            intake.teleop();
        }*/
        if (gamepad2.y || gamepad1.y) {
            a += 1;

        }
        if (gamepad2.x) {
            b += 1;
        }

        if (gamepad2.b) {
            c += 1;
        }
        if (c > 0) {
            c+=1;
            if (c< 10){
                scorer.setPosition(0.5);
                mover.setPosition(0.68);
            }
            else{
                double error = (pos1 + 60) - lift1.getCurrentPosition();
                if (error <= 0) {
                    lift1.setPower(error * kp);
                    lift2.setPower(error * kp);
                } else {
                    posStatic = lift1.getCurrentPosition();
                   //pos1 = lift1.getCurrentPosition();
                    scorer.setPosition(0.2);
                    perekid2.setPosition(0);
                    perekid1.setPosition(1);
                    c = 0;
                }
            }
        }

        if (b > 0) {
            b += 1;
            // if (b < 300) {
            scorer.setPosition(0.5);
            mover.setPosition(0.68);
            // } else {
                /*double error2 = pos3 - lift1.getCurrentPosition();
                if (error2 > 0) {
                    lift1.setPower(error2 * kp);
                    lift2.setPower(error2 * kp);
                } else {*/
            perekid2.setPosition(1);
            perekid1.setPosition(0);
            b = 0;
            // }
            //}
            //}
            //   }

        /*telemetry.addData("lift1", lift1.getCurrentPosition());
            telemetry.addData("lift2", lift2.getCurrentPosition());
            telemetry.update();*/
        }
    }
}