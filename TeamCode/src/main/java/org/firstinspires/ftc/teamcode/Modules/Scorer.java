package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;

public class Scorer extends Robot {
    Servo scorer, mover, perekid1, perekid2, scor;
    DcMotor lift1, lift2;
    Virtual4bar v4b;
    double sc1 = 0.35, sc2 = 0.6;
    double t = 30;

    public Scorer(LinearOpMode opMode) {
        super(opMode);
        scorer = hardwareMap.get(Servo.class, "skorer");
        perekid1 = hardwareMap.get(Servo.class, "perekid1");
        perekid2 = hardwareMap.get(Servo.class, "perekid2");//left
        mover = hardwareMap.get(Servo.class, "skorMover");
        scor = hardwareMap.get(Servo.class, "skorer");
        v4b = new Virtual4bar(opMode);
        lift1 = hardwareMap.get(DcMotor.class, "lift1");
        lift1.setDirection(DcMotorSimple.Direction.FORWARD);
        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //lift

        lift2 = hardwareMap.get(DcMotor.class, "lift2");
        lift2.setDirection(DcMotorSimple.Direction.REVERSE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void teleop() {
        t += 1;
        if (gamepad2.y) {
            scorer.setPosition(0.2);
        }
        if (gamepad2.a||gamepad2.x) {
            t = 0;
            // t = 10;
            scorer.setPosition(0.5);
        }

       /* if(t > 20 && t < 26) {
            double pos = lift1.getCurrentPosition() + 500;

            double error3 = pos - lift1.getCurrentPosition();
            if (error3 > 0) {
                //mover.setPosition(0.39);
                lift1.setPower(error3 * 0.4);
                lift2.setPower(error3 * 0.4);
            }
            else{*/
                // mover.setPosition(0.39);
           // }
            //perekid2.setPosition(0.98);
            //perekid1.setPosition(0.02);
       // }
        if (gamepad2.dpad_up) {
            scorer.setPosition(0);
        }
        if (gamepad2.dpad_down) {
            scorer.setPosition(0.4);
        }

      /*
        }*/

        /*if ( 20 < t && t < 22){
            scorer.setPosition(0.57);
            v4b.perekid2.setPosition(0.12);
            v4b.perekid1.setPosition(0.88);
        }*/
       /* if (gamepad2.left_trigger > 0){
            scorer.setPosition(0.52);
        }*/

    }
    public void scorerka(double a, double b){
        scorer.setPosition(a);
        opMode.sleep(500);
        scorer.setPosition(b);
    }
    public void perekidka(double a){
        v4b.perekid1.setPosition(a);
        v4b.perekid2.setPosition(1 - a);
    }
    public void scorStart(){
        scorer.setPosition(0.57);
    }
}
