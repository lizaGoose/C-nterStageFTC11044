package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Robot1;

public class V4bMover extends Robot {
    Servo zahvat_perekid;
   Scorer scor;
   Virtual4bar v4b;
   Robot1 R;
   Goose goose;
   Intake intake;



   Lift lift;
    double n = 150, k = 10, lopata_stopper = 1, lopata_stopper_intake = 1;
Intake_mover intmov;
   ElapsedTime t;
    public enum State_Intake{
        NOTHING,
        GET_VALUE
    }
    public State_Intake state = State_Intake.NOTHING;


    double kp = 0.2;

    Intake_mover intakeMover;
    public V4bMover(LinearOpMode opMode) {
        super(opMode);
        //zahvat_perekid = hardwareMap.get(Servo.class, "zahvat_perekid");
        scor = new Scorer(opMode);
       intmov = new Intake_mover(opMode);
       intake = new Intake(opMode);

    }
    public void teleop(){
        if(intmov.vidvizh.getCurrentPosition() < -300){
            lopata_stopper_intake = 0;
        }
        else {
            lopata_stopper_intake = 1;
        }
            if (gamepad2.dpad_left){
                lopata_stopper = 0;
            }
            if(gamepad2.dpad_right){
                lopata_stopper = 1;
            }
            if (gamepad2.dpad_up){
                n = 0;
                lopata_stopper = 1;
            }


        n += 1;
        k += 1;
        if(n < 5) {
            intmov.vidvizh.setPower(1);

            if(intmov.vidvizh.getCurrentPosition() < 30) {
                intmov.vidvizh.setPower(0.6);
            }
        } else if (5 < n && n < 50) {
            scor.scorer.setPosition(0.57);
          //  zahvat_perekid.setPosition(0.65);
        } else if(50 < n && n < 65) {
            intmov.vidvizh.setPower(0.6);
            intmov.vidvizh.setPower(1);
           // zahvat_perekid.setPosition(0);

        }
        if(lopata_stopper == 1 && lopata_stopper_intake == 1){
            if(gamepad2.x){
                k = 0;
            }
        }
        else {
            k = 10;
        }
        if(k<10){
           // zahvat_perekid.setPosition(0.65);
        }
        else if(k > 10 && k < 12) {
            //zahvat_perekid.setPosition(0);
        }

        else if ((lopata_stopper == 0 && lopata_stopper_intake == 0)||((gamepad2.left_trigger - gamepad2.right_trigger)* 0.7 !=0)){
           // zahvat_perekid.setPosition(0);
        }

        /*if(gamepad2.right_stick_y > 0){
            zahvat_perekid.setPosition(0.3);
        }*/

       /* telemetry.addData("time", n);
        telemetry.addData("intake_mover", intmov.vidvizh.getCurrentPosition());
        telemetry.addData("lopata", lopata_stopper);
        telemetry.update();*/

    }

}
