package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;

public class SkorMover extends Robot {
    Servo mover, scor;
    Virtual4bar v4b;
    double sc1 = 0.35, sc2 = 0.6;
    double t = 0;

    public SkorMover(LinearOpMode opMode) {
        super(opMode);
        mover = hardwareMap.get(Servo.class, "skorMover");
        scor = hardwareMap.get(Servo.class, "skorer");
        v4b = new Virtual4bar(opMode);
    }

    public void teleop() {

        if (gamepad2.right_bumper) {
            t += 1;
          //  scor.setPosition(0);
         //   mover.setPosition(0.73);
            scor.setPosition(0.5);
           mover.setPosition(1);
        }
        if (gamepad2.right_trigger > 0.3 ) {
            t += 1;
            // t = 10;
           // scor.setPosition(0.4);
            scor.setPosition(0.5);
            mover.setPosition(0.35);

           // mover.setPosition(0.08);
        }
        if (gamepad2.right_stick_y > 0 || gamepad2.right_stick_y < 0){
            t = 0;
        }
      /*  if(gamepad*2.x){
            t += 1;
            scor.setPosition(0.5);
        }*/
       /* if(t == 0){

            mover.setPosition(0.68);
        }*/


    }
}