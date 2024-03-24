package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Sk0rer{

    HardwareMap hardwareMap;

    LinearOpMode linearOpMode;

    Servo scorer, scorer1;

    public enum State{
        OPEN,
        OPEN_FRONT,
        OPEN_REAR,
        CLOSE
    }
    public State stateScorer = State.OPEN;
    public Sk0rer(LinearOpMode linearOpMode){

        this.linearOpMode = linearOpMode;
        hardwareMap = linearOpMode.hardwareMap;
        scorer = hardwareMap.get(Servo.class, "skorer");
        scorer1 = hardwareMap.get(Servo.class, "skorer1");

    }
    public void teleop(){

       switch (stateScorer){

           case OPEN:
               scorer.setPosition(0.15);
               scorer1.setPosition(0.55);
               break;
           case OPEN_REAR:
               scorer.setPosition(0.15);
               break;
           case OPEN_FRONT:
               scorer1.setPosition(0.55);
               break;
           case CLOSE:
               scorer.setPosition(0.6);
               scorer1.setPosition(0);
               break;
       }
    }
}
