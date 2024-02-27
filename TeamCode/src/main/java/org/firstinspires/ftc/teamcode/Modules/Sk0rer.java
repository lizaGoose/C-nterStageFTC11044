package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Sk0rer{

    HardwareMap hardwareMap;

    LinearOpMode linearOpMode;

    Servo scorer;

    public enum State{
        OPEN,
        CLOSE
    }
    public State stateScorer = State.OPEN;
    public Sk0rer(LinearOpMode linearOpMode){

        this.linearOpMode = linearOpMode;
        hardwareMap = linearOpMode.hardwareMap;
        scorer = hardwareMap.get(Servo.class, "skorer");
    }
    public void teleop(){

       switch (stateScorer){
           case OPEN:
               scorer.setPosition(0.3);
               break;
           case CLOSE:
               scorer.setPosition(0.5);
               break;
       }
    }
}
