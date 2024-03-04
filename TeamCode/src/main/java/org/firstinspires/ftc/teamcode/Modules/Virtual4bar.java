package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Virtual4bar{

    HardwareMap hardwareMap;

    LinearOpMode linearOpMode;

    Servo perekid1, perekid2;

    public enum State{
        SET_DOWN_POSITION,
        SET_UP_POSITION,
        DO_NOTHING
    }
    public State stateV4b = State.DO_NOTHING;
   public Virtual4bar(LinearOpMode linearOpMode){
       this.linearOpMode = linearOpMode;
       hardwareMap = linearOpMode.hardwareMap;
       perekid1 = hardwareMap.get(Servo.class, "perekid1");
       perekid2 = hardwareMap.get(Servo.class, "perekid2");
   }

   public void teleop(){
       switch (stateV4b){
           case SET_UP_POSITION:
               perekid2.setPosition(0);
               perekid1.setPosition(1);
               break;
           case SET_DOWN_POSITION:
               perekid2.setPosition(1);
               perekid1.setPosition(0);
               break;
           case DO_NOTHING:
               break;
       }
   }
}
