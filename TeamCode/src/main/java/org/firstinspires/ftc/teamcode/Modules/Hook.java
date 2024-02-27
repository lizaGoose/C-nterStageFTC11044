package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hook {
    HardwareMap hardwareMap;

    LinearOpMode linearOpMode;

    Servo hook;

    public enum State{
        CLOSE,
        OPEN
    }
    public State stateHook = State.CLOSE;
    public  Hook(LinearOpMode linearOpMode){

        this.linearOpMode = linearOpMode;
        hardwareMap = linearOpMode.hardwareMap;
        hook = hardwareMap.get(Servo.class, "servoHook");
    }
    public void teleop(){
        switch (stateHook){
            case CLOSE:
                hook.setPosition(0.35);
                break;
            case OPEN:
                hook.setPosition(0.15);
                break;
        }
    }
}
