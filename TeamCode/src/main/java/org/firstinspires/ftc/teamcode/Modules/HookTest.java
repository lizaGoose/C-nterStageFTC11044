package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HookTest {
    HardwareMap hardwareMap;

    LinearOpMode linearOpMode;

    Servo hook;

    public enum State{
        CLOSE,
        OPEN
    }
    public State stateHook = State.CLOSE;
    public  HookTest(LinearOpMode linearOpMode){

        this.linearOpMode = linearOpMode;
        hardwareMap = linearOpMode.hardwareMap;
        hook = hardwareMap.get(Servo.class, "servoHook");
    }
    public void teleop(){

        if(linearOpMode.gamepad2.left_stick_y > 0){
            stateHook = State.OPEN;
        }
        if(linearOpMode.gamepad2.left_stick_y < 0){
           stateHook = State.CLOSE;
        }
        switch (stateHook){
            case CLOSE:
                hook.setPosition(0.6);
                break;
            case OPEN:
                hook.setPosition(0.38);
                break;
        }
    }
}
