package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Sk0rMover{

    HardwareMap hardwareMap;
    LinearOpMode linearOpMode;

    ServoImplEx mover;

    public enum State{
        SET_RIGHT_POSITION,
        SET_LEFT_POSITION,
        SET_MIDDLE_POSITION
    }
    public State stateSkorMov = State.SET_MIDDLE_POSITION;
    public Sk0rMover(LinearOpMode linearOpMode){
        this.linearOpMode = linearOpMode;
        hardwareMap = linearOpMode.hardwareMap;
        mover = hardwareMap.get(ServoImplEx.class, "skorMover");
        mover.setPwmRange(new PwmControl.PwmRange(500, 2500));
    }
    public void teleop(){
        switch (stateSkorMov){
            case SET_LEFT_POSITION:
                mover.setPosition(0);
                break;
            case SET_RIGHT_POSITION:
                mover.setPosition(0.95);
                break;
            case SET_MIDDLE_POSITION:
                mover.setPosition(0.5);
                break;
        }
    }
}
