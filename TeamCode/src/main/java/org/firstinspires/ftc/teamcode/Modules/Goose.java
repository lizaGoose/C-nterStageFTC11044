package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;

public class Goose {
    HardwareMap hardwareMap;
    LinearOpMode linearOpMode;

    Gamepad gamepad1;
    Servo podves1;
    Servo podves2;
    double pd11 = 0, pd12 = 1, e = 50, pd21 = 0, pd22 = 0.75;
    public enum State{
        SHOOT,
        PUT_UP,
        PUT_DOWN,
        EMPTY
    }
    public State state = State.EMPTY;
    public Goose(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
        hardwareMap = linearOpMode.hardwareMap;
        gamepad1 = linearOpMode.gamepad1;
        podves1 = hardwareMap.get(Servo.class, "podves1");
        podves2 = hardwareMap.get(Servo.class, "podves2");
    }
    public void teleop(){

        if (gamepad1.dpad_down) {
            state = State.PUT_DOWN;
        }

        if (gamepad1.dpad_up) {
            state = State.PUT_UP;
        }



        switch (state){
            case EMPTY:
                break;
            case SHOOT:
                podves2.setPosition(0.4);
                break;
            case PUT_UP:
                podves1.setPosition(pd11);
                podves2.setPosition(pd22);
                break;
            case PUT_DOWN:
                podves1.setPosition(pd12);
                podves2.setPosition(pd21);
                break;
        }
    }

}
