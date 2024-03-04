package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;

public class SpikeScorer extends Robot {
    Servo scor;

    public SpikeScorer(LinearOpMode opMode) {
        super(opMode);
        scor = hardwareMap.get(Servo.class, "Gram");
    }

    public void scor(){
        scor.setPosition(0);
    }
    public void teleop(){
        if (gamepad1.dpad_down){
            scor.setPosition(0);
        }
        if(gamepad1.dpad_up){
            scor.setPosition(1);
        }
    }

}
