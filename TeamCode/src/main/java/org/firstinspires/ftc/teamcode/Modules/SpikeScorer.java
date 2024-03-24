package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.Robot;

public class SpikeScorer extends Robot {
    ServoImplEx scor;

    public SpikeScorer(LinearOpMode opMode) {
        super(opMode);
        scor = hardwareMap.get(ServoImplEx.class, "Gram");
        scor.setPwmRange(new PwmControl.PwmRange(500, 2600));

    }

    public void scor(){scor.setPosition(0.6);}
    public void teleop(){
        if(gamepad1.dpad_down){
            scor.setPosition(0.6);
        }
        if(gamepad1.dpad_up){
            scor.setPosition(0.9);
        }
    }

}
