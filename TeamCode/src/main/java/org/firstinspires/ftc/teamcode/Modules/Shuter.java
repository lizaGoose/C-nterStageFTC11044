package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;

public class Shuter{
    HardwareMap hardwareMap;

    LinearOpMode linearOpMode;
    Servo shuter;

    public enum State{
        SHOOT,
        CLOSE,
        DO_NOTHING
    }

    public State stateShuter = State.DO_NOTHING;

    public Shuter(LinearOpMode linearOpMode) {

        this.linearOpMode = linearOpMode;
        hardwareMap = linearOpMode.hardwareMap;
        shuter = hardwareMap.get(Servo.class, "Shuter");
    }

    public void teleop() {
      /*  if (linearOpMode.gamepad1.x){
            stateShuter = State.SHOOT;
        }*/

        switch (stateShuter){
            case DO_NOTHING:
                break;
            case CLOSE:
                shuter.setPosition(0);
                break;
            case SHOOT:
                ElapsedTime t = new ElapsedTime();
                if(t.milliseconds() < 100) {
                    shuter.setPosition(0.3);
                }
                else{
                    shuter.setPosition(0.3);
                }
                break;
        }
    }
}
