package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;

public class Wall extends Robot {
    Servo wall;

    public enum State{
        OPEN,
        CLOSE
    }
    public State stateWall = State.OPEN;
    public Wall(LinearOpMode opMode) {
        super(opMode);
        wall = hardwareMap.get(Servo.class, "servoWall");
    }
    public  void teleop(){
      switch (stateWall){
          case OPEN:
              wall.setPosition(0.7);
              break;
          case CLOSE:
              wall.setPosition(0.2);
              break;
      }
    }
}
