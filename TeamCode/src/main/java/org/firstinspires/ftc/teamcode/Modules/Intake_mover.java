package org.firstinspires.ftc.teamcode.Modules;

import android.hardware.Sensor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;

public class Intake_mover {
    LinearOpMode linearOpMode;
    HardwareMap hardwareMap;
    DcMotor vidvizh;

    double position = 0, blockExtend = 0;
   public enum State{
       SET_STICK_POWER,
       SET_ZERO_POWER,
       GET_ZERO_VALUE
   }

   public State stateIntMov = State.GET_ZERO_VALUE;
    public Intake_mover(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
        hardwareMap = linearOpMode.hardwareMap;
        vidvizh = hardwareMap.get(DcMotor.class, "vidvizhenie_zahvata");
        vidvizh.setDirection(DcMotorSimple.Direction.FORWARD);
        vidvizh.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vidvizh.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void teleop() {
        if (linearOpMode.gamepad1.dpad_right){
            position = vidvizh.getCurrentPosition();
            blockExtend = 0;
        }
        if (linearOpMode.gamepad1.dpad_left){
            blockExtend +=1;
        }

       switch (stateIntMov){
           case GET_ZERO_VALUE:
               if (vidvizh.getCurrentPosition() < -0){
                   vidvizh.setPower(1);
               }
               else {
                   vidvizh.setPower(0);
               }
               break;
           case SET_ZERO_POWER:
               vidvizh.setPower(0);
               break;
           case SET_STICK_POWER:
               vidvizh.setPower(linearOpMode.gamepad1.right_stick_y);
               break;
       }
    }


}
