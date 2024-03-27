package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ExtendAuto {
    HardwareMap hardwareMap;

    LinearOpMode linearOpMode;

    DcMotor vidvizh;

    Servo hook;

    public double errorExtend, pos1 = 0, blockLift = 0, kp = 0.008;
    public double setPositionExtend  = 0;

    public ExtendAuto(LinearOpMode linearOpMode){
        this.linearOpMode = linearOpMode;
        hardwareMap = linearOpMode.hardwareMap;

        vidvizh = hardwareMap.get(DcMotor.class, "vidvizhenie_zahvata");
        vidvizh.setDirection(DcMotorSimple.Direction.FORWARD);
        vidvizh.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vidvizh.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardwareMap = linearOpMode.hardwareMap;
        hook = hardwareMap.get(Servo.class, "servoHook");
    }
    public int GetPose(int thePositionExtend){setPositionExtend = thePositionExtend;
        return thePositionExtend;
    }

    public void Auto(){
        errorExtend = setPositionExtend - vidvizh.getCurrentPosition();

        vidvizh.setPower(errorExtend*kp);
       // hook.setPosition(0.35);
    }
}