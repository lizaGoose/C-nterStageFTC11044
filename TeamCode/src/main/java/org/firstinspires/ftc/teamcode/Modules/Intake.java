package org.firstinspires.ftc.teamcode.Modules;

import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;

public class Intake{
    HardwareMap hardwareMap;

    LinearOpMode linearOpMode;

    DcMotor intake;

    public enum State{
        SET_ZERO_POWER,
        SET_MAX_POWER,
        REVRSE
    }
    public State stateIntake = State.SET_ZERO_POWER;
    public Intake(LinearOpMode linearOpMode){
        this.linearOpMode = linearOpMode;
        hardwareMap = linearOpMode.hardwareMap;
        intake = hardwareMap.get(DcMotor.class, "zahvat");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void teleop(){
        switch (stateIntake){
            case REVRSE:
                intake.setPower(-1);
                break;
            case SET_MAX_POWER:
                intake.setPower(1);
                break;
            case SET_ZERO_POWER:
                intake.setPower(0);
                break;
        }
    }
}

