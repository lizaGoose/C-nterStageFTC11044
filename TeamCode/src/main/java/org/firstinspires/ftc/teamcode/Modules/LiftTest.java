package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LiftTest {
    HardwareMap hardwareMap;

    LinearOpMode linearOpMode;

    DcMotor lift1, lift2;

    public double position, error, pos1 = 0, blockLift = 0;

    public enum State{
        SET_GAMEPAD_POWER,
        SET_ZERO_POWER,
        SET_ZERO_VALUE,
        HOLD_POSITION,
        DO_NOTHING
    }
    public State stateLift = State.HOLD_POSITION;
    public LiftTest(LinearOpMode linearOpMode){
        this.linearOpMode = linearOpMode;
        hardwareMap = linearOpMode.hardwareMap;
        lift1 = hardwareMap.get(DcMotor.class, "lift1");
        lift1.setDirection(DcMotorSimple.Direction.FORWARD);
        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //lift

        lift2 = hardwareMap.get(DcMotor.class, "lift2");
        lift2.setDirection(DcMotorSimple.Direction.REVERSE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void teleop(){
        lift1.setPower(linearOpMode.gamepad2.right_stick_y);
        lift2.setPower(linearOpMode.gamepad2.left_stick_y);
    }
}
