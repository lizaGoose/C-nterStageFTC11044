package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class L1ft {
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
    public L1ft(LinearOpMode linearOpMode){
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
        if(linearOpMode.gamepad2.right_stick_y != 0){
            position = lift1.getCurrentPosition();
        }
        if (linearOpMode.gamepad2.dpad_right) {
            pos1 = lift1.getCurrentPosition();
            blockLift = 0;
        }
        if (linearOpMode.gamepad2.dpad_down){
            blockLift += 1;
        }
        if(blockLift > 0){
            stateLift = State.SET_GAMEPAD_POWER;
        }
        else if (blockLift == 0) {
            if (lift1.getCurrentPosition() <= pos1 && linearOpMode.gamepad2.right_stick_y > 0) {
                stateLift = State.SET_ZERO_POWER;
            } else {
                if (linearOpMode.gamepad2.right_stick_y == 0) {
                    stateLift = State.HOLD_POSITION;
                } else {
                    stateLift = State.SET_GAMEPAD_POWER;
                }
            }
        }


        switch (stateLift){
            case SET_ZERO_POWER:
                lift1.setPower(0);
                lift2.setPower(0);
                break;
            case SET_ZERO_VALUE:
                if(lift1.getCurrentPosition() > pos1){
                    lift1.setPower(-0.5);
                    lift2.setPower(-0.5);
                }
                else {
                    lift1.setPower(0);
                    lift2.setPower(0);
                }
                break;
            case SET_GAMEPAD_POWER:
                lift2.setPower(-linearOpMode.gamepad2.right_stick_y * 0.8);
                lift1.setPower(-linearOpMode.gamepad2.right_stick_y * 0.8);
                break;
            case HOLD_POSITION:
                error = position - lift1.getCurrentPosition();
                lift1.setPower(error * 0.005);
                lift2.setPower(error * 0.005);
                break;
                case DO_NOTHING:
                break;
        }
    }
}
