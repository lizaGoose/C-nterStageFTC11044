package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.Modules.Goose;
import org.firstinspires.ftc.teamcode.Modules.Hook;
import org.firstinspires.ftc.teamcode.Modules.Intake;
import org.firstinspires.ftc.teamcode.Modules.Intake_mover;
import org.firstinspires.ftc.teamcode.Modules.L1ft;
import org.firstinspires.ftc.teamcode.Modules.Shuter;
import org.firstinspires.ftc.teamcode.Modules.Sk0rMover;
import org.firstinspires.ftc.teamcode.Modules.Sk0rer;
import org.firstinspires.ftc.teamcode.Modules.SpikeScorer;
import org.firstinspires.ftc.teamcode.Modules.Virtual4bar;
import org.firstinspires.ftc.teamcode.Modules.Wall;
import org.firstinspires.ftc.teamcode.Robot;

public class TestIntake extends Robot {
    Goose goose;
    Intake intake;
    Intake_mover intmov;

    Virtual4bar virtual4bar;

    Sk0rMover skorMover;

    L1ft lift;

    Sk0rer scorer;

    Shuter shuter;

    Hook hook;

    Wall wall;

    SpikeScorer spsc;

    double c = 0,timeIntake = 75, h = 0, checkIntake = 0, a = 1,t=0, setPos = 0, b = 0, d = 0;
    DigitalChannel lineSensor2;
    public enum State{
        STATE,
        SMART_BUTTON,
        B,
        Y,
        A,
        X,
        SHOOT

    }
    public org.firstinspires.ftc.teamcode.Modules.Modules.State state = org.firstinspires.ftc.teamcode.Modules.Modules.State.STATE;
    public TestIntake(LinearOpMode opMode) {
        super(opMode);
        goose = new Goose(opMode);
        intake = new Intake(opMode);
        intmov = new Intake_mover(opMode);
        lift = new L1ft(opMode);
        virtual4bar = new Virtual4bar(opMode);
        skorMover = new Sk0rMover(opMode);
        scorer = new Sk0rer(opMode);
        shuter = new Shuter(opMode);
        hook = new Hook(opMode);
        wall = new Wall(opMode);
        spsc = new SpikeScorer(opMode);
        lineSensor2 = hardwareMap.get(DigitalChannel.class, "line_digital2");
        lineSensor2.setMode(DigitalChannel.Mode.INPUT);

    }
    public void teleop(){
        if (lineSensor2.getState() != true) {
            telemetry.addLine("first is Pressed");
        } else {
            telemetry.addLine("first is not Pressed");
        }
        telemetry.addData("mov", intmov.vidvizh.getCurrentPosition());
        telemetry.addData("lift", lift.lift1.getCurrentPosition());
        //telemetry.addData("angle", robot.targetAngle - robot.drive.getPoseEstimate().getHeading());
        telemetry.update();


        intmov.teleop();

        if(gamepad1.dpad_right){
            intmov.stateIntMov = Intake_mover.State.GET_ZERO_VALUE;
        }
        if (gamepad1.right_stick_y !=0){
            intmov.stateIntMov = Intake_mover.State.SET_STICK_POWER;
        }

    }
}