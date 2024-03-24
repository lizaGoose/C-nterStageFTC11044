package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.Robot;

public class Modules extends Robot {
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

    double c = 0,timeIntake = 75, h = 0, checkIntake = 0, a = 1,t=0, setPos = 0, b = 0, d = 0, e = 0;
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
    public State state = State.STATE;
    public Modules(LinearOpMode opMode) {
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


        goose.teleop();
        shuter.teleop();
        skorMover.teleop();
        intake.teleop();
        intmov.teleop();
        virtual4bar.teleop();
        scorer.teleop();
        hook.teleop();
        wall.teleop();
        spsc.teleop();
        //lift.teleop();

        if (gamepad1.y) {
            state = State.SMART_BUTTON;
        }

        if (gamepad2.x){
            state = State.X;
        }

        if (gamepad2.y){
            setPos = lift.lift1.getCurrentPosition() + 70;
            state = State.Y;
        }
        if (state != State.B && state != State.Y){
            lift.teleop();
        }
        if (gamepad2.b){
            state = State.B;
        }
        if (gamepad2.a){
            state = State.A;
        }
        if (gamepad2.dpad_up){
            scorer.stateScorer = Sk0rer.State.OPEN_FRONT;
            state = State.STATE;
        }
        if (gamepad2.dpad_left){
            scorer.stateScorer = Sk0rer.State.OPEN_REAR;
            state = State.STATE;
        }
        if (gamepad2.right_bumper) {
            scorer.stateScorer = Sk0rer.State.CLOSE;
            skorMover.stateSkorMov = Sk0rMover.State.SET_LEFT_POSITION;
            state = State.STATE;
        }
        if (gamepad2.right_trigger > 0.3 ) {
            scorer.stateScorer = Sk0rer.State.CLOSE;
            skorMover.stateSkorMov = Sk0rMover.State.SET_RIGHT_POSITION;
            state = State.STATE;
        }
        if (gamepad1.b){
            state = State.SHOOT;
        }

        if (intmov.vidvizh.getCurrentPosition() >= -20 && gamepad1.right_stick_y == 0) {
            hook.stateHook = Hook.State.CLOSE;
        }
        else {
            hook.stateHook = Hook.State.OPEN;
        }


        if(gamepad1.left_bumper){
            if(intmov.stateIntMov != Intake_mover.State.GET_ZERO_VALUE) {
                intake.stateIntake = Intake.State.SET_MAX_POWER;
                intmov.stateIntMov = Intake_mover.State.SET_STICK_POWER;
            }
        }
        else if(gamepad1.right_bumper){

            if(intmov.stateIntMov != Intake_mover.State.GET_ZERO_VALUE) {
                intake.stateIntake = Intake.State.REVRSE;
                intmov.stateIntMov = Intake_mover.State.SET_STICK_POWER;
            }
        }
        else{

            intake.stateIntake = Intake.State.SET_ZERO_POWER;
            if(intmov.stateIntMov != Intake_mover.State.GET_ZERO_VALUE) {
                intmov.stateIntMov = Intake_mover.State.SET_STICK_POWER;
            }
        }

        if(intmov.blockExtend != 0 && d<10){
            intmov.stateIntMov = Intake_mover.State.SET_STICK_POWER;
        }
        else {
            if (intmov.vidvizh.getCurrentPosition() >= (intmov.position-300) && intmov.vidvizh.getCurrentPosition() <= intmov.position) {
                wall.stateWall = Wall.State.OPEN;
            } else if (intmov.vidvizh.getCurrentPosition() < (intmov.position-300)) {
                wall.stateWall = Wall.State.CLOSE;
            }

            if (intmov.vidvizh.getCurrentPosition() >= (intmov.position-10)/* && d<10*/) {
                if (gamepad1.right_stick_y > 0) {
                    intmov.stateIntMov = Intake_mover.State.SET_ZERO_POWER;
                } else if (gamepad1.right_stick_y < 0) {
                    intmov.stateIntMov = Intake_mover.State.SET_STICK_POWER;
                }
                else if(gamepad1.right_stick_y == 0){
                    intmov.stateIntMov = Intake_mover.State.SET_STICK_POWER;
                }
            } else {
                if(intmov.stateIntMov != Intake_mover.State.GET_ZERO_VALUE) {
                    intmov.stateIntMov = Intake_mover.State.SET_STICK_POWER;
                }
            }

            if (intmov.vidvizh.getCurrentPosition() <= (intmov.position-2100)/* && d<10*/) {
                if (gamepad1.right_stick_y < 0) {
                    intmov.stateIntMov = Intake_mover.State.SET_ZERO_POWER;
                } else if (gamepad1.right_stick_y > 0) {
                    intmov.stateIntMov = Intake_mover.State.SET_STICK_POWER;
                }
                else if(gamepad1.right_stick_y == 0){
                    intmov.stateIntMov = Intake_mover.State.SET_STICK_POWER;
                }
            } else {
                if(intmov.stateIntMov != Intake_mover.State.GET_ZERO_VALUE) {
                    intmov.stateIntMov = Intake_mover.State.SET_STICK_POWER;
                }
            }
        }



        if (lineSensor2.getState() != true && gamepad1.left_bumper){
            d+=1;
        }
        if (d >= 10){
            intmov.stateIntMov = Intake_mover.State.GET_ZERO_VALUE;
            intake.stateIntake = Intake.State.SET_ZERO_POWER;
            d = 0;
        }
        if (d==0 && (gamepad1.right_stick_y !=0||intmov.vidvizh.getCurrentPosition() >=intmov.position)){
            intmov.stateIntMov = Intake_mover.State.SET_STICK_POWER;
        }


        switch (state){
            case STATE:
                virtual4bar.stateV4b = Virtual4bar.State.DO_NOTHING;
                t = 0;
                b = 0;
                break;
            case SMART_BUTTON:
                c+=1;
                if (c< 20){
                    scorer.stateScorer = Sk0rer.State.OPEN;
                    skorMover.stateSkorMov = Sk0rMover.State.SET_MIDDLE_POSITION;
                }
                else if(c > 20 && c < 40){
                    virtual4bar.stateV4b = Virtual4bar.State.SET_DOWN_POSITION;
                }

                if(timeIntake>0){
                    intmov.stateIntMov = Intake_mover.State.SET_STICK_POWER;
                    timeIntake +=1;

                    if (timeIntake < 20){
                        intake.stateIntake = Intake.State.SET_MAX_POWER;
                    }
                    else {
                        intake.stateIntake = Intake.State.SET_ZERO_POWER;
                        timeIntake = 0;
                        h = 0;
                    }
                }
                if(lineSensor2.getState() != true){
                    checkIntake +=1;
                    intmov.stateIntMov = Intake_mover.State.SET_STICK_POWER;
                }

                if (checkIntake ==0){
                    intmov.stateIntMov = Intake_mover.State.SET_STICK_POWER;
                    intake.stateIntake = Intake.State.SET_MAX_POWER;
                }
                else if (checkIntake > 10){

                    if (intmov.vidvizh.getCurrentPosition() < -200 && gamepad1.right_stick_y == 0){
                        intmov.vidvizh.setPower(1);
                        intake.stateIntake = Intake.State.SET_ZERO_POWER;
                    }
                    else if ( intmov.vidvizh.getCurrentPosition() > -200 && intmov.vidvizh.getCurrentPosition() < 0 && gamepad1.right_stick_y == 0){
                        wall.stateWall = Wall.State.OPEN;
                        intmov.stateIntMov = Intake_mover.State.GET_ZERO_VALUE;
                        intake.stateIntake = Intake.State.SET_ZERO_POWER;

                    }
                    else{
                        intmov.stateIntMov = Intake_mover.State.SET_STICK_POWER;
                        if(lineSensor2.getState() == true){
                            checkIntake = 0;
                            c = 0;
                            timeIntake += 1;
                        }
                        else {
                            intake.stateIntake = Intake.State.SET_MAX_POWER;
                        }
                    }
                }
                break;
            case B:
                a+=1;
                scorer.stateScorer = Sk0rer.State.OPEN;
                skorMover.stateSkorMov = Sk0rMover.State.SET_MIDDLE_POSITION;
                if(a > 10){
                    if(lift.lift1.getCurrentPosition() > lift.pos1){
                        lift.lift1.setPower(-0.5);
                        lift.lift2.setPower(-0.5);
                    }
                    else {
                        virtual4bar.stateV4b = Virtual4bar.State.SET_DOWN_POSITION;
                        lift.position = lift.lift1.getCurrentPosition();
                    a=0;
                        state = State.STATE;
                    }

                }

                break;
            case Y:
                b+=1;
                scorer.stateScorer = Sk0rer.State.OPEN;
                if(b > 12) {
                    if (lift.lift1.getCurrentPosition() < setPos) {
                        lift.lift1.setPower(0.8);
                        lift.lift2.setPower(0.8);
                    } else {
                        lift.position = lift.lift1.getCurrentPosition();
                        state = State.STATE;
                    }
                }
                break;
            case A:
                scorer.stateScorer = Sk0rer.State.CLOSE;
                break;
            case X:
                e+=1;
                scorer.stateScorer = Sk0rer.State.CLOSE;
                skorMover.stateSkorMov = Sk0rMover.State.SET_MIDDLE_POSITION;
                if(e>8) {
                    virtual4bar.stateV4b = Virtual4bar.State.SET_UP_POSITION;
                    e = 0;
                }
                break;
            case SHOOT:
                t+=1;
                goose.state = Goose.State.SHOOT;
                if(t>20){
                    shuter.stateShuter = Shuter.State.CLOSE;
                    state = State.STATE;
                }
                break;
        }
    }
}
