package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;

public class IntakeSecondVersion extends Robot {
    CRServo intake;

    Servo perekid1, perekid2, scor, wall, hook, scor1;

    ServoImplEx mover;
    Intake intk;
    public boolean PixeIsIn = false;
    Lift lift;

    Gamepad gamepad1;
    DcMotor zahvat,lift1, lift2;;

    double n = 50, k = 0, d = 0, e = 0, t = 0, z = 0, y = 0, a = 0, b = 0, pos2 = 400, pos3 = 200,kp = 0.8, c = 0, posInit = 0, off = 0,
            on = 0, hookUp = 0, hookDown = 0, checkIntake = 0, timeIntake = 75, f = 0, g = 0, h = 0;
    DigitalChannel lineSensor, lineSensor2;
    DcMotor vidvizh;

    public IntakeSecondVersion(LinearOpMode opMode) {
        super(opMode);
        Lift lifts = new Lift(opMode);
        gamepad1 = opMode.gamepad1;
        zahvat = hardwareMap.get(DcMotor.class, "zahvat");
        zahvat.setDirection(DcMotorSimple.Direction.FORWARD);
        zahvat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        zahvat.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lineSensor = hardwareMap.get(DigitalChannel.class, "line_digital");
        lineSensor2 = hardwareMap.get(DigitalChannel.class, "line_digital2");
        lineSensor.setMode(DigitalChannel.Mode.INPUT);
        lineSensor2.setMode(DigitalChannel.Mode.INPUT);
        vidvizh = hardwareMap.get(DcMotor.class, "vidvizhenie_zahvata");
        vidvizh.setDirection(DcMotorSimple.Direction.FORWARD);
        vidvizh.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vidvizh.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        perekid1 = hardwareMap.get(Servo.class, "perekid1");
        perekid2 = hardwareMap.get(Servo.class, "perekid2");//left
        mover = hardwareMap.get(ServoImplEx.class, "skorMover");
        mover.setPwmRange(new PwmControl.PwmRange(500, 2500));
        scor = hardwareMap.get(Servo.class, "skorer");
        scor1 = hardwareMap.get(Servo.class, "skorer1");
        lift1 = hardwareMap.get(DcMotor.class, "lift1");
        lift1.setDirection(DcMotorSimple.Direction.FORWARD);
        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //lift

        lift2 = hardwareMap.get(DcMotor.class, "lift2");
        lift2.setDirection(DcMotorSimple.Direction.REVERSE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift = new Lift(opMode);

        wall = hardwareMap.get(Servo.class, "servoWall");
        hook = hardwareMap.get(Servo.class, "servoHook");
        // intk = new Intake(opMode);
    }

    public void Sensor() {

       /* if(lineSensor2.getState() != true){
            t+=1;
            z = 0;
        }
        if(gamepad2.x){
            z+=1;
        }
        if(t>0 && z == 0){
                intake.setPower(-1);
        }
       else if(z > 0){
           t = 0;
           z+=1;
           if (z < 100) {
               intake.setPower(-1);
               zahvat.setPower(0);
           }
           else {
               zahvat.setPower(0);
              intake.setPower(0);
              n = 0;
              z = 0;
           }
       }*/


        /*if (lineSensor.getState() != true && lineSensor2.getState() != true){
            e+=1;
            if (e < 5) {
                intake.setPower(-1);
            }
            else if ( 6< e && e <12){
                intake.setPower(1);
            }
           else {
                intake.setPower(0);
                zahvat.setPower(0);
                if(vidvizh.getCurrentPosition() < -80){
                    vidvizh.setPower(1);
                }

            }
        }
        if(e > 12){
             if(vidvizh.getCurrentPosition() > -80 || gamepad1.dpad_left) {
                if (lineSensor.getState() != true || lineSensor2.getState() != true) {
                    intake.setPower(-1);
                }
            }
        }
        if (lineSensor2.getState() == true && lineSensor.getState() == true) {
            e = 0;
            n = 0;
            intake.setPower(0);
        }*/
    }

    public void teleop() {
        if (lineSensor2.getState() != true) {
            telemetry.addLine("first is Pressed");
        } else {
            telemetry.addLine("first is not Pressed");
        }
        if (lineSensor.getState() != true) {
            telemetry.addLine("second is Pressed");
        } else {
            telemetry.addLine("second is not Pressed");
        }
        telemetry.addData("mov", vidvizh.getCurrentPosition());
        telemetry.addData("lift", lift1.getCurrentPosition());
        telemetry.update();

        if (gamepad2.dpad_right){
            posInit = lift1.getCurrentPosition();
        }



        if (/*vidvizh.getCurrentPosition() < -80 || */gamepad1.y) {
           h+=1;

           /* mover.setPosition(0.39);
            perekid2.setPosition(0.53);
            perekid1.setPosition(0.47);
           scor.setPosition(0);*/
        }
        n+=1;
        /*if(n < 50){
            intake.setPower(-1);
        }*/
        if (gamepad2.x){
            n = 0;
            b = 0;
            c+=1;

            a = 0;
            d = 0;
            y = 0;
            k = 0;
            t = 0;
            e = 0;
            mover.setPosition(0.68);

        }

        if (c>0){
            c+=1;
            if (c<50){
                intake.setPower(0);
                zahvat.setPower(0);
            }
        }
        if (gamepad2.b){
            //z +=1;
          //  b+=1;
        }
        if(gamepad2.y){
            z+=1;
        }
        /*if(c > 0){
         double error2 = vidvizh.getCurrentPosition() + pos3 ;
            if(error2 > 0) {
                vidvizh.setPower(error2*kp);
            }
            else {
                c = 0;
            }
        }*/

       /*  if (z >0){
            z +=1;
            mover.setPosition(0.39);
            if(z<15){
                scor.setPosition(0);
            }
           else{
                if (lift1.getCurrentPosition() >= 0) {
                    lift1.setPower(-1);
                    lift2.setPower(-1);
                } else {
                    z = 0;
                    perekid2.setPosition(0.98);
                    perekid1.setPosition(0.02);
                }
            }*/
        //}
        if (b > 0&&b < 20){
            b+=1;
            scor.setPosition(0.5);
        } else if ( b > 20 && b < 30) {
            b +=1;
            mover.setPosition(0.68);
            scor.setPosition(0.2);
        }
       /* else if(b>5&&b<30){

            perekid2.setPosition(0.55);
            perekid1.setPosition(0.45);
        }*/
        else {
            b=0;
        }
        if (z == 0){
            lift.teleop();

        }
        if (vidvizh.getCurrentPosition() >= -20 && gamepad1.right_stick_y == 0) {
            hook.setPosition(0.35);
        }
        else {
            hook.setPosition(0.15);
        }

        if (gamepad1.left_bumper || gamepad1.right_bumper){
            h = 0;
        }

        if (vidvizh.getCurrentPosition() >=-300 && vidvizh.getCurrentPosition() <= 0){
            wall.setPosition(0.7);
        }
        else if(vidvizh.getCurrentPosition() < -300){
            wall.setPosition(0.2);
        }
        if (h > 0) {
            SmartButton2();
        } else {

            if (vidvizh.getCurrentPosition() >= -10) {
                if (gamepad1.right_stick_y > 0) {
                    vidvizh.setPower(0);
                }
                else if(gamepad1.right_stick_y <= 0){
                    vidvizh.setPower(gamepad1.right_stick_y);
                }
            }
            else{
                vidvizh.setPower(gamepad1.right_stick_y);
            }

            if (gamepad1.left_bumper) {
                intake.setPower(-1);
                zahvat.setPower(1);
                h = 0;
                timeIntake = 0;
                checkIntake = 0;
            } else if (gamepad1.right_bumper) {
                intake.setPower(1);
                zahvat.setPower(-1);
                h = 0;
                timeIntake = 0;
                checkIntake = 0;
            } else {
                intake.setPower(0);
                zahvat.setPower(0);
            }
        }


       /* if(n ==0 && (vidvizh.getCurrentPosition() > -80 || gamepad1.y) && gamepad1.right_stick_y != 0){
            k+=1;
            perekid2.setPosition(0.53);
            perekid1.setPosition(0.47);
            scor.setPosition(0);
        }

      /*  if (gamepad2.x){
            k = 0;
            d = 0;
        }

        if ((k > 0 || d > 0) && n == 0){
            zahvat.setPower(1);
            intake.setPower(-1);
            vidvizh.setPower(0.3);

        }
        else if (n>0) {
            k = 0;
            d = 0;
            Sensor();
           // zahvat.setPower(1);
        }*/
    }

    public void SmartButton() {
        if (lineSensor.getState() != true && lineSensor2.getState() != true) {
            e += 1;
        }

        if (gamepad1.left_bumper || gamepad1.right_bumper){
            a = 0;
            d = 0;
            y = 0;
            k = 0;
            t = 0;
            e = 0;
        }
        if (e == 0 && d != 0) {

            vidvizh.setPower(gamepad1.right_stick_y);

            if (lineSensor.getState() == true && lineSensor2.getState() == true) {
                e = 0;
                scor.setPosition(0);
                mover.setPosition(0.68);
                perekid2.setPosition(0.4);
                perekid1.setPosition(0.6);
                zahvat.setPower(1);
            }
            if (lineSensor2.getState() != true) {
                t += 1;
            }
            if (t > 0 && lineSensor.getState() == true) {
                intake.setPower(-1);
            }
            if (t > 0 && lineSensor.getState() != true) {
                k += 1;
           /* if (k < 10) {
                intake.setPower(0.5);
            } else {*/
                intake.setPower(0);

            }

        } else if (e > 0) {
            e += 1;
            zahvat.setPower(0);
            if (e < 15) {
                intake.setPower(1);
            } else if (e > 15 && e < 27) {
                intake.setPower(-1);

            } else {
                if (a == 0) {
                    if (lift1.getCurrentPosition() >= posInit){
                        lift1.setPower(-1);
                        lift2.setPower(-1);
                    }
                    if (vidvizh.getCurrentPosition() < -80) {
                        vidvizh.setPower(1);
                        intake.setPower(0);
                    } else {
                        a += 1;

                    }
                }
                if (a != 0) {
                    vidvizh.setPower(gamepad1.right_stick_y);

                    if (lineSensor.getState() != true || lineSensor2.getState() != true) {
                           y+=1;
                          if (y > 5 && y < 100) {
                              // intake.setPower(-1);
                           }
                           else {
                               intake.setPower(0);
                               a = 0;
                               d = 0;
                               y = 0;
                               k = 0;
                               t = 0;
                               e = 0;
                           }
                    } else {
                          /*  y +=1;
                            if (y < 10){
                                intake.setPower(-1);
                            }
                            else {
                                a +=1;

                                scor.setPosition(0);
                                mover.setPosition(0.39);
                                perekid2.setPosition(0.02);
                                perekid1.setPosition(0.98);
                                if(a< 10){
                                    intake.setPower(-1);

                                }
                                else {
                                intake.setPower(0)*/
                       /* y += 1;
                        if (y < 25) {
                            intake.setPower(-1);
                        }*/// else {
                            a += 1;

                          //  scor.setPosition(0);
                           // mover.setPosition(0.39);
                          //  perekid2.setPosition(0.02);
                           // perekid1.setPosition(0.98);
                            if (a < 10) {
                                intake.setPower(-1);

                            } else {
                                intake.setPower(0);
                               // scor.setPosition(0.5);
                                a = 0;
                                d = 0;
                                y = 0;
                                k = 0;
                                t = 0;
                                e = 0;
                            }
                                    /*d = 0;
                                    k = 0;
                                    t = 0;
                                    e = 0;
                                    y = 0;
                                    a = 0;
                                }

                            }*/
                        }
                    }


                }

           /* if(lineSensor.getState() != true && lineSensor2.getState() == true){
                k+=1;
                if(k<10){
                    intake.setPower(-0.5);
                }
                else {
                    intake.setPower(0);
                }
            }

            if (lineSensor.getState() != true && lineSensor2.getState() != true){
                e+=1;
                if (e < 5) {
                    intake.setPower(-0.5);
                }
                else if ( 6< e && e <12){
                    intake.setPower(0.5);
                }
                else {
                    intake.setPower(0);
                    zahvat.setPower(0);
                }
            }
            if ((lineSensor.getState() != true || lineSensor2.getState() != true) && vidvizh.getCurrentPosition() > -80) {
                intake.setPower(-1);
            }
            else{
                perekid2.setPosition(0.02);
                perekid1.setPosition(0.98);
                a = 0;
                d = 0;
            }*/
        }
    }
    public void SmartButton2(){
        c+=1;
        if (c< 20){
            scor.setPosition(0.2);
            mover.setPosition(0.68);
        }
        else if(c > 20 && c < 40){
         /*   double error = posInit - lift1.getCurrentPosition();
            if (error < 0) {
                lift1.setPower(error * kp);
                lift2.setPower(error * kp);
            } else {*/
                perekid2.setPosition(0);
                perekid1.setPosition(1);
            //}
        }
      //  vidvizh.setPower(gamepad1.right_stick_y);
        if(timeIntake>0){
            vidvizh.setPower(gamepad1.right_stick_y);
            timeIntake +=1;

            if (timeIntake < 20){
                zahvat.setPower(1);

            }
            else {
                zahvat.setPower(0);
                timeIntake = 0;
                h = 0;
            }
        }
        if(lineSensor2.getState() != true){
            checkIntake +=1;
            vidvizh.setPower(gamepad1.right_stick_y);
        }

        if (checkIntake ==0){
            vidvizh.setPower(gamepad1.right_stick_y);
            zahvat.setPower(1);
        }
       else if (checkIntake > 10){

            if (vidvizh.getCurrentPosition() < -200 && gamepad1.right_stick_y == 0){
                vidvizh.setPower(1);
                zahvat.setPower(0);
            }
            else if ( vidvizh.getCurrentPosition() > -200 && vidvizh.getCurrentPosition() < 0 && gamepad1.right_stick_y == 0){
                wall.setPosition(0.7);
                vidvizh.setPower(1);
                zahvat.setPower(0);

            }
            else{
                vidvizh.setPower(gamepad1.right_stick_y);
               if(lineSensor2.getState() == true){
                    checkIntake = 0;
                   // d = 0;
                    c = 0;
                    timeIntake += 1;
                   // zahvat.setPower(0);
                }
               else {

                       zahvat.setPower(1);
               }
            }
        }
    }
    public void Autonomous(){

        ElapsedTime time = new  ElapsedTime();
        while (time.milliseconds() < 10000) {

            if (lineSensor2.getState() != true) {
                PixeIsIn = true;
            }
            if (PixeIsIn == false) {
                zahvat.setPower(1);
                intake.setPower(-1);
            }
            if (PixeIsIn == true) {
                while (lineSensor.getState() != true || lineSensor2.getState() != true) {
                    zahvat.setPower(1);
                    intake.setPower(-1);
                }
                zahvat.setPower(0);
                intake.setPower(0);
                scor.setPosition(0.8);
            }
        }
    }
    public void Autonomous2(){
        perekid2.setPosition(0);
        perekid1.setPosition(1);

    }
    public void SetLeftMov(){
            mover.setPosition(0);
    }
    public void SetCenterMov(){
        mover.setPosition(0.5);
    }
    public void SetRightMov(){
        mover.setPosition(0.95);
    }
    public void AutonomousPerekid(){
        perekid2.setPosition(1);
        perekid1.setPosition(0);

    }
    public void Senso(){
        perekid2.setPosition(1);
        perekid1.setPosition(0);

    }
    public void ScorerOpen(){
        scor.setPosition(0.15);
        scor1.setPosition(0.55);

    }
    public void ScorerClose(){
        scor.setPosition(0.7);
        scor1.setPosition(0);

    }
    public void AutoWallClose(){
        wall.setPosition(0.2);
    }
    public void AutoWallOpen(){
        wall.setPosition(0.7);
    }
    public void Autonomoys3(){
        mover.setPosition(1);

    }
    public void Autonomous4(){
        scor.setPosition(0.5);
    }
    public void Autonomous5(){
        mover.setPosition(0.68);
    }
    public void Autonomous6(){
        perekid2.setPosition(1);
        perekid1.setPosition(0);
    }
    public void perekidSETsenter(){
        perekid2.setPosition(0.5);
        perekid1.setPosition(0.5);
    }

    public void CloseScor() {
        scor.setPosition(0.8);
    }
    public void OpenScor(){
        scor.setPosition(0.2);
    }
    public void OpenHook(){
        hook.setPosition(0.15);
    }
    public void MovSetCenter(){
        mover.setPosition(0.5);
    }
    public void Autonomoys7(){
        mover.setPosition(0);
    }
    public void CloseWall(){
        wall.setPosition(0.2);
    }
    public void IntakeFromSteak(){
        ElapsedTime t = new ElapsedTime();
        while (t.milliseconds()<200){
            zahvat.setPower(-1);
        }
        zahvat.setPower(0);

    }
    public void Steak(){
        ElapsedTime t = new ElapsedTime();
        if (t.milliseconds() < 300){
            intake.setPower(0);
            zahvat.setPower(0);
        }
        if(lineSensor2.getState() == true){
            intake.setPower(-1);
            zahvat.setPower(1);
        }
        if(t.milliseconds() < 4000 && t.milliseconds() > 300) {
            zahvat.setPower(1);
            intake.setPower(-1);
        }
        scor.setPosition(0.8);
        zahvat.setPower(-1);
        intake.setPower(1);
        opMode.sleep(1500);
       /* while (lineSensor.getState() == true){
            intake.setPower(1);
            zahvat.setPower(0);
        }*/
       /* while (t.milliseconds() < 1000){
            intake.setPower(-1);
            zahvat.setPower(-1);
        }*/
        intake.setPower(0);
        zahvat.setPower(0);
    }
    public void lift(){
        double error2 = 400 - lift1.getCurrentPosition();
        while (error2 > 0) {
            lift1.setPower(error2 * kp);
            lift2.setPower(error2 * kp);
            error2 = 500 - lift1.getCurrentPosition();
        }
            lift1.setPower(0);
            lift2.setPower(0);
    }

    public void liftF(){
        double error2 = 150 - lift1.getCurrentPosition();
        while (error2 > 0) {
            lift1.setPower(error2 * kp);
            lift2.setPower(error2 * kp);
            error2 = 270 - lift1.getCurrentPosition();
        }
        lift1.setPower(0);
        lift2.setPower(0);
    }
    public void lift2(){
        double error2 = 0 - lift1.getCurrentPosition();
        while (error2 < 0) {
            lift1.setPower(error2 * kp * 0.002);
            lift2.setPower(error2 * kp * 0.0004);
            error2 = 0 - lift1.getCurrentPosition();
        }
        lift1.setPower(0);
        lift2.setPower(0);
    }

    public void vidv(){
        double error2 = 2000 + vidvizh.getCurrentPosition();
        if (error2 > 0) {
            vidvizh.setPower(-error2 * kp);
            error2 = 0 + vidvizh.getCurrentPosition();
        }
        vidvizh.setPower(0);
    }
    public void vidv2(){
        double error2 = vidvizh.getCurrentPosition();
        if (error2 < 0) {
            vidvizh.setPower(-error2 * kp);
            error2 = 0 + vidvizh.getCurrentPosition();
        }
        vidvizh.setPower(0);
    }

    public void IntakePixels(){
       if(lineSensor.getState() != true){
           f+=1;
        }
       if (f >0){
           zahvat.setPower(0);
       }
        zahvat.setPower(1);
    }
    public void intikeWhileRunning(){
        zahvat.setPower(1);
        intake.setPower(-1);
    }
    public void stopIntake(){
        zahvat.setPower(0);
        intake.setPower(0);
    }
}


     /*   if(k<10){

        }
       /* if((gamepad2.left_trigger - gamepad2.right_trigger) > 0){
            k= 11;
        }*/



