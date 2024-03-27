package org.firstinspires.ftc.teamcode.opModes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Modules.Intake;
import org.firstinspires.ftc.teamcode.Modules.L1ft;
import org.firstinspires.ftc.teamcode.Modules.LiftTest;

@TeleOp
public class TeleEasy  extends LinearOpMode {
   LiftTest lift;
    @Override
    public void runOpMode() throws InterruptedException {
        lift = new LiftTest(this);

        waitForStart();

        while (!isStopRequested()){
           lift.teleop();
        }
    }
}
