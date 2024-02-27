package org.firstinspires.ftc.teamcode.opModes.teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Modules.Goose;
import org.firstinspires.ftc.teamcode.Modules.Intake;
import org.firstinspires.ftc.teamcode.Modules.IntakeSecondVersion;
import org.firstinspires.ftc.teamcode.Modules.Intake_mover;
import org.firstinspires.ftc.teamcode.Modules.Lift;
import org.firstinspires.ftc.teamcode.Modules.Modules;
import org.firstinspires.ftc.teamcode.Modules.Scorer;
import org.firstinspires.ftc.teamcode.Modules.SkorMover;
import org.firstinspires.ftc.teamcode.Modules.SpikeScorer;
import org.firstinspires.ftc.teamcode.Modules.V4bMover;
import org.firstinspires.ftc.teamcode.Modules.Virtual4bar;
import org.firstinspires.ftc.teamcode.Modules.Shuter;
import org.firstinspires.ftc.teamcode.Robot1;
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {
    @Override

    public void runOpMode() {

        Robot1 R = new Robot1(this);
        Modules mod = new Modules(this);

        waitForStart();

        while (!isStopRequested()) {
            R.control();
            mod.teleop();
        }
    }
}

