package org.firstinspires.ftc.teamcode.opModes.teleop;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Modules.Goose;
import org.firstinspires.ftc.teamcode.Modules.HookTest;
import org.firstinspires.ftc.teamcode.Modules.Intake;
import org.firstinspires.ftc.teamcode.Modules.IntakeSecondVersion;
import org.firstinspires.ftc.teamcode.Modules.Intake_mover;
import org.firstinspires.ftc.teamcode.Modules.Lift;
import org.firstinspires.ftc.teamcode.Modules.Modules;
import org.firstinspires.ftc.teamcode.Modules.Scorer;
import org.firstinspires.ftc.teamcode.Modules.SkorMover;
import org.firstinspires.ftc.teamcode.Modules.SpikeScorer;
import org.firstinspires.ftc.teamcode.Modules.TestIntake;
import org.firstinspires.ftc.teamcode.Modules.V4bMover;
import org.firstinspires.ftc.teamcode.Modules.Virtual4bar;
import org.firstinspires.ftc.teamcode.Modules.Shuter;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Robot1;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.TwoWheelTrackingLocalizer;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {
    SampleMecanumDrive drive;
    @Override

    public void runOpMode() {

        Robot1 R = new Robot1(this);
        Modules mod = new Modules(this);
        drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        while (!isStopRequested()) {
            R.control();
            mod.teleop();
        }
    }
}

