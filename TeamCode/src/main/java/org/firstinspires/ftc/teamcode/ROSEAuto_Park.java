
package org.firstinspires.ftc.teamcode;

// RR-specific imports

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "ROSEAuto_Park", group = "Autonomous")
public class ROSEAuto_Park extends LinearOpMode {
    double startPosition;
    public DcMotor  arm = null;
//    public Servo pitch = null;

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(24, -60, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        arm = hardwareMap.get(DcMotor.class, "arm");
        arm.setDirection(DcMotor.Direction.FORWARD);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        TrajectoryActionBuilder initToBar = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(36, -24))
                .splineTo(new Vector2d(24, 0), Math.toRadians(180));

//        TrajectoryActionBuilder tab = drive.actionBuilder(initialPose)
//                .lineToX(-72);

        if (isStopRequested())
            return;

        arm.setPower(0.4);
        arm.setTargetPosition(400);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        pitch.setPosition(1);

        waitForStart();
        Actions.runBlocking(
                new SequentialAction(
                        initToBar.build()));

    }
}