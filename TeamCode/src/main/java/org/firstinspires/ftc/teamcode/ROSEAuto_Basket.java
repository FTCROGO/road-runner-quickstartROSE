
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
import com.qualcomm.robotcore.hardware.Servo;

@Config


@Autonomous(name = "ROSEAuto_Basket", group = "Autonomous")
public class ROSEAuto_Basket extends LinearOpMode {
    double startPosition;
    public DcMotor  arm = null;
    public DcMotor  extension = null;
    public Servo    munch = null;
    public Servo    pitch = null;

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-24, -60   , Math.toRadians(270));
        Pose2d basketPose = new Pose2d(-55, -55, Math.toRadians(135));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        arm = hardwareMap.get(DcMotor.class, "arm");
        extension = hardwareMap.get(DcMotor.class, "extension");
        munch = hardwareMap.get(Servo.class, "munch");
        pitch = hardwareMap.get(Servo.class, "pitch");

        arm.setDirection(DcMotor.Direction.FORWARD);
        extension.setDirection(DcMotor.Direction.REVERSE);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        munch.setPosition(0.5);
        pitch.setPosition(0.5);

        TrajectoryActionBuilder initToBasket = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-50, -50))
                .turn(Math.toRadians(135))
                .strafeTo(new Vector2d(-55, -55));
        TrajectoryActionBuilder basketToBar = drive.actionBuilder(basketPose)
                .strafeTo(new Vector2d(-36, -24))
                .splineTo(new Vector2d(-24, 0), Math.toRadians(0));
//        TrajectoryActionBuilder tab = drive.actionBuilder(initialPose)
//                .strafeTo(new Vector2d(50, 50))
//                .turn(Math.toRadians(130))
//                .strafeTo(new Vector2d(68, 65));



        if (isStopRequested())
            return;


        waitForStart();

//basket 1
        arm.setPower(0.4);
        arm.setTargetPosition(3300);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        extension.setPower(0.2);
        extension.setTargetPosition(2000);
        extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sleep(5000);

        Actions.runBlocking(
                new SequentialAction(
                        initToBasket.build()
                )
        );

        pitch.setPosition(1);
        munch.setPosition(1);

        sleep(1000);

        extension.setPower(0.2);
        extension.setTargetPosition(100);
        extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Actions.runBlocking(
                new SequentialAction(
                        basketToBar.build()
                )
        );
    }
}