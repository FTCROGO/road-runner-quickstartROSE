
package org.firstinspires.ftc.teamcode;

// RR-specific imports

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "ROSEAuto_Basket", group = "Autonomous")
public class ROSEAuto_Basket extends LinearOpMode {
    double startPosition;
    public DcMotor  mArm = null;
    public DcMotor  mLS = null;
    public Servo    Intake = null;

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(24, 66   , Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        mArm = hardwareMap.get(DcMotor.class, "mArm");
        mLS = hardwareMap.get(DcMotor.class, "mLS");
        Intake = hardwareMap.get(Servo.class, "Intake");

        mArm.setDirection(DcMotor.Direction.FORWARD);
        mLS.setDirection(DcMotor.Direction.REVERSE);

        mArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mLS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mLS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        mArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mLS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Intake.setPosition(0.5);

//        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
//                .lineToX(-72);
////                .lineToYSplineHeading(33, Math.toRadians(0))
////                .waitSeconds(2)
////                .setTangent(Math.toRadians(90))
////                .lineToY(48)
////                .setTangent(Math.toRadians(0))
////                .lineToX(32)
////                .strafeTo(new Vector2d(44.5, 30))
////                .turn(Math.toRadians(180))
////                .lineToX(47.5)
////                .waitSeconds(3);

//        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
//                .lineToX(37)
//                .setTangent(Math.toRadians(0))
//                .lineToX(18)
//                .waitSeconds(3)
//                .setTangent(Math.toRadians(0))
//                .lineToXSplineHeading(46, Math.toRadians(180))
//                .waitSeconds(3);
//
//        TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose)
//                .lineToYSplineHeading(33, Math.toRadians(180))
//                .waitSeconds(2)
//                .strafeTo(new Vector2d(46, 30))
//                .waitSeconds(3);

//        Action trajectoryActionCloseOut = tab1.endTrajectory().fresh()
//                .strafeTo(new Vector2d(-72, 72))
//                .build();
        TrajectoryActionBuilder tab = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(50, 50))
                .turn(Math.toRadians(130))
                .strafeTo(new Vector2d(68, 65));
//                .lineToY(40)
//                .turn(Math.toRadians(145))
//                .lineToX(-24);
                //.build();


                //.strafeTo(new Vector2d(0, 14));
//                .build();

        if (isStopRequested())
            return;

//        Action trajectoryActionChosen;
//        if (startPosition == 1) {
//            trajectoryActionChosen = tab1.build();
//        } else if (startPosition == 2) {
//            trajectoryActionChosen = tab2.build();
//        } else {
//            trajectoryActionChosen = tab3.build();
//        }

        waitForStart();

        mArm.setPower(0.4);
        mArm.setTargetPosition(3300);
        mArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        mLS.setPower(0.2);
        mLS.setTargetPosition(2000);
        mLS.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sleep(5000);

        Actions.runBlocking(
                new SequentialAction(
                        tab.build()));
                    //trajectoryActionChosen,
//                    trajectoryActionCloseOut));


        Intake.setPosition(1);

        sleep(1000);
    }
}