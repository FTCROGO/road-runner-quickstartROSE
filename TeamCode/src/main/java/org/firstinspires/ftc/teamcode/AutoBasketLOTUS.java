
package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@Autonomous(name = "AutoBasketLOTUS", group = "Autonomous")
public class AutoBasketLOTUS extends LinearOpMode {

    // extension class
    public class MLS {
        private final DcMotorEx mLS;

        public MLS(HardwareMap hardwareMap) {
            mLS = hardwareMap.get(DcMotorEx.class, "mLS");
            mLS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mLS.setDirection(DcMotorSimple.Direction.REVERSE);
            mLS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mLS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public class MLSUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    mLS.setPower(0.4);
                    initialized = true;
                }

                double pos = mLS.getCurrentPosition();
                packet.put("mLSPos", pos);
//                telemetry.addData("linear pos", extension.getCurrentPosition());
//                telemetry.update();
                if (pos < 2000)
                    return true;
                else {
                    mLS.setPower(0);
                    return false;
                }
            }
        }
        public Action mLSUp() {
            return new MLSUp();
        }


        public class MLSDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    mLS.setPower(-0.1);
                    initialized = true;
                }

                double pos = mLS.getCurrentPosition();
                packet.put("mLSPos", pos);
//                telemetry.addData("linear pos", extension.getCurrentPosition());
//                telemetry.update();
                if (pos > 200)
                    return true;
                else {
                    mLS.setPower(0);
                    return false;
                }
            }
        }

        public Action mLSDown() {
            return new MLSDown();
        }
    }



    // arm class
    public class MArm {
        private final DcMotorEx mArm;

        public MArm(HardwareMap hardwareMap) {
            mArm = hardwareMap.get(DcMotorEx.class, "mArm");
            mArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mArm.setDirection(DcMotorSimple.Direction.FORWARD);
            mArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }


        public class MArmUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    mArm.setPower(0.6);
                    initialized = true;
                }

                double pos = mArm.getCurrentPosition();
                packet.put("mArm pos", pos);
                telemetry.addData("mArm pos", mArm.getCurrentPosition());
                telemetry.update();
                if (pos < 3200)
                    return true;
                else {
                    mArm.setPower(0);
                    return false;
                }
            }
        }

        public Action mArmUp() {
            return new MArmUp();
        }


        public class MArmDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    mArm.setPower(-0.4);
                    initialized = true;
                }

                double pos = mArm.getCurrentPosition();
                packet.put("mArmPos", pos);
                //telemetry.addData("arm pos", arm.getCurrentPosition());
                //telemetry.update();
                if (pos > 200)
                    return true;
                else {
                    mArm.setPower(0);
                    return false;
                }
            }
        }

        public Action mArmDown() {
            return new MArmDown();
        }
    }




    // munch class
    public static class Munch {
        private final Servo munch;

        public Munch(HardwareMap hardwareMap) {
            munch = hardwareMap.get(Servo.class, "munch");
        }


        public class CloseMunch implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                munch.setPosition(-0.5);
                return false;
            }
        }

        public Action closeMunch() {
            return new CloseMunch();
        }


        public class OpenMunch implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                munch.setPosition(0.5);
                return false;
            }
        }

        public Action openMunch() {
            return new OpenMunch();
        }
    }




    // yaw class
    public static class Yaw {
        private final Servo yaw;

        public Yaw(HardwareMap hardwareMap) {
            yaw = hardwareMap.get(Servo.class, "yaw");
        }


        public class UpYaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                yaw.setPosition(0.6);
                return false;
            }
        }

        public Action upYaw() {
            return new UpYaw();
        }


        public class SideYaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                yaw.setPosition(0.93);
                return false;
            }
        }

        public Action sideYaw() {
            return new SideYaw();
        }
    }




    // pitch class
    public static class Pitch {
        private final Servo pitch;

        public Pitch(HardwareMap hardwareMap) {
            pitch = hardwareMap.get(Servo.class, "pitch");
        }


        public class UpPitch implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                pitch.setPosition(0.25);
                return false;
            }
        }

        public Action upPitch() {
            return new UpPitch();
        }


        public class DownPitch implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                pitch.setPosition(0.4);
                return false;
            }
        }

        public Action downPitch() {
            return new DownPitch();
        }
    }




    //double startPosition;

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(24, -60, Math.toRadians(90));
        Pose2d basketPose = new Pose2d(-55, -55, Math.toRadians(225));
        Pose2d farPose = new Pose2d(-36, -24, Math.toRadians(180));
        Pose2d mediumPose = new Pose2d(-48, -24, Math.toRadians(180));
        Pose2d closePose = new Pose2d(-60, -24, Math.toRadians(180));
//        Pose2d initialPoseRight = new Pose2d(0, -60, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        MLS mLS = new MLS(hardwareMap);
        MArm mArm = new MArm(hardwareMap);
        Munch munch = new Munch(hardwareMap);
        Yaw yaw = new Yaw(hardwareMap);
        Pitch pitch = new Pitch(hardwareMap);


        TrajectoryActionBuilder toBasketInit = drive.actionBuilder(initialPose)
// to basket from init
                .strafeTo(new Vector2d(-15, -60))
                .turn(Math.toRadians(135));
//                .strafeTo(new Vector2d(-55, -55));
//                .strafeTo(new Vector2d(56, -60));
        TrajectoryActionBuilder toFar = drive.actionBuilder(basketPose)
// to far sample from basket
                .strafeTo(new Vector2d(-36, -24))
                .turn(Math.toRadians(-45));

        TrajectoryActionBuilder toBasketFar = drive.actionBuilder(farPose)
// to basket from far
                .strafeTo(new Vector2d(-55, -55))
                .turn(Math.toRadians(45));

        TrajectoryActionBuilder toMedium = drive.actionBuilder(basketPose)
// to medium sample from basket
                .strafeTo(new Vector2d(-48, -24))
                .turn(Math.toRadians(-45));

        TrajectoryActionBuilder toBasketMedium = drive.actionBuilder(mediumPose)
// to basket from medium
                .strafeTo(new Vector2d(-55, -55))
                .turn(Math.toRadians(45));

        TrajectoryActionBuilder toClose = drive.actionBuilder(basketPose)
// to close sample from basket
                .strafeTo(new Vector2d(-60, -24))
                .turn(Math.toRadians(-45));

        TrajectoryActionBuilder toBasketClose = drive.actionBuilder(closePose)
// to basket from init
                .strafeTo(new Vector2d(-55, -55))
                .turn(Math.toRadians(45));

//        TrajectoryActionBuilder toBar = drive.actionBuilder(initialPoseRight)
////                .strafeTo(new Vector2d(42, 0))
////                .strafeTo(new Vector2d(24, 0));
//                .strafeTo(new Vector2d(60, -60));



        if (isStopRequested()) return;

        waitForStart();

        //telemetry.addData("linear pos", extension.getCurrentPosition());
        //telemetry.update();

        Actions.runBlocking(
                new SequentialAction(
                        munch.closeMunch(),
//                        mArm.mArmUp(),
//                        pitch.upPitch(),
//                        toBar.build(),

                        toBasketInit.build(),
//// score in basket
                        mLS.mLSUp(),
                        pitch.downPitch(),
                        munch.openMunch(),
                        mLS.mLSDown()

//                        toFar.build()
//// intake far sample
//                        pitch.downPitch(),
//                        arm.armDown(),
//                        munch.closeMunch(),
//                        arm.armUp(),
//                        pitch.upPitch(),
//
//                        toBasketFar.build()
//
////// score far in basket
//                        extension.extensionUp(),
//                        munch.openMunch(),
//                        extension.extensionDown(),

//                        toMedium.build(),

//// intake medium sample
//                        pitch.downPitch(),
//                        arm.armDown(),
//                        munch.closeMunch(),
//                        arm.armUp(),
//                        pitch.upPitch(),

//                        toBasketMedium.build(),

//// score medium in basket
//                        extension.extensionUp(),
//                        munch.openMunch(),
//                        extension.extensionDown(),

//                        toClose.build(),

//// intake close sample
//                        pitch.downPitch(),
//                        arm.armDown(),
//                        munch.closeMunch(),
//                        arm.armUp(),
//                        pitch.upPitch(),

//                        toBasketClose.build()

//// score close in basket
//                        extension.extensionUp(),
//                        munch.openMunch(),
//                        extension.extensionDown()
//                         moving thing end pos
//                        pitch.downPitch(),
//                        arm.armDown()
                )
        );
    }
}