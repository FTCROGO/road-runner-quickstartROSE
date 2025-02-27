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
@Autonomous(name = "touchBar", group = "Autonomous")
public class touchBar extends LinearOpMode {

    // extension class
    public class Extension {
        private final DcMotorEx extension;

        public Extension(HardwareMap hardwareMap) {
            extension = hardwareMap.get(DcMotorEx.class, "extension");
            extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            extension.setDirection(DcMotorSimple.Direction.REVERSE);
//            extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        public class ExtensionUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    extension.setPower(0.4);
                    initialized = true;
                }

                double pos = extension.getCurrentPosition();
                packet.put("extensionPos", pos);
                telemetry.addData("linear pos", extension.getCurrentPosition());
                telemetry.update();
                if (pos < 2000)
                    return true;
                else {
                    extension.setPower(0);
                    return false;
                }
            }
        }
        public Action extensionUp() {
            return new ExtensionUp();
        }


        public class ExtensionDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    extension.setPower(-0.1);
                    initialized = true;
                }

                double pos = extension.getCurrentPosition();
                packet.put("extensionPos", pos);
//                telemetry.addData("linear pos", extension.getCurrentPosition());
//                telemetry.update();
                if (pos > 200)
                    return true;
                else {
                    extension.setPower(0);
                    return false;
                }
            }
        }

        public Action extensionDown() {
            return new ExtensionDown();
        }
    }



    // arm class
    public class Arm {
        private final DcMotorEx arm;

        public Arm(HardwareMap hardwareMap) {
            arm = hardwareMap.get(DcMotorEx.class, "arm");
            arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            arm.setDirection(DcMotorSimple.Direction.FORWARD);
//            arm.setMode(DcMotor.RunMode.RESET_ENCODERS);
        }


        public class ArmUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    arm.setPower(0.6);
                    initialized = true;
                }

                double pos = arm.getCurrentPosition();
                packet.put("armPos", pos);
                telemetry.addData("arm pos", arm.getCurrentPosition());
                telemetry.update();
//                if (pos < 3000)
                if (pos < 500)
                    return true;
                else {
                    arm.setPower(0);
                    return false;
                }
            }
        }

        public Action armUp() {
            return new ArmUp();
        }


        public class ArmDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    arm.setPower(-0.4);
                    initialized = true;
                }

                double pos = arm.getCurrentPosition();
                packet.put("armPos", pos);
                //telemetry.addData("arm pos", arm.getCurrentPosition());
                //telemetry.update();
                if (pos > 200)
                    return true;
                else {
                    arm.setPower(0);
                    return false;
                }
            }
        }

        public Action armDown() {
            return new ArmDown();
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


        public class ParYaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                yaw.setPosition(0.6);
                return false;
            }
        }

        public Action parYaw() {
            return new ParYaw();
        }


        public class PerpYaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                yaw.setPosition(0.93);
                return false;
            }
        }

        public Action perpYaw() {
            return new PerpYaw();
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
        Pose2d initialPose = new Pose2d(-24, -60, Math.toRadians(90));
//        Pose2d basketPose = new Pose2d(-55, -55, Math.toRadians(225));
//        Pose2d farPose = new Pose2d(-36, -24, Math.toRadians(180));
//        Pose2d mediumPose = new Pose2d(-48, -24, Math.toRadians(180));
//        Pose2d closePose = new Pose2d(-60, -24, Math.toRadians(180));
        Pose2d initialPoseRight = new Pose2d(0, -60, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Extension extension = new Extension(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        Munch munch = new Munch(hardwareMap);
        Yaw yaw = new Yaw(hardwareMap);
        Pitch pitch = new Pitch(hardwareMap);


//        TrajectoryActionBuilder toBasketInit = drive.actionBuilder(initialPose)
//// to basket from init
//                .strafeTo(new Vector2d(-50, -50))
//                .turn(Math.toRadians(135))
//                .strafeTo(new Vector2d(-55, -55));
//
//        TrajectoryActionBuilder toFar = drive.actionBuilder(basketPose)
//// to far sample from basket
//                .strafeTo(new Vector2d(-36, -24))
//                .turn(Math.toRadians(-45));
//
//        TrajectoryActionBuilder toBasketFar = drive.actionBuilder(farPose)
//// to basket from far
//                .strafeTo(new Vector2d(-55, -55))
//                .turn(Math.toRadians(45));
//
//        TrajectoryActionBuilder toMedium = drive.actionBuilder(basketPose)
//// to medium sample from basket
//                .strafeTo(new Vector2d(-48, -24))
//                .turn(Math.toRadians(-45));
//
//        TrajectoryActionBuilder toBasketMedium = drive.actionBuilder(mediumPose)
//// to basket from medium
//                .strafeTo(new Vector2d(-55, -55))
//                .turn(Math.toRadians(45));
//
//        TrajectoryActionBuilder toClose = drive.actionBuilder(basketPose)
//// to close sample from basket
//                .strafeTo(new Vector2d(-60, -24))
//                .turn(Math.toRadians(-45));
//
//        TrajectoryActionBuilder toBasketClose = drive.actionBuilder(closePose)
//// to basket from init
//                .strafeTo(new Vector2d(-55, -55))
//                .turn(Math.toRadians(45));

        TrajectoryActionBuilder toBar = drive.actionBuilder(initialPoseRight)
//                .strafeTo(new Vector2d(42, 0))
//                .strafeTo(new Vector2d(24, 0));
                .strafeTo(new Vector2d(60, -60));



        if (isStopRequested()) return;

        waitForStart();

        //telemetry.addData("linear pos", extension.getCurrentPosition());
        //telemetry.update();

        Actions.runBlocking(
                new SequentialAction(
                        arm.armUp(),
                        //pitch.upPitch(),
                        toBar.build()

//                        toBasketInit.build(),
// score in basket
//                        extension.extensionUp(),
//                        pitch.downPitch(),
//                        munch.openMunch(),
//                        extension.extensionDown()

//                        toFar.build(),
//// intake far sample
//                        pitch.downPitch(),
//                        arm.armDown(),
//                        munch.closeMunch(),
//                        arm.armUp(),
//                        pitch.upPitch(),
//
//                        toBasketFar.build(),
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
