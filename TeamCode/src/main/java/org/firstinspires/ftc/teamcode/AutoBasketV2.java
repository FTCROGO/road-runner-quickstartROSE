
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
@Autonomous(name = "AutoBasketV2", group = "Autonomous")
public class AutoBasketV2 extends LinearOpMode {

    // extension class
    public class Extension {
        private final DcMotorEx extension;

        public Extension(HardwareMap hardwareMap) {
            extension = hardwareMap.get(DcMotorEx.class, "extension");
            extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            extension.setDirection(DcMotorSimple.Direction.REVERSE);
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
                if (pos < 850)
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
                    extension.setPower(-0.8);
                    initialized = true;
                }

                double pos = extension.getCurrentPosition();
                packet.put("extensionPos", pos);
//                telemetry.addData("linear pos", extension.getCurrentPosition());
//                telemetry.update();
                if (pos > 0)
                    return true;
                else {
                    extension.setPower(0.4);
                    return false;
                }
            }
        }

        public Action extensionDown() {
            return new ExtensionDown();
        }
    }



// arm class
    public static class Arm {
        private final DcMotorEx arm;

        public Arm(HardwareMap hardwareMap) {
            arm = hardwareMap.get(DcMotorEx.class, "arm");
            arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            arm.setDirection(DcMotorSimple.Direction.FORWARD);
        }


        public class ArmUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    arm.setPower(0.4);
                    initialized = true;
                }

                double pos = arm.getCurrentPosition();
                packet.put("armPos", pos);
                //telemetry.addData("arm pos", arm.getCurrentPosition());
                //telemetry.update();
                if (pos < 850)
                    return true;
                else {
                    arm.setPower(0);
                    return false;
                }
            }
        }

        public ArmUp armUp() {
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
                    if (pos > 0)
                        return true;
                    else {
                        arm.setPower(1);
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
                munch.setPosition(0.55);
                return false;
            }
        }

        public Action closeMunch() {
            return new CloseMunch();
        }


        public class OpenMunch implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                munch.setPosition(1.0);
                return false;
            }
        }

        public Action openMunch() {
            return new OpenMunch();
        }
    }




    //double startPosition;

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-36, -64   , Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Munch munch = new Munch(hardwareMap);
        Extension extension = new Extension(hardwareMap);
        Arm arm = new Arm(hardwareMap);


        TrajectoryActionBuilder toBasket = drive.actionBuilder(initialPose)
// to basket from init
                .strafeTo(new Vector2d(-50, -50))
                .turn(Math.toRadians(130))
                .strafeTo(new Vector2d(-68, -65));

        TrajectoryActionBuilder toFar = drive.actionBuilder(initialPose)
// to far sample from basket
                .strafeTo(new Vector2d(-24, -24))
                .turn(Math.toRadians(90));

        TrajectoryActionBuilder toMedium = drive.actionBuilder(initialPose)
// to medium sample from basket
                .strafeTo(new Vector2d(-24, -18))
                .turn(Math.toRadians(90));

        TrajectoryActionBuilder toClose = drive.actionBuilder(initialPose)
// to close sample from basket
                .strafeTo(new Vector2d(-30, -18))
                .turn(Math.toRadians(90));



//        Action trajectoryActionCloseOut = tab1.endTrajectory().fresh()
//                .build();

//        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
//                .strafeTo(new Vector2d(50, 50))
//                .turn(Math.toRadians(130))
//                .lineToY(40)
//                .lineToX(-24)
//                .lineToYSplineHeading(33, Math.toRadians(0))
//                .waitSeconds(2)
//                .setTangent(Math.toRadians(90))
//                .lineToXSplineHeading(46, Math.toRadians(180))

//                .strafeTo(new Vector2d(50, 50))
//                .turn(Math.toRadians(130));
//                .lastPoseUnmapped.extensionUp();

        // actions that need to happen in init
//        Actions.runBlocking(munch.closeMunch());




        if (isStopRequested()) return;

        waitForStart();

        //telemetry.addData("linear pos", extension.getCurrentPosition());
        //telemetry.update();

        Actions.runBlocking(
                new SequentialAction(
                        toBasket.build(),
// score in basket
                        arm.armUp()
//                        extension.extensionUp(),
//                        munch.openMunch(),
//                        extension.extensionDown(),
//
//                        toFar.build(),
//// intake far sample
//                        arm.armDown(),
//                        munch.closeMunch(),
//                        arm.armUp(),
//
//                        toBasket.build(),
//
//// score far in basket
//                        arm.armUp(),
//                        extension.extensionUp(),
//                        munch.openMunch(),
//                        extension.extensionDown(),
//
//                        toMedium.build(),
//
//// intake medium sample
//                        arm.armDown(),
//                        munch.closeMunch(),
//                        arm.armUp(),
//
//                        toBasket.build(),
//
//// score medium in basket
//                        arm.armUp(),
//                        extension.extensionUp(),
//                        munch.openMunch(),
//                        extension.extensionDown(),
//
//                        toClose.build(),
//
//// intake close sample
//                        arm.armDown(),
//                        munch.closeMunch(),
//                        arm.armUp(),
//
//                        toBasket.build(),
//
//// score close in basket
//                        arm.armUp(),
//                        extension.extensionUp(),
//                        munch.openMunch(),
//                        extension.extensionDown()
                )
        );
    }
}