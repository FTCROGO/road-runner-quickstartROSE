//package org.firstinspires.ftc.teamcode;
//
//import androidx.annotation.NonNull;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//
//@Config
//@Autonomous(name = "AutoBasketV2", group = "Autonomous")
//public class AutoBasketV3 extends LinearOpMode {
//
//    public class Extention {
//        private final DcMotorEx extension;
//
//        public Extension(HardwareMap hardwareMap) {
//            extension = hardwareMap.get(DcMotorEx.class, "extension");
//            extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            extension.setDirection(DcMotorSimple.Direction.REVERSE);
//            extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            extension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        }
//
//        public class ExtensionUp implements Action {
//            private boolean initialized = false;
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                if (!initialized) {
//                    extension.setPower(0.4);
//                    initialized = true;
//                }
//
//                double pos = extension.getCurrentPosition();
//                packet.put("extensionPos", pos);
//                if (pos < 2000)
//                    return true;
//                else {
//                    extension.setPower(0);
//                    return false;
//                }
//            }
//        }
//        public Action extensionUp() {
//            return new ExtensionUp();
//        }
//
//
//        public class ExtensionDown implements Action {
//            private boolean initialized = false;
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                if (!initialized) {
//                    extension.setPower(-0.1);
//                    initialized = true;
//                }
//
//                double pos = extension.getCurrentPosition()
//            }
//        }
//    }
//}
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
