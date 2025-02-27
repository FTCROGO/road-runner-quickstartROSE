/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Robot: TeleOpROSE", group="Robot")
//@Disabled
public class TeleOpROSE extends LinearOpMode {

    public DcMotor  leftFront  = null;
    public DcMotor  rightFront  = null;
    public DcMotor  leftBack    = null;
    public DcMotor  rightBack   = null;
    public DcMotor  arm = null;
    public DcMotor  extension  = null;
    public Servo pitch   = null;
    public Servo munch   = null;
    public Servo yaw     = null;

    double pitchOffset = -1;
    double munchOffset = -1;
    double yawOffset = -1;


    @Override
    public void runOpMode() {
        double frontLeft;
        double frontRight;
        double backLeft;
        double backRight;
        double drive;
        double strafe;
        double turn;
        double maxBack;
        double maxFront;
        double armPower;
        double extensionPower;
        //double deMunchified = 0;

        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront  = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack  = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");
        arm = hardwareMap.get(DcMotor.class, "arm");
        extension  = hardwareMap.get(DcMotor.class, "extension");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.REVERSE);
        extension.setDirection(DcMotor.Direction.REVERSE);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pitch  = hardwareMap.get(Servo.class, "pitch");
        //pitch.setPosition(1);
        munch  = hardwareMap.get(Servo.class, "munch");
        //Munch.setPosition(0);
        yaw  = hardwareMap.get(Servo.class, "yaw");
        //yaw.setPosition(1.5);

        telemetry.addData(">", "Robot Ready.  Press START.");
        telemetry.update();



        waitForStart();

        while (opModeIsActive()) {

// Drivetrain -- forward/backward (y-axis joystick 1), left/right (x-axis joystick 1), turn (x-axis joystick 2)
            drive  = -gamepad1.left_stick_y/1.5;
            turn   =  gamepad1.right_stick_x/1.5;
            strafe =  gamepad1.left_stick_x/1.5;

            frontLeft  = drive + strafe + turn;
            frontRight = drive - strafe - turn;
            backLeft   = drive - strafe + turn;
            backRight  = drive + strafe - turn;

            maxBack = Math.max(Math.abs(backLeft), Math.abs(backRight));
            if (maxBack > 1.0)
            {
                backLeft   /= maxBack;
                backRight  /= maxBack;
            }

            maxFront = Math.max(Math.abs(frontLeft), Math.abs(frontRight));
            if (maxFront > 1.0)
            {
                frontLeft  /= maxFront;
                frontRight /= maxFront;
            }

            leftFront.setPower(frontLeft);
            rightFront.setPower(frontRight);
            leftBack.setPower(backLeft);
            rightBack.setPower(backRight);


// Arm - up (Y), down (A)
            armPower = gamepad2.left_stick_y/1.5;

            if (armPower > 1.0)
                armPower   /= armPower;

            arm.setPower(armPower);


    // Arm lower limit -------------------------------------------
//            if (arm.getCurrentPosition() < 10) {
//                arm.setTargetPosition(10);
//                arm.setPower(0.3);
//            }
//            else if (arm.getCurrentPosition() < 5) {
//                arm.setPower(0);
//            }

    // Arm upper limit --------------------------------------------
//            if (arm.getCurrentPosition() > 7130) {
//                arm.setTargetPosition(7130);
//                arm.setPower(-0.3);
//            }
//            else if (arm.getCurrentPosition() > 7140) {
//                arm.setPower(0);
//            }


// Linear Slide - up (dpad up), down (dpad down), zero position (x)
            extensionPower = -gamepad2.right_stick_y;

            if (extensionPower > 1.0)
                extensionPower   /= extensionPower;

            extension.setPower(extensionPower);

    // Linear slide power decreases as position gets higher -------------------------
//            if (mLS.getCurrentPosition() > a && mLSPower > 0)
//                mLSPower   /= 1.5;
//            else if (mLS.getCurrentPosition() > b && mLSPower > 0)
//                mLSPower   /= 1.75;
//            else if (mLS.getCurrentPosition() > c && mLSPower > 0)
//                mLSPower   /= 2;

    // Linear slide power cut in half as gets close to lower limit ------------------
//            if (mLS.getCurrentPosition() < d && mLSPower < 0)
//                mLSPower   /= 2;

    // Linear lower limit --------------------------------------------
//            if (mLS.getCurrentPosition() < 10) {
//                mLS.setTargetPosition(10);
//                mLS.setPower(0.5);
//            }
//            else if (mLS.getCurrentPosition() < 5) {
//                mLS.setPower(0);
//            }

    // Linear upper limit ---------------------------------------------
//            if (mLS.getCurrentPosition() > 29140) {
//                mLS.setTargetPosition(29140);
//                mLS.setPower(0.5);
//            }
//            else if (mLS.getCurrentPosition() > 29150)
//                mLS.setPower(0);


//Munch - open (left bumper), close (right bumpers) --------------------------
            if (gamepad2.left_bumper)
                munchOffset += 0.2;
            else if (gamepad2.right_bumper)
                munchOffset -= 0.2;
            //else
            //    munchOffset = 0;

            munchOffset = Range.clip(munchOffset, -0.5, 0.5);
            munch.setPosition(0.5 + munchOffset);

//Yaw -  left (left bumper), right (right bumpers)--------------------------
            yawOffset += (gamepad2.left_trigger - gamepad2.right_trigger)/100;

            yawOffset = Range.clip(yawOffset, 0.6, 0.93);
            yaw.setPosition(yawOffset);

//Pitch - left (left bumper), right (right bumpers)--------------------------
            pitchOffset += (gamepad1.left_trigger - gamepad1.right_trigger)/100;

            pitchOffset = Range.clip(pitchOffset, 0.25, 2);
            pitch.setPosition(pitchOffset);


// Send telemetry message to signify robot running;
            //telemetry.addData("intake pos",  "Offset = %.2f", intakeOffset);

            telemetry.addData("front left power",  "%.2f", frontLeft);
            telemetry.addData("fl pow", "%.2f", leftFront.getPower());
            telemetry.addData("front right power", "%.2f", frontRight);
            telemetry.addData("fr pow", "%.2f", rightFront.getPower());
            telemetry.addData("back left power",  "%.2f", backLeft);
            telemetry.addData("bl pow", "%.2f", leftFront.getPower());
            telemetry.addData("back right power", "%.2f", backRight);
            telemetry.addData("br pow", "%.2f", rightBack.getPower());

            telemetry.addData("arm power",  "%.2f", armPower);
            telemetry.addData("linear power", "%.2f", extensionPower);
            telemetry.addData("arm pos", arm.getCurrentPosition());
            telemetry.addData("linear pos", extension.getCurrentPosition());

            telemetry.addData("munch pos",  "Offset = %.2f", munchOffset);
            telemetry.addData("pitch pos",  "Offset = %.2f", pitchOffset);
            telemetry.addData("yaw pos",  "Offset = %.2f", yawOffset);

            telemetry.update();

        }
    }
}