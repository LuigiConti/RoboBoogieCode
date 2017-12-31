///* Copyright (c) 2017 FIRST. All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without modification,
// * are permitted (subject to the limitations in the disclaimer below) provided that
// * the following conditions are met:
// *
// * Redistributions of source code must retain the above copyright notice, this list
// * of conditions and the following disclaimer.
// *
// * Redistributions in binary form must reproduce the above copyright notice, this
// * list of conditions and the following disclaimer in the documentation and/or
// * other materials provided with the distribution.
// *
// * Neither the name of FIRST nor the names of its contributors may be used to endorse or
// * promote products derived from this software without specific prior written permission.
// *
// * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
// * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
// * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// */
//
//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorController;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
//
///**
// * This file illustrates the concept of driving a path based on encoder counts.
// * It uses the common Pushbot hardware class to define the drive on the robot.
// * The code is structured as a LinearOpMode
// *
// * The code REQUIRES that you DO have encoders on the wheels,
// *   otherwise you would use: PushbotAutoDriveByTime;
// *
// *  This code ALSO requires that the drive Motors have been configured such that a positive
// *  power command moves them forwards, and causes the encoders to count UP.
// *
// *   The desired path in this example is:
// *   - Drive forward for 48 inches
// *   - Spin right for 12 Inches
// *   - Drive Backwards for 24 inches
// *   - Stop and close the claw.
// *
// *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
// *  that performs the actual movement.
// *  This methods assumes that each movement is relative to the last stopping place.
// *  There are other ways to perform encoder based moves, but this method is probably the simplest.
// *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
// *
// * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
// * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
// */
//
//@Autonomous(name="Pushbot: Auto Drive By Encoder", group="Pushbot")
//@Disabled
//public class testMeAuto extends LinearOpMode {
//
//    /* Declare OpMode members. */
//    org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot robot   = new HardwarePushbot();   // Use a Pushbot's hardware
//    private ElapsedTime     runtime = new ElapsedTime();
//
//    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
//    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
//    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
//    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
//                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
//    static final double     DRIVE_SPEED             = 0.6;
//    static final double     TURN_SPEED              = 0.5;
////
////    @Override
////    public void runOpMode() throws InterruptedException {
//
//        // opticalDistanceSensor = hardwareMap.opticalDistanceSensor.get("sensor_ods");
////
////        motorFrontRight = hardwareMap.dcMotor.get("front_right");
////        motorFrontLeft = hardwareMap.dcMotor.get("front_left");
////        motorBackRight = hardwareMap.dcMotor.get("back_right");
////        motorBackLeft = hardwareMap.dcMotor.get("back_left");
////
////        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
////        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
////        motorFrontRight.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
////        motorFrontLeft.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
////        motorBackRight.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
////        motorBackLeft.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
////        waitForStart();
////        // Reset enoders to zero
////        motorFrontLeft.setMode(DcMotorController.RunMode.RESET_ENCODERS);
////        motorFrontRight.setMode(DcMotorController.RunMode.RESET_ENCODERS);
////        motorBackLeft.setMode(DcMotorController.RunMode.RESET_ENCODERS);
////        motorBackRight.setMode(DcMotorController.RunMode.RESET_ENCODERS);
////        waitOneFullHardwareCycle(); // We use these in attempt to gain stability.
////
////
////        motorFrontLeft.setTargetPosition(2440);
////        motorFrontRight.setTargetPosition(2440);
////        motorBackLeft.setTargetPosition(2440);
////        motorBackRight.setTargetPosition(2440);
////
////        leftFrontPos = motorLeftFront.getCurrentPosition();
////        leftBackPos = motorLeftBack.getCurrentPosition();
////        rightFrontPos = motorFrontRight.getCurrentPosition();
////        rightBackPos = motorBackRight.getCurrentPosition();
////
////        motorFrontRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
////        motorFrontLeft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
////        motorBackRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
////        motorBackLeft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
////
////        while(leftBackPos < 2440 && leftFrontPos < 2440 && rightBackPos < 2440 && rightFrontPos < 2440){
////            motorFrontLeft.setPower(-0.5);
////            motorFrontRight.setPower(-0.5);
////            motorBackLeft.setPower(-0.5);
////            motorBackRight.setPower(-0.5);
////
////            leftFrontPos = motorLeftFront.getCurrentPosition();
////            leftBackPos = motorLeftBack.getCurrentPosition();
////            rightFrontPos = motorFrontRight.getCurrentPosition();
////            rightBackPos = motorBackRight.getCurrentPosition();
////        }
////
////        telemetry.addData("2 ", "motorFrontLeft:  " + String.format("%d", motorFrontLeft.getTargetPosition()));
////        telemetry.addData("3 ", "motorFrontRight:  " + String.format("%d", motorFrontRight.getTargetPosition()));
////        telemetry.addData("4 ", "motorBackLeft:  " + String.format("%d", motorBackLeft.getTargetPosition()));
////        telemetry.addData("5 ", "motorBackRight:  " + String.format("%d", motorBackRight.getTargetPosition()));
////    }
////}
