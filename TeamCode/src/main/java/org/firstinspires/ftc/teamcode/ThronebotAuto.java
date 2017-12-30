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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backwards for 1 Second
 *   - Stop and close the claw.
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
         * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
            * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

    @Autonomous(name="Thronebot: Autonomous", group="Throne")

    public class ThronebotAuto extends LinearOpMode {

        /* Declare OpMode members. */
        org.firstinspires.ftc.teamcode.HardwareThronebot robot   = new org.firstinspires.ftc.teamcode.HardwareThronebot();   // Use a Pushbot's hardware
        private ElapsedTime runtime = new ElapsedTime();


    static final double     FORWARD_SPEED = .5;
    static final double     BACKWARD_SPEED= -.5;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        // INITIAL
        robot.frontLeft.setPower(0);
        robot.backLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setPower(1);
        robot.backLeft.setPower(1);
        robot.frontRight.setPower(1);
        robot.backRight.setPower(1);
        runtime.reset();
        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        // Step 1:  Drive forward for 1 seconds
        while(runtime.seconds() <1) {
            telemetry.addData("Status: ", "Forward 1 " + runtime.seconds());    //
            telemetry.update();
        }

        robot.frontLeft.setPower(0);
        robot.backLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backRight.setPower(0);
        sleep(3000);
        robot.frontLeft.setPower(-1);
        robot.backLeft.setPower(-1);
        robot.frontRight.setPower(-1);
        robot.backRight.setPower(-1);
        runtime.reset();
        // Step 2: Drive Backwards for 1 seconds
        while(runtime.seconds() <1) {
            telemetry.addData("Status: ", "BACKWARD " + runtime.seconds());    //
            telemetry.update();
        }

        robot.frontLeft.setPower(0);
        robot.backLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backRight.setPower(0);
        sleep(3000);
        robot.frontLeft.setPower(-1);
        robot.backLeft.setPower(1);
        robot.frontRight.setPower(1);
        robot.backRight.setPower(-1);
        runtime.reset();
        // Step 3(mecanum thingy): Strafe left for 1 seconds
        while(runtime.seconds() <1) {
            telemetry.addData("Status: ", "STRAFE LEFT " + runtime.seconds());    //
            telemetry.update();
        }
        runtime.reset();

        robot.frontLeft.setPower(0);
        robot.backLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backRight.setPower(0);
        sleep(3000);
        robot.frontLeft.setPower(1);
        robot.backLeft.setPower(-1);
        robot.frontRight.setPower(-1);
        robot.backRight.setPower(1);
        runtime.reset();
        // Step 4(mecanum thingy): Strafe right for 1 seconds
        while(runtime.seconds() <1) {
            telemetry.addData("Status: ", "STRAFE LEFT " + runtime.seconds());    //
            telemetry.update();
        }
        runtime.reset();

//
//        while (opModeIsActive() && (runtime.seconds() < 3.0)) {
//            telemetry.addData("Status: ", "FORWARD" + runtime.seconds());
//            telemetry.update();
//        }
//
//        // Step 2:  Spin right for 1.3 seconds
//        robot.leftDrive.setPower(FORWARD_SPEED);
//        robot.rightDrive.setPower(BACKWARD_SPEED);
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 1.3)) {
//            telemetry.addData("Status:", "TURN" + runtime.seconds());
//            telemetry.update();
//        }
//
//        // Step 3:  Drive Backwards for 3 Second
//        robot.leftDrive.setPower(FORWARD_SPEED);
//        robot.rightDrive.setPower(FORWARD_SPEED);
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 3.0)) {
//            telemetry.addData("Status: ", "BACKWARD" + runtime.seconds());
//            telemetry.update();
//        }
//
//        // Step 4:  Stop and close the claw.
//        robot.leftDrive.setPower(0);
//        robot.rightDrive.setPower(0);
//
//        telemetry.addData("Status", "Complete");
//        telemetry.update();
    }
}
