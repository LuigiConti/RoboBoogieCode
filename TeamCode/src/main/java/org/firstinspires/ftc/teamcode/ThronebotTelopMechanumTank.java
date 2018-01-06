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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.sun.tools.javac.api.ClientCodeWrapper;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Thronebot: MechanumTank", group="Thronebot")

public class ThronebotTelopMechanumTank extends OpMode{

//    public boolean g1xSlowSwitch = false;

    /* Declare OpMode members. */
    org.firstinspires.ftc.teamcode.HardwareThronebot robot = new org.firstinspires.ftc.teamcode.HardwareThronebot(); // use the class created to define a Pushbot's hardware

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        //ColorSensor colorSensor;
        //colorSensor = hardwareMap.get(ColorSensor.class, "Sensor_Color");

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        telemetry.addData("Say", "We pressed INIT!");
        robot.clawRight.setPosition(0);
        robot.clawLeft.setPosition(0);
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Controller 1
        double leftStick;
        double rightStick;
        double leftTrigger;
        double rightTrigger;
        boolean g1xSlowSwitch = false;

        // Controller 2
        double leftStick2;

        // CONTROLLER 1: Assign variables to stick and trigger

        // leftstick, rightstick, lefttrigger, righttrigger without numbers is for controller one
        if(!g1xSlowSwitch) {
            leftStick = gamepad1.left_stick_y * .25;
            rightStick = gamepad1.right_stick_y * .25;
            leftTrigger = gamepad1.left_trigger * .25;
            rightTrigger = gamepad1.right_trigger * .25;
        } else {
            leftStick = gamepad1.left_stick_y * .15;
            rightStick = gamepad1.right_stick_y * .15;
            leftTrigger = gamepad1.left_trigger * .15;
            rightTrigger = gamepad1.right_trigger * .15;
        }

        // SLOW-DOWN TOGGLE with 'x'
        if(gamepad1.x) {
            g1xSlowSwitch = !g1xSlowSwitch;
        }

        // CONTROLLER 2: Assign variables to stick, triggers, and button
        leftStick2 = gamepad2.left_stick_y;

        // display statements
        telemetry.addData("LEFT STICK VALUE: ",leftStick);
        telemetry.addData("RIGHT STICK VALUE: ",rightStick);
        telemetry.addData("LEFT TRIGGER POWER: ",leftTrigger);
        telemetry.addData("RIGHT TRIGGER POWER: ",rightTrigger);

        telemetry.addData("LEFT STICK 2 VALUE: ",leftStick2);
        telemetry.addData("GRIPPER BUTTON STATUS: ", gamepad2.a);

        // If the triggers are being pushed
        if(leftTrigger > 0 || rightTrigger > 0) {
            if (leftTrigger > 0) {
                robot.frontLeft.setPower(leftTrigger);
                robot.backLeft.setPower(-leftTrigger);
                robot.frontRight.setPower(-leftTrigger);
                robot.backRight.setPower(leftTrigger);
            } else if (rightTrigger > 0) {
                robot.frontLeft.setPower(-rightTrigger);
                robot.backLeft.setPower(rightTrigger);
                robot.frontRight.setPower(rightTrigger);
                robot.backRight.setPower(-rightTrigger);
            }
        } else {
            // movement with sticks
            robot.frontLeft.setPower(leftStick);
            robot.backLeft.setPower(leftStick);
            robot.backRight.setPower(rightStick);
            robot.frontRight.setPower(rightStick);
        }

        //Controller 2 Code:
        //If left stick is being moved
        if (leftStick2 != 0 ){
            robot.lift.setPower(leftStick2*.25);
            // If left stick is not being moved
        } else if(leftStick2 == 0) {
            robot.lift.setPower(0);

            // Claw toggle logic
            // BUTTON A
            if (gamepad2.a == true) {
                robot.clawLeft.setPosition(1);
                robot.clawRight.setPosition(1);
            } else if (gamepad2.b == true) {
                robot.clawLeft.setPosition(0);
                robot.clawRight.setPosition(0);
            } else if (gamepad2.a == false && gamepad2.b == false) {
                robot.clawLeft.setPosition(0.5);
                robot.clawRight.setPosition(0.5);
            }
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        telemetry.addData("Say", "It Stopped!");
    }
}