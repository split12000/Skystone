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

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//import java.security.KeyRep;

@TeleOp(
        name = " Mecanum Iterative",
        group = "Opmode"
)
//@Disabled
public class MecanumIterative extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    //Calls Robot Hardware and assigns a variable name
    Hardware_2019 robot   = new Hardware_2019();   // Use a Our 2019 hardware

    /**
     * Global variables you want to change must go here
     */
    boolean down = false; // x button
    boolean up = false; // y button
    boolean downsafe = false; //liftdown limit switch
    boolean rotatesafe = false; //rotatedown limit switch
    boolean triggerpress=false;
    boolean bumperpress = false;
    public int BlockStack =0;

    public void init() { //Starts this loop when you press the INIT Button
        robot.init(hardwareMap);
             robot.liftUpDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
             robot.liftRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
             robot.liftRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
             robot.liftUpDown.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
             robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE);
        runtime.reset();
    }

    @Override
    public void start() { }

    /* Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() { //Starts this loop after you press the START Button
        /**
         * Functional control of up down lift with no limit switches Meet 0 ready
         */
        double liftposition = robot.liftUpDown.getCurrentPosition();
        double liftrotposition = robot.liftRotate.getCurrentPosition();
        String liftdownx= "disabled";
        if (robot.liftdown.getState()==false){liftdownx="engaged";}
        telemetry.addData("Lift Position","%5.2f",liftposition);
        telemetry.addData("LiftRot Position", "%5.2f", liftrotposition);
        //telemetry.addData("Block Stack", BlockStack);
        telemetry.addData("Lift Down", liftdownx.toString());
        telemetry.update();

/**Main drive controls
 * Driver 1
 */
        //How far am I pushing the left joystick (how fast should go)
        double r = Math.hypot((double) -gamepad1.left_stick_x, (double) gamepad1.left_stick_y);
        //What angle am I pushing the left joystick (what angle should I strafe at)
        double robotAngle = Math.atan2((double) gamepad1.left_stick_y, (double) -gamepad1.left_stick_x) - 0.7853981633974483;
        //Am I trying to turn as well  (should I be turning left or right)
        double rightX = (double) -gamepad1.right_stick_x;
        //Math calculations using above questions
        double v1 = r * Math.cos(robotAngle) + rightX;
        double v2 = r * Math.sin(robotAngle) - rightX;
        double v3 = r * Math.sin(robotAngle) + rightX;
        double v4 = r * Math.cos(robotAngle) - rightX;

        robot.leftFDrive.setPower(-v1);
        robot.rightFDrive.setPower(-v2);
        robot.leftBDrive.setPower(-v3);
        robot.rightBDrive.setPower(-v4);

/**
 * Drag servos
  */
        if (gamepad1.a){ //release
            robot.drag1.setPosition(0);
            robot.drag2.setPosition(1);
        } else if (gamepad1.b){//grab
            robot.drag1.setPosition(1);
            robot.drag2.setPosition(0);
        }

/**Mast and Lift controls
 *
 *
 * Driver Two
 *
 *
*/
        /**
         * Functional arm rotation with limit switches and encoder limits. Meet 2 ready
         */

        //Twists lift up after verifying that rotate up limit switch is not engaged and that step count is less than 4800
        if ( gamepad2.dpad_up && robot.rotateup.getState() == true && robot.liftRotate.getCurrentPosition() < 4800){
            robot.liftRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.liftRotate.setPower(1.0);
        }
        else if (gamepad2.dpad_down && robot.rotatedown.getState() == true && robot.liftRotate.getCurrentPosition() > 10){ //Twists lift down
            robot.liftRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.liftRotate.setPower(-1.0);
        }
        //required or lift rotate motor continues to run in last direction (breaks the motor shaft)
        else robot.liftRotate.setPower(0);

        /**
         * claw controls a= open b= close
         * FUNCTIONAL Meet 2 ready
         */
        if (gamepad2.a){
            robot.claw1.setPosition(0);
            robot.claw2.setPosition(1);
        } else if (gamepad2.b){
            robot.claw1.setPosition(1);
            robot.claw2.setPosition(0);
        }

        /**
         * Lift controls with limit switches and encoder position Meet 2 ready
         * right_trigger = lift
         * left_trigger = down
         */

        if ( gamepad2.right_trigger>= 0.2 && robot.liftup.getState() && robot.liftUpDown.getCurrentPosition() <=5400) {
            triggerpress=true;
            robot.liftUpDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.liftUpDown.setPower(.9);
            robot.liftRotate.setPower(.15);
        }
        if (gamepad2.left_trigger>=0.2 && robot.liftUpDown.getCurrentPosition()>=-50){
            triggerpress=true;
            robot.liftUpDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.liftUpDown.setPower(-0.9);
            robot.liftRotate.setPower(-0.15);
        }
       if (gamepad2.left_trigger<.2 && gamepad2.right_trigger<.2 && triggerpress ){
           robot.liftUpDown.setPower(0);
           robot.liftRotate.setPower(0);
           triggerpress=false;
       }

        int x;
       int y;
        double motorDelayTime;
        //Necessary Debounce to keep bumper from being seen as multiple touches
        motorDelayTime=.15;
        if (robot.liftUpDown.getCurrentPosition()<20){
            BlockStack =0;
        }
        //skips servos unless runtime is greater than 20 ms.
        if( runtime.time() > motorDelayTime ) {

            x = robot.liftUpDown.getCurrentPosition();
            y= robot.liftRotate.getCurrentPosition();
            if (gamepad2.right_bumper ) {
                robot.liftUpDown.setTargetPosition(x + robot.floorheight);
                robot.liftUpDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.liftUpDown.setPower(.95);
                robot.liftRotate.setTargetPosition(y + robot.floorheightrotate);
                robot.liftRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.liftRotate.setPower(.3);
                bumperpress=true;
                BlockStack= BlockStack + 1;

                //don't want to drive the cable too far loose checks that we can move a full block down
            } else if (gamepad2.left_bumper && x >= robot.floorheight ) {

                robot.liftUpDown.setTargetPosition(x - robot.floorheight);
                robot.liftUpDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.liftUpDown.setPower(-.8);
                BlockStack= BlockStack - 1;
}
            runtime.reset();
             }

            /**
             * Limit switch tests that reset the encoders Meet 3 ready
             */

            if (robot.rotatedown.getState() == false) {
                robot.liftRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.liftRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
            if (robot.liftdown.getState()==false){
                robot.liftUpDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.liftUpDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
    }

    @Override
    public void stop() {
    }
}


