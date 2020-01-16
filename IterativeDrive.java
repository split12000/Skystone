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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//import java.security.KeyRep;

@TeleOp(
        name = " Old Mecanum Iterative",
        group = "Opmode"
)
@Disabled
public class IterativeDrive extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    //Calls Robot Hardware and assigns a variable name
    Hardware_2019         robot   = new Hardware_2019();   // Use a Our 2018 hardware



    public void init() { //Starts this loop when you press the INIT Button
        robot.init(hardwareMap);
             robot.liftUpDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void start() {  runtime.reset();  }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() { //Starts this loop after you press the START Button
        /**
         * Functional control of up down lift with no limit switches Meet 0 ready
         */double liftstate=0;
         boolean aIsPressed = false;
        if (robot.liftup.getState()){
            liftstate=1 ;
        }
        else {
            liftstate = 0;
        }
        double liftposition = robot.liftUpDown.getCurrentPosition();
        telemetry.addData("Lift Position","%5.2f",liftposition);
        telemetry.addData("LiftState", "%5.2f",liftstate);
        telemetry.update();

/**Main drive controls
 * Driver 1
 */
        //How far am I pushing the left joystick (how fast should go)
        double r = Math.hypot((double) gamepad1.left_stick_x, (double) gamepad1.left_stick_y);
        //What angle am I pushing the left joystick (what angle should I strafe at)
        double robotAngle = Math.atan2((double) gamepad1.left_stick_y, (double) gamepad1.left_stick_x) - 0.7853981633974483;
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


        /**Driver Two
         */
        /**
         * Need controls to
         *               Automatically prep for block
         *               Automatically Grab Block
         *               Automatically Lift and Rotate for safe travel
         *
         *               Maybe predetermined locations based on number of pushes of a button.
         *               Automatically Drop once lifted to correct position
         */

        /**
         * Functional arm rotation without limit switches. Meet 0 ready
         */
        if ( gamepad2.dpad_up && robot.rotateup.getState() == true){ //Twists lift up
            robot.liftRotate.setPower(0.5);
        }
        else if (gamepad2.dpad_down && robot.rotatedown.getState() == true){ //Twists lift down
            robot.liftRotate.setPower(-0.5);
        }
        else robot.liftRotate.setPower(0);

        /**
         * claw controls a= open b= close
         * FUNCTIONAL
         */
        if (gamepad2.a){
            robot.claw1.setPosition(0);
            robot.claw2.setPosition(1);
        } else if (gamepad2.b){
            robot.claw1.setPosition(1);
            robot.claw2.setPosition(0);
        }



       if (gamepad2.right_bumper && robot.liftup.getState() == true){ // can be robot.liftup.getState()
            robot.liftUpDown.setPower(.99);//Raises lift
        }
            else if (gamepad2.left_bumper && robot.liftdown.getState() == true){
            robot.liftUpDown.setPower(-.99); } //Lowers lift

        /* else if(gamepad2.y && robot.liftdown.getState() == true && robot.liftup.getState() == true){
            aIsPressed = true;
           robot.liftUpDown.setTargetPosition(680);
           robot.liftUpDown.setPower(0.99);
           robot.liftUpDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);

           aIsPressed = false; }
           else if (!aIsPressed && !gamepad2.y){

               robot.liftUpDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
           }*/



            else robot.liftUpDown.setPower(0); //Turns off power when neither button is pressed

        if (robot.liftdown.getState() == false) {
            robot.liftUpDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.liftUpDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }


        /**
         * Next lines open the claw and drop the lift to Grab position++
         */
        double Prep;
        Prep = 0;
        if(gamepad2.x && Prep == 0){
            robot.claw1.setPosition(1);
            robot.claw2.setPosition(0);
            runtime.reset();
           Prep = 1;
        }
        if (Prep == 1 && robot.liftdown.getState() == true ){
            robot.liftUpDown.setPower(-0.99);


        }
        if (robot.liftdown.getState() == false)
            {robot.liftUpDown.setPower(0);
        Prep = 0;}


    }




    @Override
    public void stop() {
    }
}

