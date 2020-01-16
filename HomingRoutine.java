package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="ZHomingRoutine", group ="Autonomous")
//@Disabled
public class HomingRoutine extends AutoMethod {
    @Override
        public void startOpMode() {
        robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);
        waitForStart();
        //Resets the arm encoder using the rotatedown limit switch
        robot.liftRotate.setTargetPosition(500);
        robot.liftRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftRotate.setPower(0.5);
        //make sure we are off of the limit switch
        while (robot.liftRotate.isBusy() && robot.rotateup.getState()){
            sleep (1);
        }
        robot.liftRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //come down to the limit switch
         while (robot.rotatedown.getState()==true){
             robot.liftRotate.setPower(-0.4);
         }
             robot.liftRotate.setPower(0);
             robot.liftRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         robot.liftRotate.setTargetPosition(2000);
         robot.liftRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         robot.liftRotate.setPower(0.4);
         //run up to 2000 steps so that we can reset the arm lift
         while (robot.liftRotate.isBusy() && robot.rotateup.getState()){
             sleep (1);
         }
         //Resets Lift
        robot.liftUpDown.setTargetPosition(600);
         robot.liftUpDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         robot.liftUpDown.setPower(.5);
         //make sure we are tight by lifting the arm first
         while (robot.liftUpDown.isBusy() && robot.liftup.getState()){
             sleep(1);
         }
         robot.liftUpDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         //Then run the arm down until it hits the liftdown limit switch
         while (robot.liftdown.getState()){
             robot.liftUpDown.setPower(-0.2);
         }
             robot.liftUpDown.setPower(0);
             robot.liftUpDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         //Rotate mast back to zero
        robot.liftRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (robot.rotatedown.getState()){
            robot.liftRotate.setPower(-0.4);
        }
        robot.liftRotate.setPower(0);
        robot.liftRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Open claws
        robot.claw1.setPosition(1);
        robot.claw2.setPosition(0);
        //Give the claws time to open
        sleep(2000);
    //end program
         stop();

        }
    }




