package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="BlueDepotBaseMove", group ="Autonomous")
@Disabled
public class BlueDepotBaseMove extends AutoMethod {

    @Override

    public void startOpMode() {
        robot.color =1;
        if (robot.color==0) {
            robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_RED);
        } else {robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_BLUE);}
        Prep();
        waitForStart();
    /** Step through each leg of the path, Note: Reverse movement is obtained by setting a negative distance (not speed)
     Put a hold after each turn   */

    //Step 1 move robot away from start
            gyroDrive(robot.DRIVE_SPEED, 4, 0);
    //Step 2 move robot to the right to line up with stones
            strafe_Right(robot.DRIVE_SPEED,12,0);
    //Step 3 move robot closer to stones to prepare for brick prep
            gyroDrive(robot.DRIVE_SPEED,19,0);
    //Step 4 raises mast, opens claws, lowers lift, drives forward 8 inches
            BrickPrep();
    //Step 5 grab brick in front of robot
            BrickGrab();
    //Step 6 Return to Depot
            gyroDrive(robot.DRIVE_SPEED,-20,0);

           // Strafe_Left(robot.DRIVE_SPEED,80,0);
   //Step 7 turn to skybridge
            gyroTurn(robot.TURN_SPEED,90);
            gyroHold(robot.TURN_SPEED,90,.5);
    //Step 8 drive to build platform
            gyroDrive(robot.DRIVE_SPEED,78,90);
    //Step 9 turn to build platform
            gyroTurn(robot.TURN_SPEED,0);
            gyroHold(robot.TURN_SPEED,0,0.5);
    //Step 10 drive to build platform
            gyroDrive(robot.DRIVE_SPEED,23,0);
    //Step 11 drop brick
            BrickDrop();
            //We would like to grab the build platform here
    //Step 12 back up and drive to skybridge
            gyroDrive(robot.DRIVE_SPEED,-20,0);
           gyroTurn(robot.TURN_SPEED,270);
            gyroHold(robot.TURN_SPEED,270,0.5);
            gyroDrive(robot.DRIVE_SPEED,33,270);
           // strafe_Right(robot.DRIVE_SPEED,33,0);
    //end program
            stop();

        }
    }




