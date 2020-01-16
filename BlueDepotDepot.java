package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="BlueDepotOut", group ="Autonomous")
//@Disabled
public class BlueDepotDepot extends AutoMethod {

    @Override

    public void startOpMode() {
        robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_BLUE);
        Prep();
        waitForStart();
        //start vuforia
        initVuforia();
        distance2base=90;

    /** Step through each leg of the path, Note: Reverse movement is obtained by setting a negative distance (not speed)
     Put a hold after each turn   */

    //Step 1 move robot away from start
            gyroDrive(robot.DRIVE_SPEED, 21, 0);
    //Step 2 move robot to the right to line up with stones
            strafe_Right(robot.DRIVE_SPEED,16,0);
    //Step 3 Determine brick position
            SkystoneDetectionLeft();
    //Step 4 Move to brick position
            if (Positionis=="center"){
                strafe_Right(robot.DRIVE_SPEED,6,0);
                distance2base=distance2base+8;
            }
            else {
                Strafe_Left(robot.DRIVE_SPEED,6,0);
                distance2base=distance2base-4;
            }
            gyroDrive(robot.DRIVE_SPEED,2,0);

    //Step 5 raises mast, opens claws, lowers lift, drives forward 8 inches
            BrickPrep();
    //Step 6 grab brick in front of robot
            BrickGrab();
    //Step 7 Return to Depot
            gyroDrive(robot.DRIVE_SPEED,-20,0);
    //Step 8 turn to skybridge
            gyroTurn(robot.TURN_SPEED,90);
            gyroHold(robot.TURN_SPEED,90,.5);
    //Step 9 drive to build platform
            gyroDrive(robot.DRIVE_SPEED,distance2base,90);
    //Step 10 turn to build platform
            gyroTurn(robot.TURN_SPEED,0);
            gyroHold(robot.TURN_SPEED,0,0.5);
    //Step 11 drive to build platform
            gyroDrive(robot.DRIVE_SPEED,23,0);
    //Step 12 drop brick
            BrickDrop();
            //We would like to grab the build platform here
    //Step 13 back up and drive to skybridge
            gyroDrive(robot.DRIVE_SPEED,-20,0);
            gyroTurn(robot.TURN_SPEED,270);
            gyroHold(robot.TURN_SPEED,270,0.5);
            gyroDrive(robot.DRIVE_SPEED,33,270);
    //end program
            stop();
        }
    }




