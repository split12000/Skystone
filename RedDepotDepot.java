package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.vuforia.Vuforia;

@Autonomous(name="RedDepotOut", group ="Autonomous")
//@Disabled
public class RedDepotDepot extends AutoMethod {

    @Override
    public void startOpMode() {
        robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_RED);
        Prep(); //This sets us into the 18"cube
        waitForStart();
        //start Vuforia
        initVuforia();
        distance2base=90;
    /** Step through each leg of the path, Note: Reverse movement is obtained by setting a negative distance (not speed)
     Put a hold after each turn   */
    //Step 1 move robot away from start
            gyroDrive(robot.DRIVE_SPEED, 12, 0);
    //Step 2 move robot left to line up with stones
            Strafe_Left(robot.DRIVE_SPEED,20,0);
    //Step 3 Determine where brick is
        SkystoneDetectionLeft();
    //Step 4 Move to brick
        if (Positionis=="right"){
            Strafe_Left(robot.DRIVE_SPEED,6,0);
            distance2base=distance2base-4;
        }
        else {strafe_Right(robot.DRIVE_SPEED,8,0);
            distance2base=distance2base-8;}
        gyroDrive(robot.DRIVE_SPEED,2,0);

    //Step 5 move robot closer to stones so Brick prep works
      //      gyroDrive(robot.DRIVE_SPEED,17,0);
    //Step 6 rotates mast, opens claws, and lowers lift before driving 8 inches forward
            BrickPrep();
    //Step 7 grab stone in front of robot
            BrickGrab();
    //Step 8 Return to Depot
            gyroDrive(robot.DRIVE_SPEED,-22,0);
    //Step 9 turn to skybridge
            gyroTurn(robot.TURN_SPEED,270);
            gyroHold(robot.TURN_SPEED,270,.5);
    //Step 10 drive to build platform
            gyroDrive(robot.DRIVE_SPEED,88,270);
    //Step 11 turn to build platform
            gyroTurn(robot.TURN_SPEED,0);
            gyroHold(robot.TURN_SPEED,0,0.5);
    //Step 12 drive to build platform
            gyroDrive(robot.DRIVE_SPEED,21,0);
    //Step 13 drop brick
            BrickDrop();
    //Step 14 back up and drive to skybridge
            gyroDrive(robot.DRIVE_SPEED,-20,0);
           gyroTurn(robot.TURN_SPEED,90);
            gyroHold(robot.TURN_SPEED,90,0.5);
            gyroDrive(robot.DRIVE_SPEED,30,90);

    //end program

            stop();

        }
    }




