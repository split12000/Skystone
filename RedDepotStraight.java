package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="RedDepotStraight", group ="Autonomous")
//@Disabled
public class RedDepotStraight extends AutoMethod {
    @Override
    public void startOpMode() {
        robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_RED);
        Prep(); //Sets our robot into the 18”cube
        waitForStart();
        //start Vuforia
        initVuforia();
        distance2base=76;
    /** Step through each leg of the path, Note: Reverse movement is obtained by setting a negative distance (not speed)
     Put a hold after each turn   */
    //Step 1 move robot away from start
        gyroDrive(robot.DRIVE_SPEED, 16, 0);
    //Step 2 Determine where brick is
        SkystoneDetectionLeft();
    //Step 3 Move to brick
        if (Positionis=="left"){
            strafe_Right(robot.DRIVE_SPEED, 16,0);
            distance2base=distance2base-16;
        }else if (Positionis=="center"){
            strafe_Right(robot.DRIVE_SPEED,4,0);
            distance2base=distance2base-4;
        }else {
           gyroDrive(robot.DRIVE_SPEED,2,0);
            Strafe_Left(robot.DRIVE_SPEED,8,0);
            distance2base=distance2base+8;
        }
        gyroDrive(robot.DRIVE_SPEED,2,0);
    //Step 4 drive to stones
               BrickPrep();
    //Step 5 grab stone in front of robot
            BrickGrab();
    //Step 6 turn to skybridge
            gyroTurn(robot.TURN_SPEED,270);
            gyroHold(robot.TURN_SPEED,270,.5);
    //Step 7 drive to build platform
            gyroDrive(robot.DRIVE_SPEED,distance2base,270);
    //Step 8 turn to build platform
            gyroTurn(robot.TURN_SPEED,0);
            gyroHold(robot.TURN_SPEED,0,0.5);
    //Step 9 drive to build platform
            gyroDrive(robot.DRIVE_SPEED,3,0);
    //Step 10 drop brick
            BrickDrop();
    //Step 11 back up and drive to Skybridge
            gyroTurn(robot.TURN_SPEED,90);
            gyroHold(robot.TURN_SPEED,90,0.5);
            gyroDrive(robot.DRIVE_SPEED,35,90);
   //end program
           stop();
        }
    }




