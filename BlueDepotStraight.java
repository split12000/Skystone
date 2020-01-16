package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="BlueDepotStraight", group ="Autonomous")
//@Disabled
public class BlueDepotStraight extends AutoMethod {

    @Override
    public void startOpMode() {

        robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_BLUE);
        Prep();
        waitForStart();
        //start Vuforia
        initVuforia();
        distance2base=76;
    /** Step through each leg of the path, Note: Reverse movement is obtained by setting a negative distance (not speed)
     Put a hold after each turn   */

    //Step 1 move robot away from start
            gyroDrive(robot.DRIVE_SPEED, 16, 0);
    //Step 3 Determine where brick is
            SkystoneDetectionRight();
    //Step 4 Move to brick
            if (Positionis=="left")//"right block reads y_position <-2
                 {
                strafe_Right(robot.DRIVE_SPEED,4,0);
                distance2base=distance2base+4;
                }
            else if (Positionis=="center")//center block reads y_position > -2
            {
                Strafe_Left(robot.DRIVE_SPEED,4,0);
                distance2base=distance2base-8;
            }
            else {
                Strafe_Left(robot.DRIVE_SPEED,16,0);
                distance2base=distance2base-16;
            }
            gyroDrive(robot.DRIVE_SPEED,2,0);
    //Step 5 drive to stones
            BrickPrep();
    //Step 6 grab stone in front of robot
            BrickGrab();
    //Step 7 turn to skybridge
            gyroTurn(robot.TURN_SPEED,90);
            gyroHold(robot.TURN_SPEED,90,.5);
    //Step 8 drive to build platform
            gyroDrive(robot.DRIVE_SPEED,distance2base,90);
    //Step 9 turn to build platform
            gyroTurn(robot.TURN_SPEED,0);
            gyroHold(robot.TURN_SPEED,0,0.5);
    //Step 10 drive to build platform
            gyroDrive(robot.DRIVE_SPEED,3,0);
    //Step 11 drop brick
            BrickDrop();
    //Step 12 back up and drive to skybridge
            gyroTurn(robot.TURN_SPEED,270);
            gyroHold(robot.TURN_SPEED,270,0.5);
            gyroDrive(robot.DRIVE_SPEED,35,270);
    //end program
            stop();
    }
    }




