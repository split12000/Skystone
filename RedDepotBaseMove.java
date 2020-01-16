package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="RedDepotBaseMove", group ="Autonomous")
@Disabled
public class RedDepotBaseMove extends AutoMethod {

    @Override
    public void startOpMode() {
                    robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_RED);
        Prep();
        waitForStart();
    /** Step through each leg of the path, Note: Reverse movement is obtained by setting a negative distance (not speed)
     Put a hold after each turn   */



    //Step 1 move robot away from start
            gyroDrive(robot.DRIVE_SPEED, 2, 0);
    //Step 2 move robot left to line up with stones
            Strafe_Left(robot.DRIVE_SPEED,16,0);
    //Step 3 move robot closer to stones so Brick prep works
            gyroDrive(robot.DRIVE_SPEED,17,0);
    //Step 4 rotates mast, opens claws, and lowers lift before driving 8 inches forward
            BrickPrep();
    //Step 5 grab stone in front of robot
            BrickGrab();
    //Step 6 Return to Depot
            gyroDrive(robot.DRIVE_SPEED,-22,0);

         //   strafe_Right(robot.DRIVE_SPEED,80,0);
    //Step 7 turn to skybridge
            gyroTurn(robot.TURN_SPEED,270);
            gyroHold(robot.TURN_SPEED,270,.5);
    //Step 8 drive to build platform
            gyroDrive(robot.DRIVE_SPEED,88,270);
    //Step 9 turn to build platform
            gyroTurn(robot.TURN_SPEED,0);
            gyroHold(robot.TURN_SPEED,0,0.5);
    //Step 10 drive to build platform
            gyroDrive(robot.DRIVE_SPEED,21,0);
    //Step 11 drop brick
            BrickDrop();

            //We would like to grab the build platform here
    //Step 12 back up and drive to skybridge
            gyroDrive(robot.DRIVE_SPEED,-20,0);
           gyroTurn(robot.TURN_SPEED,90);
            gyroHold(robot.TURN_SPEED,90,0.5);
            gyroDrive(robot.DRIVE_SPEED,30,90);
           // Strafe_Left(robot.DRIVE_SPEED,33,0);
    //end program

            stop();

        }
    }




