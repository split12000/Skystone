package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="BlueDepot20", group ="Autonomous")
@Disabled
public class BlueDepot20 extends AutoMethod {

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
            gyroDrive(robot.DRIVE_SPEED, 20, 0);
    //Step 2 moves robot to the right to line up with stones
            strafe_Right(robot.DRIVE_SPEED,12,0);
    //Step 3 drive to stones
            BrickPrep();
    //Step 4 grab stone in front of robot
            BrickGrab();
    //Step 5 turn to skybridge
            gyroTurn(robot.TURN_SPEED,90);
            gyroHold(robot.TURN_SPEED,90,.5);
    //Step 6 drive to build platform
            gyroDrive(robot.DRIVE_SPEED,89,90);
    //Step 7 turn to build platform
            gyroTurn(robot.TURN_SPEED,0);
            gyroHold(robot.TURN_SPEED,0,0.5);
    //Step 8 drive to build platform
            gyroDrive(robot.DRIVE_SPEED,3,0);
    //Step 9 drop brick
            BrickDrop();
    //Step 10 back up and drive to skybridge
            gyroTurn(robot.TURN_SPEED,270);
            gyroHold(robot.TURN_SPEED,270,0.5);
            gyroDrive(robot.DRIVE_SPEED,35,270);
    //end program
            stop();

        }
    }




