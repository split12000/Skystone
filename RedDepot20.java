package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="RedDepot20", group ="Autonomous")
@Disabled
public class RedDepot20 extends AutoMethod {

    @Override
    public void startOpMode() {
        robot.color = 0;
        if (robot.color==0) {
            robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_RED);
        } else {robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_BLUE);}
        Prep();
        waitForStart();
    /** Step through each leg of the path, Note: Reverse movement is obtained by setting a negative distance (not speed)
     Put a hold after each turn   */

    //Step 1 move robot away from start
            gyroDrive(robot.DRIVE_SPEED, 18, 0);
    //Step 2 move robot left to match up with stones
            Strafe_Left(robot.DRIVE_SPEED,18,0);
    //Step 3 drive to stones
            BrickPrep();
    //Step 4 grab stone in front of robot
            BrickGrab();
    //Step 5 turn to skybridge
            gyroTurn(robot.TURN_SPEED,270);
            gyroHold(robot.TURN_SPEED,270,.5);
    //Step 6 drive to build platform
            gyroDrive(robot.DRIVE_SPEED,92,270);
    //Step 7 turn to build platform
            gyroTurn(robot.TURN_SPEED,0);
            gyroHold(robot.TURN_SPEED,0,0.5);
    //Step 8 drive to build platform
            gyroDrive(robot.DRIVE_SPEED,3,0);
    //Step 9 drop brick
            BrickDrop();
    //Step 10 back up and drive to skybridge
            gyroTurn(robot.TURN_SPEED,90);
            gyroHold(robot.TURN_SPEED,90,0.5);
            gyroDrive(robot.DRIVE_SPEED,37,90);
    //end program
            stop();

        }
    }




