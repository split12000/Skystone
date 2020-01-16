package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="RedDepot5", group ="Autonomous")
@Disabled
public class RedDepot5 extends AutoMethod {

    @Override
    public void startOpMode() {
        robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_RED);
        Prep(); //Sets our robot into the 18”cube
        waitForStart();

    /** Step through each leg of the path, Note: Reverse movement is obtained by setting a negative distance (not speed)
     Put a hold after each turn   */

    //Step 1 move robot away from start
            gyroDrive(robot.DRIVE_SPEED, 24, 0);
    //Step 2 turn robot to drive to skybridge
            gyroTurn(robot.TURN_SPEED, 270);
            gyroHold(robot.TURN_SPEED, 270,.5);
   //Step 3 drive to skybridge
            gyroDrive(robot.DRIVE_SPEED,36,270);
    //end program
            stop();
        }
    }




