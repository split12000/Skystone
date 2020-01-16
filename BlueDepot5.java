package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="BlueDepot5", group ="Autonomous")
@Disabled
public class BlueDepot5 extends AutoMethod {

    @Override

        public void startOpMode() {
        robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_BLUE);
        Prep();
        waitForStart();

    /** Step through each leg of the path, Note: Reverse movement is obtained by setting a negative distance (not speed)
     Put a hold after each turn   */

    //Step 1 move robot away from start
            gyroDrive(robot.DRIVE_SPEED, 24, 0);
    //Step 2 turn robot to drive to skybridge
            gyroTurn(robot.TURN_SPEED, 90);
            gyroHold(robot.TURN_SPEED, 90,.5);
   //Step 3 drive to skybridge
            gyroDrive(robot.DRIVE_SPEED,36,90);
    //end program
            stop();
        }
    }




