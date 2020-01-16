package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="StrafeTest", group ="Autonomous")
@Disabled
public class StrafeTest extends AutoMethod {

    @Override

    public void startOpMode() {
        robot.pattern = RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_RED;
        robot.blinkinLedDriver.setPattern(robot.pattern);
        waitForStart();

    /** Step through each leg of the path, Note: Reverse movement is obtained by setting a negative distance (not speed)
     Put a hold after each turn   */
/**
 * Use this number to calculate Strafe fudge factor.
 * Start by setting fudge factor to 1 and then divide distance requested by distance done.
 * Repeat multiple times for more accuracy
 */
    //Step 1 move robot to the right side 36 inches away from start
           Strafe_Left(robot.DRIVE_SPEED,36,0);

    //end program
            stop();

        }
    }




