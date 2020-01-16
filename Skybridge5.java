package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="Skybridge5", group ="Autonomous")
//@Disabled
public class Skybridge5 extends AutoMethod {

    @Override
    public void startOpMode() {
        robot.pattern = RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_RED;
        robot.blinkinLedDriver.setPattern(robot.pattern);
        waitForStart();
        Prep();

    /** Step through each leg of the path, Note: Reverse movement is obtained by setting a negative distance (not speed)
     Put a hold after each turn   */
    //Step 1 move robot away from start
            gyroDrive(robot.DRIVE_SPEED, 12, 0);
    //end program
            stop();
        }
    }




