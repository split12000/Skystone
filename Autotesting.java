package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="TEST", group ="Autonomous")
@Disabled
public class Autotesting extends AutoMethod {

    @Override
    public void startOpMode() {
        Prep();
        initVuforia();
        //SkystoneArray();
    waitForStart();
    /** Step through each leg of the path, Note: Reverse movement is obtained by setting a negative distance (not speed)
     Put a hold after each turn   */



    //Step 1 move robot away from start
          //  gyroDrive(robot.DRIVE_SPEED, 14, 0);
            runtime.reset();
            SkystoneDetectionRight();

        if (Positionis=="right"){
            gyroTurn(robot.TURN_SPEED,45);
            gyroHold(robot.TURN_SPEED,45,.5);}


    /*p 2 turn robot to ensure that the two left minerals are the two seen
            gyroTurn(robot.TURN_SPEED, 90);
            gyroHold(robot.TURN_SPEED, 90,.5);

            Strafe_Left(robot.DRIVE_SPEED,36,90);

*/
            stop();
        }
    }




