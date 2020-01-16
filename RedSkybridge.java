package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="RedSkybridge", group ="Autonomous")
//@Disabled
public class RedSkybridge extends AutoMethod {

    @Override
    public void startOpMode() {

            robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_RED);
        Prep();
        waitForStart();
        //start Vuforia
        initVuforia();
        distance2base=68;
    /** Step through each leg of the path, Note: Reverse movement is obtained by setting a negative distance (not speed)
     Put a hold after each turn   */

    //Step 1 move robot away from start
            gyroDrive(robot.DRIVE_SPEED, 12, 0);
    //Step 2 move robot left to match up with stones
            Strafe_Left(robot.DRIVE_SPEED,18,0);
    //Step 3 Determine where brick is
            SkystoneDetectionLeft();
    //Step 4 Drive to brick
        telemetry.addData("Skystone Position", Positionis);
        //telemetry.addData("Visible Target", "none");
        telemetry.update();
            if (Positionis=="right"){
                Strafe_Left(robot.DRIVE_SPEED, 8,0);
                distance2base=distance2base+8;
                 }
            else if (Positionis=="center"){
                strafe_Right(robot.DRIVE_SPEED,6,0);
                distance2base=distance2base-4;
                 }
            else {
                Strafe_Left(robot.DRIVE_SPEED,18,0);
                distance2base=distance2base-14;
            }
      //  gyroDrive(robot.DRIVE_SPEED, 2,0);
    //Step 5 drive forward to stone while positioning arm
            BrickPrep();
    //Step 6 grab stone in front of robot and positions arm down
            BrickGrab();
    //Step 7 turn to skybridge
            gyroTurn(robot.TURN_SPEED,270);
            gyroHold(robot.TURN_SPEED,270,.5);
    //Step 8 drive to build platform
            gyroDrive(robot.DRIVE_SPEED,distance2base,270);
    //Step 9 turn to build platform
            gyroTurn(robot.TURN_SPEED,0);
            gyroHold(robot.TURN_SPEED,0,0.5);
    //Step 10 drive to build platform
            gyroDrive(robot.DRIVE_SPEED,3,0);
    //Step 11 drop brick
          BrickDrop();
    //Step 12 back up and drive to skybridge
            gyroTurn(robot.TURN_SPEED,90);
            gyroHold(robot.TURN_SPEED,90,0.5);
            gyroDrive(robot.DRIVE_SPEED,37,90);
    //end program
            stop();
        }
    }




