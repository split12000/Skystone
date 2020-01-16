/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode;
/*Program adapted and changed by Alissa Jurisch with help from our adult mentors
 */

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

/**
 * This code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 * This code requires Mecanum wheels.
 *
 *  This code ALSO requires that you have a REV Robotics hub with internal gyro with the name "imu"
 *
 *  This code requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 *  In order to calibrate the Gyro correctly, the robot must remain stationary during calibration.
 *  This is performed when the INIT button is pressed on the Driver Station.
 *  This code assumes that the robot is stationary when the INIT button is pressed.
 *  If this is not the case, then the INIT should be performed again.
 *  If your robot lands and moves around you will want to INIT again.
 *
 *  Note: all angles are referenced to the initial coordinate frame set during the
 *  the Gyro Calibration process.
 *
 */


@Disabled
public abstract class AutoMethod extends LinearOpMode {
    //Calls Robot Hardware and assigns a variable name
    Hardware_2019 robot = new Hardware_2019();   // Use a Our 2018 hardware
    PIDFCoefficients pidfCoefficients = new PIDFCoefficients(1, 1, 1, 0);

    public ElapsedTime runtime = new ElapsedTime();


    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    // Personal Vuforia key
    private static final String VUFORIA_KEY = " Ab8zBE//////AAABmYzZTQzGpU5Tsnyh/Ry01qRHmnySK+o+6WFiZHmhftfiAIqtSFTeTtD9cd3TpVuxtJ1YMaMEawjd/ZioW9jjAFF3sthi7POTW0tXO1ubdhU5Fsl+q1IS8ss457LoEZnGwbNzh6/oM+gCSf8yRBTUIxyVnP7WOb4trPFsmewgAiWVrhIp1aBz/4SMtPCgAbKiV4Ksecu8po2LGIT1epNcCO179kpwOnBUjyZnPwJwHQ/eo7bJZUZZ/h3SqKa436YXb/7NdVn4LZfYl50bk9T1ZngaL8XJzq37Of0Z+rOiuO4LUDkU/FPW3J3+g8hqGNQw3BCLIun3hb9KKR4JpCji/JOOeOW8X/nGSyNBSNSwIlFx ";
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;
    /**
     * Select which camera   FRONT is the correct choice for a webcam.
     * Valid choices are:  BACK or FRONT
     */
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = FRONT;
    private OpenGLMatrix lastLocation = null;
    public boolean targetVisible = false;
    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField = 36 * mmPerInch;

    String Positionis = "";


    private float phoneXRotate = 0;
    private float phoneYRotate = 0;
    private float phoneZRotate = 0;
    WebcamName webcamName = null;

    public float distance2base;


    @Override
    public void runOpMode() {
        robot.init(hardwareMap);


        // Initialize Rev internal IMU variables
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        /**Ensure the robot it stationary, then reset the encoders and calibrate the gyro.*/
        robot.leftFDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Let driver know what we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        /**Retrieve and initialize the IMU. We expect the IMU to be attached internally to the I2C port
         *on a Rev Expansion Hub, configured to be a sensor of type "IMU",
         *and named "imu".
         */
        robot.imu = hardwareMap.get(BNO055IMU.class, "imu");
        //Where the IMU is calibrated. May copy to after robot placed on field if having issues with repeatability.
        robot.imu.initialize(parameters);
        // Tells Driver that IMU is calibrating.
        telemetry.addData("Mode", "calibrating...");
        telemetry.update();
        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !robot.imu.isGyroCalibrated()) {
            sleep(5);
            idle();
            telemetry.addData(">", "Robot Ready.");
            telemetry.update();

            //Wait for the game to start (driver presses PLAY
            // waitForStart();
            robot.leftFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.liftUpDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.liftRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.liftUpDown.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            //calls subclass???
            startOpMode();
        }
    }

    public abstract void startOpMode();

    public void Prep() {
//makes sure we fit into 18" cube
        robot.liftRotate.setTargetPosition(450); //Set to new arm position on 12/11
        robot.liftRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftRotate.setPower(.5);
        sleep(500);
        robot.liftUpDown.setTargetPosition(700); //Set to new arm position on 12/11
        robot.liftUpDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftUpDown.setPower(.5);
    }

    /**
     * Method to prep to pick up skystone based on position known
     * 1) opens claws
     * 2) lowers lift
     * 3) drives forward 8 inches to reach stone
     */
    public void BrickPrep() {
        if (opModeIsActive()) {
            //Rotate arm up
            robot.liftRotate.setTargetPosition(4600);
            robot.liftRotate.setPower(1);
            robot.liftUpDown.setTargetPosition(-250);
            robot.liftUpDown.setPower(-1);
            while (robot.liftRotate.isBusy() && robot.rotateup.getState()) {
            }
            //Open Claws
            robot.claw1.setPosition(1);
            robot.claw2.setPosition(0);
            //robot.liftRotate.setPower(0);
            robot.liftUpDown.setTargetPosition(-1000);
            robot.liftUpDown.setPower(-1);
            while (robot.liftUpDown.isBusy()) {
            }
            // robot.liftUpDown.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            gyroDrive(robot.DRIVE_SPEED, 10, 0);
            sleep(100);

        }
        robot.liftUpDown.setPower(0);
    }

    /**
     * Method to pick up skystone based on position known
     * 1) closes claws
     * 2) raises lift
     * 3) rotates back to safe for travel under skybridge
     */
    public void BrickGrab() {
        if (opModeIsActive()) {
            //close claw
            robot.claw1.setPosition(0);
            robot.claw2.setPosition(1);
            sleep(500);
            //Rotate down to travel under Skybridge

            robot.liftRotate.setTargetPosition(6);
            robot.liftRotate.setPower(-0.8);
            while (robot.liftRotate.isBusy() && robot.rotatedown.getState()) {
            }
            gyroDrive(robot.DRIVE_SPEED, -5, 0);
            robot.liftRotate.setPower(0);
        }
    }

    /**
     * Method to drop skystone based on position known
     * 1) raises mast
     * 2) opens and closes claws
     * 3) rotates back to safe for travel under skybridge
     * 4) backs up robot to miss skybridge
     */
    public void BrickDrop() {
        if (opModeIsActive()) {
            gyroDrive(robot.DRIVE_SPEED, 8, 0);
            //rotate mast up
            robot.liftRotate.setTargetPosition(600);
            robot.liftRotate.setPower(0.9);
            while (robot.liftRotate.isBusy() && robot.rotateup.getState()) {
            }
            robot.liftUpDown.setTargetPosition(75);
            robot.liftUpDown.setPower(.9);
            while (robot.liftUpDown.isBusy()) {
            }

            //Open Claws
            robot.claw1.setPosition(1);
            robot.claw2.setPosition(0);

            robot.liftRotate.setPower(0);
            robot.liftUpDown.setTargetPosition(250);
            robot.liftUpDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftUpDown.setPower(0.5);
            while (robot.liftUpDown.isBusy() && robot.liftup.getState()) {
            }
            gyroDrive(robot.DRIVE_SPEED, -10, 0);
            robot.liftUpDown.setPower(0);

            robot.liftRotate.setTargetPosition(10);
            robot.liftRotate.setPower(-0.8);
            while (robot.liftRotate.isBusy() && robot.rotatedown.getState()) {
            }
            robot.liftRotate.setPower(0);
            robot.claw1.setPosition(0);
            robot.claw2.setPosition(1);
        }
    }

    /**
     * Method to drive on a fixed compass bearing (angle), based on encoder counts.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the desired position
     * 2) Driver stops the opmode running.
     *
     * @param speed    Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive(double speed,
                          double distance,
                          double angle) {

        int newLeftFTarget;
        int newRightFTarget;
        int newLeftBTarget;
        int newRightBTarget;

        int moveCounts;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * robot.COUNTS_PER_INCH);

            newLeftFTarget = robot.leftFDrive.getCurrentPosition() + moveCounts;
            newRightFTarget = robot.rightFDrive.getCurrentPosition() + moveCounts;
            newLeftBTarget = robot.leftBDrive.getCurrentPosition() + moveCounts;
            newRightBTarget = robot.rightBDrive.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.leftFDrive.setTargetPosition(newLeftFTarget);
            robot.rightFDrive.setTargetPosition(newRightFTarget);
            robot.leftBDrive.setTargetPosition(newLeftBTarget);
            robot.rightBDrive.setTargetPosition(newRightBTarget);

            robot.leftFDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.leftFDrive.setPower(speed);
            robot.rightFDrive.setPower(speed);
            robot.leftBDrive.setPower(speed);
            robot.rightBDrive.setPower(speed);

            // keep looping while we are still active, and ALL motors are running.
            while (opModeIsActive() &&
                    (robot.leftFDrive.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, robot.P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.leftFDrive.setPower(leftSpeed);
                robot.rightFDrive.setPower(rightSpeed);
                robot.rightBDrive.setPower(leftSpeed);
                robot.leftBDrive.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                telemetry.addData("Target", "%7d:%7d", newLeftFTarget, newRightFTarget, newLeftBTarget, newRightBTarget);
                telemetry.addData("Actual", "%7d:%7d:%7d:%7d", robot.leftFDrive.getCurrentPosition(),
                        robot.rightFDrive.getCurrentPosition(), robot.leftBDrive.getCurrentPosition(), robot.rightBDrive.getCurrentPosition());
                telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            robot.leftFDrive.setPower(0);
            robot.rightFDrive.setPower(0);
            robot.leftBDrive.setPower(0);
            robot.rightBDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void strafe_Right(double speed,
                             double distance,
                             double angle) {

        int newLeftFTarget;
        int newRightFTarget;
        int newLeftBTarget;
        int newRightBTarget;

        int moveCounts;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * robot.COUNTS_PER_INCH * robot.STRAFE);

            newLeftFTarget = robot.leftFDrive.getCurrentPosition() + moveCounts;
            newRightFTarget = robot.rightFDrive.getCurrentPosition() - moveCounts;
            newLeftBTarget = robot.leftBDrive.getCurrentPosition() - moveCounts;
            newRightBTarget = robot.rightBDrive.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.leftFDrive.setTargetPosition(newLeftFTarget);
            robot.rightFDrive.setTargetPosition(newRightFTarget);
            robot.leftBDrive.setTargetPosition(newLeftBTarget);
            robot.rightBDrive.setTargetPosition(newRightBTarget);

            robot.leftFDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), -1.0, 1.0); //changed max from 0.05 to .5
            robot.leftFDrive.setPower(speed);
            robot.rightFDrive.setPower(-speed);
            robot.leftBDrive.setPower(-speed);
            robot.rightBDrive.setPower(speed);

            // keep looping while we are still active, and ALL motors are running.
            while (opModeIsActive() &&
                    (robot.leftFDrive.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, robot.P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed + steer / 3;
                rightSpeed = speed - steer / 3;

                // Normalize speeds if either one exceeds +/- 1.0;
                Range.clip(-1, 1, leftSpeed);
                Range.clip(-1, 1, rightSpeed);
/*                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= -max;
                }*/

                robot.leftFDrive.setPower(leftSpeed);
                robot.rightFDrive.setPower(-rightSpeed);
                robot.rightBDrive.setPower(leftSpeed);
                robot.leftBDrive.setPower(-rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                telemetry.addData("Target", "%7d:%7d", newLeftFTarget, newRightFTarget, newLeftBTarget, newRightBTarget);
                telemetry.addData("Actual", "%7d:%7d", robot.leftFDrive.getCurrentPosition(),
                        robot.rightFDrive.getCurrentPosition());
                telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            robot.leftFDrive.setPower(0);
            robot.rightFDrive.setPower(0);
            robot.leftBDrive.setPower(0);
            robot.rightBDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void Strafe_Left(double speed,
                            double distance,
                            double angle) {

        int newLeftFTarget;
        int newRightFTarget;
        int newLeftBTarget;
        int newRightBTarget;

        int moveCounts;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {


            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * robot.COUNTS_PER_INCH * robot.STRAFE);

            newLeftFTarget = robot.leftFDrive.getCurrentPosition() - moveCounts;
            newRightFTarget = robot.rightFDrive.getCurrentPosition() + moveCounts;
            newLeftBTarget = robot.leftBDrive.getCurrentPosition() + moveCounts;
            newRightBTarget = robot.rightBDrive.getCurrentPosition() - moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.leftFDrive.setTargetPosition(newLeftFTarget);
            robot.rightFDrive.setTargetPosition(newRightFTarget);
            robot.leftBDrive.setTargetPosition(newLeftBTarget);
            robot.rightBDrive.setTargetPosition(newRightBTarget);

            robot.leftFDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0); //changed max from 0.05 to .5
            robot.leftFDrive.setPower(-speed);
            robot.rightFDrive.setPower(speed);
            robot.leftBDrive.setPower(speed);
            robot.rightBDrive.setPower(-speed);

            // keep looping while we are still active, and ALL motors are running.
            while (opModeIsActive() &&
                    (robot.leftFDrive.isBusy())) {

                // adjust relative speed based on heading error.

                error = getError(angle);
                steer = getSteer(error, robot.P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer / 3;
                rightSpeed = speed + steer / 3;

                // Normalize speeds if either one exceeds +/- 1.0;
                Range.clip(-1, 1, leftSpeed);
                Range.clip(-1, 1, rightSpeed);
                /*
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }
*/
                robot.leftFDrive.setPower(-leftSpeed);
                robot.rightFDrive.setPower(rightSpeed);
                robot.rightBDrive.setPower(-leftSpeed);
                robot.leftBDrive.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                telemetry.addData("Target", "%7d:%7d", newLeftFTarget, newRightFTarget, newLeftBTarget, newRightBTarget);
                telemetry.addData("Actual", "%7d:%7d", robot.leftFDrive.getCurrentPosition(),
                        robot.rightFDrive.getCurrentPosition());
                telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            robot.leftFDrive.setPower(0);
            robot.rightFDrive.setPower(0);
            robot.leftBDrive.setPower(0);
            robot.rightBDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Method to spin on central axis to point in a new direction.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the heading (angle)
     * 2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle Absolute Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn(double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, robot.P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     * Method to obtain & hold a heading for a finite amount of time
     * Move will stop once the requested time has elapsed
     *
     * @param speed    Desired speed of turn.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     * @param holdTime Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold(double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, robot.P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        robot.leftFDrive.setPower(0);
        robot.rightFDrive.setPower(0);
        robot.leftBDrive.setPower(0);
        robot.rightBDrive.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed  Desired speed of turn.
     * @param angle  Absolute Angle (in Degrees) relative to last gyro reset.
     *               0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *               If a relative angle is required, add/subtract from current heading.
     * @param PCoeff Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftFSpeed;
        double rightFSpeed;
        double leftBSpeed;
        double rightBSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= robot.HEADING_THRESHOLD) {
            steer = 0.0;
            leftFSpeed = 0.0;
            rightFSpeed = 0.0;
            leftBSpeed = 0.0;
            rightBSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);

            rightFSpeed = speed * steer;
            leftFSpeed = -rightFSpeed;
            rightBSpeed = rightFSpeed;
            leftBSpeed = -rightFSpeed;

        }

        // Send desired speeds to motors.
        robot.leftFDrive.setPower(leftFSpeed);
        robot.rightFDrive.setPower(rightFSpeed);
        robot.leftBDrive.setPower(leftBSpeed);
        robot.rightBDrive.setPower(rightBSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftFSpeed, rightFSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        /**
         * Note Expansion hubs must be mounted horizontally and in the up position to use IMU function
         */
        robotError = targetAngle - robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        //above line returns an Orientation object. Adding first angle allows for Z angle to be subtracted as itself.
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error  Error angle in robot relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public void initVuforia() {
        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code on the next line, between the double quotes.
         */
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam");

        //  Instantiate the Vuforia engine
        /*
         * Retrieve the camera we are to use.
         */
        webcamName = hardwareMap.get(WebcamName.class, "Webcam");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameterscamera = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameterscamera.vuforiaLicenseKey = VUFORIA_KEY;

        /**
         * We also indicate which camera on the RC we wish to use.
         */
        parameterscamera.cameraName = webcamName;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameterscamera);
    }

    // Loading trackables is not necessary for the Tensor Flow Object Detection engine.

    public void SkystoneDetectionLeft() {

        SkystoneArray();
            // Provide feedback as to where the robot is located (if we know).

            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);


                double y_position = translation.get(1);

                if (y_position < -2){
                    Positionis = "right";
                }
                else{
                    Positionis = "center";
                }

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            } else {
                Positionis = "left";
            }
            telemetry.addData("Skystone Position", Positionis);
           // telemetry.addData("Visible Target", "none");
                  telemetry.update();
    }


    public void SkystoneDetectionRight() {
        SkystoneArray();

        // Provide feedback as to where the robot is located (if we know).

        if (targetVisible) {
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

            double y_position = translation.get(1);

            if (y_position < -2) {
                Positionis = "center";
            } else
                Positionis = "left";

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
        telemetry.update();
        } else {
            Positionis = "right";
        }
        telemetry.addData("Skystone Position", Positionis);
        //telemetry.addData("Visible Target", "none");

        telemetry.update();
        }
    public void SkystoneArray() {

       int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameterscamera = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 4.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameterscamera.cameraDirection);
        }
        targetsSkyStone.activate();
        runtime.reset();
        while (opModeIsActive()&&runtime.time()<2) {

            // check all the trackable targets to see which one (if any) is visible.

            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;


                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }
        }
    }


}


