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
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorPIDControlLoopCoefficientsCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorTargetPositionCommand;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

import java.lang.annotation.Target;

/**
 * This is NOT an opmode.
 *
 * This class is used to define all the specific hardware for our robot.
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower and upper case and some may have single spaces between words.
 *
 * Motor channel:   Left front motor:       "lfm"
 * Motor channel:   Right front motor:      "rfm"
 * Motor channel:   Left back motor:        "lbm"
 * Motor channel:   Right back motor:       "rbm"
 * Motor channel:   Lift Motor:             "lift"
 *
 * servo channel:   Right Claw:             "claw1"
 * servo channel:   Left Claw:              "claw2"
 *
 * Digital channel: Lift Up:                "liftup"
 * Digital channel: Lift Down:              "liftdown"
 * Digital channel: Rotate Up:              "rotateup"
 * Digital channel: Rotate Down:            "rotatedown"
 *
 * I2C channel:  imu                        "imu"
 *
 * Webcam                                   "webcam"
 */
public class Hardware_2019 {

    //Mecanum motors
    public DcMotorEx leftFDrive;
    public DcMotorEx rightFDrive;
    public DcMotorEx leftBDrive;
    public DcMotorEx rightBDrive;
    public DcMotorEx liftRotate;
    public DcMotorEx liftUpDown;

    //servos
    public Servo claw1;
    public Servo claw2;
    public Servo drag1;
    public Servo drag2;

    //limit switches
    public DigitalChannel liftup;
    public DigitalChannel liftdown;
    public DigitalChannel rotateup;
    public DigitalChannel rotatedown;

    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;
    RevBlinkinLedDriver.BlinkinPattern pattern2;

    //IMU
    public BNO055IMU imu;
    public Orientation lastAngles = new Orientation();


    //Initialize Camera variables.
    static WebcamName webcamName;
    VuforiaLocalizer vuforia;


    // Motor Parameters and Calculations
    public static final double COUNTS_PER_MOTOR_REV_40 = 1120;  //REV HD 40:1 motor information
    public static final double COUNTS_PER_MOTOR_REV_20 = 560;     //REV HD 20:1 motor information
    public static final double DRIVE_GEAR_REDUCTION = 2;  //This is <1.0 if geared up  1.3 starting point for REV HD motor
    public static final double WHEEL_DIAMETER_INCHES = 4.0;  //This is for figuring circumference

    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV_20 * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    public static final double STRAFE = 1.24299; //fudge factor for slip and for the fact we are no longer driving 4*3.14 inches per revolution

    public static final double HEADING_THRESHOLD = 1;      // Fudge factor. As tight as we can make it with gyro
    public static final double P_TURN_COEFF = 0.07;     // Larger is less stable
    public static final double P_DRIVE_COEFF = 0.1;     // Larger is less stable

    public static final double DRIVE_SPEED = 0.95;
    public static final double TURN_SPEED = .95;

    public double color; //if 0 run red lights else run blue

    public int floorheight = 600; //Height of a block
    public int floorheightrotate=50; // Amount of pull back we are compensating for on the rotate motor
    public int BlockStack;

    // Field Parameters for Camera
    public static final float mmPerInch = 25.4f;
    public static final float mmFTCFieldWidth = (12 * 6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    public static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor
    // local OpMode members.
    HardwareMap hwMap;

    private ElapsedTime period = new ElapsedTime();

    // Constructor
    public Hardware_2019() {

        hwMap = null;
    }

    // Initialize standard Hardware interfaces
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // name our motors
        leftBDrive = (DcMotorEx) hwMap.dcMotor.get("lbm");      //Hub 3 Motor 0
        leftFDrive = (DcMotorEx) hwMap.dcMotor.get("lfm");      //Hub 3 Motor 1
        rightFDrive = (DcMotorEx) hwMap.dcMotor.get("rfm");     //Hub 3 Motor 2
        rightBDrive = (DcMotorEx) hwMap.dcMotor.get("rbm");     //Hub 3 Motor 3
        liftRotate = (DcMotorEx) hwMap.dcMotor.get("liftrot");  //Hub 2 Motor 0   //Yellowjacket
        liftUpDown = (DcMotorEx) hwMap.dcMotor.get("liftupmotor");   //Hub 2 Motor 1   //Rev 60

        //name our servos
        blinkinLedDriver = hwMap.get(RevBlinkinLedDriver.class, "blinkin");  //Hub 3 Servo 0
        claw1 = hwMap.servo.get("claw1");  //Hub 3 Servo 1
        claw2 = hwMap.servo.get("claw2");  //Hub 3 Servo 2
        drag1 = hwMap.servo.get("drag1");
        drag2 = hwMap.servo.get("drag2");

        //name our sensors
        liftup =hwMap.digitalChannel.get("liftup");         //Hub 2 Digital 7
        liftdown = hwMap.digitalChannel.get("liftdown");    //Hub 2 Digital 5
        rotateup = hwMap.digitalChannel.get("rotateup");    //Hub 2 Digital 3
        rotatedown = hwMap.digitalChannel.get("rotatedown");//Hub 2 Digital 1

        // name our front Webcam.
        webcamName = hwMap.get(WebcamName.class, "Webcam");

        //Set reverse motor direction where required.
        leftFDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFDrive.setDirection(DcMotor.Direction.REVERSE);
        liftRotate.setDirection(DcMotor.Direction.REVERSE);
        liftUpDown.setDirection(DcMotor.Direction.FORWARD);

        //Set our tolerances up so that we stop hunting for the end of a move
        leftFDrive.setTargetPositionTolerance(150);
        leftBDrive.setTargetPositionTolerance(150);
        rightFDrive.setTargetPositionTolerance(150);
        rightBDrive.setTargetPositionTolerance(150);
        liftUpDown.setTargetPositionTolerance(100);
        liftRotate.setTargetPositionTolerance(150);
    }
}

