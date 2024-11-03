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

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.vision.CSVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

/**
 * This is NOT an opmode.
 *
 * This defines all of the hardware on a LIGHTSABERS robot.  It also includes usefull methods that
 * can be used across both Autonomous and TeleOp programs.
 */
@Config
public class LightSabersHardware
{
    //Variables for Servos and some Motors
    private OpMode myOpMode = null;

    public DcMotorEx droneMotor = null;
    public DcMotor LiftMotor = null;
    public TouchSensor touch;
    public DistanceSensor sensorRange;
    public Servo pixel;
    public Servo drone;
    public Servo claw_right;
    public Servo claw_left;
    public Servo shoulder_right;
    public Servo shoulder_left;
    public Servo wrist_right;
    public Servo wrist_left;

    //position of servos
    //Less is more down
    public static double shoulder_left_Down = 0.13;
    public static double shoulder_left_Up = 0.5;
    //More is more down
    public static double shoulder_right_Down = 0.87;
    public static double shoulder_right_Up = 0.5;
    //Less is more down
    public static double wrist_left_Pu = 0.92;//.86
    public static double wrist_left_Drive = 0.94; //.88
    public static double wrist_left_Drive_A = 0.91;
    public static double wrist_left_Score = 0.11; //.07
    //More is more down
    //Right is 2 less than necessary for 1.0
    public static double wrist_right_Pu = 0.15;//.14
    public static double wrist_right_Drive = 0.13; //.12
    public static double wrist_right_Drive_A = 0.07;
    public static double wrist_right_Score = 0.89; //.86

    public static double claw_left_Open = 0.27;  //.28
    public static double claw_left_Close = 0.22; //.23

    public static double claw_right_Open = 0.73; //.7
    public static double claw_right_Close = 0.79; //.76

    public static double droneVel = 2000;

    public boolean up = true;
    public boolean wrist_pickup = false;
    public boolean hand_close = false;
    public boolean hand_open = false;

    public static double liftLength = 0;

    //April Tag Variables
    public static final boolean USE_WEBCAM = true;
    public WebcamName webcam1;
    public AprilTagProcessor aprilTag;  /**The variable to store our instance of the AprilTag processor. */
    public CSVisionProcessor visionProcessor;  /**The variable to store our instance of the OpenCV processor.*/
    public VisionPortal myVisionPortal;  /**The variable to store our instance of the vision portal.*/

    public int DESIRED_TAG_ID = -1; // Choose the tag you want to approach or set to -1 for ANY tag.
    public AprilTagDetection desiredTag = null; // Used to hold the data for a detected AprilTag
    public boolean targetFound = false;

    /* local OpMode members. */

    public ElapsedTime runtime = new ElapsedTime();
    public ElapsedTime liftTime  = new ElapsedTime();
    public ElapsedTime handTime  = new ElapsedTime();
    public ElapsedTime visionTime  = new ElapsedTime();
    public ElapsedTime aprilTime = new ElapsedTime();

    //pixelPos is equivalent to the correct AprilTagID
    public int pixelPos = 0;
    public int right_dis = 0;
    public int left_strafe = 0;
    public int right_forward;
    public int liftNum = 285;
    public boolean backdropStrafe = false;
    //These are the offset numbers for the AprilTag driving
    public final double DESIRED_DISTANCE = 22.0;
    public final double DESIRED_DISTANCE_X = 4.0; //  this is how close the camera should get to the target (inches)

    public static boolean TagSeen = true;

    /* Constructor */
    public LightSabersHardware(OpMode opmode){
        myOpMode = opmode;
    }

    /* Initialize standard Hardware interfaces */
    public void init() {

        //MOTORS

        /** Drivetrain motors are defined in MacanumDrive and TeleOp programs. */

        LiftMotor = myOpMode.hardwareMap.get(DcMotorEx.class, "lift_motor");
        LiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        LiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        droneMotor = myOpMode.hardwareMap.get(DcMotorEx.class,"drone_motor");
        droneMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //SERVOS
        drone = myOpMode.hardwareMap.servo.get("drone_servo");
        claw_right = myOpMode.hardwareMap.servo.get("claw_right");
        claw_left = myOpMode.hardwareMap.servo.get("claw_left");
        shoulder_right = myOpMode.hardwareMap.servo.get("shoulder_right");
        shoulder_left = myOpMode.hardwareMap.servo.get("shoulder_left");
        wrist_right = myOpMode.hardwareMap.servo.get("wrist_right");
        wrist_left = myOpMode.hardwareMap.servo.get("wrist_left");

        //SENSORS
        touch = myOpMode.hardwareMap.get(TouchSensor.class, "lift_limit");
    }

    /** Vision */
    public void initDoubleVision() {
        /* AprilTag Configuration */
        aprilTag = new AprilTagProcessor.Builder().build();

        /* OpenCV Configuration */
        visionProcessor = new CSVisionProcessor();

        /* Camera Configuration */
        //webcam2 = opMode.hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam1 = myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1");
        CameraName switchableCamera = ClassFactory.getInstance()
                .getCameraManager().nameForSwitchableCamera(webcam1);

        myVisionPortal = new VisionPortal.Builder()
                .setCamera(switchableCamera)
                .addProcessors(visionProcessor, aprilTag)
                .build();
    }


    /** Intake Commands */
    public void closeFingers(){
        claw_left.setPosition(claw_left_Close);
        claw_right.setPosition(claw_right_Close);
    }

    public void openFingers(){
        claw_left.setPosition(claw_left_Open);
        claw_right.setPosition(claw_right_Open);
    }

    public void wristPickUp(){
        wrist_right.setPosition(wrist_right_Pu);
        wrist_left.setPosition(wrist_left_Pu);
    }

    public void wristDrive(){
        wrist_right.setPosition(wrist_right_Drive);
        wrist_left.setPosition(wrist_left_Drive);
    }

    public void wristScore(){
        wrist_right.setPosition(wrist_right_Score);
        wrist_left.setPosition(wrist_left_Score);
    }

    public void shoulderUp(){
        shoulder_right.setPosition(shoulder_right_Up);
        shoulder_left.setPosition(shoulder_left_Up);
    }

    public void shoulderAuto() {
        shoulder_right.setPosition(0.87);
        shoulder_left.setPosition(0.07);
    }

    public void shoulderDown(){
        shoulder_right.setPosition(shoulder_right_Down);
        shoulder_left.setPosition(shoulder_left_Down);
    }

    /** Encoder Lift Methods */

    // Drives lift to set encoder target position
    public void liftDrive(double speed, int LiftMotorTarget, double timeoutS) {                                                                                                                                                                                                                 // :)

        // Reset Lift Encoder
        LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set Lift Target Position
        LiftMotor.setTargetPosition(LiftMotorTarget);

        // Turn On RUN_TO_POSITION
        LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        LiftMotor.setPower(Math.abs(speed));

        while ((runtime.seconds() < timeoutS) && LiftMotor.isBusy()) {
            myOpMode.telemetry.addData("Target: ", "to %7d", LiftMotorTarget);
            myOpMode.telemetry.addData("Postion:", "at %7d", LiftMotor.getCurrentPosition());
            myOpMode.telemetry.update();
        }

        // Minimize Lift power to hold lift at position
        LiftMotor.setPower(0.05);
        LiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // Changes encoder mode and takes lift back to the bottom
    public void liftReset(double speed, double timeoutS){
        //Turn Lift motor off
        LiftMotor.setPower(0.0);

        // Turn off RUN_TO_POSITION
        LiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Drive Lift back to bottom
        while(touch.isPressed()) {
            LiftMotor.setPower(speed);
        }
        LiftMotor.setPower(0);
    }

}