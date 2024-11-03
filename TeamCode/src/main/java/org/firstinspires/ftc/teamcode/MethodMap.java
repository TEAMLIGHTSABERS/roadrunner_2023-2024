package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.NewHardwareMap;
import org.firstinspires.ftc.teamcode.vision.CSVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class MethodMap {

    /**
     * These are the variables used in our method map and in our opModes. Call these using method.(variable name)
     */
    LinearOpMode opMode;

    NewHardwareMap robot =   new NewHardwareMap();


    IMU               imu;                            // IMU device
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, correction;


    public ElapsedTime     runtime = new ElapsedTime();
    public static final double     PI = Math.PI;
    public static final double     COUNTS_PER_MOTOR_REV    = 537.7;    // eg: TETRIX Motor Encoder
    public static final double     DRIVE_GEAR_REDUCTION    = 1.0;     // This is < 1.0 if geared UP 0.025
    public static final double     WHEEL_DIAMETER_INCHES   = 3.78;     // For figuring circumference
    public static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * PI);
    public static final double     DRIVE_SPEED             = 0.8;
    public static final double     STRAFE_SPEED            = 0.6;
    public static final double     TURN_SPEED              = 0.25;
    public static final double     ROBOT_WIDTH = 13.7;
    public static final double     TURN_VALUE = 90;
    public static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    public static final double     P_TURN_COEFF            = 0.02;     // Larger is more responsive, but also less stable
    public static final double     P_DRIVE_COEFF           = 0.012;     // Larger is more responsive, but also less stable



    public double sensorNum = 40;
    public double sensorNumIn = 0;

    //pixelPos is equivelant to the correct AprilTagID
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


    //April Tag Variables
    public static final boolean USE_WEBCAM = true;

    public WebcamName webcam1;

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    public AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the OpenCV processor.
     */
    public CSVisionProcessor visionProcessor;

    /**
     * The variable to store our instance of the vision portal.
     */
    public VisionPortal myVisionPortal;

    public int DESIRED_TAG_ID = -1; // Choose the tag you want to approach or set to -1 for ANY tag.

    public AprilTagDetection desiredTag = null; // Used to hold the data for a detected AprilTag

    public boolean targetFound = false;

    /**
     * This is the constructor that is used to not only brings the opMode and NewHardwareMap classes into our Method Map,
     * but is also then called into our opModes allowing us to send information back and forth between the two classes
     */

    public MethodMap(LinearOpMode opMode, NewHardwareMap hardwareMap, AprilTagProcessor aprilTag,
                     CSVisionProcessor visionProcessor, VisionPortal myVisionPortal) {
        this.opMode = opMode;
        this.robot = hardwareMap;
        this.aprilTag = aprilTag;
        this.visionProcessor = visionProcessor;
        this.myVisionPortal = myVisionPortal;

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        /*parameters.mode                = IMU.SensorMode.IMU;
        parameters.angleUnit           = IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;*/

        imu = opMode.hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(orientationOnRobot);

        imu.initialize(parameters);

        opMode.telemetry.addData("Mode", "calibrating...");
        opMode.telemetry.update();

        imu.resetYaw();

        robot.FmotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FmotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BmotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BmotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        opMode.telemetry.addData("Mode", "faster than a fish in the Smoky Mountains");
        //telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        opMode.telemetry.update();

        robot.FmotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.FmotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BmotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BmotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    /**
     * This is our method database. You call these by using method.(Method name)
     */

    //April Tag Methods
    public void initDoubleVision() {
        // -----------------------------------------------------------------------------------------
        // AprilTag Configuration
        // -----------------------------------------------------------------------------------------

        aprilTag = new AprilTagProcessor.Builder().build();

        // -----------------------------------------------------------------------------------------
        // OpenCV Configuration
        // -----------------------------------------------------------------------------------------

        visionProcessor = new CSVisionProcessor();

        // -----------------------------------------------------------------------------------------
        // Camera Configuration
        // -----------------------------------------------------------------------------------------

        //webcam2 = opMode.hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam1 = opMode.hardwareMap.get(WebcamName.class, "Webcam 1");
        CameraName switchableCamera = ClassFactory.getInstance()
                .getCameraManager().nameForSwitchableCamera(webcam1);

        myVisionPortal = new VisionPortal.Builder()
                .setCamera(switchableCamera)
                .addProcessors(visionProcessor, aprilTag)
                .build();


    }// end initDoubleVision()



    /**
     * Add telemetry about AprilTag detections.
     */
    public void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        opMode.telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                opMode.telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                opMode.telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                opMode.telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                opMode.telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                opMode.telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                opMode.telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

    }   // end method telemetryAprilTag()

    /*
    Manually set the camera gain and exposure.
    This can only be called AFTER calling initAprilTag(), and only works for Webcams;
   */

    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (myVisionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (myVisionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            opMode.telemetry.addData("Camera", "Waiting");
            opMode.telemetry.update();
            while (!opMode.isStopRequested() && (myVisionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                opMode.sleep(20);
            }
            opMode.telemetry.addData("Camera", "Ready");
            opMode.telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!opMode.isStopRequested())
        {
            ExposureControl exposureControl = myVisionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                opMode.sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            opMode.sleep(20);
            GainControl gainControl = myVisionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            opMode.sleep(20);
        }
    }

    //Driving Methods

    /*public void liftDrive(double power, int time, boolean hold) throws InterruptedException {
        robot.LiftMotor.setPower(power);
        opMode.sleep(time);
        if(hold == true) {
            robot.LiftMotor.setPower(0.05);
        }
        else{
            robot.LiftMotor.setPower(0);
        }
    }*/

    /*public void gyroTelem() {
        opMode.telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        opMode.telemetry.update();
    }*/

    public void driveStraight(double speed,double inches,double timeoutS){                                                                                                                                                                                              //:(
        RobotLog.d("DS START");
        encoderDrive(speed, inches, inches, inches, inches, timeoutS);
        RobotLog.d("DS STOP");
    }

    public void strafe(double speed,double inches,double timeoutS){                                                                                                                                                                                              //Drive like a programmer
        encoderDrive(speed, inches, -inches, -inches, inches, timeoutS);
    }

    public void turnLeft (double speed,double degrees, double timeoutS){
        double inches = (PI/180) * (degrees)*  (ROBOT_WIDTH / 2);
        encoderDrive(speed, -inches, inches, -inches, inches, timeoutS);
    }

    public void turnRight (double speed, double degrees, double timeoutS){
        turnLeft (speed, -degrees, timeoutS);
    }


    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double FLeftInches, double FRightInches, double BLeftInches, double BRightInches, double timeoutS) {                                                                                                                                                                                                                 // :)

        int FLeftTarget;
        int FRightTarget;
        int BRightTarget;
        int BLeftTarget;


        // Ensure that the opmode is still active
        //while (opMode.opModeIsActive()) {

        //resetEncoders();
        robot.FmotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FmotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BmotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BmotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Determine new target position, and pass to motor controller
        FLeftTarget  = (int)(-FLeftInches * COUNTS_PER_INCH);
        FRightTarget = (int)(-FRightInches * COUNTS_PER_INCH);
        BLeftTarget = (int)(-BLeftInches * COUNTS_PER_INCH);
        BRightTarget = (int)(-BRightInches * COUNTS_PER_INCH);
        //FLeftTarget  = robot.FmotorLeft.getCurrentPosition() + (int)(FLeftInches * COUNTS_PER_INCH);
        //FRightTarget = robot.FmotorRight.getCurrentPosition() + (int)(FRightInches * COUNTS_PER_INCH);
        //BLeftTarget = robot.BmotorLeft.getCurrentPosition() + (int)(BLeftInches * COUNTS_PER_INCH);
        //BRightTarget = robot.BmotorRight.getCurrentPosition() + (int)(BRightInches * COUNTS_PER_INCH);

        robot.FmotorLeft.setTargetPosition(FLeftTarget);
        robot.FmotorRight.setTargetPosition(FRightTarget);
        robot.BmotorLeft.setTargetPosition(BLeftTarget);
        robot.BmotorRight.setTargetPosition(BRightTarget);

        // Turn On RUN_TO_POSITION
        //positionEncoders();
        robot.FmotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.FmotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.BmotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.BmotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Turn off RUN_TO_POSITION
        //robot.FmotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.FmotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.BmotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.BmotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // reset the timeout time and start motion.
        runtime.reset();
        robot.FmotorLeft.setPower(Math.abs(speed));
        robot.FmotorRight.setPower(Math.abs(speed));
        robot.BmotorLeft.setPower(Math.abs(speed));
        robot.BmotorRight.setPower(Math.abs(speed));


            /*if (FLeftInches <0 && FRightInches <0) { //Backwards
                robot.BmotorLeft.setPower(-speed);
                robot.BmotorRight.setPower(-speed);
            }else if (FLeftInches <0 && FRightInches >0) { //Left
                robot.BmotorLeft.setPower(-speed);
                robot.BmotorRight.setPower(speed);
            }else if (FLeftInches >0 && FRightInches <0) { //Right
                robot.BmotorLeft.setPower(speed);
                robot.BmotorRight.setPower(-speed);
            }else{                                        //Forwards
                robot.BmotorLeft.setPower(speed);
                robot.BmotorRight.setPower(speed);
            }*/

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (opMode.opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (robot.FmotorLeft.isBusy() && robot.FmotorRight.isBusy() && robot.BmotorLeft.isBusy() && robot.BmotorRight.isBusy())) {

            // Display it for the driver.
            opMode.telemetry.addData("Target: ", "to %7d :%7d :%7d :%7d", FLeftTarget,  FRightTarget, BLeftTarget, BRightTarget);
            opMode.telemetry.addData("Postion:", "at %7d :%7d :%7d :%7d", robot.FmotorLeft.getCurrentPosition(),
                    robot.FmotorRight.getCurrentPosition(),robot.BmotorLeft.getCurrentPosition(),
                    robot.BmotorRight.getCurrentPosition());
            opMode.telemetry.update();
            RobotLog.d("%7d,%7d,%7d,%7d,%7d,", FLeftTarget,robot.FmotorLeft.getCurrentPosition(),
                    robot.FmotorRight.getCurrentPosition(),robot.BmotorLeft.getCurrentPosition(),
                    robot.BmotorRight.getCurrentPosition());
        }

        // Stop all motion;
        robot.FmotorLeft.setPower(0);
        robot.FmotorRight.setPower(0);
        robot.BmotorLeft.setPower(0);
        robot.BmotorRight.setPower(0);


        // Turn off RUN_TO_POSITION
        //resetEncoders();
        robot.FmotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.FmotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BmotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BmotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        opMode.sleep(250); // optional pause after each move


    }

    // Drives lift to set encoder target position
    public void liftDrive(double speed, int LiftMotorTarget, double timeoutS) {                                                                                                                                                                                                                 // :)

        // Reset Lift Encoder
        robot.LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set Lift Target Position
        robot.LiftMotor.setTargetPosition(LiftMotorTarget);

        // Turn On RUN_TO_POSITION
        robot.LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        robot.LiftMotor.setPower(Math.abs(speed));

        while (opMode.opModeIsActive() && (runtime.seconds() < timeoutS) && robot.LiftMotor.isBusy()) {
            opMode.telemetry.addData("Target: ", "to %7d", LiftMotorTarget);
            opMode.telemetry.addData("Postion:", "at %7d", robot.LiftMotor.getCurrentPosition());
            opMode.telemetry.update();
        }

        // Minimize Lift power to hold lift at position
        robot.LiftMotor.setPower(0.05);
        robot.LiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // Changes encoder mode and takes lift back to the bottom
    public void liftReset(double speed, double timeoutS){
        //Turn Lift motor off
        robot.LiftMotor.setPower(0.0);

        // Turn off RUN_TO_POSITION
        robot.LiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Drive Lift back to bottom
        while(robot.touch.isPressed() && opMode.opModeIsActive()) {
            robot.LiftMotor.setPower(speed);
        }
        robot.LiftMotor.setPower(0);
    }

    public void gyroDrive ( double speed,
                            double distance,
                            double angle, double timeoutS) {

        RobotLog.d("GD START");
        int FLeftTarget;
        int FRightTarget;
        int BRightTarget;
        int BLeftTarget;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        //if (opMode.opModeIsActive()) {

        robot.FmotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FmotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BmotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BmotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Determine new target position, and pass to motor controller
        FLeftTarget  = (int)(-distance * COUNTS_PER_INCH);
        FRightTarget = (int)(-distance * COUNTS_PER_INCH);
        BLeftTarget = (int)(-distance * COUNTS_PER_INCH);
        BRightTarget = (int)(-distance * COUNTS_PER_INCH);

        // Set Target and Turn On RUN_TO_POSITION
        robot.FmotorLeft.setTargetPosition(FLeftTarget);
        robot.FmotorRight.setTargetPosition(FRightTarget);
        robot.BmotorLeft.setTargetPosition(BLeftTarget);
        robot.BmotorRight.setTargetPosition(BRightTarget);

        robot.FmotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.FmotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.BmotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.BmotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // start motion.
        runtime.reset();
        //speed = Range.clip(Math.abs(speed), 0.0, 1.0);
        robot.FmotorLeft.setPower(Math.abs(speed));
        robot.FmotorRight.setPower(Math.abs(speed));
        robot.BmotorLeft.setPower(Math.abs(speed));
        robot.BmotorRight.setPower(Math.abs(speed));

        // keep looping while we are still active, and BOTH motors are running.
        while (opMode.opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (robot.FmotorLeft.isBusy() && robot.FmotorRight.isBusy() && robot.BmotorLeft.isBusy() && robot.BmotorRight.isBusy() )) {

            // adjust relative speed based on heading error.
            error = getError(angle);
            steer = getSteer(error, P_DRIVE_COEFF);

            // if driving in reverse, the motor correction also needs to be reversed
            if (distance < 0)
                steer *= -1.0;

            leftSpeed = speed - steer;
            rightSpeed = speed + steer;

            // Normalize speeds if either one exceeds +/- 1.0;
            max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
            if (max > 1.0)
            {
                leftSpeed /= max;
                rightSpeed /= max;
            }

            robot.FmotorLeft.setPower(leftSpeed);
            robot.BmotorLeft.setPower(leftSpeed);
            robot.FmotorRight.setPower(rightSpeed);
            robot.BmotorRight.setPower(rightSpeed);

            // Display drive status for the driver.
            opMode.telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
            opMode.telemetry.addData("Target",  "%7d:%7d:%7d:%7d",      FLeftTarget,  FRightTarget, BLeftTarget,  BRightTarget);
            opMode.telemetry.addData("Actual",  "%7d:%7d:%7d:%7d",      robot.FmotorLeft.getCurrentPosition(),robot.FmotorRight.getCurrentPosition(),
                    robot.BmotorLeft.getCurrentPosition(), robot.BmotorRight.getCurrentPosition());

            opMode.telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
            opMode.telemetry.update();
            RobotLog.d("%7d,%7d,%7d,%7d,%7d,", FLeftTarget,robot.FmotorLeft.getCurrentPosition(),
                    robot.FmotorRight.getCurrentPosition(),robot.BmotorLeft.getCurrentPosition(),
                    robot.BmotorRight.getCurrentPosition());
        }

        RobotLog.d("GD STOP");
        // Stop all motion;
        robot.FmotorLeft.setPower(0);
        robot.FmotorRight.setPower(0);
        robot.BmotorLeft.setPower(0);
        robot.BmotorRight.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.FmotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.FmotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BmotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BmotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //}
    }
    /*public void gyroDriveLD ( double speed,
                              double distance,
                              double angle, double timeoutS) {

        RobotLog.d("GD START");
        int FLeftTarget;
        int FRightTarget;
        int BRightTarget;
        int BLeftTarget;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        //if (opMode.opModeIsActive()) {

            robot.FmotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.FmotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.BmotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.BmotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Determine new target position, and pass to motor controller
            FLeftTarget  = (int)(-distance * COUNTS_PER_INCH);
            FRightTarget = (int)(-distance * COUNTS_PER_INCH);
            BLeftTarget = (int)(-distance * COUNTS_PER_INCH);
            BRightTarget = (int)(-distance * COUNTS_PER_INCH);

            // Set Target and Turn On RUN_TO_POSITION
            robot.FmotorLeft.setTargetPosition(FLeftTarget);
            robot.FmotorRight.setTargetPosition(FRightTarget);
            robot.BmotorLeft.setTargetPosition(BLeftTarget);
            robot.BmotorRight.setTargetPosition(BRightTarget);

            robot.FmotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.FmotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.BmotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.BmotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            runtime.reset();
            //speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.FmotorLeft.setPower(Math.abs(speed));
            robot.FmotorRight.setPower(Math.abs(speed));
            robot.BmotorLeft.setPower(Math.abs(speed));
            robot.BmotorRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and BOTH motors are running.
            while (opMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.FmotorLeft.isBusy() && robot.FmotorRight.isBusy() && robot.BmotorLeft.isBusy() && robot.BmotorRight.isBusy() )) {

                if (!robot.touch.isPressed()) {
                    robot.LiftMotor.setPower(-0.8);
                }
                else if (robot.touch.isPressed()) {
                    robot.LiftMotor.setPower(0);
                };

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.FmotorLeft.setPower(leftSpeed);
                robot.BmotorLeft.setPower(leftSpeed);
                robot.FmotorRight.setPower(rightSpeed);
                robot.BmotorRight.setPower(rightSpeed);

                // Display drive status for the driver.
                opMode.telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                opMode.telemetry.addData("Target",  "%7d:%7d:%7d:%7d",      FLeftTarget,  FRightTarget, BLeftTarget,  BRightTarget);
                opMode.telemetry.addData("Actual",  "%7d:%7d:%7d:%7d",      robot.FmotorLeft.getCurrentPosition(),robot.FmotorRight.getCurrentPosition(),
                        robot.BmotorLeft.getCurrentPosition(), robot.BmotorRight.getCurrentPosition());

                opMode.telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                opMode.telemetry.update();
                RobotLog.d("%7d,%7d,%7d,%7d,%7d,", FLeftTarget,robot.FmotorLeft.getCurrentPosition(),
                        robot.FmotorRight.getCurrentPosition(),robot.BmotorLeft.getCurrentPosition(),
                        robot.BmotorRight.getCurrentPosition());
            }

            RobotLog.d("GD STOP");
            // Stop all motion;
            robot.FmotorLeft.setPower(0);
            robot.FmotorRight.setPower(0);
            robot.BmotorLeft.setPower(0);
            robot.BmotorRight.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.FmotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.FmotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.BmotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.BmotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //}
    }*/

    /*public void gyroDriveLU ( double speed,
                              double distance,
                              double angle, double timeoutS) {

        RobotLog.d("GD START");
        int FLeftTarget;
        int FRightTarget;
        int BRightTarget;
        int BLeftTarget;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        //if (opMode.opModeIsActive()) {

            robot.FmotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.FmotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.BmotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.BmotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Determine new target position, and pass to motor controller
            FLeftTarget  = (int)(-distance * COUNTS_PER_INCH);
            FRightTarget = (int)(-distance * COUNTS_PER_INCH);
            BLeftTarget = (int)(-distance * COUNTS_PER_INCH);
            BRightTarget = (int)(-distance * COUNTS_PER_INCH);

            // Set Target and Turn On RUN_TO_POSITION
            robot.FmotorLeft.setTargetPosition(FLeftTarget);
            robot.FmotorRight.setTargetPosition(FRightTarget);
            robot.BmotorLeft.setTargetPosition(BLeftTarget);
            robot.BmotorRight.setTargetPosition(BRightTarget);

            robot.FmotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.FmotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.BmotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.BmotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);;

            // start motion.
            runtime.reset();
            //speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.FmotorLeft.setPower(Math.abs(speed));
            robot.FmotorRight.setPower(Math.abs(speed));
            robot.BmotorLeft.setPower(Math.abs(speed));
            robot.BmotorRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and BOTH motors are running.
            while (opMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.FmotorLeft.isBusy() && robot.FmotorRight.isBusy() && robot.BmotorLeft.isBusy() && robot.BmotorRight.isBusy() )) {

                if (!robot.touch3.isPressed()) {
                    robot.LiftMotor.setPower(0.6);
                }
                else if (robot.touch3.isPressed()) {
                    robot.LiftMotor.setPower(0);
                };

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.FmotorLeft.setPower(leftSpeed);
                robot.BmotorLeft.setPower(leftSpeed);
                robot.FmotorRight.setPower(rightSpeed);
                robot.BmotorRight.setPower(rightSpeed);

                // Display drive status for the driver.
                opMode.telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                opMode.telemetry.addData("Target",  "%7d:%7d:%7d:%7d",      FLeftTarget,  FRightTarget, BLeftTarget,  BRightTarget);
                opMode.telemetry.addData("Actual",  "%7d:%7d:%7d:%7d",      robot.FmotorLeft.getCurrentPosition(),robot.FmotorRight.getCurrentPosition(),
                        robot.BmotorLeft.getCurrentPosition(), robot.BmotorRight.getCurrentPosition());

                opMode.telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                opMode.telemetry.update();
                RobotLog.d("%7d,%7d,%7d,%7d,%7d,", FLeftTarget,robot.FmotorLeft.getCurrentPosition(),
                        robot.FmotorRight.getCurrentPosition(),robot.BmotorLeft.getCurrentPosition(),
                        robot.BmotorRight.getCurrentPosition());
            }

            RobotLog.d("GD STOP");
            // Stop all motion;
            robot.FmotorLeft.setPower(0);
            robot.FmotorRight.setPower(0);
            robot.BmotorLeft.setPower(0);
            robot.BmotorRight.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.FmotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.FmotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.BmotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.BmotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       // }
    }*/

    public void gyroDriveAcc ( double speed,
                               double distance,
                               double angle, double timeoutS) {

        RobotLog.d("GD START");
        int FLeftTarget;
        int FRightTarget;
        int BRightTarget;
        int BLeftTarget;
        double CurrPos;

        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;
        double  tempLeftSpeed;
        double  tempRightSpeed;

        // Ensure that the opmode is still active
        //if (opMode.opModeIsActive()) {

        robot.FmotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FmotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BmotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BmotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Determine new target position, and pass to motor controller
        FLeftTarget  = (int)(-distance * COUNTS_PER_INCH);
        FRightTarget = (int)(-distance * COUNTS_PER_INCH);
        BLeftTarget = (int)(-distance * COUNTS_PER_INCH);
        BRightTarget = (int)(-distance * COUNTS_PER_INCH);

        // Set Target and Turn On RUN_TO_POSITION
        robot.FmotorLeft.setTargetPosition(FLeftTarget);
        robot.FmotorRight.setTargetPosition(FRightTarget);
        robot.BmotorLeft.setTargetPosition(BLeftTarget);
        robot.BmotorRight.setTargetPosition(BRightTarget);

        robot.FmotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.FmotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.BmotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.BmotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);;

        // start motion.
        runtime.reset();
        //speed = Range.clip(Math.abs(speed), 0.0, 1.0);
        robot.FmotorLeft.setPower(Math.abs(speed));
        robot.FmotorRight.setPower(Math.abs(speed));
        robot.BmotorLeft.setPower(Math.abs(speed));
        robot.BmotorRight.setPower(Math.abs(speed));

        // keep looping while we are still active, and BOTH motors are running.
        while (opMode.opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (robot.FmotorLeft.isBusy() && robot.FmotorRight.isBusy() && robot.BmotorLeft.isBusy() && robot.BmotorRight.isBusy() )) {

            // adjust relative speed based on heading error.
            error = getError(angle);
            steer = getSteer(error, P_DRIVE_COEFF);

            // if driving in reverse, the motor correction also needs to be reversed
            if (distance < 0)
                steer *= -1.0;

            leftSpeed = speed - steer;
            rightSpeed = speed + steer;

            //Acc and Decc is here.
            CurrPos=Math.abs((robot.FmotorLeft.getCurrentPosition() + robot.FmotorRight.getCurrentPosition()))/2/COUNTS_PER_INCH;
            tempLeftSpeed = leftSpeed;
            tempRightSpeed = rightSpeed;
                /*if (CurrPos < 1)
                {
                    leftSpeed  = 0.2;
                    rightSpeed = 0.2;
                }*/
            if (CurrPos < 3)
            {
                leftSpeed  = tempLeftSpeed * CurrPos / 3;
                rightSpeed = tempRightSpeed * CurrPos / 3;
            }
                /*else if  (distance - CurrPos < 1)
                {
                    leftSpeed  = 0.2;
                    rightSpeed = 0.2;
                }*/
            else if ((distance - CurrPos) < 3)
            {
                leftSpeed  = tempLeftSpeed * (distance - CurrPos) / 3;
                rightSpeed = tempRightSpeed * (distance - CurrPos) / 3;
            }
            else
            {
                //Default full speed
            }

            // Normalize speeds if either one exceeds +/- 1.0;
            max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
            if (max > 1.0)
            {
                leftSpeed /= max;
                rightSpeed /= max;
            }

            robot.FmotorLeft.setPower(leftSpeed);
            robot.BmotorLeft.setPower(leftSpeed);
            robot.FmotorRight.setPower(rightSpeed);
            robot.BmotorRight.setPower(rightSpeed);

            // Display drive status for the driver.
            opMode.telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
            opMode.telemetry.addData("Target",  "%7d:%7d:%7d:%7d",      FLeftTarget,  FRightTarget, BLeftTarget,  BRightTarget);
            opMode.telemetry.addData("Actual",  "%7d:%7d:%7d:%7d",      robot.FmotorLeft.getCurrentPosition(),robot.FmotorRight.getCurrentPosition(),
                    robot.BmotorLeft.getCurrentPosition(), robot.BmotorRight.getCurrentPosition());

            opMode.telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
            opMode.telemetry.update();
            RobotLog.d("%7d,%7d,%7d,%7d,%7d,%5.2f:%5.2f", FLeftTarget,robot.FmotorLeft.getCurrentPosition(),
                    robot.FmotorRight.getCurrentPosition(),robot.BmotorLeft.getCurrentPosition(),
                    robot.BmotorRight.getCurrentPosition(), leftSpeed, rightSpeed);
        }

        RobotLog.d("GD STOP");
        // Stop all motion;
        robot.FmotorLeft.setPower(0);
        robot.FmotorRight.setPower(0);
        robot.BmotorLeft.setPower(0);
        robot.BmotorRight.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.FmotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.FmotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BmotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BmotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //}
    }

    // negative distance for left strafe, positive distance for right strafe

    public void gyroStrafe ( double speed,
                             double distance,
                             double angle, double timeoutS) {

        RobotLog.d("GS START");
        int FLeftTarget;
        int FRightTarget;
        int BRightTarget;
        int BLeftTarget;
        double  max;
        double  error;
        double  steer;
        double  frontSpeed;
        double  backSpeed;

        // Ensure that the opmode is still active
        //if (opMode.opModeIsActive()) {

        robot.FmotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FmotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BmotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BmotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Determine new target position, and pass to motor controller
        FLeftTarget  = (int)(distance * COUNTS_PER_INCH);
        FRightTarget = (int)(-distance * COUNTS_PER_INCH);
        BLeftTarget = (int)(-distance * COUNTS_PER_INCH);
        BRightTarget = (int)(distance * COUNTS_PER_INCH);

        // Set Target and Turn On RUN_TO_POSITION
        robot.FmotorLeft.setTargetPosition(FLeftTarget);
        robot.FmotorRight.setTargetPosition(FRightTarget);
        robot.BmotorLeft.setTargetPosition(BLeftTarget);
        robot.BmotorRight.setTargetPosition(BRightTarget);

        robot.FmotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.FmotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.BmotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.BmotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);;

        // start motion.
        runtime.reset();
        //speed = Range.clip(Math.abs(speed), 0.0, 1.0);
        robot.FmotorLeft.setPower(Math.abs(speed));
        robot.FmotorRight.setPower(Math.abs(speed));
        robot.BmotorLeft.setPower(Math.abs(speed));
        robot.BmotorRight.setPower(Math.abs(speed));

        // keep looping while we are still active, and BOTH motors are running.
        while (opMode.opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (robot.FmotorLeft.isBusy() && robot.FmotorRight.isBusy() && robot.BmotorLeft.isBusy() && robot.BmotorRight.isBusy() )) {

            // adjust relative speed based on heading error.
            error = getError(angle);
            steer = getSteer(error, P_DRIVE_COEFF);

            // if driving in reverse, the motor correction also needs to be reversed
            if (distance < 0)
                steer *= -1.0;

            frontSpeed = speed - steer;
            backSpeed = speed + steer;

            // Normalize speeds if either one exceeds +/- 1.0;
            max = Math.max(Math.abs(frontSpeed), Math.abs(backSpeed));
            if (max > 1.0)
            {
                frontSpeed /= max;
                backSpeed /= max;
            }

            robot.FmotorLeft.setPower(frontSpeed);
            robot.BmotorLeft.setPower(backSpeed);
            robot.FmotorRight.setPower(frontSpeed);
            robot.BmotorRight.setPower(backSpeed);

            // Display drive status for the driver.
            opMode.telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
            opMode.telemetry.addData("Target",  "%7d:%7d:%7d:%7d",      FLeftTarget,  FRightTarget, BLeftTarget,  BRightTarget);
            opMode.telemetry.addData("Actual",  "%7d:%7d:%7d:%7d",      robot.FmotorLeft.getCurrentPosition(),
                    robot.FmotorRight.getCurrentPosition(), robot.BmotorLeft.getCurrentPosition(),
                    robot.BmotorRight.getCurrentPosition());
            opMode.telemetry.addData("Speed",   "%5.2f:%5.2f",  frontSpeed, backSpeed);
            opMode.telemetry.update();
            RobotLog.d("%7d,%7d,%7d,%7d,%7d,", FLeftTarget,robot.FmotorLeft.getCurrentPosition(),
                    robot.FmotorRight.getCurrentPosition(),robot.BmotorLeft.getCurrentPosition(),
                    robot.BmotorRight.getCurrentPosition());
        }

        RobotLog.d("GS STOP");
        // Stop all motion;
        robot.FmotorLeft.setPower(0);
        robot.FmotorRight.setPower(0);
        robot.BmotorLeft.setPower(0);
        robot.BmotorRight.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.FmotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.FmotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BmotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BmotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //}
    }

    public void gyroAprilStrafe ( double speed,
                                  double distance,
                                  double angle, double timeoutS, int DESIRED_TAG_ID) {

        RobotLog.d("GS START");
        int FLeftTarget;
        int FRightTarget;
        int BRightTarget;
        int BLeftTarget;
        double  max;
        double  error;
        double  steer;
        double  frontSpeed;
        double  backSpeed;

        // Ensure that the opmode is still active
        //if (opMode.opModeIsActive()) {

        robot.FmotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FmotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BmotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BmotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Determine new target position, and pass to motor controller
        FLeftTarget  = (int)(distance * COUNTS_PER_INCH);
        FRightTarget = (int)(distance * COUNTS_PER_INCH);
        BLeftTarget = (int)(-distance * COUNTS_PER_INCH);
        BRightTarget = (int)(-distance * COUNTS_PER_INCH);

        // Set Target and Turn On RUN_TO_POSITION
        robot.FmotorLeft.setTargetPosition(FLeftTarget);
        robot.FmotorRight.setTargetPosition(FRightTarget);
        robot.BmotorLeft.setTargetPosition(BLeftTarget);
        robot.BmotorRight.setTargetPosition(BRightTarget);

        robot.FmotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.FmotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.BmotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.BmotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);;

        // start motion.
        runtime.reset();
        //speed = Range.clip(Math.abs(speed), 0.0, 1.0);
        robot.FmotorLeft.setPower(Math.abs(speed));
        robot.FmotorRight.setPower(Math.abs(speed));
        robot.BmotorLeft.setPower(Math.abs(speed));
        robot.BmotorRight.setPower(Math.abs(speed));

        // keep looping while we are still active, and BOTH motors are running.
        while (opMode.opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (robot.FmotorLeft.isBusy() && robot.FmotorRight.isBusy() && robot.BmotorLeft.isBusy() && robot.BmotorRight.isBusy() )) {

            // adjust relative speed based on heading error.
            error = getError(angle);
            steer = getSteer(error, P_DRIVE_COEFF);

            // if driving in reverse, the motor correction also needs to be reversed
            if (distance < 0)
                steer *= -1.0;

            frontSpeed = speed - steer;
            backSpeed = speed + steer;

            // Normalize speeds if either one exceeds +/- 1.0;
            max = Math.max(Math.abs(frontSpeed), Math.abs(backSpeed));
            if (max > 1.0)
            {
                frontSpeed /= max;
                backSpeed /= max;
            }

            robot.FmotorLeft.setPower(frontSpeed);
            robot.BmotorLeft.setPower(backSpeed);
            robot.FmotorRight.setPower(frontSpeed);
            robot.BmotorRight.setPower(backSpeed);

            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        break;  // don't look any further.
                    } /*else {
                        // This tag is in the library, but we do not want to track it right now.
                        opMode.telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    opMode.telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                    }*/
                }
            }

            // Display drive status for the driver.
            opMode.telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
            opMode.telemetry.addData("Target",  "%7d:%7d:%7d:%7d",      FLeftTarget,  FRightTarget, BLeftTarget,  BRightTarget);
            opMode.telemetry.addData("Actual",  "%7d:%7d:%7d:%7d",      robot.FmotorLeft.getCurrentPosition(),
                    robot.FmotorRight.getCurrentPosition(), robot.BmotorLeft.getCurrentPosition(),
                    robot.BmotorRight.getCurrentPosition());
            opMode.telemetry.addData("Speed",   "%5.2f:%5.2f",  frontSpeed, backSpeed);
            opMode.telemetry.update();
            RobotLog.d("%7d,%7d,%7d,%7d,%7d,", FLeftTarget,robot.FmotorLeft.getCurrentPosition(),
                    robot.FmotorRight.getCurrentPosition(),robot.BmotorLeft.getCurrentPosition(),
                    robot.BmotorRight.getCurrentPosition());
        }

        RobotLog.d("GS STOP");
        // Stop all motion;
        robot.FmotorLeft.setPower(0);
        robot.FmotorRight.setPower(0);
        robot.BmotorLeft.setPower(0);
        robot.BmotorRight.setPower(0);

        while(!targetFound) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        break;  // don't look any further.
                    }
                }
            }
        }

        // Turn off RUN_TO_POSITION
        robot.FmotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.FmotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BmotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BmotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //}
    }

    public void gyroSensorStrafe ( double speed,
                                   double distance,
                                   double angle, double timeoutS) {

        RobotLog.d("GS START");
        int FLeftTarget;
        int FRightTarget;
        int BRightTarget;
        int BLeftTarget;
        //double sensorNum;
        double  max;
        double  error;
        double  steer;
        double  frontSpeed;
        double  backSpeed;

        // Ensure that the opmode is still active
        //if (opMode.opModeIsActive()) {

        robot.FmotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FmotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BmotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BmotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Determine new target position, and pass to motor controller
        FLeftTarget  = (int)(-distance * COUNTS_PER_INCH);
        FRightTarget = (int)(distance * COUNTS_PER_INCH);
        BLeftTarget = (int)(distance * COUNTS_PER_INCH);
        BRightTarget = (int)(-distance * COUNTS_PER_INCH);

        // Set Target and Turn On RUN_TO_POSITION
        robot.FmotorLeft.setTargetPosition(FLeftTarget);
        robot.FmotorRight.setTargetPosition(FRightTarget);
        robot.BmotorLeft.setTargetPosition(BLeftTarget);
        robot.BmotorRight.setTargetPosition(BRightTarget);

        robot.FmotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.FmotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.BmotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.BmotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);;

        // start motion.
        runtime.reset();
        //speed = Range.clip(Math.abs(speed), 0.0, 1.0);
        robot.FmotorLeft.setPower(Math.abs(speed));
        robot.FmotorRight.setPower(Math.abs(speed));
        robot.BmotorLeft.setPower(Math.abs(speed));
        robot.BmotorRight.setPower(Math.abs(speed));

        //sensorNum = robot.sensorRange.getDistance(DistanceUnit.CM);

        // keep looping while we are still active, and BOTH motors are running.
        while (opMode.opModeIsActive() && (sensorNum > 20 ) &&
                (runtime.seconds() < timeoutS) &&
                (robot.FmotorLeft.isBusy() && robot.FmotorRight.isBusy() && robot.BmotorLeft.isBusy() && robot.BmotorRight.isBusy() )) {

            // adjust relative speed based on heading error.
            error = getError(angle);
            steer = getSteer(error, P_DRIVE_COEFF);

            // if driving in reverse, the motor correction also needs to be reversed
            if (distance < 0)
                steer *= -1.0;

            frontSpeed = speed - steer;
            backSpeed = speed + steer;

            // Normalize speeds if either one exceeds +/- 1.0;
            max = Math.max(Math.abs(frontSpeed), Math.abs(backSpeed));
            if (max > 1.0)
            {
                frontSpeed /= max;
                backSpeed /= max;
            }

            robot.FmotorLeft.setPower(frontSpeed);
            robot.BmotorLeft.setPower(backSpeed);
            robot.FmotorRight.setPower(frontSpeed);
            robot.BmotorRight.setPower(backSpeed);

            // Display drive status for the driver.
                    /*telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                    telemetry.addData("Target",  "%7d:%7d:%7d:%7d",      FLeftTarget,  FRightTarget, BLeftTarget,  BRightTarget);
                    telemetry.addData("Actual",  "%7d:%7d:%7d:%7d",      robot.FmotorLeft.getCurrentPosition(),
                            robot.FmotorRight.getCurrentPosition(), robot.BmotorLeft.getCurrentPosition(),
                            robot.BmotorRight.getCurrentPosition());
                    telemetry.addData("Speed",   "%5.2f:%5.2f",  frontSpeed, backSpeed);
                    telemetry.update();
                    RobotLog.d("%7d,%7d,%7d,%7d,%7d,", FLeftTarget,robot.FmotorLeft.getCurrentPosition(),
                                                             robot.FmotorRight.getCurrentPosition(),robot.BmotorLeft.getCurrentPosition(),
                                                             robot.BmotorRight.getCurrentPosition());*/

            //telemetry.addData("deviceName",robot.sensorRange.getDeviceName() );
            //telemetry.addData("range", String.format("%.01f cm", robot.sensorRange.getDistance(DistanceUnit.CM)));
            //telemetry.addData("range", String.format("%.01f in", robot.sensorRange.getDistance(DistanceUnit.INCH)));

            sensorNum = robot.sensorRange.getDistance(DistanceUnit.CM);
            sensorNumIn = (sensorNum/2.54);

            RobotLog.d("%.01f cm,%.01f in,",robot.sensorRange.getDistance(DistanceUnit.CM),robot.sensorRange.getDistance(DistanceUnit.INCH));

            //telemetry.update();
        }

        RobotLog.d("GS STOP");
        // Stop all motion;
        robot.FmotorLeft.setPower(0);
        robot.FmotorRight.setPower(0);
        robot.BmotorLeft.setPower(0);
        robot.BmotorRight.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.FmotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.FmotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BmotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BmotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //}
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn (double speed, double angle) {
        RobotLog.d("GT START");
        // keep looping while we are still active, and not on heading.
        while (opMode.opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.

            opMode.telemetry.update();
        }
        RobotLog.d("GT STOP");
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opMode.opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            opMode.telemetry.update();
        }

        // Stop all motion;
        robot.FmotorLeft.setPower(0);
        robot.FmotorRight.setPower(0);
        robot.BmotorLeft.setPower(0);
        robot.BmotorRight.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    public boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.FmotorLeft.setPower(-leftSpeed);
        robot.BmotorLeft.setPower(-leftSpeed);
        robot.FmotorRight.setPower(-rightSpeed);
        robot.BmotorRight.setPower(-rightSpeed);

        // Display it for the driver.
        opMode.telemetry.addData("Target", "%5.2f", angle);
        opMode.telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        opMode.telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        RobotLog.d("%5.2f,%5.2f,%5.2f,%5.2f, %5.2f", angle, error, steer, leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        //Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // calculate error in -179 to +180 range  (
        //robotError = targetAngle - angles.firstAngle;

        robotError = targetAngle - angles.getYaw(AngleUnit.DEGREES);
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1.0, 1.0);
    }
}
