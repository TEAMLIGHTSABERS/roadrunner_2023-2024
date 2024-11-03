package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.LightSabersHardware;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ParallelAction;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

// Non-RR imports

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Config
@Autonomous(name="BlueBackdrop", preselectTeleOp = "Tournament_TeleOp")

public class BlueBackdrop extends LinearOpMode {
    //Put variables and classes here
    LightSabersHardware robot =   new LightSabersHardware(this);


    public double drive_X = 0;
    public double drive_Y = 17.57;
    public double right_drop = 0;
    Pose2d initPose = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d dropPurplePixelPose = new Pose2d(0, 0, 0);
    Pose2d backFromPurplePose = new Pose2d(0, 0, 0);
    Pose2d leftMidwayPose = new Pose2d(0, 0, 0);
    Pose2d aprilPose = new Pose2d(30, 25, Math.toRadians(90));
    Pose2d parkPose = new Pose2d(0, 30, Math.toRadians(-90));
    Pose2d wallPose = new Pose2d(0, 45, Math.toRadians(-90));
    //Pose2d dropYellowPixelPose = new Pose2d(drive_X, drive_Y, Math.toRadians(90));

    Pose2d moveBeyondTrussPose = new Pose2d(15,0,0);

    public class Lift {
        public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    robot.liftTime.reset();
                    robot.LiftMotor.setPower(1.0);
                    initialized = true;
                }

                if (robot.liftTime.milliseconds() <= robot.liftLength) {
                    return true;
                } else {
                    robot.LiftMotor.setPower(0.05);
                    return false;
                }
            }
        }
        public Action LiftUp() {
            return new LiftUp();
        }

        public class LiftUpScore implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    robot.liftTime.reset();
                    robot.LiftMotor.setPower(1.0);
                    initialized = true;
                }

                if (robot.liftTime.milliseconds() <= 150) {
                    return true;
                } else {
                    robot.LiftMotor.setPower(0.05);
                    return false;
                }
            }
        }
        public Action LiftUpScore() {
            return new LiftUp();
        }

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    robot.LiftMotor.setPower(-0.8);
                    initialized = true;
                }

                if (robot.touch.isPressed()) {
                    return true;
                } else {
                    robot.LiftMotor.setPower(0);
                    return false;
                }
            }
        }
        public Action LiftDown() {
            return new LiftDown();
        }
    }

    public class Hand {
        public class HandDrive implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    robot.handTime.reset();
                    robot.wristDrive();
                    /*robot.wrist_right.setPosition(robot.wrist_right_Drive);
                    robot.wrist_left.setPosition(robot.wrist_left_Drive);*/
                    initialized = true;
                }

                if (robot.handTime.milliseconds() <= 500) {
                    return true;
                } else {
                    robot.shoulderDown();
                    /*robot.shoulder_right.setPosition(robot.shoulder_right_Down);
                    robot.shoulder_left.setPosition(robot.shoulder_left_Down);*/
                    return false;
                }
            }
        }
        public Action HandDrive() {
            return new HandDrive();
        }

        public class HandScore implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    robot.handTime.reset();
                    robot.shoulderUp();
                    /*robot.shoulder_right.setPosition(robot.shoulder_right_Up);
                    robot.shoulder_left.setPosition(robot.shoulder_left_Up);*/
                    initialized = true;
                }

                if (robot.handTime.milliseconds() <= 500) {
                    return true;
                } else {
                    robot.wristScore();
                    /*robot.wrist_right.setPosition(robot.wrist_right_Score);
                    robot.wrist_left.setPosition(robot.wrist_left_Score);*/
                    return false;
                }
            }
        }
        public Action HandScore() {
            return new HandScore();
        }
    }

    public class Claw {
        public class ClawClose implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.closeFingers();
                sleep(350);
                return false;
            }
        }
        public Action ClawClose() {
            return new ClawClose();
        }

        public class ClawOpen implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.openFingers();
                sleep(350);
                return false;
            }
        }
        public Action ClawOpen() {
            return new ClawOpen();
        }

        public class LeftClawOpen implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.claw_left.setPosition(robot.claw_left_Open);
                sleep(350);

                return false;
            }
        }
        public Action leftClawOpen() {
            return new LeftClawOpen();
        }

        public class RightClawOpen implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.claw_right.setPosition(robot.claw_right_Open);
                sleep(350);
                return false;
            }
        }
        public Action RightClawOpen() {
            return new RightClawOpen();
        }
    }


    @Override
    public void runOpMode() {
        /**
         * Initialize the hardware map and the method map constructor into the opMode
         */

        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);
        Lift lift = new Lift();
        Claw claw = new Claw();
        Hand hand = new Hand();

        robot.init();

        robot.closeFingers();
        robot.shoulderAuto();
        robot.drone.setPosition(0.4);
        robot.liftLength = 500;

        robot.initDoubleVision();

        while (!isStopRequested() && opModeInInit()) {
            telemetry.addData("Identified", robot.visionProcessor.getSelection());
            telemetry.update();
        }

        Action firstAction;


       waitForStart();

        robot.shoulderDown();
        robot.wristDrive();

        telemetry.addData("Identified", robot.visionProcessor.getSelection());

        //Deliver the pixel to the correct spike mark
        robot.myVisionPortal.setProcessorEnabled(robot.visionProcessor, false);

        setManualExposure(2, 150);

        switch (robot.visionProcessor.getSelection()) {
            case LEFT:
                robot.DESIRED_TAG_ID = 1;
                dropPurplePixelPose = new Pose2d(16, 13, Math.toRadians(0));
                backFromPurplePose = new Pose2d(8, 17, Math.toRadians(0));
                leftMidwayPose = new Pose2d(8, 25, Math.toRadians(0));
                drive_X = 15.3;
                break;
            case RIGHT:
                robot.DESIRED_TAG_ID = 3;
                dropPurplePixelPose = new Pose2d(22, -4, Math.toRadians(-45));
                backFromPurplePose = new Pose2d(16, 0, Math.toRadians(-45));
                leftMidwayPose = new Pose2d(16, 1, Math.toRadians(-45));
                drive_X = 29.3;
                right_drop = 2;
                break;
            case MIDDLE:
                robot.DESIRED_TAG_ID = 2;
                dropPurplePixelPose = new Pose2d(28, 4, Math.toRadians(0));
                backFromPurplePose = new Pose2d(24, 4, Math.toRadians(0));
                leftMidwayPose = new Pose2d(24, 5, Math.toRadians(0));
                drive_X = 21.3;
        }
        Actions.runBlocking(
                new SequentialAction(
               drive.actionBuilder(drive.pose)
                       .strafeToLinearHeading(moveBeyondTrussPose.position, moveBeyondTrussPose.heading)
                       .strafeToLinearHeading(dropPurplePixelPose.position, dropPurplePixelPose.heading)
                       .build(),
                        claw.RightClawOpen(),
                drive.actionBuilder(dropPurplePixelPose)
                        .strafeToLinearHeading(backFromPurplePose.position, backFromPurplePose.heading)
                        .build(),
                claw.ClawClose()));
        sleep(8000);
        Actions.runBlocking(
                new SequentialAction(
                drive.actionBuilder(backFromPurplePose)
                        .strafeToLinearHeading(leftMidwayPose.position, leftMidwayPose.heading)
                        .strafeToLinearHeading(aprilPose.position, aprilPose.heading)
                        .build())
                );
        robot.aprilTime.reset();
        while(!robot.targetFound && opModeIsActive() && robot.aprilTime.time() <= 3.0) {

            List<AprilTagDetection> currentDetections = robot.aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((robot.DESIRED_TAG_ID < 0) || (detection.id == robot.DESIRED_TAG_ID)) {
                        // Yes, we want to use this tag.
                        robot.targetFound = true;
                        robot.desiredTag = detection;
                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }
        }

        if(robot.targetFound = true) {
            drive_X = 30-4.7+right_drop+robot.desiredTag.ftcPose.x; // 5.7 -> 4.7, 6.7 for Right
            drive_Y = 25-17+robot.desiredTag.ftcPose.y; //11.43 -> 15.43 -> 20.43
        }

        /*drive_X = 30-4.7+right_drop+robot.desiredTag.ftcPose.x; // 5.7 -> 4.7, 6.7 for Right
        drive_Y = 25-20.43+robot.desiredTag.ftcPose.y; //11.43 -> 15.43 -> 20.43*/

        Pose2d dropYellowPixelPose = new Pose2d(drive_X, drive_Y, Math.toRadians(90));

        Actions.runBlocking(
                new ParallelAction(
                        lift.LiftUp(),
                        hand.HandScore()
                ));
        sleep(1000);
        Actions.runBlocking(
                new SequentialAction(
                drive.actionBuilder(aprilPose)
                        .strafeToLinearHeading(dropYellowPixelPose.position, dropYellowPixelPose.heading)
                        .build(),
                        claw.leftClawOpen(),
                        lift.LiftUpScore(),
                        drive.actionBuilder(dropYellowPixelPose)
                                .strafeToLinearHeading(aprilPose.position, aprilPose.heading)
                                .strafeToLinearHeading(parkPose.position, parkPose.heading)
                                .strafeToLinearHeading(wallPose.position, wallPose.heading)
                                .build()));
        Actions.runBlocking( new ParallelAction(
                        lift.LiftDown(),
                        hand.HandDrive(),
                        claw.ClawClose()
                ));
        sleep(1000);

    }

    public boolean    setManualExposure(int exposureMS, int gain) {
        // Ensure Vision Portal has been setup.
        if (robot.myVisionPortal == null) {
            return false;
        }

        // Wait for the camera to be open
        if (robot.myVisionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (robot.myVisionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            // Set exposure.  Make sure we are in Manual Mode for these values to take effect.
            ExposureControl exposureControl = robot.myVisionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);

            // Set Gain.
            GainControl gainControl = robot.myVisionPortal.getCameraControl(GainControl.class);

            if(gainControl != null) {
            gainControl.setGain(gain);
            sleep(20); }
            return (true);
        } else {
            return (false);
        }
    }

}