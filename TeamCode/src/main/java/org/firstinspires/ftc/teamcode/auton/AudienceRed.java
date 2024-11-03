package org.firstinspires.ftc.teamcode.auton;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.LightSabersHardware;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Config
@Autonomous(name="AudienceRed", preselectTeleOp = "Tournament_TeleOp")

public class AudienceRed extends LinearOpMode {
    //Put variables and classes here
    LightSabersHardware robot =   new LightSabersHardware(this);


    public double drive_X = 0;
    public double drive_Y = -63.57;
    public double right_drop = 0;
    public double white_drop = 0;
    Pose2d initPose = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d dropPurplePixelPose = new Pose2d(0, 0, 0);
    Pose2d backFromPurplePose = new Pose2d(0, 0, 0);
    Pose2d leftMidwayPose = new Pose2d(0, 0, 0);
    Pose2d frontStackPose = new Pose2d(48, 18, Math.toRadians(0));
    Pose2d frontStackTurnPose = new Pose2d(48, 17, Math.toRadians(90));
    Pose2d pixelPickupPose = new Pose2d(48, 25, Math.toRadians(90));
    Pose2d doorPose = new Pose2d(52, 0, Math.toRadians(-90));
    Pose2d throughDoorPose = new Pose2d(50, -72, Math.toRadians(-90));
    Pose2d aprilPose = new Pose2d(22, -72, Math.toRadians(-90));
    Pose2d parkPose = new Pose2d(25, -76, Math.toRadians(-90));
    //Pose2d dropYellowPixelPose = new Pose2d(drive_X, drive_Y, Math.toRadians(-90));

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
            return new LiftUpScore();
        }

        public class LiftUpStack implements Action {
            private boolean initialized = false;
            //private double stacktop = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    robot.liftTime.reset();
                    robot.LiftMotor.setPower(0.75);
                    initialized = true;
                }

                if (robot.liftTime.milliseconds() <= 130) {
                    return true;
                } else {
                    robot.LiftMotor.setPower(0.05);
                    return false;
                }
                /*if (!initialized) {
                    // Reset Lift Encoder
                    robot.LiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.LiftMotor.setPower(0.4);
                    initialized = true;
                }

                //double pos = robot.LiftMotor.getCurrentPosition();
                if (robot.LiftMotor.getCurrentPosition() <= 285){
                    return true;
                } else {
                    robot.LiftMotor.setPower(0.05);
                    sleep(350);
                    return false;
                }*/
            }
        }
        public Action LiftUpStack() {
            return new LiftUpStack();
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
                    sleep(200);
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

        robot.wristDrive();
        robot.shoulderDown();

        telemetry.addData("Identified", robot.visionProcessor.getSelection());

        //Deliver the pixel to the correct spike mark
        robot.myVisionPortal.setProcessorEnabled(robot.visionProcessor, false);

        setManualExposure(2, 150);

        switch (robot.visionProcessor.getSelection()) {
            case LEFT:
                robot.DESIRED_TAG_ID = 4;
                dropPurplePixelPose = new Pose2d(16, 12, Math.toRadians(0));
                backFromPurplePose = new Pose2d(8, 17, Math.toRadians(0));
                leftMidwayPose = new Pose2d(8, 24, Math.toRadians(0));
                white_drop = -4;
                drive_X = 29.7;
                break;
            case RIGHT:
                robot.DESIRED_TAG_ID = 6;
                Pose2d moveBeyondTrussPose = new Pose2d(15,2,0);
                dropPurplePixelPose = new Pose2d(22, -4, Math.toRadians(-45));
                backFromPurplePose = new Pose2d(16, 0, Math.toRadians(-45));
                leftMidwayPose = new Pose2d(8, 24, Math.toRadians(-45));
                right_drop = -3;
                white_drop = 12;
                drive_X = 19.7;
                break;
            case MIDDLE:
                robot.DESIRED_TAG_ID = 5;
                dropPurplePixelPose = new Pose2d(28, 4, Math.toRadians(0));
                backFromPurplePose = new Pose2d(24, 4, Math.toRadians(0));
                leftMidwayPose = new Pose2d(8, 24, Math.toRadians(0));
                white_drop = -4;
                drive_X = 25.7;
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
                drive.actionBuilder(backFromPurplePose)
                        .strafeToLinearHeading(leftMidwayPose.position, leftMidwayPose.heading)
                        .strafeToLinearHeading(frontStackPose.position, frontStackPose.heading)
                        .turnTo(Math.toRadians(95))
                        .build(),
                        lift.LiftUpStack()));
        sleep(500);

        Actions.runBlocking(
                new SequentialAction(
                        drive.actionBuilder(frontStackTurnPose)
                                .strafeToLinearHeading(pixelPickupPose.position, pixelPickupPose.heading)
                                .build(),
                        claw.ClawClose(),
                        drive.actionBuilder(pixelPickupPose)
                                .strafeToLinearHeading(doorPose.position, doorPose.heading)
                                .build(),
                        lift.LiftDown(),
                drive.actionBuilder(doorPose)
                        .strafeToLinearHeading(throughDoorPose.position, throughDoorPose.heading)
                        .strafeToLinearHeading(aprilPose.position, aprilPose.heading)
                        .build()));

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

        if (robot.targetFound = true) {
            drive_X = 10 + 15.7 + right_drop - robot.desiredTag.ftcPose.x;
            drive_Y = -72 + 21.43 - robot.desiredTag.ftcPose.y;
        }

        Pose2d dropYellowPixelPose = new Pose2d(drive_X, drive_Y, Math.toRadians(-90));
        Pose2d dropWhitePixelPose = new Pose2d(drive_X+white_drop, drive_Y, Math.toRadians(-90));

        Actions.runBlocking(
                new ParallelAction(
                        lift.LiftUp(),
                        hand.HandScore()
                ));
        sleep(500);
        Actions.runBlocking(
                new SequentialAction(
                drive.actionBuilder(aprilPose)
                        .strafeToLinearHeading(dropYellowPixelPose.position, dropYellowPixelPose.heading)
                        .build(),
                        claw.leftClawOpen(),
                        lift.LiftUpScore(),
                        drive.actionBuilder(dropYellowPixelPose)
                                .strafeToLinearHeading(dropWhitePixelPose.position, dropWhitePixelPose.heading)
                                .build(),
                        claw.RightClawOpen(),
                        drive.actionBuilder(dropYellowPixelPose)
                                //.strafeToLinearHeading(aprilPose.position, aprilPose.heading)
                                .strafeToLinearHeading(parkPose.position, parkPose.heading)
                                .turnTo(Math.toRadians(90))
                                //.strafeToLinearHeading(parkPose.position, parkPose.heading)
                                .build()));
        sleep(300);
        Actions.runBlocking(
                new ParallelAction(
                        lift.LiftDown(),
                        hand.HandDrive(),
                        claw.ClawClose()
                ));

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