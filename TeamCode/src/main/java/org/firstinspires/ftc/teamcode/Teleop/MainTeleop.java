package org.firstinspires.ftc.teamcode.Teleop;

import static java.util.Objects.nonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot.ErasmusRobot;

@Config
@TeleOp(name = "Main Teleop", group = "Teleop Test")
public final class MainTeleop extends LinearOpMode {
    public static double SHOOTERSPEED = 1500;
    public static double FEEDERUP = 0.40 ;
    public static double FEEDERDOWN = 0.22 ;

    boolean rightTriggerPulled = false ;
    boolean leftTriggerPulled = false ;

    ErasmusRobot robot ;
    public static double LOWSPEED = 0.3 ;
    public static double HIGHSPEED = 0.8 ;
    private double speedAdjustment ;
    Action currentAction ;
    TelemetryPacket packet = new TelemetryPacket();

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-24, 24, Math.PI);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

//        ErasmusRobot robot = new ErasmusRobot(hardwareMap, telemetry) ;
        robot = new ErasmusRobot(this, beginPose) ;


        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.right_trigger> 0.2) {
                if (!rightTriggerPulled) {
                    rightTriggerPulled = true ;
                    robot.SHOOTERSPEED = SHOOTERSPEED ;
                    currentAction = robot.shootAll() ;
                }
                if (nonNull(currentAction) && !currentAction.run(packet)) currentAction = null ;
            } else if (rightTriggerPulled) {
                currentAction = null ;
                robot.shutdownShooter();
                rightTriggerPulled = false ;
            }


            if (gamepad1.left_trigger> 0.2) {
                if (!leftTriggerPulled) {
                    leftTriggerPulled = true ;
                    currentAction = robot.intakeAll() ;
                }
                if (nonNull(currentAction) && !currentAction.run(packet)) currentAction = null ;
            } else if (leftTriggerPulled) {
                robot.stopIntake();
                currentAction = null ;
                leftTriggerPulled = false ;
            }

            else if (gamepad1.xWasPressed()) robot.huskyReadPattern() ;
            else if (gamepad1.y) robot.feedServo.setPosition(FEEDERUP);   // B
            else if (gamepad1.yWasReleased()) robot.feedServo.setPosition(FEEDERDOWN);  // B released

            if (gamepad1.dpadUpWasReleased()) {          // DPad Up released
                SHOOTERSPEED +=100 ;
                if (SHOOTERSPEED>2800) SHOOTERSPEED=2800 ;
            } else if (gamepad1.dpadDownWasReleased()) {        // DPad Down released
                SHOOTERSPEED -= 100;
                if (SHOOTERSPEED < 0) SHOOTERSPEED = 0;
            }


            robot.drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

//            // Read pose
//            Pose2d poseEstimate = robot.drive.localizer.getPose();
//
//            // Create a vector from the gamepad x/y inputs
//            // Then, rotate that vector by the inverse of that heading
//            Vector2d input = new Vector2d(
//                    -gamepad1.left_stick_y,
//                    -gamepad1.left_stick_x
//            ).rotated(-poseEstimate.heading);
//            input.angleCast()
//            // Pass in the rotated input + right stick value for rotation
//            // Rotation is not part of the rotated input thus must be passed in separately
//            robot.drive.setDrivePowers(new PoseVelocity2d(
//                    new Vector2d(
//                            input.x,
//                            input.y
//                    ),
//                            -gamepad1.right_stick_x
//            ));




//            if (gamepad1.rightBumperWasPressed()) {             // Right Bumper
//                shooterMotor.setVelocity(SHOOTERSPEED);
//            } else if (gamepad1.leftBumperWasPressed()) {       // Left Bumper
//                shooterMotor.setVelocity(0);
//            } else if (gamepad1.aWasPressed()) {                // A
//                shooterMotor.setVelocity(SHOOTERSPEED);
//            } else if (gamepad1.aWasReleased()) {               // A released
//                shooterMotor.setVelocity(0);
//            } else if (gamepad1.dpadUpWasReleased()) {          // DPad Up released
//                SHOOTERSPEED +=100 ;
//                if (SHOOTERSPEED>2800) SHOOTERSPEED=2800 ;
//            } else if (gamepad1.dpadDownWasReleased()) {        // DPad Down released
//                SHOOTERSPEED -=100 ;
//                if (SHOOTERSPEED<0) SHOOTERSPEED=0 ;
//            }
//            else if (gamepad1.b) feedServo.setPosition(FEEDERUP);   // B
//            else if (gamepad1.bWasReleased()) feedServo.setPosition(FEEDERDOWN);  // B released


            telemetry.addData("Shooter Velocity (OpMode)", SHOOTERSPEED) ;
            telemetry.addData("Motor Velocity = ", robot.shooterMotor.getVelocity()) ;
            telemetry.addData("Index", robot.indexerCurrentPosition) ;
            telemetry.addData("Green Index", robot.greenIndex) ;
            telemetry.addData("Pattern", robot.pattern) ;
            telemetry.addData("Distance", robot.lastDistanceReading) ;
            robot.drive.updatePoseEstimate();

            Pose2d pose = robot.drive.localizer.getPose();
            telemetry.addData("x", pose.position.x);
            telemetry.addData("y", pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
            telemetry.addData("Current", robot.shooterMotor.getCurrent(CurrentUnit.AMPS)) ;
            updateTelemetry(telemetry);
        }
    }

    private void clearAction() {
        currentAction = null ;
        robot.stopIntake();
        robot.shutdownShooter();
        speedAdjustment = HIGHSPEED ;

    }
}
