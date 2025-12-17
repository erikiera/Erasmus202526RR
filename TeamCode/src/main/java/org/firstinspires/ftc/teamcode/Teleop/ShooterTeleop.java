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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Robot.ErasmusRobot;

@Config
@TeleOp(name = "Shooter Teleop", group = "Teleop Test")
public final class ShooterTeleop extends LinearOpMode {
//    public static double SHOOTERSPEED = 1500;
    public static double FEEDERUP = 0.40 ;
    public static double FEEDERDOWN = 0.22 ;
    public static double LOWSPEED = 0.3 ;
    public static double HIGHSPEED = 0.8 ;
    private double speedAdjustment = HIGHSPEED ;

    boolean rightTriggerPulled = false ;
    boolean leftTriggerPulled = false ;

    ErasmusRobot robot ;
    Action currentAction ;
    TelemetryPacket packet = new TelemetryPacket();

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-24, 24, Math.PI);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new ErasmusRobot(this, beginPose) ;


        waitForStart();

        while (opModeIsActive()) {
            // RIGHT TRIGGER ----------------------------------------------------------
            if (gamepad1.right_trigger> 0.2) {
                if (!rightTriggerPulled) {
                    rightTriggerPulled = true ;
//                    robot.SHOOTERSPEED = SHOOTERSPEED ;
                    clearAction();
                    speedAdjustment = LOWSPEED ;
                    currentAction = robot.shootAll() ;
                }
                if (nonNull(currentAction) && !currentAction.run(packet)) currentAction = null ;
            } else if (rightTriggerPulled) {
                clearAction();
//                currentAction = null ;
//                robot.shutdownShooter();
                rightTriggerPulled = false ;
            }
            // RIGHT BUMPER  ----------------------------------------------------------
            if (gamepad1.rightBumperWasPressed()) {
                clearAction() ;
//                robot.SHOOTERSPEED = SHOOTERSPEED ;
                currentAction = robot.shootBlank() ;
            }
            if (gamepad1.right_bumper) {
                if (nonNull(currentAction) && !currentAction.run(packet)) currentAction = null ;
            }
            else if (gamepad1.rightBumperWasReleased()) {
                clearAction();
            }

            // LEFT TRIGGER ----------------------------------------------------------
            if (gamepad1.left_trigger> 0.2) {
                if (!leftTriggerPulled) {
                    leftTriggerPulled = true ;
                    clearAction();
                    speedAdjustment = LOWSPEED ;
                    currentAction = robot.intakeAll() ;
                }
                if (nonNull(currentAction) && !currentAction.run(packet)) currentAction = null ;
            } else if (leftTriggerPulled) {
                clearAction();
//                robot.stopIntake();
//                currentAction = null ;
                leftTriggerPulled = false ;
            }
            // (X) ----------------------------------------------------------
            else if (gamepad1.xWasPressed()) robot.huskyReadPattern() ;
            // (Y) ----------------------------------------------------------
            else if (gamepad1.y) robot.feedServo.setPosition(FEEDERUP);   // Y held
            else if (gamepad1.yWasReleased()) robot.feedServo.setPosition(FEEDERDOWN);  // Y released
            // DPad UP ----------------------------------------------------------
            if (gamepad1.dpadUpWasReleased()) {          // DPad Up released
                robot.SHOOTERSPEED +=100 ;
                if (robot.SHOOTERSPEED>2800) robot.SHOOTERSPEED=2800 ;
            } else if (gamepad1.dpadDownWasReleased()) {        // DPad Down released
                robot.SHOOTERSPEED -= 100;
                if (robot.SHOOTERSPEED < 0) robot.SHOOTERSPEED = 0;
            }


            robot.drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

//

            telemetry.addData("Shooter Velocity (OpMode)", robot.SHOOTERSPEED) ;
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
