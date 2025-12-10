package org.firstinspires.ftc.teamcode.Teleop;

import static java.util.Objects.nonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.ErasmusRobot;

@Config
@TeleOp(name = "Positionator", group = "Teleop Test")
public final class PositionatorTeleop extends LinearOpMode {
    public static double SHOOTERSPEED = 1300;
    public static double FEEDERUP = 0.40 ;
    public static double FEEDERDOWN = 0.22 ;

    boolean rightTriggerPulled = false ;
    Action currentAction = null ;
    TelemetryPacket packet = new TelemetryPacket();

    @Override
    public void runOpMode() throws InterruptedException {
//        ErasmusRobot robot = new ErasmusRobot(hardwareMap, telemetry) ;
        ErasmusRobot robot = new ErasmusRobot(this) ;

        waitForStart();   // ================== START ======================

        while (opModeIsActive()) {   // ================ LOOP =======================
            if (gamepad1.dpadRightWasReleased()) {
                robot.indexerCurrentPosition=robot.checkIndex(robot.indexerCurrentPosition+1) ;
                robot.moveIndexer(robot.indexerCurrentPosition);
            }
            else if (gamepad1.dpadLeftWasReleased()) {
                robot.shutdownShooter();
                sleep(300);
                robot.resetIndexer();
            }
            // ----- StageAll() --------------
            else if (gamepad1.dpadDownWasPressed()) {
                currentAction = robot.stageAll() ;
                if (!currentAction.run(packet)) currentAction = null ;
            } else if (gamepad1.dpad_down) {
                if (nonNull(currentAction)&&!currentAction.run(packet)) currentAction = null;
            } else if (gamepad1.dpadDownWasReleased()) currentAction = null;
            // ----- ShootAll() --------
            else if (gamepad1.dpadUpWasPressed()) {
                currentAction = robot.shootAll() ;
                if (!currentAction.run(packet)) currentAction = null ;
            } else if (gamepad1.dpad_up) {
                if (nonNull(currentAction)&&!currentAction.run(packet)) currentAction = null;
            } else if (gamepad1.dpadUpWasReleased()) {
                currentAction = null;
                robot.shutdownShooter();
            } else if (gamepad1.rightBumperWasPressed()) {
                robot.shooterMotor.setPower(robot.INTAKEPOWER);
            } else if (gamepad1.leftBumperWasPressed()) {
                robot.shooterMotor.setPower(0);
            }

//            if (gamepad1.right_trigger> 0.2) {
//                if (!rightTriggerPulled) {
//                    rightTriggerPulled = true ;
//                    currentAction = robot.shootBall() ;
//                }
//                if (nonNull(currentAction) && !currentAction.run(packet)) currentAction = null ;
//                //shooterMotor.setVelocity(SHOOTERSPEED);
//            } else if (rightTriggerPulled) {
//                currentAction = null ;
//                robot.shutdownShooter();
//                rightTriggerPulled = false ;
//            }
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
                else if (gamepad1.b) robot.feedServo.setPosition(FEEDERUP);   // B
                else if (gamepad1.bWasReleased()) robot.feedServo.setPosition(FEEDERDOWN);  // B released

            telemetry.addData("Green = ", robot.colorSensor.green()) ;
            telemetry.addData("Red = ", robot.colorSensor.red()) ;
            telemetry.addData("Blue = ", robot.colorSensor.blue()) ;
            telemetry.addData("Name = ", robot.colorSensor.getDeviceName()) ;
            telemetry.addData("Version = ", robot.colorSensor.getVersion()) ;
            telemetry.addData("HSV Color = ", robot.readColorSensorHSV()) ;
            telemetry.addData("Shooter Velocity = ", robot.SHOOTERSPEED) ;
            telemetry.addData("Motor Velocity = ", robot.shooterMotor.getVelocity()) ;
            updateTelemetry(telemetry);
        }
    }
}
