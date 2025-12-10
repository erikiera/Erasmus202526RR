package org.firstinspires.ftc.teamcode.Teleop;

import static java.util.Objects.nonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.Robot.ErasmusRobot;

@Config
@TeleOp(name = "Shooter LP", group = "Teleop Test")
public final class ShooterOnlyLP extends LinearOpMode {
    public static double SHOOTERSPEED = 1800;
    public static double FEEDERUP = 0.40 ;
    public static double FEEDERDOWN = 0.22 ;
    private boolean triggerRightOn = false ;
    public static double SHOOTERRAWPOWER = 1 ;
    Action currentAction = null ;

    @Override
    public void runOpMode() throws InterruptedException {
        ErasmusRobot robot = new ErasmusRobot(this) ;
        TelemetryPacket packet = new TelemetryPacket();


        DcMotorEx shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setPower(0);

        Servo feedServo = hardwareMap.get(Servo.class, "feedServo") ;

        waitForStart();

        while (opModeIsActive()) {
            robot.SHOOTERSPEED = SHOOTERSPEED ;
            if (nonNull(currentAction) && !currentAction.run(packet)) currentAction = null ;


            if (gamepad1.rightBumperWasPressed()) {             // Right Bumper
                currentAction = robot.newShoot() ;
            } else if (gamepad1.rightBumperWasReleased()) {       // Left Bumper
                currentAction = null ;
                robot.shutdownShooter();
            } else if (gamepad1.bWasReleased()) {               // A released
                shooterMotor.setPower(0);
            } else if (gamepad1.dpadUpWasReleased()) {          // DPad Up released
                SHOOTERSPEED +=100 ;
                if (SHOOTERSPEED>2800) SHOOTERSPEED=2800 ;
            } else if (gamepad1.dpadDownWasReleased()) {        // DPad Down released
                SHOOTERSPEED -=100 ;
                if (SHOOTERSPEED<0) SHOOTERSPEED=0 ;
            }
            else if (gamepad1.y) feedServo.setPosition(FEEDERUP);   // B
            else if (gamepad1.yWasReleased()) feedServo.setPosition(FEEDERDOWN);  // B released


            if(gamepad1.right_trigger>0.3) {
                if (!triggerRightOn) {
                    currentAction = null ;
                    shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    shooterMotor.setPower(SHOOTERRAWPOWER);
                }

                triggerRightOn = true ;
            }
            else if (triggerRightOn) {
                shooterMotor.setVelocity(0);
                triggerRightOn = false ;
            }



            telemetry.addData("Shooter Velocity = ", SHOOTERSPEED) ;
            telemetry.addData("Motor Velocity = ", shooterMotor.getVelocity()) ;
            updateTelemetry(telemetry);

            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), robot.drive.localizer.getPose());
            packet.put("Motor Velocity = ", shooterMotor.getVelocity()) ;
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}
