package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@TeleOp(name = "Shooter Test", group = "Teleop Test")
public final class ShooterOnly extends LinearOpMode {
    public static double SHOOTERSPEED = 1800;
    public static double FEEDERUP = 0.40 ;
    public static double FEEDERDOWN = 0.22 ;
    public static double PIDP = 7 ;
    public static double PIDI = 3 ;
    public static double PIDD = 1.5 ;
    public static double PIDF = 0 ;
    private boolean triggerRightOn = false ;
    public static double SHOOTERRAWPOWER = 0.7 ;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setVelocity(0);
        shooterMotor.setVelocityPIDFCoefficients(PIDP, PIDI, PIDD, PIDF);

        Servo feedServo = hardwareMap.get(Servo.class, "feedServo") ;

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.rightBumperWasPressed()) {             // Right Bumper
                shooterMotor.setVelocity(SHOOTERSPEED);
            } else if (gamepad1.leftBumperWasPressed()) {       // Left Bumper
                shooterMotor.setVelocity(0);
            } else if (gamepad1.aWasPressed()) {                // A
                shooterMotor.setVelocity(SHOOTERSPEED);
            } else if (gamepad1.aWasReleased()) {               // A released
                shooterMotor.setVelocity(0);
            } else if (gamepad1.dpadUpWasReleased()) {          // DPad Up released
                SHOOTERSPEED +=100 ;
                if (SHOOTERSPEED>2800) SHOOTERSPEED=2800 ;
            } else if (gamepad1.dpadDownWasReleased()) {        // DPad Down released
                SHOOTERSPEED -=100 ;
                if (SHOOTERSPEED<0) SHOOTERSPEED=0 ;
            }
            else if (gamepad1.b) feedServo.setPosition(FEEDERUP);   // B
            else if (gamepad1.bWasReleased()) feedServo.setPosition(FEEDERDOWN);  // B released


            if(gamepad1.right_trigger>0.3) {
                if (!triggerRightOn) {
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
        }
    }
}
