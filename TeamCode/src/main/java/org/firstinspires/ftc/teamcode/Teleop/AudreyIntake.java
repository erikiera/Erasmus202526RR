package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "Audrey Intake Test", group = "Teleop Test")
public final class AudreyIntake extends LinearOpMode {
    public static double INTAKEPOWER = 0.4;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx shooterMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setPower(0);

        Servo feedServo = hardwareMap.get(Servo.class, "feedServo") ;

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.rightBumperWasPressed()) {
                shooterMotor.setPower(INTAKEPOWER);
                //feedServo.setPosition(1);
            } else if (gamepad1.leftBumperWasPressed()) {
                shooterMotor.setPower(0);
                feedServo.setPosition(0.5);
            } else if (gamepad1.aWasPressed()) {
                shooterMotor.setPower(INTAKEPOWER);
                feedServo.setPosition(1);
            } else if (gamepad1.aWasReleased()) {
                shooterMotor.setPower(0);
                feedServo.setPosition(0.5);
            } else if (gamepad1.dpadUpWasReleased()) {
                INTAKEPOWER +=.1 ;
                if (INTAKEPOWER >1) INTAKEPOWER =1 ;
            } else if (gamepad1.dpadDownWasReleased()) {
                INTAKEPOWER -=.1 ;
                if (INTAKEPOWER <0) INTAKEPOWER =0 ;
            } else if (gamepad1.bWasReleased()) {
                if (shooterMotor.isBusy())  {
                    if (shooterMotor.getDirection()==DcMotorSimple.Direction.FORWARD)
                            shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                    else shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                };
            }


            telemetry.addData("Shooter Velocity = ", INTAKEPOWER) ;
            telemetry.addData("Motor Velocity = ", shooterMotor.getVelocity()) ;
            updateTelemetry(telemetry);
        }
    }
}
