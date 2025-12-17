package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.ErasmusRobot;

@Autonomous(name = "Auto Strafe", group = "Auto Test")
public final class AutoStrafe extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        ErasmusRobot robot = new ErasmusRobot(this);

        waitForStart();

        while (opModeIsActive()) {

            Actions.runBlocking(
                    robot.drive.actionBuilder(beginPose)
                            .strafeTo(new Vector2d(0, 36))
                            .strafeTo(new Vector2d(0, 36))
                            .build());
        }
    }
}
