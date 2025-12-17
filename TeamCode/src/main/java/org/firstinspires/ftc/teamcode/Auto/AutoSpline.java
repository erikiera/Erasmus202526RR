package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.ErasmusRobot;

@Autonomous(name = "Auto Spline", group = "Auto Test")
public final class AutoSpline extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        ErasmusRobot robot = new ErasmusRobot(this);

        waitForStart();

        while (opModeIsActive()) {

            Actions.runBlocking(
                    robot.drive.actionBuilder(beginPose)
                            .splineTo(new Vector2d(36, 36), 0)
                            .setReversed(true)
                            .splineTo(new Vector2d(0, 0), Math.PI)

                            .build());
        }
    }
}
