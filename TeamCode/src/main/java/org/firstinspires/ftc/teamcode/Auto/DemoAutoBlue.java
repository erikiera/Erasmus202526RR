package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.ErasmusRobot;

@Autonomous(name = "Blue Auto Demo", group = "Auto Test")
public final class DemoAutoBlue extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
//        Pose2d beginPose = new Pose2d(-63, 24, Math.PI);
        Pose2d beginPose = new Pose2d(-24, 24, Math.PI);

        ErasmusRobot robot = new ErasmusRobot(this, beginPose) ;
        robot.SHOOTERSPEED = 900 ;
        robot.greenIndex = 2 ;


        waitForStart();

        Actions.runBlocking(
                robot.drive.actionBuilder(beginPose)
                        .setReversed(true)
                        .splineTo(new Vector2d(-12, 24), 0)
                        .stopAndAdd(()->robot.huskyReadPattern())
                        .setReversed(true)
                        .splineTo(new Vector2d(36, 36), Math.PI/4)
                        .stopAndAdd(robot.shootAll())

                        .build());
    }
}
