package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot.ErasmusRobot;

@Autonomous(name = "Scratch Auto", group = "Auto Test")
public final class ScratchAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        ErasmusRobot robot = new ErasmusRobot(this) ;
//        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        Actions.runBlocking(
                robot.drive.actionBuilder(beginPose)
                        .splineTo(new Vector2d(20, 18), Math.PI/2)
                        .stopAndAdd(robot.shootBall())
                        .stopAndAdd(new SequentialAction(
                                new InstantAction(()->robot.moveIndexer(0)),
                                new SleepAction(0.9),
                                new InstantAction(()->robot.feedServo.setPosition(robot.FEEDERUP)),
                                new SleepAction(0.5)
                        ))
//                        .setReversed(true)
//                        .splineTo(new Vector2d(40, 0), 0)
//                        .setReversed(false)
//                        .splineTo(new Vector2d(20, -18), 3*Math.PI/2)
//                        .setReversed(true)
//                        .splineTo(new Vector2d(0, 0), Math.PI)
                        .build());
    }
}
