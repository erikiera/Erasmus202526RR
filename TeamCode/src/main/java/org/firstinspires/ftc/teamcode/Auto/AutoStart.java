package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.ErasmusRobot;

@Autonomous(name = "Auto Start", group = "Auto Test")
public final class AutoStart extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-12, -12, Math.PI/2);
        ErasmusRobot robot = new ErasmusRobot(this, beginPose);

        Pose2d pose = robot.drive.localizer.getPose();
        telemetry.addData("x", pose.position.x);
        telemetry.addData("y", pose.position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
        updateTelemetry(telemetry);

        waitForStart();

        while (opModeIsActive()) {

            Actions.runBlocking(
                    robot.drive.actionBuilder(beginPose)
                            .splineTo(new Vector2d(24, 24), Math.PI/2)
                            .setReversed(true)
                            .splineTo(new Vector2d(-12, -12), -Math.PI/2)

                            .build());
        }
    }
}
