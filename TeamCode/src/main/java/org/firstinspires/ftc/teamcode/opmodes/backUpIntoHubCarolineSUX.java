package org.firstinspires.ftc.teamcode.opmodes;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.DriveForTime;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
@Autonomous
public class backUpIntoHubCarolineSUX extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CrabRobot robot = new CrabRobot(this);
        double driveTime;

        waitForStart();

        // go forward
        Pose2d drivePower = new Pose2d(0.2,0,0);
        driveTime = 0.55;
        DriveForTime driveCommand = new DriveForTime(robot.mecanumDrive, drivePower, driveTime);
        robot.runCommand(driveCommand);
    }
}
