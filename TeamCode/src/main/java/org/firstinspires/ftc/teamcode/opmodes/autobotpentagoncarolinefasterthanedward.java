package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.DriveForTime;
import org.firstinspires.ftc.teamcode.commands.Spin;
import org.firstinspires.ftc.teamcode.commands.Turn;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
@Autonomous
public class autobotpentagoncarolinefasterthanedward extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CrabRobot robot = new CrabRobot(this);
        double driveTime;
        double turnAngle;
        DriveForTime driveCommand;
        Turn turnCommand;

        waitForStart();

        for(int i=0; i<5; i++)
        {
// go forward
            Pose2d drivePower = new Pose2d(.25, 0, 0);
            driveTime = 1;
            driveCommand = new DriveForTime(robot.mecanumDrive, drivePower, driveTime);
            robot.runCommand(driveCommand);

// turn 108 towards warehouse
            turnAngle = Math.toRadians(72*(i+1));
            turnCommand = new Turn(robot.mecanumDrive, turnAngle);
            robot.runCommand(turnCommand);
        }
    }
}

