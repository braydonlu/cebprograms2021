package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.DriveForTime;
import org.firstinspires.ftc.teamcode.commands.Spin;
import org.firstinspires.ftc.teamcode.commands.Turn;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
@Autonomous
public class autobotsrollout extends LinearOpMode {
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

        // turn 90 towards warehouse
        double turnAngle = Math.toRadians(87);
        Turn  turnCommand = new Turn(robot.mecanumDrive, turnAngle);
        robot.runCommand(turnCommand);

        // back to carousal
        Pose2d carousel = new Pose2d(-0.25,0,0);
        driveTime = 1.65;
        DriveForTime backToCarousel = new DriveForTime(robot.mecanumDrive, carousel, driveTime);
        robot.runCommand(backToCarousel);

        // spin carousel
        double spinPower = 0.5;
        driveTime = 3;
        Spin spinDuck = new Spin(robot.slide,spinPower, driveTime);
        robot.runCommands(spinDuck);
//obama
    }
}
