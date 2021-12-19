package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.DriveForTime;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import org.firstinspires.ftc.teamcode.commands.Spin;

@Autonomous
public class carolineBullyMe extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CrabRobot robot = new CrabRobot(this);
        double driveTime;
        //robot.dumper.setServoPosition(0.8);


        waitForStart();

       // go forwards slightly
        Pose2d drivePower = new Pose2d(-0.2,0,0);
        driveTime = 0.52;
        DriveForTime driveCommand = new DriveForTime(robot.mecanumDrive, drivePower, driveTime);
        robot.runCommand(driveCommand);

        //strafe
        Pose2d sideways = new Pose2d(0,0.4,0);
        driveTime = 2.5;
        DriveForTime goSideways = new DriveForTime(robot.mecanumDrive, sideways, driveTime);
        robot.runCommand(goSideways);

        //strafe0
        Pose2d sideways0 = new Pose2d(0,0.1,0);
        driveTime = 0.5;
        DriveForTime goSideways0 = new DriveForTime(robot.mecanumDrive, sideways0, driveTime);
        robot.runCommand(goSideways0);

        // spin carousel
        double spinPower = 0.5;
        driveTime = 2.5;
        Spin spinDuck = new Spin(robot.spinner,spinPower, driveTime);
        robot.runCommands(spinDuck);

        //sickstrafes2
        Pose2d sideways2 = new Pose2d(0,-0.5,0);
        driveTime = 0.75;
        DriveForTime goSideways2 = new DriveForTime(robot.mecanumDrive, sideways2, driveTime);
        robot.runCommand(goSideways2);

        //waitaminute
        Pose2d nopower = new Pose2d(0,0,0);
        driveTime = 2;
        DriveForTime drivenoCommand = new DriveForTime(robot.mecanumDrive, nopower, driveTime);
        robot.runCommand(drivenoCommand);

        //sickstrafes3
        Pose2d sideways3 = new Pose2d(0,-0.5,0);
        driveTime = 1.5;
        DriveForTime goSideways3 = new DriveForTime(robot.mecanumDrive, sideways3, driveTime);
        robot.runCommand(goSideways3);





        // turn 90 towards warehouse
        /*double turnAngle = Math.toRadians(87);
        Turn  turnCommand = new Turn(robot.mecanumDrive, turnAngle);
        robot.runCommand(turnCommand);

        // back to carousal
        Pose2d carousel = new Pose2d(-0.25,0,0);
        driveTime = 1.65;
        DriveForTime backToCarousel = new DriveForTime(robot.mecanumDrive, carousel, driveTime);
        robot.runCommand(backToCarousel);
        */


//obama
    }
}

