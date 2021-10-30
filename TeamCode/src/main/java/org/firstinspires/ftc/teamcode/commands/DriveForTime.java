package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.NanoClock;

import org.firstinspires.ftc.teamcode.robot.Command;
import org.firstinspires.ftc.teamcode.subsystems.SimpleMecanumDrive;

public class DriveForTime implements Command {
    SimpleMecanumDrive mecanumDrive;
    NanoClock clock;
    Pose2d drivePower;
    double initialTimeStamp;
    double driveTime;
    public DriveForTime (SimpleMecanumDrive drive, Pose2d power, double time){
        mecanumDrive=drive;
        clock=NanoClock.system();
        drivePower= power;
        driveTime=time;
    }
    @Override
    public void start() {
        mecanumDrive.setDrivePower(drivePower);
        initialTimeStamp=clock.seconds();
    }

    @Override
    public void update() {

    }

    @Override
    public void stop() {
        mecanumDrive.setDrivePower(new Pose2d());

    }

    @Override
    public boolean isCompleted() {
        double currentTime=clock.seconds()-initialTimeStamp;
        return currentTime>=driveTime;
    }
}
