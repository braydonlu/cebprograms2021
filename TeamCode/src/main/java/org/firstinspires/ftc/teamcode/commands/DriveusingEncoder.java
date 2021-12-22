package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.robot.Command;
import org.firstinspires.ftc.teamcode.subsystems.SimpleMecanumDrive;

public class DriveusingEncoder implements Command {
    SimpleMecanumDrive mecanumDrive;
    Pose2d drivePower;
    double initialTimeStamp;
    double driveTime;
    double power;
    public DriveusingEncoder(SimpleMecanumDrive drive, Pose2d distance, double power){
        mecanumDrive=drive;
        drivePower= distance;

    }
    @Override
    public void start() {
        mecanumDrive.setDrivePower(drivePower);
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
        return false;
    }
}

