package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.util.NanoClock;

import org.firstinspires.ftc.teamcode.robot.Command;
import org.firstinspires.ftc.teamcode.subsystems.duckSpinner;

public class Spin implements Command {
    duckSpinner spinner;
    NanoClock clock;
    double drivePower;
    double initialTimeStamp;
    double driveTime;
    // power: positive clockwise

    public Spin(duckSpinner drive, double power, double time) {
        spinner =drive;
        clock=NanoClock.system();
        drivePower= power;
        driveTime=time;
    }

    @Override
    public void start() {
        spinner.setPower(drivePower);
        initialTimeStamp=clock.seconds();
    }

    @Override
    public void update() {

    }

    @Override
    public void stop() {
        spinner.setPower(0);

    }

    @Override
    public boolean isCompleted() {
        double currentTime=clock.seconds()-initialTimeStamp;
        return currentTime>=driveTime;
    }
}
