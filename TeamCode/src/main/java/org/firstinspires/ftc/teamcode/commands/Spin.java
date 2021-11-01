package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.util.NanoClock;

import org.firstinspires.ftc.teamcode.robot.Command;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class Spin implements Command {
    Slide slide;
    NanoClock clock;
    double drivePower;
    double initialTimeStamp;
    double driveTime;
    // power: positive clockwise

    public Spin(Slide drive, double power, double time) {
        slide =drive;
        clock=NanoClock.system();
        drivePower= power;
        driveTime=time;
    }

    @Override
    public void start() {
        slide.setPower(drivePower);
        initialTimeStamp=clock.seconds();
    }

    @Override
    public void update() {

    }

    @Override
    public void stop() {
        slide.setPower(0);

    }

    @Override
    public boolean isCompleted() {
        double currentTime=clock.seconds()-initialTimeStamp;
        return currentTime>=driveTime;
    }
}
