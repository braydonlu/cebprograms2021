package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.util.NanoClock;

import org.firstinspires.ftc.teamcode.robot.Command;
import org.firstinspires.ftc.teamcode.subsystems.MyFirstSubsystem;

public class MySecondCommand implements Command {
    private MyFirstSubsystem subsystem;
    NanoClock clock;
    double initialTimestamp;

    public MySecondCommand(MyFirstSubsystem subsystem) {
        this.subsystem = subsystem;
        clock = NanoClock.system();
    }

    @Override
    public void start() {
        subsystem.setMotorPower(1);
        initialTimestamp = clock.seconds();
    }

    @Override
    public void update() {

    }

    @Override
    public void stop() {
        subsystem.setMotorPower(0);
    }

    @Override
    public boolean isCompleted() {
        double currentTimestamp = clock.seconds();
        return currentTimestamp - initialTimestamp >= 1;
    }
}
