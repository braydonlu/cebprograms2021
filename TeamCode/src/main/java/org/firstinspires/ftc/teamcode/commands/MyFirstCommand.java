package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robot.Command;
import org.firstinspires.ftc.teamcode.subsystems.MyFirstSubsystem;

public class MyFirstCommand implements Command {
    private MyFirstSubsystem subsystem;
    Gamepad gamepad;

    public MyFirstCommand(MyFirstSubsystem subsystem, Gamepad gamepad) {
        this.subsystem = subsystem;
        this.gamepad = gamepad;
    }

    @Override
    public void start() {

    }

    @Override
    public void update() {
        subsystem.setMotorPower(gamepad.left_stick_y);
        subsystem.setServoPosition(Math.abs(gamepad.right_stick_y));
    }

    @Override
    public void stop() {
        subsystem.setMotorPower(0);
    }

    @Override
    public boolean isCompleted() {
        return false;
    }
}
