package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Robot;

public class MyFirstRobot extends Robot {
    public final MyFirstSubsystem subsystem;

    public MyFirstRobot(LinearOpMode opMode) {
        super(opMode);

        subsystem = new MyFirstSubsystem(this);
        registerSubsystem(subsystem);
    }
}
