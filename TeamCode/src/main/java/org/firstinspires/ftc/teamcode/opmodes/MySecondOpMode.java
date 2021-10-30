package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.MyFirstCommand;
import org.firstinspires.ftc.teamcode.commands.MySecondCommand;
import org.firstinspires.ftc.teamcode.subsystems.MyFirstRobot;

public class MySecondOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MyFirstRobot robot = new MyFirstRobot(this);

        telemetry.addLine("Initialization finished");
        telemetry.update();

        waitForStart();

        robot.runCommand(new MySecondCommand(robot.subsystem));

        robot.runCommand(new MyFirstCommand(robot.subsystem, gamepad1));

        robot.stop();
    }
}
