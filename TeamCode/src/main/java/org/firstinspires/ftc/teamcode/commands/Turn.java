package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.robot.Command;
import org.firstinspires.ftc.teamcode.subsystems.SimpleMecanumDrive;

public class Turn implements Command {
    SimpleMecanumDrive mecanumDrive;
    PIDFController controller;
    PIDCoefficients coefficients = new PIDCoefficients(0.95,0,0.05);
    double errorTolerance = 0.01;
public Turn (SimpleMecanumDrive drive, double targetHeading){
    mecanumDrive= drive;
    controller= new PIDFController(coefficients);
    controller.setInputBounds(-Math.PI,Math.PI);
    controller.setOutputBounds(-1, 1);
    controller.setTargetPosition(targetHeading);
}
    @Override
    public void start() {

    }

    @Override
    public void update() {
        double turnPower = controller.update(mecanumDrive.getHeading());
        mecanumDrive.setDrivePower(new Pose2d(0,0, turnPower));
    }

    @Override
    public void stop() {
    mecanumDrive.setDrivePower(new Pose2d());

    }

    @Override
    public boolean isCompleted() {
        return controller.getLastError()<=errorTolerance;
    }
}
