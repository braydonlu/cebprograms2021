package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Robot;

public class CrabRobot extends Robot {
    public final SimpleMecanumDrive mecanumDrive;
    public final Slide slide;
    public final duckSpinner spinner;
    public final dumpServo dumper;
    public CrabRobot(LinearOpMode opMode) {
        super(opMode);
        mecanumDrive = new SimpleMecanumDrive(this);
        registerSubsystem(mecanumDrive);
        slide = new Slide(this, opMode.telemetry);
        registerSubsystem(slide);
        spinner = new duckSpinner(this);
        registerSubsystem(spinner);
        dumper = new dumpServo(this);
        registerSubsystem(dumper);

}
}
