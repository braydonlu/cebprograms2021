package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Subsystem;


public class dumpServo implements Subsystem {
    //Hardware: 1 motor, 1 encoder
    private Servo dumpServo;
    private double servoPosition = 0;

    public dumpServo(Robot robot) {
        dumpServo = robot.getServo("dumpServo");

    }



    public double getPosition() {
        return dumpServo.getPosition();
    }

    public void setServoPosition(double position) {
        this.servoPosition = position;


            // set encode to new position
    }


    @Override
    public void update(TelemetryPacket packet) {
        dumpServo.setPosition(servoPosition);
    }
}
