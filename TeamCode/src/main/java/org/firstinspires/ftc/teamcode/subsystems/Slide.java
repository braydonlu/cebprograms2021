package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Subsystem;


public class Slide implements Subsystem {
    //Hardware: 1 motor, 1 encoder
    private DcMotorEx slideMotor;
    private double slidePower= 0.2;
    public static final double  TICKS_PER_REV = 357.7;
    public static final double PULLEY_DIAMETER = 38 /25.4;
    public int level = 0;
    public static double SLIDE_LENGTH = 15.0;
    private static final double INCHES_PER_LEVEL = 3.0;
    private int targetPosition;
    private boolean needsPowerUpdate = false;


    public enum slide_state {
        LEVEL_0,
        LEVEL_1,
        LEVEL_2,
        LEVEL_3
    }

    public Slide(Robot robot) {
        slideMotor = robot.getMotor("slide");
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidePower=0.2;

    }

    public void setPower (double power ){
        slidePower = power;
    }

    public  int inchToTicks ( double inches) {

        return (int) (inches * TICKS_PER_REV / (PULLEY_DIAMETER * Math.PI));
    }
    public int getLevel() {
        return level;
    }

    public void goUp () {

        if (level < 3) {

            level = level + 1;
            targetPosition = inchToTicks (INCHES_PER_LEVEL * level);


            // set encode to new position
        }
    }

    public void goDown() {

    }

    @Override
    public void update(TelemetryPacket packet) {
        if (needsPowerUpdate) {
            slideMotor.setPower(slidePower);
        }
        slideMotor.setTargetPosition(targetPosition);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // debug only,  remove it on release
        // packet.put("Current Position", slideMotor.getCurrentPosition());
       //  packet.put("target position", slideMotor.getTargetPosition());
    }
}

