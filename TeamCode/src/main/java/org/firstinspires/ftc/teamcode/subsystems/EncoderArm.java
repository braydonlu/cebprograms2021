package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Subsystem;

public class EncoderArm implements Subsystem {
    //Hardware: 1 motor, 1 encoder
    private DcMotorEx armMotor;
    private static final double TICKS_PER_REV = 7168; // 28 * 256 = 7168

    //PID Stuff
    private PIDFController armPID;
    private static final PIDCoefficients ARM_PID_COEFFICIENTS = new PIDCoefficients(1, 0, 0);

    private static final double ARM_ACCEPTABLE_ERROR_MARGIN = 0.05;

    public enum Positions {
        RESET,
        INTAKE,
        DUMP,
    }

    public EncoderArm(Robot robot) {
        armMotor = robot.getMotor("armMotor");
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        armPID = new PIDFController(ARM_PID_COEFFICIENTS);
        //In order for the PID controller to find the most efficient way to go to the target position,
        //we need to bound the error inputted to the PID controller from -pi to pi radians
        armPID.setInputBounds(-Math.PI, Math.PI);
    }

    public void setTargetAngle(double targetAngle) {
        armPID.reset();
        armPID.setTargetPosition(targetAngle);
    }

    public void setTargetPosition(Positions position) {
        switch (position) {
            case RESET:
                setTargetAngle(0 * Math.PI / 180);
                break;
            case INTAKE:
                setTargetAngle(-150 * Math.PI / 180);
                break;
            case DUMP:
                setTargetAngle(10 * Math.PI / 180);
                break;
        }
    }
    private double getRawArmAngle() {
        // encoder position * (2pi / TICKS_PER_REV)
        return armMotor.getCurrentPosition() * (2 * Math.PI / TICKS_PER_REV);
    }

    public double getArmAngle() {
        return Angle.norm(getRawArmAngle());
    }

    public double getPIDError() { return armPID.getLastError(); }

    public boolean targetReached() {
        return Math.abs(armPID.getLastError()) <= ARM_ACCEPTABLE_ERROR_MARGIN;
    }

    @Override
    public void update(TelemetryPacket packet) {
        double armPower = armPID.update(getArmAngle());
        armMotor.setPower(armPower);
    }
}
