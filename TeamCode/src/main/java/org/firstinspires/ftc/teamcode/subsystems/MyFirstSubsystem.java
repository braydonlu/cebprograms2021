package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Subsystem;

public class MyFirstSubsystem implements Subsystem {
    private DcMotor motor;
    private Servo servo;
    private BNO055IMU imu;

    private double motorPower;
    private double servoPosition;

    public MyFirstSubsystem(Robot robot) {
        motor = robot.getMotor("motor");
        servo = robot.getServo("servo");
        imu = robot.getIMU("imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
    }

    public void setMotorPower(double motorPower) {
        this.motorPower = motorPower;
    }

    public void setServoPosition(double servoPosition) {
        this.servoPosition = servoPosition;
    }

    public double getIMUHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    @Override
    public void update(TelemetryPacket packet) {
        motor.setPower(motorPower);
        servo.setPosition(servoPosition);

        packet.put("Motor Power", motorPower);
        packet.put("Servo Position", servoPosition);
        packet.put("IMU Heading", getIMUHeading());
    }
}
