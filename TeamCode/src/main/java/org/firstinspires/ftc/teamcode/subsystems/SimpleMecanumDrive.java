 package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.hardware.CachingSensor;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Subsystem;

public class SimpleMecanumDrive implements Subsystem {

    private int MOTOR_LF= 0;
    private int MOTOR_LR= 3;
    private int MOTOR_RR =2;
    private int MOTOR_RF= 1;
            
    private DcMotorEx[] motors = new DcMotorEx[4];
    private CachingSensor<Float> headingSensor;

    private AnalogSensor scale_n, scale_p;

    private Double[] powers = {0.0, 0.0, 0.0, 0.0};

    public SimpleMecanumDrive (Robot robot) {
        motors[0] = robot.getMotor("DriveLF");
        motors[1] = robot.getMotor("DriveLR");
        motors[2] = robot.getMotor("DriveRR");
        motors[3] = robot.getMotor("DriveRF");

        BNO055IMU imu = robot.getIMU("imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        headingSensor = new CachingSensor<>(() -> imu.getAngularOrientation().firstAngle);
        robot.addListener(headingSensor);
        scale_n = robot.getAnalogSensor("NInput");
        scale_p = robot.getAnalogSensor("PInput");

        motors[0].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[1].setDirection(DcMotorSimple.Direction.REVERSE);
        for (DcMotorEx motor:motors){
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void setDrivePower(Pose2d drivePower) {
        powers[0] = drivePower.getX() - drivePower.getY() - drivePower.getHeading();
        powers[1] = drivePower.getX() + drivePower.getY() - drivePower.getHeading();

        powers[2] = drivePower.getX() - drivePower.getY() + drivePower.getHeading();
        powers[3] = drivePower.getX() + drivePower.getY() + drivePower.getHeading();
    }

    public double getHeading() {
        return headingSensor.getValue();
    }

    @Override
    public void update(TelemetryPacket packet) {
        for (int i = 0; i < 4; i++){
            motors[i].setPower(powers[i]);
        }

        packet.put("Scale N", scale_n.readRawVoltage());
        packet.put("Scale P", scale_p.readRawVoltage());
    }
}
