package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;





@TeleOp
public class MyFirstOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //INITIALIZATION SEQUENCE

        DcMotor motor = hardwareMap.get(DcMotor.class, "motor");
        Servo servo = hardwareMap.get(Servo.class, "servo");
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        telemetry.addLine("Initialization finished");
        telemetry.update();

        waitForStart();

        // TELEOP SEQUENCE
        while (!isStopRequested()) {
            motor.setPower(gamepad1.left_stick_y);
            servo.setPosition(Math.abs(gamepad1.right_stick_y));

            telemetry.addData("Motor Power", motor.getPower());
            telemetry.addData("Servo Position", servo.getPosition());
            telemetry.addData("IMU Orientation", imu.getAngularOrientation().firstAngle);
            telemetry.update();
        }

        //STOP SEQUENCE
        motor.setPower(0);
    }
}
