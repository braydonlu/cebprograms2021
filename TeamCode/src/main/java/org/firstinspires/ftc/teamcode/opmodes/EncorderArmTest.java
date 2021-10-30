package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.subsystems.EncoderArm;

@TeleOp
public class EncorderArmTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this);
        EncoderArm arm = new EncoderArm(robot);
        robot.registerSubsystem(arm);

        telemetry.addLine("Initialization Finished");
        telemetry.update();

        waitForStart();

        while (!isStopRequested()) {
            double joystickX = gamepad1.left_stick_x;
            double joystickY = gamepad1.left_stick_y;
            double joystickPythagorean = Math.sqrt(joystickX * joystickX + joystickY * joystickY);
            if (joystickPythagorean > 0.5 && arm.targetReached()) {
                double joystickAngle = Angle.norm(Math.atan2(joystickY, joystickX) + Math.PI/2);
                arm.setTargetAngle(joystickAngle);
            }
        }
    }
}
