package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Spinner;
import org.firstinspires.ftc.teamcode.subsystems.SimpleMecanumDrive;

@TeleOp
public class MecanumDriveTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this);
        SimpleMecanumDrive mecanumDrive = new SimpleMecanumDrive(robot);
        Spinner myLift = new Spinner(robot);
        robot.registerSubsystem(mecanumDrive);
        robot.registerSubsystem(myLift);

        waitForStart();

        while (!isStopRequested()) {
            robot.update();
            mecanumDrive.setDrivePower(new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x));
            if (gamepad1.left_bumper) {
                myLift.setPower(1);
            } else {
                myLift.setPower(-gamepad1.left_trigger );

            }
        }
    }
}
