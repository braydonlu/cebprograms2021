package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.subsystems.EncoderArm;

@TeleOp
public class EncoderArmTestB extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this);
        EncoderArm arm = new EncoderArm(robot);
        robot.registerSubsystem(arm);

        telemetry.addLine("Initialization Finished");
        telemetry.update();

        arm.setTargetAngle(0);

        waitForStart();

        while (!isStopRequested()) {
            robot.update();
            boolean buttonA = gamepad1.a;
            boolean buttonB = gamepad1.b;
            boolean buttonX = gamepad1.x;

            if (arm.targetReached()) {
                telemetry.addData("Current arm angle" , arm.getArmAngle());
                telemetry.update();
                if (buttonA){
                    arm.setTargetPosition(EncoderArm.Positions.RESET);

                }
                else if (buttonB){
                    arm.setTargetPosition(EncoderArm.Positions.INTAKE);
                }
                else if (buttonX){
                    arm.setTargetPosition(EncoderArm.Positions.DUMP);
                }
            }else {
                telemetry.addData("Target not reached :( arm angle:",arm.getArmAngle());
                telemetry.update();
            }
        }
    }
}
