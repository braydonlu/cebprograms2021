package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@TeleOp
public class EncoderArmTestB extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this);
        Intake arm = new Intake(robot);
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
                    arm.setTargetPosition(Intake.Positions.INTAKE);

                }
                else if (buttonB){
                    arm.setTargetPosition(Intake.Positions.INTAKE);
                }
                else if (buttonX){
                    arm.setTargetPosition(Intake.Positions.DUMP);
                }
            }else {
                telemetry.addData("Target not reached :( arm angle:",arm.getArmAngle());
                telemetry.update();
            }
        }
    }
}
