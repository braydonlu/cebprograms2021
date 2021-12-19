package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.dumpServo;

@TeleOp
public class EncoderSlideTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this);
        Slide slide = new Slide(robot, telemetry);
        dumpServo dumper = new dumpServo(robot);
        robot.registerSubsystem(slide);
        robot.registerSubsystem(dumper);
        telemetry.addLine("Initialization Finished");
        telemetry.update();


        waitForStart();

        while (!isStopRequested()) {

            boolean buttonA = gamepad1.a; //down
            boolean buttonB = gamepad1.b;
            boolean buttonY = gamepad1.y; //upper
            boolean buttonX = gamepad1.x;
            if (buttonA){
                slide.goDown();

            }
            else if (buttonY){
                slide.goUp();

            }
            else if (buttonB){
                dumper.setServoPosition(0);
            }
            else if (buttonX){
                dumper.setServoPosition(1);
            }
            telemetry.addData("current level:", slide.getLevel());
            telemetry.addData("current servo position:", dumper.getPosition());
            telemetry.update();

            robot.update();

        }
    }










































}
