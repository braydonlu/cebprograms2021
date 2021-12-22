package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.SimpleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.dumpServo;
import org.firstinspires.ftc.teamcode.subsystems.duckSpinner;
@TeleOp
public class    TeleFreightB extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this);
        SimpleMecanumDrive mecanumDrive = new SimpleMecanumDrive(robot);
        dumpServo dumper = new dumpServo(robot);
        Intake intake = new Intake(robot);
        Slide mySlide = new Slide(robot, telemetry);
        duckSpinner duckspinner = new duckSpinner(robot);
        robot.registerSubsystem(intake);
        robot.registerSubsystem(mySlide);
        robot.registerSubsystem(duckspinner);
        robot.registerSubsystem(mecanumDrive);
        robot.registerSubsystem(dumper);
        int slidecountup = 0;
        int slidecountdown = 0;
        boolean isApressed = false;

        boolean inTransfer = false;

        intake.setTargetPosition(Intake.Positions.RESET);
        //dumper.setServoPosition(0.0);

        waitForStart();

        while (!isStopRequested()) {
            telemetry.addData("slide level init: ", mySlide.getLevel());
            telemetry.addData("dumpServo Position:",dumper.getPosition());
            telemetry.update();

            boolean buttonA = gamepad2.a; //enter Align
            boolean buttonB = gamepad2.b; // exit Align
            boolean buttonX = gamepad2.x;
            boolean buttonY = gamepad2.y;
            boolean leftBumper = gamepad2.left_bumper;
            boolean rightBumper = gamepad2.right_bumper;
            boolean slowMode = gamepad1.a;
            boolean normieMode = gamepad1.b;
            float leftTrigger = gamepad1.left_trigger;
            float rightTrigger = gamepad1.right_trigger;

            robot.update();






            // Telemetry print out distL, distR
            /*
            telemetry.addData("distL:", mecanumDrive.getdistL());
            telemetry.addData("distR:", mecanumDrive.getdistR());
            telemetry.addData("Dist reached", mecanumDrive.hubReached());
            telemetry.addData("Turn reached", mecanumDrive.turnReached());

             */

           /* if ( (buttonA || !mecanumDrive.hubReached() || !mecanumDrive.turnReached()) && !buttonB) {
            //if ( (buttonA || !mecanumDrive.hubReached()) && !buttonB) {
                if (!mecanumDrive.getInAlignMode()) {
                    mecanumDrive.setTargetDist(200.0);
                    mecanumDrive.setInAlignMode(true);
                }

                telemetry.addLine("in button A loop");
            } else {
            */
            //    mecanumDrive.setInAlignMode(false);
                mecanumDrive.setDrivePower(new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x));



            if(buttonX) {
                intake.start();
                telemetry.addLine("in buttonX loop");
            }


            if(buttonY){
                if (!inTransfer) {
                    intake.stop();
                    inTransfer=true;
                    telemetry.addLine("enter transfer");
                }
                else {
                    intake.reset();
                    inTransfer = false;
                    telemetry.addLine("transfer done");
                }

            }
            if(buttonA) {
                if (!isApressed) {
                    mySlide.goUp();
                    isApressed = true;
                    telemetry.addData("going up. level: ", mySlide.getLevel());

                }

            } else {
                isApressed = false;
            }
            if(buttonB) {
                mySlide.goalldown();
                telemetry.addLine("going all down");
            }
            if(leftBumper) {
                int level = mySlide.getLevel();
                double servoPosition=0.85;
                switch (level){
                    case 1: servoPosition=0.3;
                    break;
                    case 2: servoPosition= 0.45;
                    break;
                    case 3: servoPosition= 0.48;
                    break;

                }
                dumper.setServoPosition(servoPosition);
                telemetry.addLine("dumping  ");
            }
            if (rightBumper) {
                dumper.setServoPosition(0.85);
                telemetry.addLine("resetting dumper");
            }
            if (slowMode) {
                mecanumDrive.setPowerFactor(0.2);
            }
            if (normieMode) {
                mecanumDrive.setPowerFactor(0.8);
            }
            if (leftTrigger > 0 ) {
                duckspinner.setPower(3);
            }
            if (rightTrigger > 0) {
                duckspinner.setPower(0);
            }


            telemetry.update();

        }

    }
}