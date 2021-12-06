package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.SimpleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.dumpServo;

@TeleOp
public class TeleFreightB extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this);
        SimpleMecanumDrive mecanumDrive = new SimpleMecanumDrive(robot);
        dumpServo dumper = new dumpServo(robot);
        Intake intake = new Intake(robot);
        Slide mySlide = new Slide(robot);
        robot.registerSubsystem(intake);
        robot.registerSubsystem(mySlide);
        robot.registerSubsystem(mecanumDrive);
        robot.registerSubsystem(dumper);
        int slidecountup = 0;
        int slidecountdown = 0;

        boolean inTransfer = false;

        intake.setTargetPosition(Intake.Positions.RESET);
        dumper.setServoPosition(1);

        waitForStart();

        while (!isStopRequested()) {
            boolean buttonA = gamepad1.a; //enter Align
            boolean buttonB = gamepad1.b; // exit Align
            boolean buttonX = gamepad1.x;
            boolean buttonY = gamepad1.y;
            boolean leftBumper = gamepad1.left_bumper;
            boolean rightBumper = gamepad1.right_bumper;

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
                mySlide.goUp();
                telemetry.addLine("going up");
            }
            if(buttonB) {
                mySlide.goalldown();
                telemetry.addLine("going all down");
            }
            if(leftBumper) {
                dumper.setServoPosition(0.6);
                telemetry.addLine("dumping item");
            }
            if (rightBumper) {
                dumper.setServoPosition(1);
                telemetry.addLine("resetting dumper");
            }
            /*
            telemetry.addData("hub PID error:", mecanumDrive.getDistPIDError());
            telemetry.addData("turn PID error:", mecanumDrive.getTurnPIDError() );
            telemetry.addData("In Alignment Mode", mecanumDrive.getInAlignMode());

             */


            telemetry.update();



        }
    }
}