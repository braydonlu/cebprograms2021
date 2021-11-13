package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name = "The Great Depression")
public class TheBigSad extends LinearOpMode {
    private DcMotor intake_motor;
    // private DcMotor arm_motor;
    // private DcMotor spinner;
    private DcMotor right_rear;
    private DcMotor right_front;
    private DcMotor left_front;
    private DcMotor left_rear;
    @Override
    public void runOpMode(){
        intake_motor = hardwareMap.dcMotor.get("intake_motor");
        //   arm_motor = hardwareMap.dcMotor.get("arm_motor");
        //spinner = hardwareMap.dcMotor.get("spinner");
        left_rear = hardwareMap.dcMotor.get("DriveLR");
        left_front = hardwareMap.dcMotor.get("DriveLF");
        right_front = hardwareMap.dcMotor.get("DriveRF");
        right_rear = hardwareMap.dcMotor.get("DriveRR");
        intake_motor.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        while (opModeIsActive()){
            double horizontal = 0.7 * gamepad1.left_stick_x;
            double vertical = -0.7 * gamepad1.left_stick_y;
            double turn = -0.7 * gamepad1.right_stick_x;
            left_rear.setPower(vertical + turn - horizontal);
            left_front.setPower(vertical + turn + horizontal);
            right_rear.setPower(vertical - turn + horizontal);
            right_front.setPower(vertical - turn - horizontal);

            if (gamepad1.a){
                intake_motor.setPower(1);
            }
            if(gamepad1.b){
                intake_motor.setPower(0);
            }
           /* if(gamepad1.b){
                arm_motor.setPower(1);
            }
            /
           / if (gamepad1.x){
                spinner.setPower(1);
            }*/
        }
    }
}