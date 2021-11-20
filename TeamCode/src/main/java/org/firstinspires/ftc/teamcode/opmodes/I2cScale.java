package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class I2cScale extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //AnalogInput scale_p = hardwareMap.get(AnalogInput.class, "PInput");
        //AnalogInput scale_n = hardwareMap.get(AnalogInput.class, "NInput");
        ScaleHx711 scale = hardwareMap.get(ScaleHx711.class, "scale");


        waitForStart();

        //telemetry.addData("calibration", scale.getCalibration());
        //telemetry.update();
        //sleep(1000);

       // telemetry.addData("offset", scale.getOffset());
        //telemetry.update();
        //sleep(1000);

        telemetry.addLine("Initialization Finished");
        telemetry.update();
        sleep(1000);

        //scale.setCalibration(100);
        //scale.peel();
        int zero_value=0, adjValue, value;
        while (!isStopRequested()) {

            //telemetry.addData("Scale P", scale_p.getVoltage());
           // telemetry.addData("Scale N", scale_n.getVoltage());
            //telemetry.update();
            //telemetry.addData("Calibration", scale.getCalibration());
            //telemetry.addData("Offset", scale.getOffset());
            //telemetry.addData("PF flag", scale.peelFlag());
            //= (int)scale.getValue();
            //long average = scale.average(10);
            long raw_value= scale.getValue();
            telemetry.addData(String.format("get_value hex 0x%08X, decimal: ", raw_value), raw_value);
            telemetry.addLine("");

            if (gamepad1.a)
            {
                zero_value= (int)scale.getValue();
                telemetry.addLine("zero set");
                sleep(1000);
                telemetry.update();
            }
            value= (int)scale.getValue();
            adjValue = (value > zero_value) ? value - zero_value : 0;

            telemetry.addData(String.format("zero_value hex 0x%08X, decimal: ", zero_value), zero_value);
            telemetry.addData(String.format("get_value hex 0x%08X, decimal: ", value), value);
            telemetry.addData(String.format("adjValue hex 0x%08X, decimal: ", adjValue), adjValue);
            switch (adjValue) {
                case 0x00:
                    telemetry.addLine("light cube");
                    break;
                case 0x01:
                    telemetry.addLine("Ball");
                    break;
                case 0x03:
                    telemetry.addLine("Weighted cube");
                    break;
                case 0x04:
                    telemetry.addLine("Heavy cube");
                    break;
                default:
                    telemetry.addData("INVALID", adjValue);
                    break;
            }

            //telemetry.addData("Scale read", scale.average(10));

            telemetry.update();
            sleep(1000);
        }


    }
}

