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

        telemetry.addData("calibration", scale.getCalibration());
        telemetry.update();
        sleep(1000);

        telemetry.addData("offset", scale.getOffset());
        telemetry.update();
        sleep(1000);

        telemetry.addLine("Initialization Finished");
        telemetry.update();
        sleep(1000);

        //scale.setCalibration(100);
        //scale.peel();

        while (true) {

            //telemetry.addData("Scale P", scale_p.getVoltage());
           // telemetry.addData("Scale N", scale_n.getVoltage());
            //telemetry.update();
            //telemetry.addData("Calibration", scale.getCalibration());
            //telemetry.addData("Offset", scale.getOffset());
            //telemetry.addData("PF flag", scale.peelFlag());
            long value = scale.getValue();
            telemetry.addData(String.format("get_value hex 0x%08X, decimal: ", value), value);
            //telemetry.addData("Scale read", scale.average(10));

            telemetry.update();
            sleep(1000);
        }


    }
}

