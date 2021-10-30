package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;

@TeleOp
public class ScaleTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AnalogInput scale_p = hardwareMap.get(AnalogInput.class, "PInput");
        AnalogInput scale_n = hardwareMap.get(AnalogInput.class, "NInput");
        waitForStart();

        while (true) {

            telemetry.addData("Scale P", scale_p.getVoltage());
            telemetry.addData("Scale N", scale_n.getVoltage());
            telemetry.update();
        }
    }
}
