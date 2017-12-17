package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by Stephen Ogden on 12/6/2016.
 */

@TeleOp(name = "Optical Distance", group = "Test")
@Disabled
public class OpticalDistance extends LinearOpMode{

    OpticalDistanceSensor ODSensor;

    @Override
    public void runOpMode() throws InterruptedException {

        ODSensor = hardwareMap.opticalDistanceSensor.get("ODS");

        waitForStart();

        while (opModeIsActive()) {

            //telemetry.addData("Raw Data", ODSensor.getRawLightDetected()); //Removed: no set scale
            /**
             * Returns a signal whose strength is proportional to the intensity of the light measured.
             * Note that returned values INCREASE as the light energy INCREASES. The units in which
             * this signal is returned are unspecified.
             * Return a value proportional to the amount of light detected, in unspecified units
             *
             */
            boolean beacon = false;
            if (ODSensor.getLightDetected() >= 0.01) {
                beacon = true;
            }
            telemetry.addData(" ", "0/1 1 is close, 0 is far");
            telemetry.addData("Value",ODSensor.getLightDetected());
            telemetry.addData("Beacon found?", beacon);
            /**
             * Get the amount of light detected by the sensor, scaled and cliped to a range
             * which is a pragmatically useful sensitivity. Note that returned values INCREASE as
             * the light energy INCREASES.
             * Return amount of light, on a scale of 0.0 to 1.0
             */
        //RED: 0.01

        //BLUE: 0.03
            telemetry.update();
        }
    }
}
