package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Created by Stephen Ogden on 11/14/2016.
 */
@TeleOp(name = "TAOD", group = "Test")
@Disabled

public class TouchAndOpticalDistance extends LinearOpMode {

    OpticalDistanceSensor ODSensor;  // Hardware Device Object
    TouchSensor Touch;
    ColorSensor colorSensor;
    LightSensor lightSensor;

    @Override
    public void runOpMode() throws InterruptedException {

        float hsvValues[] = {0F,0F,0F};
        final float values[] = hsvValues;
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.RelativeLayout);

        boolean blPrevState = false;
        boolean blCurrState = false;
        boolean bcPrevState = false;
        boolean bcCurrState = false;
        boolean blLedOn = true;
        boolean bcLedOn = true;

        ODSensor = hardwareMap.opticalDistanceSensor.get("ODS");
        Touch = hardwareMap.touchSensor.get("TS");
        colorSensor = hardwareMap.colorSensor.get("color sensor");
        lightSensor = hardwareMap.lightSensor.get("light sensor");

        lightSensor.enableLed(blLedOn);

        waitForStart();

        // while the op mode is active, loop and read the light levels.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {

            bcCurrState = gamepad1.x;
            blCurrState = gamepad1.y;

            if ((bcCurrState == true) && (bcCurrState != bcPrevState))  {
                bcLedOn = !bcLedOn;
                colorSensor.enableLed(bcLedOn);
            }
            if ((blCurrState == true) && (blCurrState != blPrevState))  {

                // button is transitioning to a pressed state.  Toggle LED
                blLedOn = !blLedOn;
                lightSensor.enableLed(blLedOn);
            }

            bcPrevState = bcCurrState;
            blPrevState = blCurrState;

            Color.RGBToHSV(colorSensor.red(), colorSensor.green(), colorSensor.blue(), hsvValues);

            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });

            telemetry.addData("Light LED", blLedOn ? "On" : "Off");
            telemetry.addData("Raw", lightSensor.getRawLightDetected());
            telemetry.addData("Normal", lightSensor.getLightDetected());
            telemetry.addData("Color LED", bcLedOn ? "On" : "Off");
            telemetry.addData("Clear", colorSensor.alpha());
            telemetry.addData("Red  ", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue ", colorSensor.blue());
            telemetry.addData("Hue", hsvValues[0]);
            telemetry.addData("Is the button pressed?  ", Touch.isPressed());
            telemetry.addData("Raw", ODSensor.getRawLightDetected());
            telemetry.addData("Normal", ODSensor.getLightDetected());

            telemetry.update();
        }
    }
}