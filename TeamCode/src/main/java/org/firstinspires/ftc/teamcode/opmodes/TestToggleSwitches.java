package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@Disabled
@TeleOp(name = "TEST - Toggle Switches")
public class TestToggleSwitches extends LinearOpMode {

    private DigitalChannel ToggleSwitchA;
    private DigitalChannel ToggleSwitchB;
    private DigitalChannel ToggleSwitchC;
    private DigitalChannel ToggleSwitchD;

    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        ToggleSwitchA = hardwareMap.get(DigitalChannel.class, "ToggleSwitchA");
        ToggleSwitchB = hardwareMap.get(DigitalChannel.class, "ToggleSwitchB");
        ToggleSwitchC = hardwareMap.get(DigitalChannel.class, "ToggleSwitchC");
        ToggleSwitchD = hardwareMap.get(DigitalChannel.class, "ToggleSwitchD");

        // Configure digital pin for input mode.
        ToggleSwitchA.setMode(DigitalChannel.Mode.INPUT);
        ToggleSwitchB.setMode(DigitalChannel.Mode.INPUT);
        ToggleSwitchC.setMode(DigitalChannel.Mode.INPUT);
        ToggleSwitchD.setMode(DigitalChannel.Mode.INPUT);

        telemetry.addData("Toggle Switch Test", "Press start to continue...");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                if (ToggleSwitchA.getState() == false) {
                    telemetry.addData("Toggle(1) ", "B");
                } else {
                    telemetry.addData("Toggle(1) ", "A");
                }

                if (ToggleSwitchB.getState() == false) {
                    telemetry.addData("Toggle(2) ", "B");
                } else {
                    telemetry.addData("Toggle(2) ", "A");
                }

                if (ToggleSwitchC.getState() == false) {
                    telemetry.addData("Toggle(3) ", "B");
                } else {
                    telemetry.addData("Toggle(3) ", "A");
                }

                if (ToggleSwitchD.getState() == false) {
                    telemetry.addData("Toggle(4) ", "B");
                } else {
                    telemetry.addData("Toggle(4) ", "A");
                }

                telemetry.update();
            }
        }
    }
}
