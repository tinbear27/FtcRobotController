package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Test", group = "TESTING")
public class ServoTest extends LinearOpMode {

    Servo wristServo;
    Servo clawServo;

    @Override
    public void runOpMode() {

        wristServo = hardwareMap.get(Servo.class, "WristServo");
        clawServo = hardwareMap.get(Servo.class, "ClawServo");

        waitForStart();
        while (opModeIsActive()) {

            if (gamepad1.a) {
                wristServo.setPosition(0.30);
            }
            if (gamepad1.b) {
                wristServo.setPosition(0.8);
            }

            if (gamepad1.x) {
                clawServo.setPosition(0.240);
            }
            if (gamepad1.y) {
                clawServo.setPosition(0.8);
            }

            telemetry.addData("Wrist Servo Position", "%5.2f", wristServo.getPosition());
            telemetry.addData("Wrist Servo Position", "%5.2f", clawServo.getPosition());
            telemetry.update();
        }
    }
}