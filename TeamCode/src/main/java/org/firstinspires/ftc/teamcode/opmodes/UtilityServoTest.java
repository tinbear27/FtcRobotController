package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

@TeleOp(name="Utility - Servo Test",group="Testing")
public class UtilityServoTest extends LinearOpMode {
    Servo clawServo, wristServo;
    DigitalChannel angleLimit,winchLimit;

    @Override
    public void runOpMode() {
        clawServo = hardwareMap.get(Servo.class, Constants.Arm.Claw.CLAW_ID);
        wristServo = hardwareMap.get(Servo.class, Constants.Arm.Wrist.WRIST_ID);
        angleLimit=hardwareMap.get(DigitalChannel.class, "LimitAngle");
        winchLimit=hardwareMap.get(DigitalChannel.class, "LimitWinch");

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                wristServo.setPosition(0.40);
            }

            if (gamepad1.b) {
                wristServo.setPosition(0.60);
            }

            if (gamepad1.y) {
                clawServo.setPosition(0.40);
            }

            if (gamepad1.x) {
                clawServo.setPosition(0.60);
            }

            String angleLimitText="NO";
            if(angleLimit.getState()) {
                angleLimitText="YES";
            }

            String winchLimitText="NO";
            if(winchLimit.getState()) {
                winchLimitText="YES";
            }

            telemetry.addData("Angle Limit:", angleLimitText);
            telemetry.addData("Winch Limit:", winchLimitText);
            telemetry.update();

        }
    }
}
