package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

//@Disabled
@Autonomous(group = "Utility",name="Servo Reset (50 percent)")
public class UtilityServoReset  extends LinearOpMode {
    Servo clawServo, wristServo;

    @Override
    public void runOpMode() throws InterruptedException {
        clawServo = hardwareMap.get(Servo.class, Constants.Arm.Claw.CLAW_ID);
        wristServo = hardwareMap.get(Servo.class, Constants.Arm.Wrist.WRIST_ID);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            clawServo.setPosition(0.50);
            wristServo.setPosition(0.50);
        }
    }
}