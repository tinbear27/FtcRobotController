package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

@Disabled
@Config
@TeleOp(name = "Encoders - Manual Reading", group = "TESTING")
public class EncodersManual extends LinearOpMode {
    Servo wristServo;
    Servo clawServo;
    DcMotorEx winchMotor;
    DcMotorEx armAngleMotor;

    public static double wristServoPosition=0.5;
    public static double clawServoPosition=0.24;

    @Override
    public void runOpMode() {
        wristServo = hardwareMap.get(Servo.class, Constants.Arm.Wrist.WRIST_ID);
        clawServo = hardwareMap.get(Servo.class, Constants.Arm.Claw.CLAW_ID);

        armAngleMotor = hardwareMap.get(DcMotorEx.class, Constants.Arm.Angle.ANGLE_ID);
        armAngleMotor.setDirection(Constants.Arm.Angle.ANGLE_DIRECTION);
        armAngleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        winchMotor = hardwareMap.get(DcMotorEx.class, Constants.Arm.Winch.WINCH_ID);
        winchMotor.setDirection(Constants.Arm.Winch.WINCH_DIRECTION);
        winchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Dashboard telemetry
        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        while (opModeIsActive()) {
            wristServo.setPosition(wristServoPosition);
            clawServo.setPosition(clawServoPosition);

            telemetry.addData("Arm Angle Position:",armAngleMotor.getCurrentPosition());
            telemetry.addData("Arm Extension Position:",winchMotor.getCurrentPosition());
            telemetry.addData("Wrist Position:",wristServo.getPosition());
            telemetry.addData("Claw Position:",clawServo.getPosition());
            telemetry.update();
        }
    }
}
