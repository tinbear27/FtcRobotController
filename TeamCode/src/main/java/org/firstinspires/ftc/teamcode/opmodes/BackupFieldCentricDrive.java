package org.firstinspires.ftc.teamcode.opmodes;

/*
Teleop for backup bot - field centric drive
 */

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(group="Testing",name="Backup Bot - Field-centric Drive")
public class BackupFieldCentricDrive extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotorSimple leftFront=hardwareMap.get(DcMotorSimple.class, "DriveLeftFront");
        DcMotorSimple leftBack=hardwareMap.get(DcMotorSimple.class, "DriveLeftBack");
        DcMotorSimple rightFront=hardwareMap.get(DcMotorSimple.class, "DriveRightFront");
        DcMotorSimple rightBack=hardwareMap.get(DcMotorSimple.class, "DriveRightBack");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        IMU imu=hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParam=new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
            RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
        imu.initialize(imuParam);

        waitForStart();

        while(opModeIsActive()) {
            double lx=gamepad1.left_stick_x;
            double ly=gamepad1.left_stick_y;
            double rx=gamepad1.right_stick_x;

            if (gamepad1.options) {
                imu.resetYaw();
            }

            double max=Math.max(Math.abs(lx)+Math.abs(ly)+Math.abs(rx),1);

            double robotHeading=-imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double adjustedLx=(-ly*Math.sin(robotHeading))+(lx*Math.cos(robotHeading));
            double adjustedLy=(ly*Math.cos(robotHeading))+(lx*Math.sin(robotHeading));

            adjustedLx = adjustedLx * 1.1;  // Counteract imperfect strafing

            //Apply power to motors
            leftFront.setPower((adjustedLy+adjustedLx+rx)/max);
            leftBack.setPower((adjustedLy-adjustedLx+rx)/max);
            rightFront.setPower((adjustedLy-adjustedLx-rx)/max);
            rightBack.setPower((adjustedLy+adjustedLx-rx)/max);
        }
    }
}
