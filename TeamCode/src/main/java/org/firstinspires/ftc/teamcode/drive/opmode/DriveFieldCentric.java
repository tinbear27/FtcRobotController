package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "Field-centric Drive")
public class DriveFieldCentric extends LinearOpMode {
    double headingTarget=0.0,turnPower=0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        //drive.setPoseEstimate(PoseStorage.currentPose);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();
            double headingCurrent = poseEstimate.getHeading();

            //Check for low-speed mode
            double SPEED_MULTIPLIER = 1.0;
            if (gamepad1.right_bumper) {
                SPEED_MULTIPLIER = 0.3;
                telemetry.addLine("=== SLOW MODE ===");
            }

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y * SPEED_MULTIPLIER,
                    -gamepad1.left_stick_x * SPEED_MULTIPLIER
            ).rotated(-headingCurrent);
            headingCurrent = normalizeAngle(headingCurrent);

            //Right joystick used to identify heading
            if (Math.sqrt(gamepad1.right_stick_x*gamepad1.right_stick_x + gamepad1.right_stick_y*gamepad1.right_stick_y) > 0.90) {
                headingTarget = normalizeAngle(Math.atan2(gamepad1.right_stick_y, -gamepad1.right_stick_x) + Math.PI / 2);
            }

            //Identify error between current heading and maintain heading
            double headingError = normalizeAngle(headingTarget - headingCurrent);

            //Calculate heading correction power
            //Original -- "-gamepad1.right_stick_x*SPEED_MULTIPLIER"
            if (Math.abs(headingError) < 0.10) {
                turnPower = 0.0;
            } else if(headingError>1) {
                turnPower = 1;
            } else if(headingError<-1) {
                turnPower = -1;
            } else {
                turnPower=(headingError/Math.PI)*3;
            }

            turnPower*=1.0;

            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately

            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            turnPower
                    )
            );

            // Update everything. Odometry. Etc.
            drive.update();

            // Print pose to telemetry
            //telemetry.addData("x", poseEstimate.getX());
            //telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", Math.toDegrees(headingCurrent));
            telemetry.addData("heading (rad)", headingCurrent);
            telemetry.addData("heading (target)", Math.toDegrees(headingTarget));
            telemetry.addData("heading (target; rad)", headingTarget);
            telemetry.addData("heading error", headingError);
            telemetry.addData("heading turn power", turnPower);
            telemetry.update();
        }
    }

    public double normalizeAngle(double angle) {
        if(Math.abs(angle)<=Math.PI) {
            return angle;
        }

        if(angle>(2*Math.PI)) {
            angle-=(2*Math.PI);
        } else if(angle<(-2*Math.PI)) {
            angle+=(2*Math.PI);
        }

        if(angle>Math.PI) {
            angle=-((2*Math.PI)-angle);
        } else if(angle<-Math.PI) {
            angle=angle+(2*Math.PI);
        }

        return angle;
    }
}
