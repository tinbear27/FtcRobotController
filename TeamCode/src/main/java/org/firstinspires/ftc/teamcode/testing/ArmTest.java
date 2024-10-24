package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.utility.MotionProfile;
import org.firstinspires.ftc.teamcode.utility.MotionProfileState;
import org.firstinspires.ftc.teamcode.utility.MathUtility;

@Disabled
@Config
@TeleOp(name = "Arm Test", group = "TESTING")
public class ArmTest extends LinearOpMode {

    Servo wristServo;
    Servo clawServo;
    DcMotorEx winchMotor;
    DcMotorEx armAngleMotor;

    int winchLimitMin=0;
    int winchLimitMax=700;
    double winchPower=0.50;

    int angleLimitMin=0;
    int angleLimitMax=2400;

    int angleUp=2000;
    int angleDown=200;

    //Integer angleTargetPosition=0;
    //double anglePower=0.75;
    //double angleSlowPower=0.05;


    public static double p=0.002, i=0.0, d=0.0001;
    public static double f=0.05; //Feedforward multiplier

    public static int angleTargetPosition=0;
    public static int anglePidTarget=0;

    double anglePower=0.0;
    double angleMaxPower=1.0;
    double angleFeedforward=0.0;
    public boolean angleTargetReached=true;
    public double angleTolerance=10;

    MotionProfile angleProfile;
    MotionProfileState angleState;

    private final double ticks_in_degree=8192 / 360.0;

    private ElapsedTime timer;

    public static double ANGLE_PROFILE_ACCEL=10000; //Ticks/sec/sec
    public static double ANGLE_PROFILE_DECEL=8000; //Ticks/sec/sec
    public static double ANGLE_PROFILE_VELO=10000; //Ticks/sec


    @Override
    public void runOpMode() {

        wristServo = hardwareMap.get(Servo.class, "WristServo");
        clawServo = hardwareMap.get(Servo.class, "ClawServo");

        //Initialize arm winch motor
        winchMotor = hardwareMap.get(DcMotorEx.class, "ArmWinch");
        winchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        winchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while (winchMotor.getCurrentPosition() != 0) { idle(); } //Make sure motor encoder reset before continuing

        winchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        winchMotor.setTargetPosition(winchLimitMin);
        winchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Initialize arm angle motor
        armAngleMotor = hardwareMap.get(DcMotorEx.class, "ArmAngle");
        armAngleMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        angleTargetPosition=angleLimitMin; //Initial position

        PIDController controller = new PIDController(p, i, d);

        //Dashboard telemetry
        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        while (opModeIsActive()) {
            if (timer == null) { timer = new ElapsedTime(); } //Create timer if it doesn't exist

            //Wrist Articulation
            if (gamepad1.a) {
                wristServo.setPosition(0.20);
            }
            if (gamepad1.b) {
                wristServo.setPosition(0.60);
            }

            //Claw open/close
            if (gamepad1.right_trigger>0.0) { //Close
                clawServo.setPosition(0.240);
            }
            if (gamepad1.right_bumper) { //Open
                clawServo.setPosition(0.8);
            }

            //--------------------------------------------------------------------------
            //ARM ANGLE

            int currentAnglePosition=armAngleMotor.getCurrentPosition();

            //Check for a new position request
            if(gamepad1.dpad_up && angleTargetPosition!=angleUp) {
                angleTargetPosition=angleUp;
                this.angleProfile = new MotionProfile(currentAnglePosition, angleTargetPosition, ANGLE_PROFILE_ACCEL, ANGLE_PROFILE_DECEL, ANGLE_PROFILE_VELO);
                this.timer.reset();

            } else if(gamepad1.dpad_down && angleTargetPosition!=angleDown) {
                angleTargetPosition=angleDown;
                this.angleProfile = new MotionProfile(currentAnglePosition, angleTargetPosition, ANGLE_PROFILE_ACCEL, ANGLE_PROFILE_DECEL, ANGLE_PROFILE_VELO);
                this.timer.reset();
            }

            //Get next virtual target for PID control
            if (angleProfile != null) {
                this.angleState = angleProfile.calculate(timer.time());
                anglePidTarget = (int)Math.round(angleState.x);
            }

            //Calculate feedforward
            angleFeedforward=Math.cos(Math.toRadians(currentAnglePosition/ticks_in_degree))*f;

            //Calculate power using PID and Feedforward
            double pidPower=controller.calculate(currentAnglePosition, anglePidTarget);
            this.anglePower = pidPower+angleFeedforward;
            this.anglePower = MathUtility.clamp(anglePower, -(angleMaxPower), angleMaxPower);

            this.angleTargetReached = Math.abs(currentAnglePosition - angleTargetPosition) < angleTolerance;

            //Apply power
            armAngleMotor.setPower(anglePower);

            /*
            //Change arm angle
            if(gamepad1.dpad_up) {
                angleTargetPosition=angleLimitMax;
            } else if(gamepad1.dpad_down) {
                angleTargetPosition=angleLimitMin;
            }

            controller.setPID(p,i,d);
            int currentAnglePosition=armAngleMotor.getCurrentPosition();

            double pid=controller.calculate(currentAnglePosition,angleTargetPosition);

            double ff=Math.cos(Math.toRadians(angleTargetPosition/ticks_in_degree))*f;
            double power=pid+ff;

            armAngleMotor.setPower(power);

*/

            /*
            if(gamepad1.dpad_up) {
                telemetry.addLine("UP!!!");
                angleTargetPosition=angleLimitMax;
                armAngleMotor.setTargetPosition(angleTargetPosition);
            } else if(gamepad1.dpad_down) {
                telemetry.addLine("DOWN!!!");
                angleTargetPosition=angleLimitMin;
                armAngleMotor.setTargetPosition(angleTargetPosition);
            }

            if (armAngleMotor.isBusy()) {
                if(Math.abs(currentAnglePosition-angleTargetPosition)<200) {
                    armAngleMotor.setPower(angleSlowPower);
                } else {
                    armAngleMotor.setPower(anglePower);
                }
            } else {
                armAngleMotor.setPower(0);
            }
            */

            //Extend/Retract Arm
            if(gamepad1.dpad_right) {
                winchMotor.setTargetPosition(winchLimitMax);
            } else if(gamepad1.dpad_left) {
                winchMotor.setTargetPosition(winchLimitMin);
            }

            if (winchMotor.isBusy()) {
                winchMotor.setPower(winchPower);
            } else {
                winchMotor.setPower(0);
            }

            //Update telemetry
//            telemetry.addData("Wrist Servo Position", "%5.2f", wristServo.getPosition());
//            telemetry.addData("Claw Servo Position", "%5.2f", clawServo.getPosition());
//            telemetry.addData("Winch Position", winchMotor.getCurrentPosition());
            telemetry.addData("Arm Angle Position", currentAnglePosition);
            telemetry.addData("Arm Angle Target", angleTargetPosition);
            telemetry.addData("Arm Angle PID Target", anglePidTarget);
            telemetry.addData("Feed Forward Power", "%5.2f", angleFeedforward);
            telemetry.addData("Arm Angle Power", "%5.2f", anglePower);
            telemetry.addData("Tolerance", armAngleMotor.getTargetPositionTolerance());

            telemetry.update();
        }
    }
}