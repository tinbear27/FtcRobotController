package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.Arm.Angle;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.utility.MathUtility;
import org.firstinspires.ftc.teamcode.utility.MotionProfile;
import org.firstinspires.ftc.teamcode.utility.MotionProfileState;

public class ArmAngleSubsystem extends SubsystemBase {

    private final DcMotorEx angleMotor;
    private final Telemetry thisTelemetry;

    private MotionProfile angleProfile;
    private MotionProfileState angleState;
    private ElapsedTime timer;
    private PIDController controller;

    private int angleTargetPosition=0;
    private int anglePidTarget=0;
    private double anglePower=0.0;
    private double angleFeedforward=0.0;
    private int currentAnglePosition=0;

    public ArmAngleSubsystem(final HardwareMap hMap, Telemetry telemetry) {
        angleMotor = hMap.get(DcMotorEx.class, Angle.ANGLE_ID);
        angleMotor.setDirection(Angle.ANGLE_DIRECTION);
        angleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //angleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        angleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.controller = new PIDController(Angle.p, Angle.i, Angle.d);
        this.thisTelemetry=telemetry;
    }

    public void reset() {
        angleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        angleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setPositionByIndex(int indexNum) {
        setPosition(Angle.ANGLE_POSITIONS[indexNum]);
    }

    public void setPosition(int position) {
        if(position!=angleTargetPosition) {
            this.angleTargetPosition = position;

            this.angleProfile = new MotionProfile(currentAnglePosition, angleTargetPosition, Angle.ANGLE_PROFILE_ACCEL, Angle.ANGLE_PROFILE_DECEL, Angle.ANGLE_PROFILE_VELO);

            if (timer != null) {
                this.timer.reset();
            }
        }
    }

    public int getPosition() {
        return angleMotor.getCurrentPosition();
    }

    public int getTarget() {
        return angleTargetPosition;
    }

    @Override
    public void periodic() {
        if (timer == null) { timer = new ElapsedTime(); } //Create timer if it doesn't exist

        //Get current position
        this.currentAnglePosition=getPosition();

        //Get next virtual target for PID control
        if (angleProfile != null) {
            this.angleState = angleProfile.calculate(timer.time());
            anglePidTarget = (int)Math.round(angleState.x);
        }

        //Calculate feedforward
        angleFeedforward=Math.cos(Math.toRadians(currentAnglePosition/Angle.TICKS_PER_DEGREE))*(Angle.ANGLE_FF_RETRACTED+(RobotHardware.getInstance().armWinch.getExtensionRatio()*(Angle.ANGLE_FF_EXTENDED-Angle.ANGLE_FF_RETRACTED)));
        thisTelemetry.addData("Arm Angle FF:",angleFeedforward);

        //Calculate power using PID and Feedforward
        double pidPower=controller.calculate(currentAnglePosition, anglePidTarget);
        this.anglePower = MathUtility.clamp(pidPower+angleFeedforward, -(Angle.ANGLE_POWER_MAX), Angle.ANGLE_POWER_MAX);
        thisTelemetry.addData("Arm Angle PID Power:",pidPower);
        thisTelemetry.addData("Arm Angle Applied Power:",anglePower);
        thisTelemetry.addData("Arm Angle Tolerance",angleMotor.getTargetPositionTolerance());

        //Apply power
        angleMotor.setPower(anglePower);
    }

    //Sets motor power -- only used during init (zeroing)
    public void setZeroPower(double zeroPower) {
        angleMotor.setPower(zeroPower);
    }

    public boolean reachedTarget() {
        return (Math.abs(currentAnglePosition-angleTargetPosition)<Angle.ANGLE_TOLERANCE);
    }

    /*
    public void holdStartPosition(int position) {
        angleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        angleMotor.setTargetPositionTolerance(Angle.ANGLE_HOLD_TOLERANCE);
        angleMotor.setTargetPosition(position);
        angleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        angleMotor.setPower(Angle.ANGLE_HOLD_POWER);
    }
*/

    public void setActiveRunMode() {
        setPosition(1); //Travel position
        //angleMotor.setPower(Angle.ANGLE_FF_RETRACTED+0.04);
        angleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //angleMotor.setPower(Angle.ANGLE_FF_RETRACTED+0.04);

        //Set to travel mode
        this.setPosition(1);
        //this.periodic();
    }
}
