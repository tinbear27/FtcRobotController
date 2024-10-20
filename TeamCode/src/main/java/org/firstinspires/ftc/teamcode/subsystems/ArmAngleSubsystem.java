package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants.Arm.Angle;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.utility.MathUtility;
import org.firstinspires.ftc.teamcode.utility.MotionProfile;
import org.firstinspires.ftc.teamcode.utility.MotionProfileState;

public class ArmAngleSubsystem extends SubsystemBase {

    private final DcMotorEx angleMotor;

    private MotionProfile angleProfile;
    private MotionProfileState angleState;
    private ElapsedTime timer;
    private PIDController controller;

    private int angleTargetPosition=0;
    private int anglePidTarget=0;
    private double anglePower=0.0;
    private double angleFeedforward=0.0;
    private int currentAnglePosition=0;

    public ArmAngleSubsystem(final HardwareMap hMap) {
        angleMotor = hMap.get(DcMotorEx.class, Angle.ANGLE_ID);
        angleMotor.setDirection(Angle.ANGLE_DIRECTION);
        angleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.controller = new PIDController(Angle.p, Angle.i, Angle.d);
    }

    public void setPositionByIndex(int indexNum) {
        setPosition(Angle.ANGLE_POSITIONS[indexNum]);
    }

    public void setPosition(int position) {
        if(position!=angleTargetPosition) {
            this.angleTargetPosition = position;

            this.angleProfile = new MotionProfile(currentAnglePosition, angleTargetPosition, Angle.ANGLE_PROFILE_ACCEL, Angle.ANGLE_PROFILE_DECEL, Angle.ANGLE_PROFILE_VELO);
            this.timer.reset();
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

        //Calculate power using PID and Feedforward
        double pidPower=controller.calculate(currentAnglePosition, anglePidTarget);
        this.anglePower = pidPower+angleFeedforward;
        this.anglePower = MathUtility.clamp(anglePower, -(Angle.ANGLE_POWER_MAX), Angle.ANGLE_POWER_MAX);

        //Apply power
        angleMotor.setPower(anglePower);
    }

    public boolean reachedTarget() {
        return (Math.abs(currentAnglePosition-angleTargetPosition)<Angle.ANGLE_TOLERANCE);
    }

    public void holdStartPosition(int position) {
        angleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        angleMotor.setTargetPosition(position);
        angleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        angleMotor.setPower(Angle.ANGLE_HOLD_POWER);
    }

    public void setActiveRunMode() {
        angleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.angleTargetPosition=Angle.ANGLE_POSITIONS[RobotHardware.getInstance().ARM_POSITION];
    }
}
