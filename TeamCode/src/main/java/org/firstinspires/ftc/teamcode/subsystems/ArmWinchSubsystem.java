package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Constants.Arm.Winch;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.utility.MathUtility;

public class ArmWinchSubsystem extends SubsystemBase {
    private final DcMotorEx winchMotor;
    private final Telemetry thisTelemetry;

    private int currentTarget=0;

    public ArmWinchSubsystem(final HardwareMap hMap, Telemetry telemetry) {
        this.winchMotor = hMap.get(DcMotorEx.class, Winch.WINCH_ID);
        winchMotor.setDirection(Winch.WINCH_DIRECTION);
        winchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //winchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        winchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.thisTelemetry=telemetry;
    }

    public void reset() {
        winchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        winchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setPositionByIndex(int indexNum) {
        setPosition(Winch.WINCH_POSITIONS[indexNum]);
    }

    public void setPosition(int position) {
        this.currentTarget=position;
        winchMotor.setTargetPosition(currentTarget);
        winchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public int getPosition() {
        return winchMotor.getCurrentPosition();
    }

    public int getTarget() {
        return currentTarget;
    }

    //Extension ratio (0=fully retracted, 1=fully extended)
    public double getExtensionRatio() {
        return MathUtility.clamp((double) (getPosition() - Winch.WINCH_POSITION_MIN) /(Winch.WINCH_POSITION_MAX-Winch.WINCH_POSITION_MIN),0.0,1.0);
    }

    @Override
    public void periodic() {
        if (winchMotor.isBusy()) {
            winchMotor.setPower(Winch.WINCH_POWER);
        } else {
            double ffPower=Winch.WINCH_FF_EXTENSION*getExtensionRatio()+Winch.WINCH_FF_ANGLE*Math.sin(Math.toRadians(RobotHardware.getInstance().armAngle.getPosition()/Constants.Arm.Angle.TICKS_PER_DEGREE));
            if(ffPower<0.01) { ffPower=0.0; }
            winchMotor.setPower(ffPower);
        }
    }

    //Sets motor power -- only used during init (zeroing)
    public void setZeroPower(double zeroPower) {
        winchMotor.setPower(zeroPower);
    }

    public boolean reachedTarget() {
        return (Math.abs(getPosition()-currentTarget)< Winch.WINCH_POSITION_TOLERANCE);
    }
}
