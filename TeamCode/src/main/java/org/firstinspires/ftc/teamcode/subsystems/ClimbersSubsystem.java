package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class ClimbersSubsystem  extends SubsystemBase {
    private final DcMotorEx climberLeftMotor;
    private final DcMotorEx climberRightMotor;
    private int currentTarget=0;
    private final Telemetry thisTelemetry;

    public ClimbersSubsystem(final HardwareMap hMap, Telemetry telemetry) {
        this.climberLeftMotor = hMap.get(DcMotorEx.class, Constants.Climbers.CLIMBER_LEFT_ID);
        climberLeftMotor.setDirection(Constants.Climbers.CLIMBER_LEFT_DIRECTION);
        climberLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        climberLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climberLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.climberRightMotor = hMap.get(DcMotorEx.class, Constants.Climbers.CLIMBER_RIGHT_ID);
        climberRightMotor.setDirection(Constants.Climbers.CLIMBER_RIGHT_DIRECTION);
        climberRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        climberRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climberRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.thisTelemetry=telemetry;
    }

    public void setTarget(int target) {
        this.currentTarget=target;

        if(Math.abs(getLeftPosition()-currentTarget)>(Constants.Climbers.CLIMBER_POSITION_TOLERANCE/2)) {
            climberLeftMotor.setTargetPosition(currentTarget);
            climberLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            climberLeftMotor.setPower(Constants.Climbers.CLIMBER_POWER);
        }

        if(Math.abs(getRightPosition()-currentTarget)>(Constants.Climbers.CLIMBER_POSITION_TOLERANCE/2)) {
            climberRightMotor.setTargetPosition(currentTarget);
            climberRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            climberRightMotor.setPower(Constants.Climbers.CLIMBER_POWER);
        }
    }

    public int getLeftPosition() {
        return climberLeftMotor.getCurrentPosition();
    }

    public int getRightPosition() {
        return climberRightMotor.getCurrentPosition();
    }

    public int getAveragePosition() {
        return (int)((getLeftPosition()+getRightPosition())/2);
    }

    public int getTarget() {
        return currentTarget;
    }

    @Override
    public void periodic() {
        int leftPosition=getLeftPosition();
        int rightPosition=getRightPosition();

        thisTelemetry.addData("> Climber Power (right)",climberRightMotor.getPower());
        thisTelemetry.addData("> Climber Pos Tol (right)",climberRightMotor.getTargetPositionTolerance());
        thisTelemetry.addData("> Climber Target (right)",climberRightMotor.getTargetPosition());
        thisTelemetry.addData("> Climber Direction (right)",climberRightMotor.getDirection());

        /*
        if (climberLeftMotor.isBusy() && Math.abs(leftPosition-currentTarget)>(Constants.Climbers.CLIMBER_POSITION_TOLERANCE/2) && leftPosition>=(Constants.Climbers.CLIMBER_MIN-Constants.Climbers.CLIMBER_POSITION_TOLERANCE) && leftPosition<=(Constants.Climbers.CLIMBER_MAX+Constants.Climbers.CLIMBER_POSITION_TOLERANCE)) {
            climberLeftMotor.setPower(Constants.Climbers.CLIMBER_POWER);
        } else {
            climberLeftMotor.setPower(0.0);
        }
        if (climberRightMotor.isBusy() && Math.abs(rightPosition-currentTarget)>(Constants.Climbers.CLIMBER_POSITION_TOLERANCE/2) && rightPosition>=(Constants.Climbers.CLIMBER_MIN-Constants.Climbers.CLIMBER_POSITION_TOLERANCE) && rightPosition<=(Constants.Climbers.CLIMBER_MAX+Constants.Climbers.CLIMBER_POSITION_TOLERANCE)) {
            climberRightMotor.setPower(Constants.Climbers.CLIMBER_POWER);
        } else {
            climberRightMotor.setPower(0.0);
        }
        */
    }

    public boolean reachedTarget() {
        return (Math.abs(getAveragePosition()-currentTarget)< Constants.Climbers.CLIMBER_POSITION_TOLERANCE);
    }
}
