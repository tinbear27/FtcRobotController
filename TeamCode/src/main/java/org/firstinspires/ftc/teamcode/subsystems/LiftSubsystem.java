package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class LiftSubsystem extends SubsystemBase {
    private CRServo lift;
    private AnalogInput liftEncoder;
    private final Telemetry thisTelemetry;

    private int targetPosition=0;
    private int currentPosition=0;
    private int lastAngle=0;

    public LiftSubsystem(final HardwareMap hMap, Telemetry telemetry) {
        this.lift=hMap.get(CRServo.class, Constants.Lift.LIFT_ID);
        lift.setDirection(Constants.Lift.LIFT_DIRECTION);
        this.liftEncoder=hMap.get(AnalogInput.class, Constants.Lift.LIFT_ENCODER_ID);
        this.lastAngle=getLiftEncoderAngle();
        this.thisTelemetry=telemetry;
    }

    public void setTarget(int target) {
        this.targetPosition=target;
    }

    public void raise() {
        setTarget(Constants.Lift.LIFT_MAX);
    }

    public void lower() {
        setTarget(Constants.Lift.LIFT_MIN);
    }

    public boolean reachedTarget() {
        return (Math.abs(currentPosition-targetPosition)< Constants.Lift.LIFT_POSITION_TOLERANCE);
    }

    @Override
    public void periodic() {
        //Update current position
        int currentAngle=getLiftEncoderAngle();
        if(currentAngle!=lastAngle) {
            //Simple change within same rotation
            if (Math.abs(currentAngle-lastAngle) < 180) {
                currentPosition+=-(currentAngle-lastAngle);
            } else {
                if(currentAngle<lastAngle) {
                    currentPosition-=((360-lastAngle)+currentAngle);
                } else {
                    currentPosition+=((360-currentAngle)+lastAngle);
                }
            }
            lastAngle = currentAngle;
        }

        //Apply power if outside position tolerance
        if(Math.abs(currentPosition-targetPosition)>Constants.Lift.LIFT_POSITION_TOLERANCE) {
            if(targetPosition>currentPosition) {
                lift.setPower(Constants.Lift.LIFT_POWER);
            } else {
                lift.setPower(-Constants.Lift.LIFT_POWER);
            }
        } else {
            lift.setPower(0.0);
        }
    }

    public int getLiftEncoderAngle() {
        return (int)((1-(this.liftEncoder.getVoltage() / 3.3)) * 360);
    }

    public int getCurrentPosition() {
        return currentPosition;
    }

    public int getTargetPosition() {
        return targetPosition;
    }
}
