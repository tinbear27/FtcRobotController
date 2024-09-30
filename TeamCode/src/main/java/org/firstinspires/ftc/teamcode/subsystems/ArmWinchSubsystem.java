package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Constants.Arm.Winch;

public class ArmWinchSubsystem extends SubsystemBase {
    private final DcMotorEx winchMotor;
    private int currentTarget=0;

    public ArmWinchSubsystem(final HardwareMap hMap) {
        winchMotor = hMap.get(DcMotorEx.class, Winch.WINCH_ID);
        winchMotor.setDirection(Winch.WINCH_DIRECTION);
        winchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
}
