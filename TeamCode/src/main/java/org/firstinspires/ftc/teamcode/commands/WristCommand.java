package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class WristCommand extends CommandBase {
    private final WristSubsystem thisWrist;
    private final double thisTarget;
    private ElapsedTime thisTimer;
    private double timeToFinish;

    public WristCommand(WristSubsystem wrist, double target) {
        /*
        super(
                () -> RobotHardware.getInstance().wrist.setPosition(target)
        );
        */

        this.thisWrist=wrist;
        this.thisTarget=target;
        this.thisTimer=new ElapsedTime();
    }

    @Override
    public void initialize() {
        thisWrist.setPosition(thisTarget);

        //Start timer
        thisTimer.reset();

        //Calculate amount of time it should take to make this turn
        this.timeToFinish=(Math.abs(thisTarget-thisWrist.getPosition())* Constants.Arm.Wrist.WRIST_SEC_PER_ROTATION);
    }

    @Override
    public boolean isFinished() {
        return (thisTimer.time()>=timeToFinish);
    }
}
