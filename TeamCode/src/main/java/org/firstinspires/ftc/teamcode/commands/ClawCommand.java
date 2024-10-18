package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;

public class ClawCommand extends CommandBase {
    private final ClawSubsystem thisClaw;
    private final double thisTarget;
    private ElapsedTime thisTimer;
    private double timeToFinish;

    public ClawCommand(ClawSubsystem claw, double target) {
        /*
        super(
                () -> RobotHardware.getInstance().claw.setPosition(target)
        );
         */

        this.thisClaw=claw;
        this.thisTarget=target;
        this.thisTimer=new ElapsedTime();
    }

    @Override
    public void initialize() {
        thisClaw.setPosition(thisTarget);

        //Start timer
        thisTimer.reset();

        //Calculate amount of time it should take to make this turn
        this.timeToFinish=(Math.abs(thisTarget-thisClaw.getPosition())* Constants.Arm.Claw.CLAW_SEC_PER_ROTATION);
    }

    @Override
    public boolean isFinished() {
        return (thisTimer.time()>=timeToFinish);
    }
}
