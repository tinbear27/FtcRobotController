package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

public class LiftCommand extends CommandBase {
    private final LiftSubsystem thisLift;
    private final int thisTarget;

    public LiftCommand(LiftSubsystem lift, int target) {
        this.thisLift=lift;
        this.thisTarget=target;
    }

    @Override
    public void initialize() {
        thisLift.setTarget(thisTarget);
    }

    @Override
    public boolean isFinished() {
        return thisLift.reachedTarget();
    }
}
