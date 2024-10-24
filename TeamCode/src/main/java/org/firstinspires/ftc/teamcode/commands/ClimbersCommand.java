package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ClimbersSubsystem;

public class ClimbersCommand extends CommandBase {
    private final ClimbersSubsystem thisClimbers;
    private final int thisTarget;

    public ClimbersCommand(ClimbersSubsystem climbers, int target) {
        this.thisClimbers=climbers;
        this.thisTarget=target;
    }

    @Override
    public void initialize() {
        thisClimbers.setTarget(thisTarget);
    }

    @Override
    public boolean isFinished() {
        return thisClimbers.reachedTarget();
    }
}
