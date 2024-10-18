package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.ArmAngleSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ArmWinchSubsystem;

public class ArmWinchCommand extends CommandBase {
    private final ArmWinchSubsystem thisArmWinch;
    private final int thisTarget;

    public ArmWinchCommand(ArmWinchSubsystem armWinch, int target) {
        /*
        super(
                () -> RobotHardware.getInstance().armWinch.setPosition(target)
        );
         */

        this.thisArmWinch=armWinch;
        this.thisTarget=target;
    }

    @Override
    public void initialize() {
        thisArmWinch.setPosition(thisTarget);
    }

    @Override
    public boolean isFinished() {
        return thisArmWinch.reachedTarget();
    }
}
