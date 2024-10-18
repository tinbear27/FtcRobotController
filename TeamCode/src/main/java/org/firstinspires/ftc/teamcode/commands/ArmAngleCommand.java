package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.ArmAngleSubsystem;

public class ArmAngleCommand extends CommandBase {
    private final ArmAngleSubsystem thisArmAngle;
    private final int thisTarget;

    public ArmAngleCommand(ArmAngleSubsystem armAngle, int target) {
        /*
        super(
                () -> RobotHardware.getInstance().armAngle.setPosition(target)
        );
        */

        this.thisArmAngle=armAngle;
        this.thisTarget=target;
    }

    @Override
    public void initialize() {
        thisArmAngle.setPosition(thisTarget);
    }

    @Override
    public boolean isFinished() {
        return thisArmAngle.reachedTarget();
    }
}
