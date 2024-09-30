package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class ArmAngleCommand extends InstantCommand {
    public ArmAngleCommand(int target) {
        super(
                () -> RobotHardware.getInstance().armAngle.setPosition(target)
        );
    }
}
