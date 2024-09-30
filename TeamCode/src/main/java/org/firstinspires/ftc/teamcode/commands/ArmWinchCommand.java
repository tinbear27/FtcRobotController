package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class ArmWinchCommand extends InstantCommand {
    public ArmWinchCommand(int target) {
        super(
                () -> RobotHardware.getInstance().armWinch.setPosition(target)
        );
    }
}
