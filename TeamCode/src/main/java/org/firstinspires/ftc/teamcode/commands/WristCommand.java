package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class WristCommand extends InstantCommand {
    public WristCommand(double target) {
        super(
                () -> RobotHardware.getInstance().wrist.setPosition(target)
        );
    }
}
