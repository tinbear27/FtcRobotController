package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class ClawCommand extends InstantCommand {
    public ClawCommand(double target) {
        super(
                () -> RobotHardware.getInstance().claw.setPosition(target)
        );
    }
}
