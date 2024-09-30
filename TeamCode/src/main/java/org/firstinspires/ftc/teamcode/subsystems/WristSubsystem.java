package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants.Arm.Wrist;

public class WristSubsystem extends SubsystemBase {
    private final Servo wristServo;
    private double currentPosition;

    public WristSubsystem(final HardwareMap hMap) {
        wristServo = hMap.get(Servo.class, Wrist.WRIST_ID);
    }

    public void setPositionByIndex(int indexNum) {
        setPosition(Wrist.WRIST_POSITIONS[indexNum]);
    }

    public void setPosition(double position) {
        this.currentPosition=position;
        wristServo.setPosition(position);
    }

    public double getPosition() {
        return currentPosition;
    }
}
