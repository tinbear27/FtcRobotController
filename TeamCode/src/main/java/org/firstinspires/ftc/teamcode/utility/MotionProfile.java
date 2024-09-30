package org.firstinspires.ftc.teamcode.utility;

import androidx.annotation.NonNull;

public class MotionProfile {
    //State
    public MotionProfileState state = new MotionProfileState();

    //Constraints
    public double accel=0.0;
    public double decel=0.0;
    public double velo=0.0;

    //Profile variables
    public double initialPosition;
    public double finalPosition;
    public double distance;
    public double t1, t2, t3;
    public double totalTime;
    public double t1_stop_position;
    public double max_velocity;
    public double t2_stop_position;
    public boolean flipped = false;
    public double originalPos = 0;

    public MotionProfile(double initialPosition, double finalPosition, double accel, double decel, double velo) {
        //Set constraints
        this.accel=accel;
        this.decel=decel;
        this.velo=velo;

        //Set positions
        if (finalPosition < initialPosition) {
            flipped = true;
            this.originalPos = initialPosition;
            double temp = initialPosition;
            initialPosition = finalPosition;
            finalPosition = temp;
        }
        this.initialPosition = initialPosition;
        this.finalPosition = finalPosition;
        this.distance = finalPosition - initialPosition;

        t1 = velo / accel;
        t3 = velo / decel;
        t2 = Math.abs(distance) / velo - (t1 + t3) / 2;

        if (t2 < 0) {
            this.t2 = 0;

            double a = (accel / 2) * (1 - accel / -decel);
            double c = -distance;

            t1 = Math.sqrt(-4 * a * c) / (2 * a);
            t3 = -(accel * t1) / -decel;
            t1_stop_position = (accel * Math.pow(t1, 2)) / 2;

            max_velocity = accel * t1;

            t2_stop_position = t1_stop_position;
        } else {
            max_velocity = velo;
            t1_stop_position = (velo * t1) / 2;
            t2_stop_position = t1_stop_position + t2 * max_velocity;
        }

        totalTime = t1 + t2 + t3;
    }

    public MotionProfileState calculate(final double time) {
        double position, velocity, acceleration, stage_time;
        if (time <= t1) {
            stage_time = time;
            acceleration = accel;
            velocity = acceleration * stage_time;
            position = velocity * stage_time / 2;
        } else if (time <= t1 + t2) {
            stage_time = time - t1;
            acceleration = 0;
            velocity = velo;
            position = t1_stop_position + stage_time * velocity;
        } else if (time <= totalTime) {
            stage_time = time - t1 - t2;
            acceleration = -decel;
            velocity = max_velocity - stage_time * decel;
            position = t2_stop_position + (max_velocity + velocity) / 2 * stage_time;
        } else {
            acceleration = 0;
            velocity = 0;
            position = finalPosition;
        }

        // lol
        if (time <= totalTime) {
            if (flipped) {
                state.x = originalPos - position;
            } else {
                state.x = initialPosition + position;
            }
        } else {
            if (flipped) {
                state.x = initialPosition;
            } else {
                state.x = originalPos + position;
            }
        }
        state.v = velocity;
        state.a = acceleration;
        return this.state;
    }

    @NonNull
    @Override
    public String toString() {
        return String.format("%f, %f, %f", state.x, state.v, state.a);
    }
}
