/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team503.lib.util;

import java.io.BufferedReader;
import java.io.FileReader;

import com.team503.robot.Robot;

import motionProfiling.Trajectory;

public class ProfileLoader {
    private Trajectory trajectory;
    private boolean isReversed;
    private double lookaheadDistance;

    public void storeTrajectory(String file) {

        if (!file.endsWith(".txt")) {
            file = file + ".txt";
        }
        if (!file.startsWith(Robot.bot.motionProfilingRioFolder)) {
            file = Robot.bot.motionProfilingRioFolder + file;
        }

        Trajectory trajectory = null;
        try (BufferedReader in = new BufferedReader(new FileReader(file))) {
            int length = Integer.parseInt(in.readLine());
            trajectory = new Trajectory(length);
            String[] data;
            for (int i = 0; i < length; i++) {
                String line = in.readLine();
                data = line.split(",");
                Trajectory.Segment seg = new Trajectory.Segment();
                seg.dt = Double.parseDouble(data[0]);
                seg.x = Double.parseDouble(data[1]);
                seg.y = Double.parseDouble(data[2]);
                seg.pos = Double.parseDouble(data[3]);
                seg.vel = Double.parseDouble(data[4]);
                seg.acc = Double.parseDouble(data[5]);
                seg.jerk = Double.parseDouble(data[6]);
                seg.heading = Double.parseDouble(data[7]);
                seg.curvature = Double.parseDouble(data[8]);
                trajectory.setSegment(i, seg);
            }
            this.isReversed = Boolean.parseBoolean(in.readLine());
            while (!in.readLine().equals("end"))
                ;
            double lookahead = Robot.bot.kMinLookAhead;
            try {
                lookahead = Double.parseDouble(in.readLine());
            } catch (Exception e) {
            }

            this.trajectory = trajectory;
            this.lookaheadDistance = lookahead;
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public Trajectory getTrajectory() {
        return this.trajectory;
    }

    public boolean getReversed() {
        return this.isReversed;
    }

    public double getLookaheadDistance() {
        return this.lookaheadDistance;
    }

}
