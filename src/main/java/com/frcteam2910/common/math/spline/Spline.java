package com.frcteam2910.common.math.spline;

import com.frcteam2910.common.math.Rotation2;
import com.frcteam2910.common.math.Vector2;

public abstract class Spline {

    public abstract Vector2 getPoint(double t);

    public abstract Rotation2 getHeading(double t);
}
