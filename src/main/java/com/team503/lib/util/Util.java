package com.team503.lib.util;

import java.io.BufferedReader;
import java.io.FileReader;
import java.util.List;

import com.team503.lib.geometry.Translation2d;
import com.team503.robot.Robot;
import com.team503.robot.RobotState.Bot;
import com.team503.robot.subsystems.SwerveModule;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.simple.SimpleMatrix;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

import edu.wpi.first.wpilibj.Filesystem;

/**
 * Contains basic functions that are used often.
 */
public class Util {
    private static DMatrixRMaj invertedWheelPositionMatrix;
    public static final double kEpsilon = 1e-12;
    // static {
    // invertedWheelPositionMatrix = getPseudoInverseForwardKinematicsMatrix();
    // }

    /** Prevent this class from being instantiated. */
    private Util() {
    }

    /**
     * Limits the given input to the given magnitude.
     */
    public static double limit(double v, double maxMagnitude) {
        return limit(v, -maxMagnitude, maxMagnitude);
    }

    public static double limit(double v, double min, double max) {
        return Math.min(max, Math.max(min, v));
    }

    public static String joinStrings(String delim, List<?> strings) {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < strings.size(); ++i) {
            sb.append(strings.get(i).toString());
            if (i < strings.size() - 1) {
                sb.append(delim);
            }
        }
        return sb.toString();
    }

    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, kEpsilon);
    }

    public static boolean allCloseTo(List<Double> list, double value, double epsilon) {
        boolean result = true;
        for (Double value_in : list) {
            result &= epsilonEquals(value_in, value, epsilon);
        }
        return result;
    }

    public static double normalize(double current, double test) {
        if (current > test)
            return current;
        return test;
    }

    public static double boundAngleNeg180to180Degrees(double angle) {
        // Naive algorithm
        while (angle >= 180.0) {
            angle -= 360.0;
        }
        while (angle < -180.0) {
            angle += 360.0;
        }
        return angle;
    }

    public static double boundAngle0to360Degrees(double angle) {
        // Naive algorithm hmmm angle % 360 was too simple I guess
        while (angle >= 360.0) {
            angle -= 360.0;
        }
        while (angle < 0.0) {
            angle += 360.0;
        }
        return angle;
    }

    public static double boundToScope(double scopeFloor, double scopeCeiling, double argument) {
        double stepSize = scopeCeiling - scopeFloor;
        while (argument >= scopeCeiling) {
            argument -= stepSize;
        }
        while (argument < scopeFloor) {
            argument += stepSize;
        }
        return argument;
    }

    public static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
        double lowerBound;
        double upperBound;
        double lowerOffset = scopeReference % 360;
        if (lowerOffset >= 0) {
            lowerBound = scopeReference - lowerOffset;
            upperBound = scopeReference + (360 - lowerOffset);
        } else {
            upperBound = scopeReference - lowerOffset;
            lowerBound = scopeReference - (360 + lowerOffset);
        }
        while (newAngle < lowerBound) {
            newAngle += 360;
        }
        while (newAngle > upperBound) {
            newAngle -= 360;
        }
        if (newAngle - scopeReference > 180) {
            newAngle -= 360;
        } else if (newAngle - scopeReference < -180) {
            newAngle += 360;
        }
        return newAngle;
    }

    public static boolean shouldReverse(double goalAngle, double currentAngle) {
        goalAngle = boundAngle0to360Degrees(goalAngle);
        currentAngle = boundAngle0to360Degrees(currentAngle);
        double reversedAngle = boundAngle0to360Degrees(currentAngle + 180);
        double angleDifference = Math.abs(goalAngle - currentAngle);
        double reversedAngleDifference = Math.abs(goalAngle - reversedAngle);
        angleDifference = (angleDifference > 180) ? 360 - angleDifference : angleDifference;
        reversedAngleDifference = (reversedAngleDifference > 180) ? 360 - reversedAngleDifference
                : reversedAngleDifference;
        if (reversedAngle < angleDifference) {
            System.out.println(alternateShouldReverse(goalAngle, currentAngle));
        }
        return reversedAngleDifference < angleDifference;
    }

    /**
     * written as a possible alternative to test logic (when tested in a test class,
     * it produced the exact same outputs as the other method so this is not the
     * problem)
     * 
     * @param goal
     * @param cur
     * @return
     */
    public static boolean alternateShouldReverse(double goal, double cur) {
        goal = boundAngle0to360Degrees(goal);
        cur = boundAngle0to360Degrees(cur);
        if (Math.abs(goal - cur) <= 90.0) {
            return false;
        }
        if (360 - Math.abs(goal - cur) > 90.0) {
            return true;
        }
        return false;
    }

    public static double deadBand(double val, double deadband) {
        return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
    }

    public static SwerveModule readSwerveJSON(String file) throws Exception {
        file += (!file.endsWith(".json")) ? ".json" : "";
        // parsing file "JSONExample.json"
        Object obj = new JSONParser()
                .parse(new FileReader(Filesystem.getDeployDirectory().getAbsolutePath() + "/SwerveModules/" + file));

        // typecasting obj to JSONObject
        JSONObject jo = (JSONObject) obj;

        // getting firstName and lastName
        String moduleName = (String) jo.get("name");
        int driveMotorID = Math.toIntExact((Long) jo.get("driveMotorID"));
        int turnMotorID = Math.toIntExact((Long) jo.get("turnMotorID"));
        double p = (double) jo.get("P");
        double i = (double) jo.get("I");
        double d = (double) jo.get("D");
        double f = (double) jo.get("F");
        int startEnc = Math.toIntExact((Long) jo.get("startingEncoderClick"));
        int cv = Math.toIntExact((Long) jo.get("cruiseVelocity"));
        int ca = Math.toIntExact((Long) jo.get("cruiseAccel"));
        boolean tcd = (boolean) jo.get("turnCountsDecreasing");
        boolean driveInvert = (boolean) jo.get("DriveInverted");
        boolean driveEncInvert = (boolean) jo.get("DriveEncoderInverted");
        boolean turnMotorInvert = (boolean) jo.get("TurnMotorInverted");
        boolean turnEncInvert = (boolean) jo.get("TurnEncoderInverted");

        SwerveModule m = new SwerveModule(driveMotorID, turnMotorID, p, i, d, f, startEnc, cv, ca, tcd, driveInvert,
                driveEncInvert, turnMotorInvert, turnEncInvert);
        return m;
    }

    public static String readRobotName() {
        final String dir = "/home/lvuser/robotInfo.txt";
        String robotName;
        try (BufferedReader in = new BufferedReader(new FileReader(dir))) {
            robotName = in.readLine();
        } catch (Exception e) {
            System.err.println("Unable to fetch Robot name automatically.");
            robotName = "";
        }

        return robotName;

    }

    public static Bot parseRobotNameToEnum(final String robotName) {
        return Bot.valueOf(robotName);
    }

    public static DMatrixRMaj setPseudoInverseForwardKinematicsMatrix() {
        double r1x = Robot.bot.kWheelbaseWidth / 2;
        double r1y = Robot.bot.kWheelbaseLength / 2;
        double r2x = -r1x;
        double r2y = r1y;
        double r3x = r2x;
        double r3y = -r1y;
        double r4x = r1x;
        double r4y = r3y;
        double[][] wheelPositions = { { 1, 0, -r1y }, { 0, 1, r1x }, { 1, 0, -r2y }, { 0, 1, r2x }, { 1, 0, -r3y },
                { 0, 1, r3x }, { 1, 0, -r4y }, { 0, 1, r4x } };

        DMatrixRMaj wheelPositionMatrix = new DMatrixRMaj(wheelPositions);
        DMatrixRMaj output = new DMatrixRMaj(wheelPositions);
        CommonOps_DDRM.pinv(wheelPositionMatrix, output);
        Util.invertedWheelPositionMatrix = output;
        return output;
    }

    public static Translation2d getVelocity(SimpleMatrix wheelVelocities) {
        SimpleMatrix simpleMatrix = invertedWheelPositionMatrix.createLike();
        SimpleMatrix cardinalVelocities = simpleMatrix.mult(wheelVelocities);
        return new Translation2d(cardinalVelocities.get(0, 0), cardinalVelocities.get(1, 0));
    }

    // converts angle from 0 forwards and clockwise to 0 to the right and counter
    // clockwise
    public static double unitCircleify(double angle) {
        return 90 - angle;
    }

    /**
     * @return converted coordinates (x,y,theta)
     * @param cSystem     coordinate system to convert to
     * @param translation translation
     * @param theta       input theta
     */
    public static double[] convertCoordinateSystem(CoordinateSystem cSystem, Translation2d translation, double theta) {
        double x = translation.getX();
        double y = translation.getY();
        return convertCoordinateSystem(cSystem, x, y, theta);
    }

    /**
     * @return converted coordinates (x,y,theta)
     * @param cSystem coordinate system to convert to
     * @param x       input x
     * @param y       input y
     * @param theta   input theta
     */
    public static double[] convertCoordinateSystem(CoordinateSystem cSystem, double x, double y, double theta) {
        double[] converted = new double[3];
        if (cSystem.equals(CoordinateSystem.UNIT_CIRCLE)) {
            // Converts rotated to unit circle
            converted[0] = -y;
            converted[1] = x;
            converted[2] = 90.0 + theta;
        } else if (cSystem.equals(CoordinateSystem.ROTATED)) {
            // Converts unit circle to rotated
            converted[0] = y;
            converted[1] = -x;
            converted[2] = theta - 90.0;
        }

        return converted;
    }

    public static enum CoordinateSystem {
        UNIT_CIRCLE, ROTATED;
    }

}
