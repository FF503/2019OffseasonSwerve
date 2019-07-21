package com.team503.lib.util;

import java.io.FileReader;
import java.util.List;

import com.team503.robot.subsystems.SwerveModule;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

import edu.wpi.first.wpilibj.Filesystem;



/**
 * Contains basic functions that are used often.
 */
public class Util {

    public static final double kEpsilon = 1e-12;

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
    
    public static double normalize(double current, double test){
    	if(current > test) return current;
    	return test;
    }
    
    public static double boundAngleNeg180to180Degrees(double angle){
        // Naive algorithm
        while(angle >= 180.0) {angle -= 360.0;}
        while(angle < -180.0) {angle += 360.0;}
        return angle;
    }
    
    public static double boundAngle0to360Degrees(double angle){
        // Naive algorithm hmmm angle % 360 was too simple I guess
        while(angle >= 360.0) {angle -= 360.0;}
        while(angle < 0.0) {angle += 360.0;}
        return angle;
    }
    
    public static double boundToScope(double scopeFloor, double scopeCeiling, double argument){
    	double stepSize = scopeCeiling - scopeFloor;
    	while(argument >= scopeCeiling) {argument -= stepSize;}
    	while(argument < scopeFloor) {argument += stepSize;}
    	return argument;
    }
    
    public static double placeInAppropriate0To360Scope(double scopeReference, double newAngle){
    	double lowerBound;
        double upperBound;
        double lowerOffset = scopeReference % 360;
        if(lowerOffset >= 0){
        	lowerBound = scopeReference - lowerOffset;
        	upperBound = scopeReference + (360 - lowerOffset);
        }else{
        	upperBound = scopeReference - lowerOffset; 
        	lowerBound = scopeReference - (360 + lowerOffset);
        }
        while(newAngle < lowerBound){
        	newAngle += 360; 
        }
        while(newAngle > upperBound){
        	newAngle -= 360; 
        }
        if(newAngle - scopeReference > 180){
        	newAngle -= 360;
        }else if(newAngle - scopeReference < -180){
        	newAngle += 360;
        }
        return newAngle;
    }
    
    public static boolean shouldReverse(double goalAngle, double currentAngle){
    	goalAngle = boundAngle0to360Degrees(goalAngle);
    	currentAngle = boundAngle0to360Degrees(currentAngle);
    	double reversedAngle = boundAngle0to360Degrees(currentAngle + 180);
    	double angleDifference = Math.abs(goalAngle - currentAngle);
    	double reversedAngleDifference = Math.abs(goalAngle - reversedAngle);
    	angleDifference = (angleDifference > 180) ? 360-angleDifference : angleDifference;
    	reversedAngleDifference = (reversedAngleDifference > 180) ? 360-reversedAngleDifference : reversedAngleDifference;
    	return reversedAngleDifference < angleDifference;
    }
    
    public static double deadBand(double val, double deadband){
        return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
    }

    public static SwerveModule readSwerveJSON(String file) throws Exception{ 
        file += (!file.endsWith(".json")) ? ".json" : "";
        // parsing file "JSONExample.json" 
        Object obj = new JSONParser().parse(new FileReader(Filesystem.getDeployDirectory().getAbsolutePath() + "/SwerveModules/" + file)); 
          
        // typecasting obj to JSONObject 
        JSONObject jo = (JSONObject) obj; 
          
        // getting firstName and lastName 
        String moduleName = (String) jo.get("name"); 
        int driveMotorID = Math.toIntExact((Long) jo.get("driveMotorID"));
        int  turnMotorID = Math.toIntExact((Long) jo.get("turnMotorID"));
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

        SwerveModule m = new SwerveModule(driveMotorID, turnMotorID, p, i, d, f, startEnc, cv, ca, tcd, driveInvert, driveEncInvert, turnMotorInvert, turnEncInvert);
        return m;
    }
}
