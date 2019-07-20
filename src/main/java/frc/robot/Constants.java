/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Arrays;
import java.util.List;

import com.team254.lib.geometry.Translation2d;


public class Constants {
    /* All distance measurements are in inches, unless otherwise noted */

    //Swerve Calculations Constants (measurements are in inches)
    public static final double kWheelbaseLength = 21.0;
    public static final double kWheelbaseWidth  = 21.0;

    //Swerve Module Positions (relative to the center of the drive base)
	public static final Translation2d kVehicleToModuleZero = new Translation2d(kWheelbaseLength/2, -kWheelbaseWidth/2);
	public static final Translation2d kVehicleToModuleOne = new Translation2d(-kWheelbaseLength/2, -kWheelbaseWidth/2);
	public static final Translation2d kVehicleToModuleTwo = new Translation2d(-kWheelbaseLength/2, kWheelbaseWidth/2);
	public static final Translation2d kVehicleToModuleThree = new Translation2d(kWheelbaseLength/2, kWheelbaseWidth/2);

    public static final List<Translation2d> kModulePositions = Arrays.asList(kVehicleToModuleZero,
			kVehicleToModuleOne, kVehicleToModuleTwo, kVehicleToModuleThree);
}
