/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

public class RobotHardware {
	public SwerveDriveProperties swerveProps = new SwerveDriveProperties();

	public class SwerveDriveProperties{
		public FRONT_LEFT frontLeft = new FRONT_LEFT();
		public FRONT_RIGHT frontRight = new FRONT_RIGHT();
		public BACK_LEFT backLeft = new BACK_LEFT();
		public BACK_RIGHT backRight = new BACK_RIGHT();
		public class ModulePropertySet{
			public int az;
			public int dm;
			public double p;
			public double i;
			public double d;
			public double Cx;
			public double Cy;
			ModulePropertySet(double p,double  i ,double  d,double  Cx,double Cy, int az, int dm){
			
				this.p = 0.0;
				this.i = 0.0;
				this.d = 0.0;
				this.Cx = 0.0;
				this.Cy = 0.0;
				this.az = 0; 
				this.dm = 1;
			}
			
		}
		
		public class FRONT_LEFT{
			public ModulePropertySet props = new ModulePropertySet(0.0,0.0,0.0,0.0,0.0,5,1);
		}
		public class FRONT_RIGHT{
			public ModulePropertySet props = new ModulePropertySet(0.0,0.0,0.0,0.0,0.0,6,2);
		}
		public class BACK_LEFT{
			public ModulePropertySet props = new ModulePropertySet(0.0,0.0,0.0,0.0,0.0,8,4);
		}
		public class BACK_RIGHT{
			public ModulePropertySet props = new ModulePropertySet(0.0,0.0,0.0,0.0,0.0,7,3);
		}
		
	}
	
}