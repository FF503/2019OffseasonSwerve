// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// package com.team503.lib.io;

// import com.team503.lib.geometry.Translation2d;
// import com.team503.lib.util.FFDashboard;
// import com.team503.robot.subsystems.Pigeon;
// import com.team503.robot.subsystems.SwerveDrive;
// import com.team503.robot.subsystems.SwerveDrive.DriveMode;

// import edu.wpi.first.networktables.EntryListenerFlags;

// /**
//  * Add your docs here.
//  */
// public class PrecisionDriveController {
//     private static FFDashboard table = new FFDashboard("Input");
//     private static boolean enable = false;

//     public static void initalize() {
//         table.getEntry("New Input").addListener(event -> {
//             boolean checkBoolean = event.getEntry().getBoolean(false);
//             if (enable && checkBoolean) {
//                 SwerveDrive.getInstance().setMode(DriveMode.KeyboardControl);
//                 double forward = table.getNumber("Forward", 0.0);
//                 double strafe = table.getNumber("Strafe", 0.0);
//                 double rotation = table.getNumber("Rotation", 0.0);
//                 Translation2d translationalVector = new Translation2d(strafe, forward);
//                 // if (rotation != 0.0) {
//                     SwerveDrive.getInstance().drive(translationalVector, rotation);
//                     SwerveDrive.getInstance().rotate(Pigeon.getInstance().getYaw());
//                 // } else {
//                     // SwerveDrive.getInstance().drive(translationalVector);
//                 // }
//             }
//             event.getEntry().setBoolean(false);
//         }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
//     }

//     static {
//         initalize();
//     }

//     public static void activatePrecisionDrive() {
//         toggleEnable(true);
//     }

//     public static void disablePrecisionDrive() {
//         toggleEnable(false);
//     }

//     private static void toggleEnable(boolean enable) {
//         PrecisionDriveController.enable = enable;
//     }
// }
