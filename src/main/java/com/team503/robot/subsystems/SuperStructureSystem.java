/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team503.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;



/**
 * Parent interface for all motion magic subsystems: Arm, Wrist, Entension
 */
public interface SuperStructureSystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public abstract void setTargetPosition(double tgt);

  public abstract boolean getMagicStall();

  public abstract TalonSRX getTalon();

  public abstract double getMagicError();
}
