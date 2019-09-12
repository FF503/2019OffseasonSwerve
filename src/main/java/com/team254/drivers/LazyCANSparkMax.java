package com.team254.drivers;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

/**
 * This class is a thin wrapper around the CANTalon that reduces CAN bus / CPU
 * overhead by skipping duplicate set commands. (By default the Talon flushes
 * the Tx buffer on every set call).
 */
public class LazyCANSparkMax extends CANSparkMax {
    protected double mLastSet = Double.NaN;
    protected ControlMode mLastControlMode = ControlMode.PercentOutput;
    protected int mSlotID = 0;

    protected CANEncoder mEncoder;
    protected CANPIDController mPidController;

    public LazyCANSparkMax(int deviceNumber) {
        super(deviceNumber, MotorType.kBrushless);
        super.restoreFactoryDefaults();
        mEncoder = this.getEncoder();
        mPidController = this.getPIDController();

        mPidController.setOutputRange(-1, 1);
        mPidController.setSmartMotionMinOutputVelocity(0, 0);
        mPidController.setSmartMotionMinOutputVelocity(0, 1);
    }

    public double getLastSet() {
        return mLastSet;
    }

    public ControlMode getControlMode() {
        return mLastControlMode;
    }

    public void set(ControlMode mode, double value) {
        // if (value != mLastSet) {
        // mLastSet = value;
        // super.set(value);
        // }
        if (value != mLastSet || mode != mLastControlMode) {
            mLastSet = value;
            mLastControlMode = mode;

            if (mode == ControlMode.PercentOutput) {
                super.set(value);
            } else {
                mPidController.setReference(value, ControlType.kSmartMotion, mSlotID);
            }
        }
    }

    public void setSensorPhase(boolean phase) {
        mEncoder.setPositionConversionFactor(phase ? 1 : -1);
        mEncoder.setVelocityConversionFactor(phase ? 1 : -1);
    }

    public void setP(int slotID, double kP) {
        mPidController.setP(kP, slotID);
    }

    public void setI(int slotID, double kI) {
        mPidController.setI(kI, slotID);
    }

    public void setD(int slotID, double kD) {
        mPidController.setD(kD, slotID);
    }

    public void setIzone(int slotID, double kIz) {
        mPidController.setIZone(kIz, slotID);
    }

    public void setFF(int slotID, double kFF) {
        mPidController.setFF(kFF, slotID);
    }

    public void setCruiseVelocity(int slotID, double velocity) {
        mPidController.setSmartMotionMaxVelocity(velocity, slotID);
    }

    public void setAcceleration(int slotID, double acceleration) {
        mPidController.setSmartMotionMaxAccel(acceleration, slotID);
    }

    public void selectProfileSlot(int slotID) {
        mSlotID = slotID;
    }

    public int getSelectedProfileSlot() {
        return mSlotID;
    }

    public void setEncoderPosition(double position) {
        mEncoder.setPosition(position);
    }

    public double getEncoderPosition() {
        return mEncoder.getPosition();
    }

    public double getEncoderVelocity() {
        return mEncoder.getVelocity();
    }

    public double getClosedLoopError() {
        return getEncoderPosition() - mLastSet;
    }

    public enum ControlMode {
        SmartMotion, PercentOutput;
    }

}