package com.team503.robot.subsystems.requests;

public class EmptyRequest extends Request{
	
	@Override
	public void act(){
		//empty, as the name suggests
	}
	
	@Override
	public boolean isFinished(){
		return true;
	}
	
}
