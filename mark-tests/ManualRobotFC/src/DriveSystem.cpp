//
//  Robot.cpp
//  
//
//  Created by Andrew Fisher-Shin on 1/27/15.
//
//

#include "WPILib.h"
#include "Math.h"
#include "Constants.h"

class DriveSystem {
    public:
    //Constructor
    DriveSystem(Encoder *left, Encoder *right, SpeedController *ml, SpeedController *mr) {
        m_pLeftEncoder = left;
        m_pRightEncoder = right;
        m_pscMotorLeft = ml;
        m_pscMotorRight = mr;
        distanceMultiplier = 0;
        pidDrive = true;
        
        m_pLeftEncoder->SetPIDSourceParameter(m_pLeftEncoder->kRate);
        m_pRightEncoder->SetPIDSourceParameter(m_pRightEncoder->kRate);
        double distPerPulse  = 1.0 / ENCODER_RESOLUTION;
        m_pLeftEncoder->SetDistancePerPulse(distPerPulse);
        m_pRightEncoder->SetDistancePerPulse(distPerPulse);
        m_pLeftEncoder->SetSamplesToAverage(5);
        m_pRightEncoder->SetSamplesToAverage(5);
		m_pLeftEncoder->SetMinRate(0.25);
		m_pRightEncoder->SetMinRate(0.25);
        controlLeft = new PIDController(PROPORTIONAL_TERM, INTEGRAL_TERM, DIFFERENTIAL_TERM,
        		m_pLeftEncoder, m_pscMotorLeft);
        controlRight = new PIDController(PROPORTIONAL_TERM, INTEGRAL_TERM, DIFFERENTIAL_TERM,
        		m_pRightEncoder, m_pscMotorRight);
    }

    DriveSystem(SpeedController *ml, SpeedController *mr) {
           m_pscMotorLeft = ml;
           m_pscMotorRight = mr;
           distanceMultiplier = 0;
           pidDrive = false;

       }

    //Destructor
    ~DriveSystem() {
        if(pidDrive == true) {
        	delete m_pLeftEncoder;
        	delete m_pRightEncoder;
        	delete controlLeft;
        	delete controlRight;
        }
        delete m_pscMotorLeft;
        delete m_pscMotorRight;
    }

    void SetDriveInstruction(float velocity, float angle) {
        this->velocity = velocity;
        this->angle = angle / 2.0f;
    }
    
    float GetRobotSpeedInRPM() {
        return ((m_pLeftEncoder->GetRate() - m_pRightEncoder->GetRate()) / 2.0) * 60.0;
    }

    float GetRobotSpeedInRPS() {
        return ((m_pLeftEncoder->GetRate() - m_pRightEncoder->GetRate()) / 2.0);
    }


    float GetRobotSpeedInFPS()
    {
    	return(GetRobotSpeedInRPS()*PI*WHEEL_DIAMETER/12.0);
    }

    float GetLeftPIDOutput(){
    	return(controlLeft->Get());
    }


    float GetRightPIDOutput(){
    	return(controlRight->Get());
    }

    float GetLeftPIDSetpoint(){
    	return(controlLeft->GetSetpoint());
    }


    float GetRightPIDSetpoint(){
    	return(controlRight->GetSetpoint());
    }

    float GetLeftPIDError(){
    	return(controlLeft->GetError());
    }
    
    float GetRightPIDError(){
    	return(controlRight->GetError());
    }

    void SetWheelDiameter(float diameter) {
        float distPerRev = diameter * PI;
        distanceMultiplier = distPerRev / 1024;
    }
    
    double GetDistanceTraveledRot() {
        return fabs((double)m_pLeftEncoder->Get() - (double)startingTick) * distanceMultiplier;
    }

    double GetDistanceTraveledFeet() {
        return fabs((double)m_pLeftEncoder->Get() - (double)startingTick) * distanceMultiplier * PI * WHEEL_DIAMETER;
    }
    
    void StartRecordingDistance() {
        startingTick = m_pLeftEncoder->Get();
    }
    
    void SetPIDDrive(bool pid) {
        // When in PID mode, send speeds for instructions, else send motor power values
//      m_pLeftEncoder->Reset();
//      m_pRightEncoder->Reset();

    	pidDrive = pid;
        if (pid && !controlLeft->IsEnabled()) {
        	m_pLeftEncoder->SetPIDSourceParameter(m_pLeftEncoder->kRate);
        	m_pRightEncoder->SetPIDSourceParameter(m_pRightEncoder->kRate);
            controlLeft->Enable();
            controlRight->Enable();
        } else if (controlLeft->IsEnabled()) {
            controlLeft->Disable();
            controlRight->Disable();
        }
    }
    
    
    void Update() {
    SetMotorSpeedLeft(GetLeftDriveMotorSpeed());
    SetMotorSpeedRight(GetRightDriveMotorSpeed());
    }
    
    float GetRightEncoder() {
        return m_pRightEncoder->GetRate();
    }
    
    float GetLeftEncoder() {
        return m_pLeftEncoder->GetRate();
    }
    
    float GetLeftDriveMotorSpeed() {
        double speed = velocity;
         speed += angle * .7f;
        return speed;
    }
    
    float GetRightDriveMotorSpeed() {
        double speed = velocity;
        speed -= angle * .7f;
        return speed;
    }

    private:
    Encoder *m_pLeftEncoder;
    Encoder *m_pRightEncoder;
    SpeedController *m_pscMotorLeft;
    SpeedController *m_pscMotorRight;
    PIDController *controlLeft;
    PIDController *controlRight;
    double distanceMultiplier;
    double velocity;
    double angle;
    int startingTick;
    bool pidDrive;

    
    void SetMotorSpeedLeft(float speed) {
    	char myString[64];

        if (pidDrive) {
    		//sprintf(myString, "pid SP L: %5.2f\n", -speed);
    		//SmartDashboard::PutString("DB/String 8", myString);
            controlLeft->SetSetpoint(-speed);
        } else {
    		//sprintf(myString, "setpoint L: %5.2f\n", -speed);
    		//SmartDashboard::PutString("DB/String 8", myString);
            m_pscMotorLeft->Set(-speed);
        }
    }
    
    void SetMotorSpeedRight(float speed) {
    	char myString[64];

        if (pidDrive) {
    		//sprintf(myString, "pid SP R: %5.2f\n", speed);
    		//SmartDashboard::PutString("DB/String 9", myString);
            controlRight->SetSetpoint(speed);
        } else {
    		//sprintf(myString, "setpoint R: %5.2f\n", speed);
    		//SmartDashboard::PutString("DB/String 9", myString);
            m_pscMotorRight->Set(speed);
        }
    }
};
