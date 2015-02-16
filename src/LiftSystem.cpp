#include "WPILib.h"
#include "Constants.h"

class LiftSystem
{
public:
	typedef enum {opened_narrow_GP,
		opened_wide_GP,
		closed_idle,
		closed_C_Pos_One,
		closed_C_Pos_Two,
		closed_C_Pos_Three,
		closed_C_Pos_Step,
		lowering_Pos_One,
		elevating_Pos_Two,
		lowering_Pos_Two,
		elevating_Pos_Three,
		elevating_Pos_Step,
		lowering_Pos_Step} RobotState;

	typedef enum {narrow_idle,
		narrow_closing,
		opening_wide,
		return_narrow} OpenedNarrowState;

	typedef enum {wide_idle,
		wide_closing,
		opening_narrow,
		return_wide} OpenedWideState;

	typedef enum {closed_raising,
		releasing} ClosedState;

	typedef enum {release_lowering,
		waiting,
		release_opening_narrow,
		release_opening_wide} LiftState;

	typedef enum { fork_opened_narrow,
		fork_opened_wide } PrevForkState;

	LiftSystem(CANTalon *pforkMotor, VictorSP *pliftMotor, Encoder *forkEnc, Encoder *liftEnc,\
			DigitalInput *forkLimitMin, DigitalInput *forkLimitMax, DigitalInput *liftLimitMin,\
			DigitalInput *liftLimitMax)
	{
		forkMotor = pforkMotor;
		liftMotor = pliftMotor;
		forkEncoder = forkEnc;
		liftEncoder = liftEnc;

		forkLimitSwitchMin = forkLimitMin;
		forkLimitSwitchMax = forkLimitMax;
		liftLimitSwitchMin = liftLimitMin;
		liftLimitSwitchMax = liftLimitMax;

		prevForkState = fork_opened_wide;

		/*fOpNarrowVal = ;
		fOpWideVal = ;

		lPosGroundOneVal = ;
		lPosCOneVal = ;
		lPosCTwoVal = ;
		lPosCThreeVal = ;
		lPosCStepVal = ;
		liftOffsetVal = ;

		forkCurrentSpike = ; */
	}

	~LiftSystem()
	{
		delete forkMotor;
		delete liftMotor;
		delete forkEncoder;
		delete liftEncoder;
		delete forkLimitSwitchMin;
		delete forkLimitSwitchMax;
		delete liftLimitSwitchMin;
		delete liftLimitSwitchMax;
	}

	LiftState GetLiftState()
	{
		return liftState;
	}

	void SetPrevForkState(PrevForkState state)
	{
		prevForkState = state;
	}

	//Limit Switches:
	bool GetForkLimitSwitchMin()
	{
		return forkLimitSwitchMin->Get();
	}

	bool GetForkLimitSwitchMax()
	{
		return forkLimitSwitchMax->Get();
	}

	bool GetLiftLimitSwitchMin()
	{
		return liftLimitSwitchMin->Get();
	}

	bool GetLiftLimitSwitchMax()
	{
		return liftLimitSwitchMax->Get();
	}

	//Motors
	void SetForkMotor(float val)
	{
		forkMotor->Set(val);
	}

	void SetLiftMotor(float val)
	{
		liftMotor->Set(val);
	}

	void Close()
	{
		close = true;
	}

	void OpenNarrow()
	{
		openNarrow = true;
	}

	void OpenWide()
	{
		openWide = true;
	}

	void MovePosOne()
	{
		movePosOne = true;
	}

	void MovePosTwo()
	{
		movePosTwo = true;
	}

	void MovePosThree()
	{
		movePosThree = true;
	}

	void MovePosStep()
	{
		movePosStep = true;
	}

	void Update()
	{
		switch (robotState)
		{
			case opened_narrow_GP:
				if (close)
					openedNarrowState = narrow_closing;
				else if (openWide)
					openedNarrowState = opening_wide;
				switch(openedNarrowState)
				{
					case narrow_idle:
						SetForkMotor(0.0f);
						break;

					case narrow_closing:
						prevForkState = fork_opened_narrow;
						if (!(forkMotor->GetOutputCurrent() >= forkCurrentSpike))
							SetForkMotor(1.0f);
						else
						{
							SetForkMotor(0.0f);
							robotState = closed_idle;
						}

						break;

					case opening_wide:
						if (forkEncoder->Get() != fOpWideVal)
							SetForkMotor(1.0f);
						else
						{
							SetForkMotor(0.0f);
							robotState = opened_wide_GP;
						}

						break;

					case return_narrow:
						if (forkEncoder->Get() != fOpNarrowVal)
							SetForkMotor(1.0f);
						else
							openedNarrowState = narrow_idle;

						break;
				}
				break;

			case opened_wide_GP:
				if (close)
					openedWideState = wide_closing;
				else if (openNarrow)
					openedWideState = opening_narrow;
				switch(openedWideState)
				{
					case wide_idle:
						SetForkMotor(0.0f);
						break;

					case wide_closing:
					{
						prevForkState = fork_opened_wide;
						if (!(forkMotor->GetOutputCurrent() >= forkCurrentSpike))
							SetForkMotor(-1.0l);
						else
						{
							SetForkMotor(0.0f);
							robotState = closed_idle;
						}
					}
					break;

					case opening_narrow:
						if (forkEncoder->Get() != fOpNarrowVal)
							SetForkMotor(-1.0f);
						else
						{
							SetForkMotor(0.0f);
							robotState = opened_narrow_GP;
						}
						break;

					case return_wide:
						if (forkEncoder->Get() != fOpWideVal)
							SetForkMotor(1.0f);
						else
							openedWideState = wide_idle;
						break;

				}
				break;

			case closed_idle:
				closedState = closed_raising;
				switch (closedState)
				{
					case closed_raising:
						if (!(liftEncoder->Get() >= lPosCOneVal))
							SetLiftMotor(1.0f);
						else
						{
							SetLiftMotor(0.0f);
							robotState = closed_C_Pos_One;
						}
						break;

					case releasing:
						break;
				}
				break;

			case closed_C_Pos_One:
				if (movePosTwo)
					robotState = elevating_Pos_Two;
				else if (movePosThree)
					robotState = elevating_Pos_Three;
				else if (movePosStep)
					robotState = elevating_Pos_Step;
				else if (release)
					liftState = release_lowering;
				switch (liftState)
				{
					case release_lowering:
						//run motors to position one on lift and release tote slightly
						liftState = waiting;
						break;

					case waiting:
						if (openNarrow)
							liftState = release_opening_narrow;
						else if (openWide)
							liftState = release_opening_wide;
						break;

					case release_opening_narrow:
						openedNarrowState = opening_wide;
						robotState = opened_narrow_GP;
						break;

					case release_opening_wide:
						openedWideState = opening_narrow;
						robotState = opened_wide_GP;
						break;
				}
				break;

			case closed_C_Pos_Two:
				if (movePosOne)
					robotState = lowering_Pos_One;
				else if (movePosThree)
					robotState = elevating_Pos_Three;
				else if (movePosStep)
					robotState = lowering_Pos_Step;
				else if (release)
					liftState = release_lowering;
				switch (liftState)
				{
					case release_lowering:
						//run motors to position one on lift and release tote slightly
						liftState = waiting;
						break;

					case waiting:
						if (openNarrow)
							liftState = release_opening_narrow;
						else if (openWide)
							liftState = release_opening_wide;
						break;

					case release_opening_narrow:
						openedNarrowState = opening_wide;
						robotState = opened_narrow_GP;
						break;

					case release_opening_wide:
						openedWideState = opening_narrow;
						robotState = opened_wide_GP;
						break;
				}
				break;

			case closed_C_Pos_Three:
				if (movePosOne)
					robotState = lowering_Pos_One;
				else if (movePosTwo)
					robotState = lowering_Pos_Two;
				else if (movePosStep)
					robotState = lowering_Pos_Step;
				else if (release)
					liftState = release_lowering;
				switch (liftState)
				{
					case release_lowering:
						//run motors to position one on lift and release tote slightly
						liftState = waiting;
						break;

					case waiting:
						if (openNarrow)
							liftState = release_opening_narrow;
						else if (openWide)
							liftState = release_opening_wide;
						break;

					case release_opening_narrow:
						openedNarrowState = opening_wide;
						robotState = opened_narrow_GP;
						break;

					case release_opening_wide:
						openedWideState = opening_narrow;
						robotState = opened_wide_GP;
						break;
				}
				break;

			case closed_C_Pos_Step:
				if (movePosOne)
					robotState = lowering_Pos_One;
				else if (movePosTwo)
					robotState = elevating_Pos_Two;
				else if (movePosThree)
					robotState = elevating_Pos_Three;
				else if (release)
					liftState = release_lowering;
				switch (liftState)
				{
					case release_lowering:
						//run motors to position one on lift and release tote slightly
						liftState = waiting;
						break;

					case waiting:
						if (openNarrow)
							liftState = release_opening_narrow;
						else if (openWide)
							liftState = release_opening_wide;
						break;

					case release_opening_narrow:
						openedNarrowState = opening_wide;
						robotState = opened_narrow_GP;
						break;

					case release_opening_wide:
						openedWideState = opening_narrow;
						robotState = opened_wide_GP;
						break;
				}
				break;

			case lowering_Pos_One:
				if (liftEncoder->Get() != lPosCOneVal)
					SetLiftMotor(-1.0f);
				else robotState = closed_C_Pos_One;
				break;

			case elevating_Pos_Two:
				if (liftEncoder->Get() != lPosCTwoVal)
					SetLiftMotor(1.0f);
				else robotState = closed_C_Pos_Two;
				break;

			case lowering_Pos_Two:
				if (liftEncoder->Get() != lPosCTwoVal)
					SetLiftMotor(-1.0f);
				else robotState = closed_C_Pos_Two;
				break;

			case elevating_Pos_Three:
				if (liftEncoder->Get() != lPosCThreeVal)
					SetLiftMotor(1.0f);
				else robotState = closed_C_Pos_Three;
				break;

			case elevating_Pos_Step:
				if (liftEncoder->Get() != lPosCStepVal)
					SetLiftMotor(1.0f);
				robotState = closed_C_Pos_Step;
				break;

			case lowering_Pos_Step:
				if (liftEncoder->Get() != lPosCStepVal)
					SetLiftMotor(-1.0f);
				robotState = closed_C_Pos_Step;
				break;
		}

		if (GetForkLimitSwitchMin())
		{
			if (prevForkState == fork_opened_narrow)
				openedNarrowState = return_narrow;
			else if (prevForkState == fork_opened_wide)
				openedWideState = return_wide;
		}

		if (GetForkLimitSwitchMax())
		{

		}

		if (GetLiftLimitSwitchMin())
		{

		}

		if (GetLiftLimitSwitchMax())
		{

		}
	}
private:
		CANTalon *forkMotor;
		VictorSP *liftMotor;

		Encoder *forkEncoder;
		Encoder *liftEncoder;

		DigitalInput *forkLimitSwitchMin;
		DigitalInput *forkLimitSwitchMax;
		DigitalInput *liftLimitSwitchMin;
		DigitalInput *liftLimitSwitchMax;

		RobotState robotState;
		OpenedNarrowState openedNarrowState;
		OpenedWideState openedWideState;
		ClosedState closedState;
		LiftState liftState;
		PrevForkState prevForkState;

		int fOpNarrowVal;
		int fOpWideVal;
		int currForkPosVal;
		int releaseOffsetVal;

		int lPosGroundOneVal;
		int lPosCOneVal;
		int lPosCTwoVal;
		int lPosCThreeVal;
		int lPosCStepVal;

		bool openNarrow;
		bool openWide;
		bool close;
		bool release;
		bool movePosOne;
		bool movePosTwo;
		bool movePosThree;
		bool movePosStep;

		double forkCurrentSpike;
};
