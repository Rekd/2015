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
		closed_C_Pos_three,
		closed_C_Pos_Step,
		released} RobotState;

	typedef enum {narrow_idle,
		narrow_closing,
		opening_wide} OpenedNarrowState;

	typedef enum {wide_idle,
		wide_closing,
		opening_narrow} OpenedWideState;

	typedef enum {closed_raising,
		releasing,
		release_lowering} ClosedState;

	typedef enum {lift_idle,
		lift_moving,
		waiting,
		release_opening_narrow,
		elease_opening_wide} LiftState;

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

					lPosOneVal = ;
					lPosTwoVal = ;
					lPosThreeVal = ;
					lPosFourVal = ;
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

				//Lift Position Values
				int GetLiftPosOneVal()
				{
					if (forkState != closed)
						return lPosOneVal;
					else
						return (lPosOneVal + liftOffsetVal);
				}

				int GetLiftPosTwoVal()
				{
					if (forkState != closed)
						return lPosTwoVal;
					else
						return (lPosTwoVal + liftOffsetVal);
				}

				int GetLiftPosThreeVal()
				{
					if (forkState != closed)
						return lPosThreeVal;
					else
						return (lPosThreeVal + liftOffsetVal);
				}

				int GetLiftPosStepVal()
				{
					if (forkState != closed)
						return lPosStepVal;
					else
						return (lPosStepVal + liftOffsetVal);
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

				}

				void MovePosThree()
				{

				}

				void MovePosStep()
				{

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
									SetForkMotor(0);
									break;

								case narrow_closing:
									prevForkState = fork_opened_narrow;
									SetForkMotor(-1.0);
									if (forkMotor->GetOutputCurrent() >= forkCurrentSpike)
									{
										SetForkMotor(0);
										robotState = closed_raising;
									}
									break;

								case opening_wide:
									prevForkState = fork_opened_narrow;
									SetForkMotor(1.0);
									if (forkEncoder->Get() >= fOpWideVal)
									{
										SetForkMotor(0);
										robotState = opened_wide_GP;
									}
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
									SetForkMotor(0);
									break;

								case wide_closing:
								{
									SetForkMotor(-1.0l);
									if (forkMotor->GetOutputCurrent() >= forkCurrentSpike)
									{
										SetForkMotor(0);
										robotState = closed_raising;
									}
								}
								break;

								case opening_narrow:
									prevForkState = fork_opened_wide;;
									SetForkMotor(-1.0);
									if (forkEncoder->Get() >= fOpNarrowVal)
									{
										SetForkMotor(0);
										robotState = opened_narrow_GP;
									}
									break;

									if (close)
									{
										prevForkState = fork_opened_wide;
										SetForkMotor(-1.0);
										if (forkMotor->GetOutputCurrent() >= forkCurrentSpike)
										{
											SetForkMotor(0);
											closedState = closed_raising;
										}
									}
									break;
							}
							break;

						case closed_idle:
							closedState = closed_raising;
							switch (closedState)
							{
								case closed_raising:
									SetLiftMotor(1.0);
									if (liftEncoder->Get() >= (currPosVal + liftOffsetVal))
									{
										SetLiftMotor(0);
										robotState = closed_C_Pos_One;
									}
									break;

								case closed_C_Pos_One:
									if (movePosTwo)
									{

									}
									else if (movePosThree)
									{

									}
									else if (movePosStep)
									{

									}
									else if (release)
									{

									}
									break;

								case released:
									if (openNarrow)
									{
										SetForkMotor(-1.0l);
										if (forkEncoder->Get() <= fOpNarrowVal)
										{
											SetForkMotor(0);
											forkState = opened_narrow;
										}
									}
									else if (openWide)
									{
										prevForkState = fork_opened_narrow;
										SetForkMotor(1.0l);
										if (forkEncoder->Get() >= fOpWideVal)
										{
											SetForkMotor(0);
											forkState = opened_wide;
										}
									}
									break;
							}
							/*
							 * 							if (release)
								ClosedState = closed_raising;
							{
								SetLiftMotor(-1.0);
								if (liftEncoder->Get() <= (currLiftPosVal - liftOffsetVal))
								{
									SetLiftMotor(0);
									SetForkMotor(1.0);
									if (forkEncoder->Get() >= (currForkPosVal + releaseOffsetVal))
									{
										SetForkMotor(0);
										forkState = released;
									}
								}
							}
							 */
					}

					if (GetForkLimitSwitchMin())
					{
						if (prevForkState == fork_opened_narrow)
							GetOpenNarrowButton();
						else if (prevForkState == fork_opened_wide)
							GetOpenWideButton();
					}

					if (GetForkLimitSwitchMax())
					{
						forkState = opened_wide;
					}

					if (GetMovePositionOneButton())
					{
						switch (liftState)
						{
							case position_one:
								break;

							case position_two:
								SetLiftMotor(-1.0);
								if (liftEncoder->Get() <= GetLiftPosOneVal())
								{
									SetLiftMotor(0);
									liftState = position_one;
								}
								break;

							case position_three:
								SetLiftMotor(-1.0);
								if (liftEncoder->Get() <= GetLiftPosOneVal())
								{
									SetLiftMotor(0);
									liftState = position_one;
								}
								break;

							case position_four:
								SetLiftMotor(-1.0);
								if (liftEncoder->Get <= GetLiftPosOneVal())
								{
									SetLiftMotor(0);
									liftState = position_one;
								}
								break;

						}
					}

					if (GetMovePositionTwoButton())
					{
						switch (liftState)
						{
							case position_one:
								SetLiftMotor(1.0);
								if (liftEncoder->Get() >= GetLiftPosTwoVal())
								{
									SetLiftMotor(0);
									liftState = position_two;
								}
								break;

							case position_two:
								break;

							case position_three:
								SetLiftMotor(-1.0);
								if (liftEncoder->Get() <= GetLiftPosTwoVal())
								{
									SetLiftMotor(0);
									liftState = position_two;
								}
								break;

							case position_four:
								SetLiftMotor(-1.0);
								if (liftEncoder->Get <= GetLiftPosTwoVal())
								{
									SetLiftMotor(0);
									liftState = position_two;
								}
								break;

						}
					}

					if (GetMovePositionThreeButton())
					{
						switch (liftState)
						{
							case position_one:
								SetLiftMotor(1.0);
								if (liftEncoder->Get() >= GetLiftPosThreeVal())
								{
									SetLiftMotor(0);
									liftState = position_three;
								}
								break;

							case position_two:
								SetLiftMotor(1.0);
								if (liftEncoder->Get() >= GetLiftPosThreeVal())
								{
									SetLiftMotor(0);
									liftState = position_three;
								}
								break;

							case position_three:
								break;

							case position_four:
								SetLiftMotor(-1.0);
								liftState = lowering;
								if (liftEncoder->Get() <= GetLiftPosThreeVal())
								{
									SetLiftMotor(0);
									liftState = position_three;
								}
								break;

						}
					}

					if (GetMovePositionFourButton())
					{
						switch (liftState)
						{
							case position_one:
								SetLiftMotor(1.0);
								if (liftEncoder->Get() <= GetLiftPosFourVal())
								{
									SetLiftMotor(0);
									liftState = position_step;
								}
								break;

							case position_two:
								SetLiftMotor(1.0);
								if (liftEncoder->Get() <= GetLiftPosFourVal())
								{
									SetLiftMotor(0);
									liftState = position_step;
								}
								break;

							case position_three:
								SetLiftMotor(1.0);
								if (liftEncoder->Get() <= GetLiftPosFourVal())
								{
									SetLiftMotor(0);
									liftState = position_step;
								}
								break;

							case position_four:
								break;

						}
					}

					switch (liftState)
					{
						case position_one:
							currLiftPosVal = GetLiftPosOneVal();
							break;

						case position_two:
							currLiftPosVal = GetLiftPosTwoVal();
							break;

						case position_three:
							currLiftPosVal = GetLiftPosThreeVal();
							break;

						case position_four:
							currLiftPosVal = GetLiftPosFourVal();
							break;
					}

					if (GetLiftLimitSwitchMin())
						liftState = position_one;

					if (GetLiftLimitSwitchMax())
						liftState = position_step;


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
				ReleasedState releasedState;
				LiftState liftState;
				PrevForkState prevForkState;

				int fOpNarrowVal;
				int fOpWideVal;
				int currForkPosVal;
				int releaseOffsetVal;

				int lPosOneVal;
				int lPosTwoVal;
				int lPosThreeVal;
				int lPosFourVal;
				int currLiftPosVal;
				int liftOffsetVal;

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
