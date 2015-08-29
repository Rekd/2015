/*
 * CamSystem.h
 *
 *  Created on: Feb 14, 2015
 *      Author: mll
 */

#ifndef SRC_CAMSYSTEM_H_
#define SRC_CAMSYSTEM_H_
#include "WPILib.h"
#include "constants.h"
#include "Vision/BaeUtilities.h"
#include "Vision/FrcError.h"
#include "Vision/VisionAPI.h"

#define DEBUG 1

using namespace std;
#define MIN_PARTICLE_AREA 200.0
// 1/2 of the field of view of the Axis M1013 camera
// from spec sheet - other calibrated #define VIEW_ANGLE 33.5
//#define VIEW_ANGLE 49.0
#define VIEW_ANGLE 59.0
// The width of a tote
#define TARGET_WIDTH 16.75
#define FOV_PIX_WIDTH 640
#define FOV_PIX_HEIGHT 480


class CamSystem {
public:
	CamSystem();
	virtual ~CamSystem();

	float GetDistance();
	void Scan();
private:
	float distance;
	IMAQdxSession session;
	IMAQdxError imaqError;
	AxisCamera *camera;
	double   twoTanTheta;  // pre-calculated tan(theta) value for this camera
	Image    *inputImage;
	Image    *thresholdImage;
	Image    *particleImage;


	struct TargetReport {
		int verticalIndex;
		int horizontalIndex;
		bool Hot;
		double totalScore;
		double leftScore;
		double rightScore;
		double tapeWidthScore;
		double verticalScore;
	};

	typedef struct {
		int rectangularity;
		float aspectRatioVertical;
		float aspectRatioHorizontal;
	} Scores;


	bool scoreParticle(ParticleAnalysisReport *report);
	double computeDistance (ParticleAnalysisReport *report);

};

#endif /* SRC_CAMSYSTEM_H_ */
