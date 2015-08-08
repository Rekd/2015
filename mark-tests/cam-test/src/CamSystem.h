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

class CamSystem {
public:
	CamSystem();
	virtual ~CamSystem();

	float GetDistance();
	void Scan();
private:
	float distance;
	IMAQdxSession session;
	Image *frame;
	IMAQdxError imaqError;
	AxisCamera *camera;

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
	double computeDistance (BinaryImage *image, ParticleAnalysisReport *report);

};

#endif /* SRC_CAMSYSTEM_H_ */
