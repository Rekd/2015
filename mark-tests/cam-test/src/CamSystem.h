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

	bool scoreParticle(ParticleAnalysisReport *report);

};

#endif /* SRC_CAMSYSTEM_H_ */
