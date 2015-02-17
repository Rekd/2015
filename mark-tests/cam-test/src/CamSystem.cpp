/*
 * CamSystem.cpp
 *
 *  Created on: Feb 14, 2015
 *      Author: mll
 */

#include <CamSystem.h>
#include <Math.h>



CamSystem::CamSystem()
{
// create an image
	frame = imaqCreateImage(IMAQ_IMAGE_RGB, 0);

// open the camera at the IP address assigned. This is the IP address that the camera
// can be accessed through the web interface.
	camera = new AxisCamera("10.9.80.11");

}

CamSystem::~CamSystem() {
	// TODO Auto-generated destructor stub
}

float CamSystem::GetDistance() {
	return distance;
}

void CamSystem::Scan() {
	int targets[MAX_PARTICLES];
	int targetCount = 0;
	Threshold threshold(0, 255, 50, 255, 70, 255);
	ParticleFilterCriteria2 criteria[] = {{IMAQ_MT_AREA, AREA_MINIMUM, 65535, false, false}};
	ColorImage *image;
	vector<ParticleAnalysisReport> *reports;  // vector of particle reports
	char myString[64];


//			AxisCamera &camera = AxisCamera::GetInstance();	//To use the Axis camera uncomment this line

	image = camera.GetImage();

#if DEBUG_SAVE
	image->Write("/raw.bmp");
#endif
	BinaryImage *thresholdImage = image->ThresholdHSV(threshold);
#if DEBUG_SAVE
	thresholdImage->Write("/threshold.bmp");
#endif
	BinaryImage *filteredImage = thresholdImage->ParticleFilter(criteria, 1);
#if DEBUG_SAVE
	filteredImage->Write("Filtered.bmp");
#endif

	reports = filteredImage->GetOrderedParticleAnalysisReports();
	targetCount = 0;

#if DEBUG
	message("reports->size = %d", reports->size());
#endif
	if (reports->size() > 0) {
		for (unsigned int i = 0; i < MAX_PARTICLES && i < reports->size(); i++) {
			ParticleAnalysisReport *report = &(reports->at(i));
//Score each particle on rectangularity and aspect ratio

			if (CamSystem::scoreParticle(report))
				targets[targetCount++] = i;
		}
		sprintf(myString, "&d targets id'd\n", targetCount);
		SmartDashboard::PutString("DB/String 0", myString);

		if (targetCount > 0)
		{
			ParticleAnalysisReport *distanceReport = &(reports->at(0));
			distance = computeDistance(filteredImage, distanceReport);
			sprintf(myString, "Distance: %f in\n", distance);
			SmartDashboard::PutString("DB/String 2", myString);

			sprintf(myString, "Top 0: %d\n", target[0].boundingRect.top);
			SmartDashboard::PutString("DB/String 2", myString);
			sprintf(myString, "Bottom: %d\n", target[0].boundingRect.top+report.boundingRect.height);
			SmartDashboard::PutString("DB/String 3", myString);
			sprintf(myString, "Left: %d\n", target[0].boundingRect.left);
			SmartDashboard::PutString("DB/String 4", myString);
			sprintf(myString, "Right: %d\n", target[0].boundingRect.left+report.boundingRect.width);
			SmartDashboard::PutString("DB/String 5", myString);

			if (targetCount > 1)
			{
				sprintf(myString, "Top 1: %d\n", target[1].boundingRect.top);
				SmartDashboard::PutString("DB/String 6", myString);
				sprintf(myString, "Bottom: %d\n", target[1].boundingRect.top+report.boundingRect.height);
				SmartDashboard::PutString("DB/String 7", myString);
				sprintf(myString, "Left: %d\n", target[1].boundingRect.left);
				SmartDashboard::PutString("DB/String 8", myString);
				sprintf(myString, "Right: %d\n", target[1].boundingRect.left+report.boundingRect.width);
				SmartDashboard::PutString("DB/String 9", myString);
			}
		}
	}
}

double CamSystem::computeDistance (BinaryImage *image, ParticleAnalysisReport *report)
{
	double rectLong, height;
	int targetHeight;

	imaqMeasureParticle(image->GetImaqImage(), report->particleIndex, 0, IMAQ_MT_EQUIVALENT_RECT_LONG_SIDE, &rectLong);
	//using the smaller of the estimated rectangle long side and the bounding rectangle height results in better performance
	//on skewed rectangles
	height = min(report->boundingRect.height, rectLong);
	targetHeight = 32;

	return Y_IMAGE_RES * targetHeight / (height * 12 * 2 * tan(VIEW_ANGLE*PI/(180*2)));
}

boolean CamSystem::scoreParticle(ParticleAnalysisReport *report)
{
	if (report->particleArea > MIN_PARTICLE_AREA)
		return true;
	else
		return false;
}
