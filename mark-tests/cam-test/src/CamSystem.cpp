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
	char myString[64];
// create an image
	//frame = frcCreateImage(IMAQ_IMAGE_RGB);
	sprintf(myString, "In CamSystem Const\n");
	SmartDashboard::PutString("DB/String 0", myString);

// open the camera at the IP address assigned. This is the IP address that the camera
// can be accessed through the web interface.
	camera = new AxisCamera("10.9.80.22");
	sprintf(myString, "camera init\n");
	SmartDashboard::PutString("DB/String 0", myString);
}

CamSystem::~CamSystem() {
	// TODO Auto-generated destructor stub
	free(camera);
}

float CamSystem::GetDistance() {
	return distance;
}

void CamSystem::Scan() {
	int targets[MAX_PARTICLES];
	int targetCount = 0;
	TargetReport target;
	int ret;
	Threshold threshold(0, 255, 50, 255, 70, 255);
	ParticleFilterCriteria2 criteria[] = {{IMAQ_MT_AREA, AREA_MINIMUM, 65535, false, false}};
	vector<ParticleAnalysisReport> *reports;  // vector of particle reports
	char myString[64];
	Image  *inputImage;
	Image  *thresholdImage;
	Image  *particleImage;
	Range redRange, greenRange, blueRange;
	int   numParticles, i;
	ParticleAnalysisReport report;


//			AxisCamera &camera = AxisCamera::GetInstance();	//To use the Axis camera uncomment this line

	sprintf(myString, "starting vision\n");
	SmartDashboard::PutString("DB/String 0", myString);

	ret = camera->GetImage(inputImage);
	if (!ret)
	{
#if DEBUG
		sprintf(myString, "problem getting image\n");
		SmartDashboard::PutString("DB/String 0", myString);
#endif
	}

#if DEBUG
	//inputImage->Write("/home/lvuser/raw.bmp");
#endif

	redRange.minValue = 0;
	redRange.maxValue = 255;
	greenRange.minValue = 50;
	greenRange.maxValue = 255;
	blueRange.minValue = 70;
	blueRange.maxValue = 255;
//	thresholdImage = frcCreateImage	(IMAQ_IMAGE_U8);

	sprintf(myString, "threshold\n");
	SmartDashboard::PutString("DB/String 0", myString);

	ret = frcColorThreshold	(thresholdImage, inputImage,
			255, IMAQ_RGB, &redRange, &greenRange, &blueRange);
	if (!ret)
	{
#if DEBUG
		sprintf(myString, "problem thresholding image\n");
		SmartDashboard::PutString("DB/String 0", myString);
#endif
	}

#if DEBUG
	//thresholdImage->Write("/home/lvuser/threshold.bmp");
#endif
	sprintf(myString, "particles\n");
	SmartDashboard::PutString("DB/String 0", myString);

	ret = frcParticleFilter (particleImage, thresholdImage, criteria, 1,
			0, &numParticles);
	if (!ret)
	{
#if DEBUG
		sprintf(myString, "problem with particle filter\n");
		SmartDashboard::PutString("DB/String 0", myString);
#endif
	}

#if DEBUG
	//particleImage->Write("Filtered.bmp");
#endif

	for (i=0; i<numParticles; i++)
	{
		ret = frcParticleAnalysis(particleImage, i, &report);
		if (!ret)
		{
	#if DEBUG
			sprintf(myString, "prbm w/part.analysis %d\n", i);
			SmartDashboard::PutString("DB/String 0", myString);
	#endif
		}
		sprintf(myString, "%d particles\n", i);
		SmartDashboard::PutString("DB/String 1", myString);



	}

#if 0
	reports = filteredImage->GetOrderedParticleAnalysisReports();
	targetCount = 0;

#if DEBUG
	sprintf(myString, "reports->size = %d", reports->size());
	SmartDashboard::PutString("DB/String 0", myString);
#endif
	if (reports->size() > 0)
	{
		for (unsigned int i = 0; i < MAX_PARTICLES && i < reports->size(); i++)
		{
			scores = new Scores[reports->size()];
			ParticleAnalysisReport *report = &(reports->at(i));
//Score each particle on rectangularity and aspect ratio
			scores[i].rectangularity = scoreRectangularity(report);
			scores[i].aspectRatioVertical = scoreAspectRatio(filteredImage, report, true);
			scores[i].aspectRatioHorizontal = scoreAspectRatio(filteredImage, report, false);

			if (CamSystem::scoreParticle(report))
				targets[targetCount++] = i;
		}
		sprintf(myString, "%d targets id'd\n", targetCount);
		SmartDashboard::PutString("DB/String 0", myString);

		if (targetCount > 0)
		{
			ParticleAnalysisReport *distanceReport = &(reports->at(0));
			distance = CamSystem::computeDistance(filteredImage, distanceReport);
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
#endif
}


#if 0
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

int scoreAspectRatio(BinaryImage *image, ParticleAnalysisReport *report, double *vert, double *horiz) {


	double rectLong, rectShort, idealAspectRatio, aspectRatio;
	idealAspectRatio = vertical ? (4.0/32) : (23.5/4);	//Vertical reflector 4" wide x 32" tall, horizontal 23.5" wide x 4" tall

	imaqMeasureParticle(image->GetImaqImage(), report->particleIndex, 0, IMAQ_MT_EQUIVALENT_RECT_LONG_SIDE, &rectLong);
	imaqMeasureParticle(image->GetImaqImage(), report->particleIndex, 0, IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE, &rectShort);

	//Divide width by height to measure aspect ratio
	if(report->boundingRect.width > report->boundingRect.height) {
		//particle is wider than it is tall, divide long by short
		aspectRatio = ratioToScore(((rectLong/rectShort)/idealAspectRatio));
	} else {
		//particle is taller than it is wide, divide short by long
		aspectRatio = ratioToScore(((rectShort/rectLong)/idealAspectRatio));
	}
	return aspectRatio;		//force to be in range 0-100
}
#endif
