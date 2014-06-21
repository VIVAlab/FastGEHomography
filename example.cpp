/* Includes */
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <vector>
#include "SPaRTaRefC.h"
#include "SPaRTaSSE2.h"



/* Namespaces */
using namespace cv;
using namespace std;




/* Main */
int main(int argc, char* argv[]){
	vector<Point2f> modelPoints;
	vector<Point2f> rtPoints;
	Mat             homography;
	float           H[3*4];
	Mat             tmpHomog = Mat(3, 3, CV_32FC1, H, 4*sizeof(float));
	
	
	
	
	
	/**
	 * Fill modelPoints vector with points from model, and at the corresponding index in the
	 * runtime points vector add the matching point detected at runtime.
	 */
	
	/* .... */
	
	float*          mdPtr              = (float*)&modelPoints[0];
	float*          rtPtr              = (float*)&rtPoints[0];
	char*           inl                = NULL;
	float*          extrinsicGuess     = NULL;
	int             wantInlierList     = !inl;
	int             haveExtrinsicGuess = !extrinsicGuess;
	unsigned        N                  = rtPoints.size();
	float           maxReprojDist      = 3.0;
	unsigned        maxIter            = 2000;
	unsigned        prosacConvergence  = maxIter;
	float           confidence         = 0.995;
	unsigned        minNumInliers      = N*0.1;
	double          beta               = 0.35;
	unsigned        flags              = HEST_FLAG_ENABLE_NR;
	unsigned        initialCapacity    = 1000;
	unsigned        numInliers;
	int             homographySuccess;
	
	
	
	
	/**
	 * Run homography. Can be called from C code.
	 */
	
#ifdef __SSE2__
	PROSAC_HEST_SSE2 p;
	hestSSE2Init(&p);/* Called once only for the lifetime of p */
	if(flags & HEST_FLAG_ENABLE_NR){
		hestSSE2EnsureCapacity(&p, initialCapacity, beta);
	}
	/* START TIMING CODE GOES HERE.... */
	numInliers = hestSSE2(&p,
	                      mdPtr,
	                      rtPtr,
	                      wantInlierList     ? inl            : NULL,
	                      N,
	                      maxReprojDist,
	                      maxIter,
	                      prosacConvergence,
	                      confidence,
	                      minNumInliers,
	                      beta,
	                      flags,
	                      haveExtrinsicGuess ? extrinsicGuess : NULL,
	                      H);
	/* ....END TIMING CODE GOES HERE */
	hestSSE2Fini(&p);/* Called once only for the lifetime of p */
#else
	PROSAC_HEST_REFC p;
	hestRefCInit(&p);/* Called once only for the lifetime of p */
	if(flags & HEST_FLAG_ENABLE_NR){
		hestRefCEnsureCapacity(&p, initialCapacity, beta);
	}
	/* START TIMING CODE GOES HERE.... */
	numInliers = hestRefC(&p,
	                      mdPtr,
	                      rtPtr,
	                      wantInlierList     ? inl            : NULL,
	                      N,
	                      maxReprojDist,
	                      maxIter,
	                      prosacConvergence,
	                      confidence,
	                      minNumInliers,
	                      beta,
	                      flags,
	                      haveExtrinsicGuess ? extrinsicGuess : NULL,
	                      H);
	/* ....END TIMING CODE GOES HERE */
	hestRefCFini(&p);/* Called once only for the lifetime of p */
#endif
	
	/**
	 * Make a cv::Mat clone, because tmpHomog currently manages memory on the stack.
	 */
	
	homography        = tmpHomog.clone();
	homographySuccess = numInliers > 0;
	
	/* ......... */
	
	
}