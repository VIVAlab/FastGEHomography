/* Includes */
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <vector>
#include "rhorefc.h"


/* Namespaces */
using namespace cv;
using namespace std;

/* Main */
int main(int argc, char* argv[]){

    vector<Point2f> src; 	/* should be filled */
    vector<Point2f> dst;	/* should be filled */
    int npoints = 100;

    Mat    tempMask;
    bool   result;
    const double ransacReprojThreshold = 3;
    double beta = 0.35; /* 0.35 is a value that often works. */
    const int maxIters = 2000;
    const double confidence = 0.995;

    /* Create temporary output matrix (RHO outputs a single-precision H only). */
    Mat tmpH = Mat(3, 3, CV_32FC1);

    /* Create output mask. */
    tempMask = Mat(npoints, 1, CV_8U);

#ifdef __SSE2__
	// SSE version here
#else
    /**
     * Make use of the RHO estimator API.
     *
     * This is where the math happens. A homography estimation context is
     * initialized, used, then finalized.
     */

    RHO_HEST_REFC* p = rhoRefCInit();

    /**
     * Optional. Ideally, the context would survive across calls to
     * findHomography(), but no clean way appears to exit to do so. The price
     * to pay is marginally more computational work than strictly needed.
     */

    rhoRefCEnsureCapacity(p, npoints, beta);

    /**
     * The critical call. All parameters are heavily documented in rhorefc.h.
     *
     * Currently, NR (Non-Randomness criterion) and Final Refinement (with
     * internal, optimized Levenberg-Marquardt method) are enabled.
     */

    result = !!rhoRefC(p,
                      (const float*)&src[0],
                      (const float*)&dst[0],
                      (char*)       tempMask.data,
                      (unsigned)    npoints,
                      (float)       ransacReprojThreshold,
                      (unsigned)    maxIters,
                      (unsigned)    maxIters,
                      confidence,
                      4U,
                      beta,
                      RHO_FLAG_ENABLE_NR | RHO_FLAG_ENABLE_FINAL_REFINEMENT,
                      NULL,
                      (float*)tmpH.data);

    /**
     * Cleanup.
     */

    rhoRefCFini(p);
#endif

    /* Convert float homography to double precision. */
    tmpH.convertTo(_H, CV_64FC1);

    /* Maps non-zero mask elems to 1, for the sake of the testcase. */
    for(int k=0;k<npoints;k++){
        tempMask.data[k] = !!tempMask.data[k];
    }
    tempMask.copyTo(_tempMask);

    return result;
}
