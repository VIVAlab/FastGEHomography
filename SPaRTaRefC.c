/* Includes */
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stddef.h>
#include <limits.h>
#include <math.h>
#include "SPaRTaRefC.h"



/* Defines */
#define MEM_ALIGN               32
#define HSIZE                   (3*4*sizeof(float))
#define MIN_DELTA_CHNG          0.1
#define REL_CHNG(a, b)          (fabs((a) - (b))/(a))
#define CHNG_SIGNIFICANT(a, b)  (REL_CHNG(a, b) > MIN_DELTA_CHNG)
#define CHI_STAT                2.706
#define CHI_SQ                  1.645



/* Data Structures */



/* Prototypes */
static inline void*  almalloc(size_t nBytes);
static inline void   alfree(void* ptr);

static inline int    sacInitRun(PROSAC_HEST_REFC* restrict p,
                                const float* restrict      src,
                                const float* restrict      dst,
                                char* restrict             bestInl,
                                unsigned                   N,
                                float                      maxD,
                                unsigned                   maxI,
                                unsigned                   rConvg,
                                double                     cfd,
                                unsigned                   minInl,
                                double                     beta,
                                unsigned                   flags,
                                const float*               guessH,
                                float*                     finalH);
static inline void   sacFiniRun(PROSAC_HEST_REFC* p);
static inline int    sacIsNREnabled(PROSAC_HEST_REFC* p);
static inline int    sacIsRefineEnabled(PROSAC_HEST_REFC* p);
static inline int    sacIsFinalRefineEnabled(PROSAC_HEST_REFC* p);
static inline int    sacPhaseEndReached(PROSAC_HEST_REFC* p);
static inline void   sacGoToNextPhase(PROSAC_HEST_REFC* p);
static inline void   sacGetPROSACSample(PROSAC_HEST_REFC* p);
static inline int    sacIsSampleDegenerate(PROSAC_HEST_REFC* p);
static inline void   sacGenerateModel(PROSAC_HEST_REFC* p);
static inline int    sacIsModelDegenerate(PROSAC_HEST_REFC* p);
static inline void   sacEvaluateModelSPRT(PROSAC_HEST_REFC* p);
static inline void   sacUpdateSPRT(PROSAC_HEST_REFC* p);
static inline void   sacDesignSPRTTest(PROSAC_HEST_REFC* p);
static inline int    sacIsBestModel(PROSAC_HEST_REFC* p);
static inline int    sacIsBestModelGoodEnough(PROSAC_HEST_REFC* p);
static inline void   sacSaveBestModel(PROSAC_HEST_REFC* p);
static inline void   sacInitNonRand(double    beta,
                                    unsigned  start,
                                    unsigned  N,
                                    unsigned* nonRandMinInl);
static inline void   sacNStarOptimize(PROSAC_HEST_REFC* p);
static inline void   sacUpdateBounds(PROSAC_HEST_REFC* p);
static inline void   sacOutputModel(PROSAC_HEST_REFC* p);

static inline double sacInitPEndFpI(const unsigned ransacConvg,
                                    const unsigned n,
                                    const unsigned m);
static inline void   sacRndSmpl(unsigned  sampleSize,
                                unsigned* currentSample,
                                unsigned  dataSetSize);
static inline double sacRandom(void);
static inline unsigned sacCalcIterBound(double   confidence,
                                        double   inlierRate,
                                        unsigned sampleSize,
                                        unsigned maxIterBound);
static void          hFuncRefC(float* packedPoints, float* H);



/* Functions */

/**
 * Initialize the estimator context, by allocating the aligned buffers
 * internally needed.
 * 
 * @param [in/out] p  The uninitialized estimator context to initialize.
 * @return 0 if successful; non-zero if an error occured.
 */

int  hestRefCInit(PROSAC_HEST_REFC* p){
	p->smpl       = almalloc(4*sizeof(*p->smpl));
	p->H          = almalloc(HSIZE);
	p->bestH      = almalloc(HSIZE);
	p->pkdPts     = almalloc(4*2*2*sizeof(*p->pkdPts));
	p->nrTBL      = NULL;
	p->nrSize     = 0;
	p->nrBeta     = 0.0;
	
	int ret = p->smpl   &&
	          p->H      &&
	          p->bestH  &&
	          p->pkdPts;
	
	if(!ret){
		hestRefCFini(p);
	}
	
	return ret;
}


/**
 * Ensure that the estimator context's internal table for non-randomness
 * criterion is at least of the given size, and uses the given beta. The table
 * should be larger than the maximum number of matches fed into the estimator.
 * 
 * A value of N of 0 requests deallocation of the table.
 * 
 * @param [in] p     The initialized estimator context
 * @param [in] N     If 0, deallocate internal table. If > 0, ensure that the
 *                   internal table is of at least this size, reallocating if
 *                   necessary.
 * @param [in] beta  The beta-factor to use within the table.
 * @return 1 if successful; 0 if an error occured.
 */

int  hestRefCEnsureCapacity(PROSAC_HEST_REFC* p, unsigned N, double beta){
	unsigned* tmp;
	
	
	if(N == 0){
		/* Deallocate table */
		alfree(p->nrTBL);
		p->nrTBL  = NULL;
		p->nrSize = 0;
	}else{
		/* Ensure table at least as big as N and made for correct beta. */
		if(p->nrTBL && p->nrBeta == beta && p->nrSize >= N){
			/* Table already correctly set up */
		}else{
			if(p->nrSize < N){
				/* Reallocate table because it is too small. */
				tmp = almalloc(N*sizeof(unsigned));
				if(!tmp){
					return 0;
				}
				
				/* Must recalculate in whole or part. */
				if(p->nrBeta != beta){
					/* Beta changed; recalculate in whole. */
					sacInitNonRand(beta, 0, N, tmp);
					alfree(p->nrTBL);
				}else{
					/* Beta did not change; Copy over any work already done. */
					memcpy(tmp, p->nrTBL, p->nrSize*sizeof(unsigned));
					sacInitNonRand(beta, p->nrSize, N, tmp);
					alfree(p->nrTBL);
				}
				
				p->nrTBL  = tmp;
				p->nrSize = N;
				p->nrBeta = beta;
			}else{
				/* Might recalculate in whole, or not at all. */
				if(p->nrBeta != beta){
					/* Beta changed; recalculate in whole. */
					sacInitNonRand(beta, 0, p->nrSize, p->nrTBL);
					p->nrBeta = beta;
				}else{
					/* Beta did not change; Table was already big enough. Do nothing. */
					/* Besides, this is unreachable. */
				}
			}
		}
	}
	
	return 1;
}


/**
 * Finalize the estimator context, by freeing the aligned buffers used
 * internally.
 * 
 * @param [in] p  The initialized estimator context to finalize.
 */

void hestRefCFini(PROSAC_HEST_REFC* p){
	alfree(p->smpl);
	alfree(p->H);
	alfree(p->bestH);
	alfree(p->pkdPts);
	alfree(p->nrTBL);
}


/**
 * Estimates the homography using the given context, matches and parameters to
 * PROSAC.
 * 
 * @param [in/out] p       The context to use for homography estimation. Must
 *                             be already initialized. Cannot be NULL.
 * @param [in]     src     The pointer to the source points of the matches.
 *                             Must be aligned to 4 bytes. Cannot be NULL.
 * @param [in]     dst     The pointer to the destination points of the matches.
 *                             Must be aligned to 16 bytes. Cannot be NULL.
 * @param [out]    inl     The pointer to the output mask of inlier matches.
 *                             Must be aligned to 16 bytes. May be NULL.
 * @param [in]     N       The number of matches.
 * @param [in]     maxD    The maximum distance.
 * @param [in]     maxI    The maximum number of PROSAC iterations.
 * @param [in]     rConvg  The RANSAC convergence parameter.
 * @param [in]     cfd     The required confidence in the solution.
 * @param [in]     minInl  The minimum required number of inliers.
 * @param [in]     beta    The beta-parameter for the non-randomness criterion.
 * @param [in]     flags   A union of flags to control the estimation.
 * @param [in]     guessH  An extrinsic guess at the solution H, or NULL if
 *                         none provided.
 * @param [out]    finalH  The final estimation of H, or the zero matrix if
 *                         the minimum number of inliers was not met.
 *                         Cannot be NULL.
 * @return                 The number of inliers if the minimum number of
 *                         inliers for acceptance was reached; 0 otherwise.
 */

unsigned hestRefC(PROSAC_HEST_REFC* restrict p,       /* Homography estimation context. */
                  const float* restrict      src,     /* Source points */
                  const float* restrict      dst,     /* Destination points */
                  char* restrict             bestInl, /* Inlier mask */
                  unsigned                   N,       /*  = src.length = dst.length = inl.length */
                  float                      maxD,    /*   3.0 */
                  unsigned                   maxI,    /*  2000 */
                  unsigned                   rConvg,  /*  2000 */
                  double                     cfd,     /* 0.995 */
                  unsigned                   minInl,  /*     4 */
                  double                     beta,    /*  0.35 */
                  unsigned                   flags,   /*     0 */
                  const float*               guessH,  /* Extrinsic guess, NULL if none provided */
                  float*                     finalH){ /* Final result. */
	
	/**
	 * Setup
	 */
	
	if(!sacInitRun(p, src, dst, bestInl, N, maxD, maxI, rConvg, cfd, minInl, beta, flags, guessH, finalH)){
		sacFiniRun(p);
		return 0;
	}
	
	
	/**
	 * PROSAC Loop
	 */
	
	for(p->i=0; p->i < p->maxI; p->i++){
		if(sacPhaseEndReached(p)){
			sacGoToNextPhase(p);
		}
		
		sacGetPROSACSample(p);
		if(sacIsSampleDegenerate(p)){
			continue;
		}
		
		sacGenerateModel(p);
		if(sacIsModelDegenerate(p)){
			continue;
		}
		
		sacEvaluateModelSPRT(p);
		sacUpdateSPRT(p);
		if(sacIsBestModel(p)){
			if(sacIsRefineEnabled(p)){
				/* sacRefine(p) */
			}
			
			sacSaveBestModel(p);
			sacUpdateBounds(p);
			
			if(sacIsNREnabled(p)){
				sacNStarOptimize(p);
			}
		}
	}
	
	
	/**
	 * Teardown
	 */
	
	if(sacIsFinalRefineEnabled(p)){
		/* sacRefineFinal(p) */
	}
	
	sacOutputModel(p);
	sacFiniRun(p);
	return sacIsBestModelGoodEnough(p) ? p->bestNumInl : 0;
}


/**
 * Allocate memory aligned to a boundary of MEMALIGN.
 */

static inline void*  almalloc(size_t nBytes){
	if(nBytes){
		unsigned char* ptr = malloc(MEM_ALIGN + nBytes);
		if(ptr){
			unsigned char* adj = (unsigned char*)(((intptr_t)(ptr+MEM_ALIGN))&((intptr_t)(-MEM_ALIGN)));
			ptrdiff_t diff = adj - ptr;
			adj[-1] = diff - 1;
			return adj;
		}
	}
	
	return NULL;
}

/**
 * Free aligned memory
 */

static inline void   alfree(void* ptr){
	if(ptr){
		unsigned char* cptr = ptr;
		free(cptr - (ptrdiff_t)cptr[-1] - 1);
	}
}


/**
 * Initialize SAC for a run.
 * 
 * Passed all the arguments of hest.
 */

static inline int    sacInitRun(PROSAC_HEST_REFC* restrict p,
                                const float* restrict      src,
                                const float* restrict      dst,
                                char* restrict             bestInl,
                                unsigned                   N,
                                float                      maxD,
                                unsigned                   maxI,
                                unsigned                   rConvg,
                                double                     cfd,
                                unsigned                   minInl,
                                double                     beta,
                                unsigned                   flags,
                                const float*               guessH,
                                float*                     finalH){
	p->src          = src;
	p->dst          = dst;
	p->allocBestInl = !bestInl;
	p->bestInl      = bestInl ? bestInl : almalloc(N);
	p->inl          = almalloc(N);
	p->N            = N;
	p->maxD         = maxD;
	p->maxI         = maxI;
	p->rConvg       = rConvg;
	p->cfd          = cfd;
	p->minInl       = minInl < 4 ? 4 : minInl;
	p->beta         = beta;
	p->flags        = flags;
	p->guessH       = guessH;
	p->finalH       = finalH;
	
	if(p->guessH){
		memcpy(p->H, p->guessH, HSIZE);
	}
	memset(p->bestH,  0, HSIZE);
	memset(p->finalH, 0, HSIZE);
	
	if(!p->inl){/* Malloc failure */
		return 0;
	}
	if(sacIsNREnabled(p) && !hestRefCEnsureCapacity(p, N, beta)){
		return 0;
	}
	
	p->phNum      = 4;
	p->phEndI     = 1;
	p->phEndFpI   = sacInitPEndFpI(p->rConvg, p->N, 4);
	p->phMax      = p->N;
	p->phNumInl   = 0;
	p->bestNumInl = 0;
	p->numInl     = 0;
	p->numModels  = 0;
	p->Ntested    = 0;
	p->Ntestedtotal = 0;
	p->good       = 1;
	p->t_M        = 25;
	p->m_S        = 1;
	p->epsilon    = 0.1;
	p->delta      = 0.01;
	sacDesignSPRTTest(p);
	
	return 1;
}

/**
 * Finalize SAC run.
 * 
 * @param p
 */

static inline void   sacFiniRun(PROSAC_HEST_REFC* p){
	if(p->allocBestInl){
		alfree(p->bestInl);
	}
	alfree(p->inl);
}

/**
 * Check whether non-randomness criterion is enabled.
 * 
 * @param p
 * @return Zero if disabled; non-zero if not.
 */

static inline int    sacIsNREnabled(PROSAC_HEST_REFC* p){
	return p->flags & HEST_FLAG_ENABLE_NR;
}

/**
 * Check whether best-model-so-far refinement is enabled.
 * 
 * @param p
 * @return Zero if disabled; non-zero if not.
 */

static inline int    sacIsRefineEnabled(PROSAC_HEST_REFC* p){
	return p->flags & HEST_FLAG_ENABLE_REFINEMENT;
}

/**
 * Check whether final-model refinement is enabled.
 * 
 * @param p
 * @return Zero if disabled; non-zero if not.
 */

static inline int    sacIsFinalRefineEnabled(PROSAC_HEST_REFC* p){
	return p->flags & HEST_FLAG_ENABLE_FINAL_REFINEMENT;
}

/**
 * @brief sacPhaseEndReached
 * @param p
 * @return 
 */

static inline int    sacPhaseEndReached(PROSAC_HEST_REFC* p){
	return p->i >= p->phEndI && p->phNum < p->phMax;
}

/**
 * @brief sacGoToNextPhase
 * @param p
 */

static inline void   sacGoToNextPhase(PROSAC_HEST_REFC* p){
	double next;
	unsigned  m = 4;
	
	p->phNum++;
	next        = (p->phEndFpI * p->phNum)/(p->phNum - m);
	p->phEndI  += ceil(next - p->phEndFpI);
	p->phEndFpI = next;
}

/**
 * @brief sacGetPROSACSample
 * @param p
 */

static inline void   sacGetPROSACSample(PROSAC_HEST_REFC* p){
	if(p->i > p->phEndI){
		sacRndSmpl(4, p->smpl, p->phNum);/* Used to be phMax */
	}else{
		sacRndSmpl(3, p->smpl, p->phNum-1);
		p->smpl[3] = p->phNum-1;
	}
}

/**
 * @brief sacIsSampleDegenerate
 * @param p
 * @return 
 */

static inline int    sacIsSampleDegenerate(PROSAC_HEST_REFC* p){
	unsigned i0 = p->smpl[0], i1 = p->smpl[1], i2 = p->smpl[2], i3 = p->smpl[3];
	struct{float x,y;}* pkdPts = (void*)p->pkdPts, *src = (void*)p->src, *dst = (void*)p->dst;
	
	/**
	 * Pack the matches selected by the SAC algorithm.
	 * Must be packed  points[0:7]  = {srcx0, srcy0, srcx1, srcy1, srcx2, srcy2, srcx3, srcy3}
	 *                 points[8:15] = {dstx0, dsty0, dstx1, dsty1, dstx2, dsty2, dstx3, dsty3}
	 * Gather 4 points into the vector
	 */
	
	pkdPts[0] = src[i0];
	pkdPts[1] = src[i1];
	pkdPts[2] = src[i2];
	pkdPts[3] = src[i3];
	pkdPts[4] = dst[i0];
	pkdPts[5] = dst[i1];
	pkdPts[6] = dst[i2];
	pkdPts[7] = dst[i3];
	
	/**
	 * If the matches' source points have common x and y coordinates, abort.
	 */
	
	if(pkdPts[0].x == pkdPts[1].x || pkdPts[1].x == pkdPts[2].x ||
	   pkdPts[2].x == pkdPts[3].x || pkdPts[0].x == pkdPts[2].x ||
	   pkdPts[1].x == pkdPts[3].x || pkdPts[0].x == pkdPts[3].x ||
	   pkdPts[0].y == pkdPts[1].y || pkdPts[1].y == pkdPts[2].y ||
	   pkdPts[2].y == pkdPts[3].y || pkdPts[0].y == pkdPts[2].y ||
	   pkdPts[1].y == pkdPts[3].y || pkdPts[0].y == pkdPts[3].y){
		return 1;
	}
	
	/* If the matches do not satisfy the strong geometric constraint, abort. */
	/* (0 x 1) * 2 */
	float cross0s0 = pkdPts[0].y-pkdPts[1].y;
	float cross0s1 = pkdPts[1].x-pkdPts[0].x;
	float cross0s2 = pkdPts[0].x*pkdPts[1].y-pkdPts[0].y*pkdPts[1].x;
	float dots0    = cross0s0*pkdPts[2].x + cross0s1*pkdPts[2].y + cross0s2;
	float cross0d0 = pkdPts[4].y-pkdPts[5].y;
	float cross0d1 = pkdPts[5].x-pkdPts[4].x;
	float cross0d2 = pkdPts[4].x*pkdPts[5].y-pkdPts[4].y*pkdPts[5].x;
	float dotd0    = cross0d0*pkdPts[6].x + cross0d1*pkdPts[6].y + cross0d2;
	if(((int)dots0^(int)dotd0) < 0){
		return 1;
	}
	/* (0 x 1) * 3 */
	float cross1s0 = cross0s0;
	float cross1s1 = cross0s1;
	float cross1s2 = cross0s2;
	float dots1    = cross1s0*pkdPts[3].x + cross1s1*pkdPts[3].y + cross1s2;
	float cross1d0 = cross0d0;
	float cross1d1 = cross0d1;
	float cross1d2 = cross0d2;
	float dotd1    = cross1d0*pkdPts[7].x + cross1d1*pkdPts[7].y + cross1d2;
	if(((int)dots1^(int)dotd1) < 0){
		return 1;
	}
	/* (2 x 3) * 0 */
	float cross2s0 = pkdPts[2].y-pkdPts[3].y;
	float cross2s1 = pkdPts[3].x-pkdPts[2].x;
	float cross2s2 = pkdPts[2].x*pkdPts[3].y-pkdPts[2].y*pkdPts[3].x;
	float dots2    = cross2s0*pkdPts[0].x + cross2s1*pkdPts[0].y + cross2s2;
	float cross2d0 = pkdPts[6].y-pkdPts[7].y;
	float cross2d1 = pkdPts[7].x-pkdPts[6].x;
	float cross2d2 = pkdPts[6].x*pkdPts[7].y-pkdPts[6].y*pkdPts[7].x;
	float dotd2    = cross2d0*pkdPts[4].x + cross2d1*pkdPts[4].y + cross2d2;
	if(((int)dots2^(int)dotd2) < 0){
		return 1;
	}
	/* (2 x 3) * 1 */
	float cross3s0 = cross2s0;
	float cross3s1 = cross2s1;
	float cross3s2 = cross2s2;
	float dots3    = cross3s0*pkdPts[1].x + cross3s1*pkdPts[1].y + cross3s2;
	float cross3d0 = cross2d0;
	float cross3d1 = cross2d1;
	float cross3d2 = cross2d2;
	float dotd3    = cross3d0*pkdPts[5].x + cross3d1*pkdPts[5].y + cross3d2;
	if(((int)dots3^(int)dotd3) < 0){
		return 1;
	}
	
	/* Otherwise, accept */
	return 0;
}

/**
 * Compute homography of matches in p->pkdPts with hFuncRefC and store in p->H.
 * 
 * @param p
 */

static inline void   sacGenerateModel(PROSAC_HEST_REFC* p){
	hFuncRefC(p->pkdPts, p->H);
}

/**
 * @brief sacIsModelDegenerate
 * @param p
 * @return 
 */

static inline int    sacIsModelDegenerate(PROSAC_HEST_REFC* p){
	int degenerate;
	float* H = p->H;
	float f=H[0]+H[1]+H[2]+H[4]+H[5]+H[6]+H[8]+H[9];
	
	/* degenerate = isnan(f); */
	degenerate = f!=f;/* Only NaN is not equal to itself. */
	/* degenerate = 0; */
	
	
	return degenerate;
}

/**
 * @brief sacEvaluateModelSPRT
 * @param p
 */

static inline void   sacEvaluateModelSPRT(PROSAC_HEST_REFC* p){
	unsigned i;
	unsigned isInlier;
	double   lambda = 1.0;
	float    distSq = p->maxD*p->maxD;
	const float* src = p->src;
	const float* dst = p->dst;
	char*    inl = p->inl;
	float*   H   = p->H;
	
	
	p->numModels++;
	
	p->numInl   = 0;
	p->Ntested  = 0;
	p->good     = 1;
	
	
	/* SCALAR */
	for(i=0;i<p->N && p->good;i++){
		/* Backproject */
		float x=src[i*2],y=src[i*2+1];
		float X=dst[i*2],Y=dst[i*2+1];
		
		float reprojX=H[0]*x+H[1]*y+H[2]; /*  ( X_1 )     ( H_11 H_12    H_13  ) (x_1)       */
		float reprojY=H[4]*x+H[5]*y+H[6]; /*  ( X_2 )  =  ( H_21 H_22    H_23  ) (x_2)       */
		float reprojZ=H[8]*x+H[9]*y+H[10];/*  ( X_3 )     ( H_31 H_32 H_33=1.0 ) (x_3 = 1.0) */
		
		/* reproj is in homogeneous coordinates. To bring back to "regular" coordinates, divide by Z. */
		reprojX/=reprojZ;
		reprojY/=reprojZ;
		
		/* Compute distance */
		reprojX-=X;
		reprojY-=Y;
		reprojX*=reprojX;
		reprojY*=reprojY;
		float reprojDist = reprojX+reprojY;
		
		/* ... */
		isInlier   = reprojDist <= distSq;
		p->numInl += isInlier;
		*inl++     = isInlier;
		
		
		/* SPRT */
		lambda *= isInlier ? p->lambdaAccept : p->lambdaReject;
		p->good = lambda <= p->A;
		/* If !p->good, the threshold A was exceeded, so we're rejecting */
	}
	
	
	p->Ntested       = i;
	p->Ntestedtotal += i;
}

/**
 * Update either the delta or epsilon SPRT parameters, depending on the events
 * that transpired in the previous evaluation.
 * 
 * If a "good" model that is also the best was encountered, update epsilon,
 * since
 */

static inline void   sacUpdateSPRT(PROSAC_HEST_REFC* p){
	if(p->good){
		if(sacIsBestModel(p)){
			p->epsilon = (double)p->numInl/p->N;
			sacDesignSPRTTest(p);
		}
	}else{
		double newDelta = (double)p->numInl/p->Ntested;
		
		if(newDelta > 0 && CHNG_SIGNIFICANT(p->delta, newDelta)){
			p->delta = newDelta;
			sacDesignSPRTTest(p);
		}
	}
}

/**
 * Numerically compute threshold A from the estimated delta, epsilon, t_M and
 * m_S values.
 * 
 * Epsilon:  Denotes the probability that a randomly chosen data point is
 *           consistent with a good model.
 * Delta:    Denotes the probability that a randomly chosen data point is
 *           consistent with a bad model.
 * t_M:      Time needed to instantiate a model hypotheses given a sample.
 *           (Computing model parameters from a sample takes the same time
 *            as verification of t_M data points)
 * m_S:      The number of models that are verified per sample.
 */

static inline double designSPRTTest(double delta, double epsilon, double t_M, double m_S){
	double An, C, K, prevAn;
	unsigned i;
	
	/**
	 * Randomized RANSAC with Sequential Probability Ratio Test, ICCV 2005
	 * Eq (2)
	 */
	
	C = (1-delta)  *  log((1-delta)/(1-epsilon)) +
	    delta      *  log(  delta  /  epsilon  );
	
	/**
	 * Randomized RANSAC with Sequential Probability Ratio Test, ICCV 2005
	 * Eq (6)
	 * K = K_1/K_2 + 1 = (t_M*C)/m_S + 1
	 */
	
	K = t_M*C/m_S + 1;
	
	/**
	 * Randomized RANSAC with Sequential Probability Ratio Test, ICCV 2005
	 * Paragraph below Eq (6)
	 * 
	 * A* = lim_{n -> infty} A_n, where
	 *     A_0     = K1/K2 + 1             and
	 *     A_{n+1} = K1/K2 + 1 + log(A_n)
	 * The series converges fast, typically within four iterations.
	 */
	
	An = K;
	i  = 0;
	
	do{
		prevAn = An;
		An = K + log(An);
	}while((An-prevAn > 1.5e-8)  &&  (++i < 10));
	
	/**
	 * Return A = An_stopping, with n_stopping < 10
	 */
	
	return An;
}

/**
 * Design the SPRT test. Shorthand for
 *     A = sprt(delta, epsilon, t_M, m_S);
 * 
 * Sets p->A, p->lambdaAccept and p->lambdaReject
 */

static inline void   sacDesignSPRTTest(PROSAC_HEST_REFC* p){
	p->A = designSPRTTest(p->delta, p->epsilon, p->t_M, p->m_S);
	p->lambdaReject = ((1.0 - p->delta) / (1.0 - p->epsilon));
	p->lambdaAccept = ((   p->delta   ) / (    p->epsilon  ));
}

/**
 * Return whether the current model is the best model so far.
 */

static inline int    sacIsBestModel(PROSAC_HEST_REFC* p){
	return p->numInl > p->bestNumInl;
}

/**
 * Returns whether the current-best model is good enough to be an
 * acceptable best model, by checking whether it meets the minimum
 * number of inliers.
 */

static inline int    sacIsBestModelGoodEnough(PROSAC_HEST_REFC* p){
	return p->bestNumInl >= p->minInl;
}

/**
 * 
 */

static inline void   sacSaveBestModel(PROSAC_HEST_REFC* p){
	p->bestNumInl = p->numInl;
	memcpy(p->bestH,    p->H,  HSIZE);
	memcpy(p->bestInl, p->inl, p->N);
}

/**
 * 
 */

static inline void   sacInitNonRand(double    beta,
                                    unsigned  start,
                                    unsigned  N,
                                    unsigned* nonRandMinInl){
	unsigned m = 4;
	unsigned n = m+1 > start ? m+1 : start;
	double   beta_beta1_sq_chi = sqrt(beta*(1.0-beta)) * CHI_SQ;
	
	for(; n < N; n++){
		double   mu      = n * beta;
		double   sigma   = sqrt(n)* beta_beta1_sq_chi;
		unsigned i_min   = ceil(m + mu + sigma);
		
		nonRandMinInl[n] = i_min;
	}
}

/**
 * 
 */

static inline void   sacNStarOptimize(PROSAC_HEST_REFC* p){
	unsigned min_sample_length = 10*2; /*(p->N * INLIERS_RATIO) */
	unsigned best_n       = p->N;
	unsigned test_n       = best_n;
	unsigned bestNumInl   = p->bestNumInl;
	unsigned testNumInl   = bestNumInl;
	
	for(;test_n > min_sample_length && testNumInl;test_n--){
		if(testNumInl*best_n > bestNumInl*test_n){
			if(testNumInl < p->nrTBL[test_n]){
				break;
			}
			best_n      = test_n;
			bestNumInl  = testNumInl;
		}
		testNumInl -= !!p->bestInl[test_n-1];
	}
	
	if(bestNumInl*p->phMax > p->phNumInl*best_n){
		p->phMax    = best_n;
		p->phNumInl = bestNumInl;
		p->maxI     = sacCalcIterBound(p->cfd,
		                               (double)p->phNumInl/p->phMax,
		                               4,
		                               p->maxI);
	}
}

/**
 * 
 */

static inline void   sacUpdateBounds(PROSAC_HEST_REFC* p){
	p->maxI = sacCalcIterBound(p->cfd,
	                           (double)p->bestNumInl/p->N,
	                           4,
	                           p->maxI);
}

/**
 * @brief sacOutputModel
 * @param p
 */

static inline void   sacOutputModel(PROSAC_HEST_REFC* p){
	if(!sacIsBestModelGoodEnough(p)){
		memset(p->bestH, 0, HSIZE);
	}
	
	if(p->finalH){
		memcpy(p->finalH, p->bestH, HSIZE);
	}
}

/**
 * Compute the real-valued number of samples per phase, given the RANSAC convergence speed,
 * data set size and sample size.
 */

static inline double sacInitPEndFpI(const unsigned ransacConvg,
                                    const unsigned n,
                                    const unsigned m){
	double numer=1, denom=1;
	
	unsigned i;
	for(i=0;i<m;i++){
		numer *= m-i;
		denom *= n-i;
	}
	
	return ransacConvg*numer/denom;
}

/**
 * Choose, without repetition, sampleSize integers in the range [0, numDataPoints).
 */

static inline void sacRndSmpl(unsigned  sampleSize,
                              unsigned* currentSample,
                              unsigned  dataSetSize){
	/**
	 * If sampleSize is very close to dataSetSize, we use selection sampling.
	 * Otherwise we use the naive sampling technique wherein we select random
	 * indexes until sampleSize of them are distinct.
	 */
	
	if(sampleSize*2>dataSetSize){
		/**
		 * Selection Sampling:
		 * 
		 * Algorithm S (Selection sampling technique). To select n records at random from a set of N, where 0 < n ≤ N.
		 * S1. [Initialize.] Set t ← 0, m ← 0. (During this algorithm, m represents the number of records selected so far,
		 *                                      and t is the total number of input records that we have dealt with.)
		 * S2. [Generate U.] Generate a random number U, uniformly distributed between zero and one.
		 * S3. [Test.] If (N – t)U ≥ n – m, go to step S5.
		 * S4. [Select.] Select the next record for the sample, and increase m and t by 1. If m < n, go to step S2;
		 *               otherwise the sample is complete and the algorithm terminates.
		 * S5. [Skip.] Skip the next record (do not include it in the sample), increase t by 1, and go back to step S2.
		 */
		
		unsigned m=0,t=0;
		
		for(m=0;m<sampleSize;t++){
			double U=sacRandom();
			if((dataSetSize-t)*U < (sampleSize-m)){
				currentSample[m++]=t;
			}
		}
	}else{
		/**
		 * Naive sampling technique. Generate indexes until sampleSize of them are distinct.
		 */
		
		unsigned i, j;
		for(i=0;i<sampleSize;i++){
			int inList;
			
			do{
				currentSample[i]=dataSetSize*sacRandom();
				
				inList=0;
				for(j=0;j<i;j++){
					if(currentSample[i] == currentSample[j]){
						inList=1;
						break;
					}
				}
			}while(inList);
		}
	}
}

/**
 * Generates a random double uniformly distributed in the range [0, 1].
 */

static inline double sacRandom(void){
#ifdef _WIN32
	return rand()/RAND_MAX;
#else
	return ((double)random())/INT_MAX;
#endif
}

/**
 * Estimate the number of iterations required based on the requested confidence,
 * proportion of inliers in the best model so far and sample size.
 * 
 * Clamp return value at maxIterationBound.
 */

static inline unsigned sacCalcIterBound(double   confidence,
                                        double   inlierRate,
                                        unsigned sampleSize,
                                        unsigned maxIterBound){
	unsigned retVal;
	
	/**
	 * Formula chosen from http://en.wikipedia.org/wiki/RANSAC#The_parameters :
	 * 
	 * \[ k = \frac{\log{(1-confidence)}}{\log{(1-inlierRate**sampleSize)}} \]
	 */
	
	double atLeastOneOutlierProbability = 1.-pow(inlierRate, (double)sampleSize);
	
	/**
	 * There are two special cases: When argument to log() is 0 and when it is 1.
	 * Each has a special meaning.
	 */
	
	if(atLeastOneOutlierProbability>=1.){
		/**
		 * A certainty of picking at least one outlier means that we will need
		 * an infinite amount of iterations in order to find a correct solution.
		 */
		
		retVal = maxIterBound;
	}else if(atLeastOneOutlierProbability<=0.){
		/**
		 * The certainty of NOT picking an outlier means that only 1 iteration
		 * is needed to find a solution.
		 */
		
		retVal = 1;
	}else{
		/**
		 * Since 1-confidence (the probability of the model being based on at
		 * least one outlier in the data) is equal to
		 * (1-inlierRate**sampleSize)**numIterations (the probability of picking
		 * at least one outlier in numIterations samples), we can isolate
		 * numIterations (the return value) into
		 */
		
		retVal = ceil(log(1.-confidence)/log(atLeastOneOutlierProbability));
	}
	
	/**
	 * Clamp to maxIterationBound.
	 */
	
	return retVal <= maxIterBound ? retVal : maxIterBound;
}


/* Transposed, C */
static void hFuncRefC(float* packedPoints,/* Source (four x,y float coordinates) points followed by
                                             destination (four x,y float coordinates) points, aligned by 32 bytes */
                      float* H){          /* Homography (three 16-byte aligned rows of 3 floats) */
	float x0=*packedPoints++;
	float y0=*packedPoints++;
	float x1=*packedPoints++;
	float y1=*packedPoints++;
	float x2=*packedPoints++;
	float y2=*packedPoints++;
	float x3=*packedPoints++;
	float y3=*packedPoints++;
	float X0=*packedPoints++;
	float Y0=*packedPoints++;
	float X1=*packedPoints++;
	float Y1=*packedPoints++;
	float X2=*packedPoints++;
	float Y2=*packedPoints++;
	float X3=*packedPoints++;
	float Y3=*packedPoints++;
	
	float x0X0=x0*X0, x1X1=x1*X1, x2X2=x2*X2, x3X3=x3*X3;
	float x0Y0=x0*Y0, x1Y1=x1*Y1, x2Y2=x2*Y2, x3Y3=x3*Y3;
	float y0X0=y0*X0, y1X1=y1*X1, y2X2=y2*X2, y3X3=y3*X3;
	float y0Y0=y0*Y0, y1Y1=y1*Y1, y2Y2=y2*Y2, y3Y3=y3*Y3;
	
	
	/**
	 *  [0]   [1] Hidden   Prec
	 *  x0    y0    1       x1
	 *  x1    y1    1       x1
	 *  x2    y2    1       x1
	 *  x3    y3    1       x1
	 * 
	 * Eliminate ones in column 2 and 5.
	 * R(0)-=R(2)
	 * R(1)-=R(2)
	 * R(3)-=R(2)
	 * 
	 *  [0]   [1] Hidden   Prec
	 * x0-x2 y0-y2  0       x1+1
	 * x1-x2 y1-y2  0       x1+1
	 *  x2    y2    1       x1
	 * x3-x2 y3-y2  0       x1+1
	 * 
	 * Eliminate column 0 of rows 1 and 3
	 * R(1)=(x0-x2)*R(1)-(x1-x2)*R(0),     y1'=(y1-y2)(x0-x2)-(x1-x2)(y0-y2)
	 * R(3)=(x0-x2)*R(3)-(x3-x2)*R(0),     y3'=(y3-y2)(x0-x2)-(x3-x2)(y0-y2)
	 * 
	 *  [0]   [1] Hidden   Prec
	 * x0-x2 y0-y2  0      x1+1
	 *   0    y1'   0      x2+3
	 *  x2    y2    1       x1
	 *   0    y3'   0      x2+3
	 * 
	 * Eliminate column 1 of rows 0 and 3
	 * R(3)=y1'*R(3)-y3'*R(1)
	 * R(0)=y1'*R(0)-(y0-y2)*R(1)
	 * 
	 *  [0]   [1] Hidden   Prec
	 *  x0'    0    0      x3+5
	 *   0    y1'   0      x2+3
	 *  x2    y2    1       x1
	 *   0     0    0      x4+7
	 * 
	 * Eliminate columns 0 and 1 of row 2
	 * R(0)/=x0'
	 * R(1)/=y1'
	 * R(2)-= (x2*R(0) + y2*R(1))
	 * 
	 *  [0]   [1] Hidden   Prec
	 *   1     0    0      x6+10
	 *   0     1    0      x4+6
	 *   0     0    1      x4+7
	 *   0     0    0      x4+7
	 */
	
	/**
	 * Eliminate ones in column 2 and 5.
	 * R(0)-=R(2)
	 * R(1)-=R(2)
	 * R(3)-=R(2)
	 */
	
	/*float minor[4][2] = {{x0-x2,y0-y2},
						 {x1-x2,y1-y2},
						 {x2   ,y2   },
						 {x3-x2,y3-y2}};*/
	/*float major[8][3] = {{x2X2-x0X0,y2X2-y0X0,(X0-X2)},
						 {x2X2-x1X1,y2X2-y1X1,(X1-X2)},
						 {-x2X2    ,-y2X2    ,(X2   )},
						 {x2X2-x3X3,y2X2-y3X3,(X3-X2)},
						 {x2Y2-x0Y0,y2Y2-y0Y0,(Y0-Y2)},
						 {x2Y2-x1Y1,y2Y2-y1Y1,(Y1-Y2)},
						 {-x2Y2    ,-y2Y2    ,(Y2   )},
						 {x2Y2-x3Y3,y2Y2-y3Y3,(Y3-Y2)}};*/
	float minor[2][4] = {{x0-x2,x1-x2,x2   ,x3-x2},
	                     {y0-y2,y1-y2,y2   ,y3-y2}};
	float major[3][8] = {{x2X2-x0X0,x2X2-x1X1,-x2X2    ,x2X2-x3X3,x2Y2-x0Y0,x2Y2-x1Y1,-x2Y2    ,x2Y2-x3Y3},
	                     {y2X2-y0X0,y2X2-y1X1,-y2X2    ,y2X2-y3X3,y2Y2-y0Y0,y2Y2-y1Y1,-y2Y2    ,y2Y2-y3Y3},
	                     { (X0-X2) , (X1-X2) , (X2   ) , (X3-X2) , (Y0-Y2) , (Y1-Y2) , (Y2   ) , (Y3-Y2) }};
	
	/**
	 * int i;
	 * for(i=0;i<8;i++) major[2][i]=-major[2][i];
	 * Eliminate column 0 of rows 1 and 3
	 * R(1)=(x0-x2)*R(1)-(x1-x2)*R(0),     y1'=(y1-y2)(x0-x2)-(x1-x2)(y0-y2)
	 * R(3)=(x0-x2)*R(3)-(x3-x2)*R(0),     y3'=(y3-y2)(x0-x2)-(x3-x2)(y0-y2)
	 */
	
	float scalar1=minor[0][0], scalar2=minor[0][1];
	minor[1][1]=minor[1][1]*scalar1-minor[1][0]*scalar2;
	
	major[0][1]=major[0][1]*scalar1-major[0][0]*scalar2;
	major[1][1]=major[1][1]*scalar1-major[1][0]*scalar2;
	major[2][1]=major[2][1]*scalar1-major[2][0]*scalar2;
	
	major[0][5]=major[0][5]*scalar1-major[0][4]*scalar2;
	major[1][5]=major[1][5]*scalar1-major[1][4]*scalar2;
	major[2][5]=major[2][5]*scalar1-major[2][4]*scalar2;
	
	scalar2=minor[0][3];
	minor[1][3]=minor[1][3]*scalar1-minor[1][0]*scalar2;
	
	major[0][3]=major[0][3]*scalar1-major[0][0]*scalar2;
	major[1][3]=major[1][3]*scalar1-major[1][0]*scalar2;
	major[2][3]=major[2][3]*scalar1-major[2][0]*scalar2;
	
	major[0][7]=major[0][7]*scalar1-major[0][4]*scalar2;
	major[1][7]=major[1][7]*scalar1-major[1][4]*scalar2;
	major[2][7]=major[2][7]*scalar1-major[2][4]*scalar2;
	
	/**
	 * Eliminate column 1 of rows 0 and 3
	 * R(3)=y1'*R(3)-y3'*R(1)
	 * R(0)=y1'*R(0)-(y0-y2)*R(1)
	 */
	
	scalar1=minor[1][1];scalar2=minor[1][3];
	major[0][3]=major[0][3]*scalar1-major[0][1]*scalar2;
	major[1][3]=major[1][3]*scalar1-major[1][1]*scalar2;
	major[2][3]=major[2][3]*scalar1-major[2][1]*scalar2;
	
	major[0][7]=major[0][7]*scalar1-major[0][5]*scalar2;
	major[1][7]=major[1][7]*scalar1-major[1][5]*scalar2;
	major[2][7]=major[2][7]*scalar1-major[2][5]*scalar2;
	
	scalar2=minor[1][0];
	minor[0][0]=minor[0][0]*scalar1-minor[0][1]*scalar2;
	
	major[0][0]=major[0][0]*scalar1-major[0][1]*scalar2;
	major[1][0]=major[1][0]*scalar1-major[1][1]*scalar2;
	major[2][0]=major[2][0]*scalar1-major[2][1]*scalar2;
	
	major[0][4]=major[0][4]*scalar1-major[0][5]*scalar2;
	major[1][4]=major[1][4]*scalar1-major[1][5]*scalar2;
	major[2][4]=major[2][4]*scalar1-major[2][5]*scalar2;
	
	/**
	 * Eliminate columns 0 and 1 of row 2
	 * R(0)/=x0'
	 * R(1)/=y1'
	 * R(2)-= (x2*R(0) + y2*R(1))
	 */
	
	scalar1=minor[0][0];
	major[0][0]/=scalar1;
	major[1][0]/=scalar1;
	major[2][0]/=scalar1;
	major[0][4]/=scalar1;
	major[1][4]/=scalar1;
	major[2][4]/=scalar1;
	
	scalar1=minor[1][1];
	major[0][1]/=scalar1;
	major[1][1]/=scalar1;
	major[2][1]/=scalar1;
	major[0][5]/=scalar1;
	major[1][5]/=scalar1;
	major[2][5]/=scalar1;
	
	
	scalar1=minor[0][2];scalar2=minor[1][2];
	major[0][2]-=major[0][0]*scalar1+major[0][1]*scalar2;
	major[1][2]-=major[1][0]*scalar1+major[1][1]*scalar2;
	major[2][2]-=major[2][0]*scalar1+major[2][1]*scalar2;
	
	major[0][6]-=major[0][4]*scalar1+major[0][5]*scalar2;
	major[1][6]-=major[1][4]*scalar1+major[1][5]*scalar2;
	major[2][6]-=major[2][4]*scalar1+major[2][5]*scalar2;
	
	/* Only major matters now. R(3) and R(7) correspond to the hollowed-out rows. */
	scalar1=major[0][7];
	major[1][7]/=scalar1;
	major[2][7]/=scalar1;
	
	scalar1=major[0][0];major[1][0]-=scalar1*major[1][7];major[2][0]-=scalar1*major[2][7];
	scalar1=major[0][1];major[1][1]-=scalar1*major[1][7];major[2][1]-=scalar1*major[2][7];
	scalar1=major[0][2];major[1][2]-=scalar1*major[1][7];major[2][2]-=scalar1*major[2][7];
	scalar1=major[0][3];major[1][3]-=scalar1*major[1][7];major[2][3]-=scalar1*major[2][7];
	scalar1=major[0][4];major[1][4]-=scalar1*major[1][7];major[2][4]-=scalar1*major[2][7];
	scalar1=major[0][5];major[1][5]-=scalar1*major[1][7];major[2][5]-=scalar1*major[2][7];
	scalar1=major[0][6];major[1][6]-=scalar1*major[1][7];major[2][6]-=scalar1*major[2][7];
	
	
	/* One column left (Two in fact, but the last one is the homography) */
	scalar1=major[1][3];
	
	major[2][3]/=scalar1;
	scalar1=major[1][0];major[2][0]-=scalar1*major[2][3];
	scalar1=major[1][1];major[2][1]-=scalar1*major[2][3];
	scalar1=major[1][2];major[2][2]-=scalar1*major[2][3];
	scalar1=major[1][4];major[2][4]-=scalar1*major[2][3];
	scalar1=major[1][5];major[2][5]-=scalar1*major[2][3];
	scalar1=major[1][6];major[2][6]-=scalar1*major[2][3];
	scalar1=major[1][7];major[2][7]-=scalar1*major[2][3];
	
	
	/* Homography is done. */
	H[0]=major[2][0];
	H[1]=major[2][1];
	H[2]=major[2][2];
	
	H[4]=major[2][4];
	H[5]=major[2][5];
	H[6]=major[2][6];
	
	H[8]=major[2][7];
	H[9]=major[2][3];
	H[10]=1.0;
}

