/* Include Guards */
#ifndef __SPARTASSE2_H__
#define __SPARTASSE2_H__



/* Includes */





/* Defines */
#ifdef __cplusplus

/* C++ does not have the restrict keyword. */
#ifdef restrict
#undef restrict
#endif
#define restrict

#else

/* C99 and over has the restrict keyword. */
#if !defined(__STDC_VERSION__) || __STDC_VERSION__ < 199901L
#define restrict
#endif

#endif


/* Flags */
#ifndef HEST_FLAG_NONE
#define HEST_FLAG_NONE                        (0U<<0)
#endif
#ifndef HEST_FLAG_ENABLE_NR
#define HEST_FLAG_ENABLE_NR                   (1U<<0)
#endif
#ifndef HEST_FLAG_ENABLE_REFINEMENT
#define HEST_FLAG_ENABLE_REFINEMENT           (1U<<1)
#endif
#ifndef HEST_FLAG_ENABLE_FINAL_REFINEMENT
#define HEST_FLAG_ENABLE_FINAL_REFINEMENT     (1U<<2)
#endif



/* Data structures */

/**
 * Homography Estimation context.
 */

typedef struct{
	/* Virtual Arguments */
	const float* restrict src;
	const float* restrict dst;
	int                allocBestInl;
	char* restrict     bestInl;
	unsigned           N;
	float              maxD;
	unsigned           maxI;
	unsigned           rConvg;
	double             cfd;
	unsigned           minInl;
	double             beta;
	unsigned           flags;
	const float*       guessH;
	float*             finalH;
	
	/* PROSAC */
	unsigned           i;               /* Iteration Number */
	unsigned           phNum;           /* Phase Number */
	unsigned           phEndI;          /* Phase End Iteration */
	double             phEndFpI;        /* Phase floating-point End Iteration */
	unsigned           phMax;           /* Termination phase number */
	unsigned           phNumInl;        /* Number of inliers for termination phase */
	unsigned           bestNumInl;      /* Best    number of inliers */
	unsigned           numInl;          /* Current number of inliers */
	unsigned           numModels;       /* Number of models tested */
	unsigned* restrict smpl;            /* Sample */
	float* restrict    H;               /* Current homography */
	float* restrict    bestH;           /* Best homography */
	float* restrict    pkdPts;          /* Packed points */
	char* restrict     inl;             /* Inliers to current model */
	unsigned* restrict nrTBL;           /* Non-Randomness: Table */
	unsigned           nrSize;          /* Non-Randomness: Size */
	double             nrBeta;          /* Non-Randomness: Beta */
	
	/* SPRT */
	double             t_M;             /* t_M */
	double             m_S;             /* m_S */
	double             epsilon;         /* Epsilon */
	double             delta;           /* delta */
	double             A;               /* SPRT Threshold */
	unsigned           Ntested;         /* Number of points tested */
	unsigned           Ntestedtotal;    /* Number of points tested in total */
	int                good;            /* Good/bad flag */
	double             lambdaAccept;    /* Accept multiplier */
	double             lambdaReject;    /* Reject multiplier */
	double             lambdaTBL[16];   /* Multiplier LUT */
} PROSAC_HEST_SSE2;



/* Extern C */
#ifdef __cplusplus
extern "C" {
#endif



/* Functions */

/**
 * Initialize the estimator context, by allocating the aligned buffers
 * internally needed.
 * 
 * @param [in/out] p  The uninitialized estimator context to initialize.
 * @return 0 if successful; non-zero if an error occured.
 */

int  hestSSE2Init(PROSAC_HEST_SSE2* p);


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
 * @return 0 if successful; non-zero if an error occured.
 */

int  hestSSE2EnsureCapacity(PROSAC_HEST_SSE2* p, unsigned N, double beta);


/**
 * Finalize the estimator context, by freeing the aligned buffers used
 * internally.
 * 
 * @param [in] p  The initialized estimator context to finalize.
 */

void hestSSE2Fini(PROSAC_HEST_SSE2* p);


/**
 * Estimates the homography using the given context, matches and parameters to
 * PROSAC.
 * 
 * The given context must have been initialized.
 * 
 * The matches are provided as two arrays of N single-precision, floating-point
 * (x,y) points. Points with corresponding offsets in the two arrays constitute
 * a match. The homography estimation attempts to find the 3x3 matrix H which
 * best maps the homogeneous-coordinate points in the source array to their
 * corresponding homogeneous-coordinate points in the destination array.
 * 
 *     Note: At least 4 matches must be provided (N >= 4).
 *     Note: A point in either array takes up 2 floats. The first of two stores
 *           the x-coordinate and the second of the two stores the y-coordinate.
 *           Thus, the arrays resemble this in memory:
 *           
 *           src =    [x0, y0, x1, y1, x2, y2, x3, y3, x4, y4, ...]
 *           Matches:     |       |       |       |       |
 *           dst =    [x0, y0, x1, y1, x2, y2, x3, y3, x4, y4, ...]
 *     Note: The matches are expected to be provided sorted by quality, or at
 *           least not be worse-than-random in ordering.
 * 
 * A pointer to the base of an array of N bytes can be provided. It serves as
 * an output mask to indicate whether the corresponding match is an inlier to
 * the returned homography, if any. A zero indicates an outlier; A non-zero
 * value indicates an inlier.
 * 
 * The PROSAC estimator requires a few parameters of its own. These are:
 * 
 *     - The maximum distance that a source point projected onto the destination
 *           plane can be from its putative match and still be considered an
 *           inlier. Must be non-negative.
 *           A sane default is 3.0.
 *     - The maximum number of PROSAC iterations. This corresponds to the
 *           largest number of samples that will be drawn and tested.
 *           A sane default is 2000.
 *     - The RANSAC convergence parameter. This corresponds to the number of
 *           iterations after which PROSAC will start sampling like RANSAC.
 *           A sane default is 2000.
 *     - The confidence threshold. This corresponds to the probability of
 *           finding a correct solution. Must be bounded by [0, 1].
 *           A sane default is 0.995.
 *     - The minimum number of inliers acceptable. Only a solution with at
 *           least this many inliers will be returned. The minimum is 4.
 *           A sane default is 10% of N.
 *     - The beta-parameter for the non-randomness termination criterion.
 *           Ignored if non-randomness criterion disabled, otherwise must be
 *           bounded by (0, 1).
 *           A sane default is 0.35.
 *     - Optional flags to control the estimation. Available flags are:
 *           HEST_FLAG_NONE:
 *               No special processing.
 *           HEST_FLAG_ENABLE_NR:
 *               Enable non-randomness criterion. If set, the beta parameter
 *               must also be set.
 *           HEST_FLAG_ENABLE_REFINEMENT:
 *               Enable refinement of each new best model, as they are found.
 *           HEST_FLAG_ENABLE_FINAL_REFINEMENT:
 *               Enable one final refinement of the best model found before
 *               returning it.
 * 
 * The PROSAC estimator additionally accepts an extrinsic initial guess of H,
 * and outputs a final estimate at H provided it was able to find one with a
 * minimum of supporting inliers. If it was not, it outputs the all-zero
 * matrix.
 * 
 * The extrinsic guess at and final estimate of H are both in the same form:
 * A 3x3 single-precision floating-point matrix with step 4. Thus, it is a
 * 12-element array of floats, with the elements as follows:
 * 
 *     [ H00, H01, H02, <pad>,
 *       H10, H11, H12, <pad>,
 *       H20, H21, H22, <pad> ]
 * 
 * The function returns the number of inliers if it was able to find a
 * homography with at least the minimum required support, and 0 if it was not.
 * 
 * 
 * @param [in/out] p       The context to use for homography estimation. Must
 *                             be already initialized. Cannot be NULL.
 * @param [in]     src     The pointer to the source points of the matches.
 *                             Must be aligned to 16 bytes. Cannot be NULL.
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

unsigned hestSSE2(PROSAC_HEST_SSE2* restrict p,       /* Homography estimation context. */
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
                  float*                     finalH); /* Final result. */




/* Extern C */
#ifdef __cplusplus
}
#endif




#endif
