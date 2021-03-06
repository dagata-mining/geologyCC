//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qM3C2                       #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#            COPYRIGHT: UNIVERSITE EUROPEENNE DE BRETAGNE                #
//#                                                                        #
//##########################################################################

#ifndef LIBPOINTMATCHER_TOOLS_HEADER
#define LIBPOINTMATCHER_TOOLS_HEADER

//CCCoreLib
#include <GenericIndexedCloud.h>
#include <GenericProgressCallback.h>
#include <DgmOctree.h>
#include <pointmatcher/PointMatcher.h>
#include <nabo/nabo.h>
#include <CCCoreLib.h>

class ccGenericPointCloud;
class NormsIndexesTableType;
class ccScalarField;
class ccPointCloud;
class ccMainAppInterface;

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;

//! M3C2 normals computation related tools
class LibpointmatcherNormals
{
public:

	//! Normals computation mode
	/** \warning Don't change the associated values! (for parameter files)
	**/
	enum ComputationMode
	{
		DEFAULT_MODE			= 0, //compute normals on core points
		USE_CLOUD1_NORMALS		= 1,
		MULTI_SCALE_MODE		= 2,
		VERT_MODE				= 3,
		HORIZ_MODE				= 4,
		USE_CORE_POINTS_NORMALS	= 5,
	};

	//! Computes normals on core points only
	/** See qCC's ccNormalVectors::ComputeCloudNormals.
		\warning normals orientation is not resolved!
	**/
	static bool ComputeCorePointsNormals(	CCCoreLib::GenericIndexedCloud* corePoints,
											NormsIndexesTableType* corePointsNormals,
											ccGenericPointCloud* sourceCloud,
											const std::vector<PointCoordinateType>& sortedRadii,
											bool& invalidNormals,
											int maxThreadCount = 0,
											ccScalarField* normalScale = 0,
											CCCoreLib::GenericProgressCallback* progressCb = 0,
											CCCoreLib::DgmOctree* inputOctree = 0);
	
	//! Re-orients normal vectors so that they all 'look' towards the nearest point of another cloud
	static bool UpdateNormalOrientationsWithCloud(	CCCoreLib::GenericIndexedCloud* normCloud,
													NormsIndexesTableType& normsCodes,
													CCCoreLib::GenericIndexedCloud* orientationCloud,
													int maxThreadCount = 0,
													CCCoreLib::GenericProgressCallback* progressCb = 0);

	//! Makes all normals horizontal
	static void MakeNormalsHorizontal(NormsIndexesTableType& normsCodes);
};

//! M3C2 generic tools
class LibpointmatcherTools
{
public:

	//! Computes statistics on a neighbors set
	/** Either the mean distance and std. dev. (if useMedian is false)
		or the median and interquartile range (if useMedian is true).
		See http://en.wikipedia.org/wiki/Interquartile_range
	**/
	static void ComputeStatistics(	CCCoreLib::DgmOctree::NeighboursSet& set,
									bool useMedian,
									double& meanOrMedian,
									double& stdDevOrIQR);

	//! M3C2 parameters that can be guessed automatically by 'probing'
	struct GuessedParams
	{
		int preferredDimension;
		double normScale;
		double projScale;
		double projDepth;
	};

	//! Tries to guess some M3C2 parameters by randomly 'probing' the cloud
	static bool GuessBestParams(ccPointCloud* cloud1,
								ccPointCloud* cloud2,
								unsigned minPoints4Stats,
								GuessedParams& params,
								bool fastMode,
								ccMainAppInterface* app = 0,
								unsigned probingCount = 1000);

	// Converts a CloudCompare Entity to a Point Matcher Entity
	static DP ccToPointMatcher(ccPointCloud* cloud);
	static DP ccToPointMatcherSubsample(ccPointCloud* cloud);
	// Converts a CloudCompare Entity to a Point Matcher Entity
	static DP ccNormalsToPointMatcher(ccPointCloud* cloud);
	static DP ccNormalsToPointMatcherSubsample(ccPointCloud* cloud);
	// Converts a pointMatcher Entity to a Cloudcompare ReferenceCloud 
	static CCCoreLib::ReferenceCloud* pointmatcherToCC(DP* cloud, ccPointCloud* ref);
	static CCCoreLib::ReferenceCloud* pointmatcherToCCSubsample(DP* cloud, ccPointCloud* ref);
	// Subsamples from Libpointmatcher
	static DP filter(DP cloud, std::vector< std::shared_ptr<PM::DataPointsFilter>> filters, std::shared_ptr<PM::DataPointsFilter> normalParams, std::vector<bool> needNormals, bool hasNormalDescriptors );
	//! returns the bounds of a DP cloud (Xmax,Ymax,Zmax,Xmin,Ymin,Zmin)
	static std::vector<float> getBounds(DP* cloud);
	//! Returns a filter based on the bounds and the extra padding 
	static std::shared_ptr<PM::DataPointsFilter> boundsFilter(std::vector<float>, bool, float);
	
};

#endif //Q_M3C2_TOOLS_HEADER
