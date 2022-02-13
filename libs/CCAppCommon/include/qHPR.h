//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qHPR                        #
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
//#                  COPYRIGHT: Daniel Girardeau-Montaut                   #
//#                                                                        #
//##########################################################################

#ifndef Q_HPR_PLUGIN_HEADER
#define Q_HPR_PLUGIN_HEADER


//CCCoreLib
#include <ReferenceCloud.h>
#include <ccPointCloud.h>
#include <CCCoreLib.h>


//! Wrapper to the "Hidden Point Removal" algorithm for approximating points visibility in an N dimensional point cloud, as seen from a given viewpoint
/** "Direct Visibility of Point Sets", Sagi Katz, Ayellet Tal, and Ronen Basri.
	SIGGRAPH 2007
	http://www.mathworks.com/matlabcentral/fileexchange/16581-hidden-point-removal
**/

namespace qHPR 
{
	//! Katz et al. algorithm
	CCCoreLib::ReferenceCloud* removeHiddenPoints(CCCoreLib::GenericIndexedCloudPersist * theCloud, CCVector3d viewPoint, double fParam);
	void doAction(ccPointCloud* cloud, CCVector3d viewPoint);
}



#endif
