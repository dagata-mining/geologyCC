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

#include "qHPR.h"


//Qt
#include <QtGui>
#include <QMainWindow>

//qCC_db
#include <ccPointCloud.h>
#include <ccOctree.h>
#include <ccOctreeProxy.h>
#include <ccProgressDialog.h>
#include <cc2DViewportObject.h>


//qCC
#include <ccGLWindow.h>

//CCCoreLib
#include <CloudSamplingTools.h>

//Qhull
extern "C"
{
#include <qhull_a.h>
}

namespace qHPR{
void doAction(ccPointCloud* cloud, CCVector3d viewPoint)
{

	//unique parameter: the octree subdivision level
	int octreeLevel =10;
	assert(octreeLevel >= 0 && octreeLevel <= CCCoreLib::DgmOctree::MAX_OCTREE_LEVEL);

	//compute octree if cloud hasn't any
	ccOctree::Shared theOctree = cloud->getOctree();
	if (!theOctree)
	{
		theOctree = cloud->computeOctree();
	}

	if (!theOctree)
	{
		ccLog::Error("Couldn't compute octree!");
		return;
	}


	//HPR
	QScopedPointer<CCCoreLib::ReferenceCloud> visibleCells;
	{
		QScopedPointer<CCCoreLib::ReferenceCloud> theCellCenters( CCCoreLib::CloudSamplingTools::subsampleCloudWithOctreeAtLevel(	cloud,
																											static_cast<unsigned char>(octreeLevel),
																											CCCoreLib::CloudSamplingTools::NEAREST_POINT_TO_CELL_CENTER,
																											0,
																											theOctree.data()) );
		if (!theCellCenters)
		{
			ccLog::Error("Error while simplifying point cloud with octree!");
			return;
		}

		visibleCells.reset(removeHiddenPoints(theCellCenters.data(), viewPoint, 3.5));


		//warning: after this point, visibleCells can't be used anymore as a
		//normal cloud (as it's 'associated cloud' has been deleted).
		//Only its indexes are valid! (they are corresponding to octree cells)
	}

	if (visibleCells)
	{
		//DGM: we generate a new cloud now, instead of playing with the points visiblity! (too confusing for the user)
		/*if (!cloud->isVisibilityTableInstantiated() && !cloud->resetVisibilityArray())
		{
			m_app->dispToConsole("Visibility array allocation failed! (Not enough memory?)",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return;
		}

		ccPointCloud::VisibilityTableType* pointsVisibility = cloud->getTheVisibilityArray();
		assert(pointsVisibility);
		pointsVisibility->fill(POINT_HIDDEN);
		*/

		CCCoreLib::ReferenceCloud visiblePoints(theOctree->associatedCloud());

		unsigned visiblePointCount = 0;
		unsigned visibleCellsCount = visibleCells->size();

		CCCoreLib::DgmOctree::cellIndexesContainer cellIndexes;
		if (!theOctree->getCellIndexes(static_cast<unsigned char>(octreeLevel), cellIndexes))
		{
			ccLog::Error("Couldn't fetch the list of octree cell indexes! (Not enough memory?");
			return;
		}

		for (unsigned i = 0; i < visibleCellsCount; ++i)
		{
			//cell index
			unsigned index = visibleCells->getPointGlobalIndex(i);

			//points in this cell...
			CCCoreLib::ReferenceCloud Yk(theOctree->associatedCloud());
			theOctree->getPointsInCellByCellIndex(&Yk, cellIndexes[index], static_cast<unsigned char>(octreeLevel));
			//...are all visible
			/*unsigned count = Yk.size();
			for (unsigned j=0;j<count;++j)
				pointsVisibility->setValue(Yk.getPointGlobalIndex(j),POINT_VISIBLE);
			visiblePointCount += count;
			*/
			if (!visiblePoints.add(Yk))
			{
				ccLog::Error("Not enough memory!");
				return;
			}
		}

		visibleCells.reset(nullptr);

		ccLog::Print(QString("[HPR] Visible points: %1").arg(visiblePointCount));

		if (visiblePoints.size() == cloud->size())
		{
			ccLog::Error("No points were removed!");
		}
		else
		{
			//create cloud from visibility selection
			cloud = cloud->partialClone(&visiblePoints);
			if (cloud)
			{
				cloud->setVisible(true);
				cloud->setName(QString("visible_points"));
				cloud->setEnabled(false);
			
			}
			else
			{
				ccLog::Error("Not enough memory!");
			}
		}
	}

	//currently selected entities appearance may have changed!
	//m_app->refreshAll();
}

CCCoreLib::ReferenceCloud * removeHiddenPoints(CCCoreLib::GenericIndexedCloudPersist * theCloud, CCVector3d viewPoint, double fParam)
{
	assert(theCloud);

	unsigned nbPoints = theCloud->size();
	if (nbPoints == 0)
		return nullptr;

	//less than 4 points? no need for calculation, we return the whole cloud
	if (nbPoints < 4)
	{
		CCCoreLib::ReferenceCloud* visiblePoints = new CCCoreLib::ReferenceCloud(theCloud);
		if (!visiblePoints->addPointIndex(0, nbPoints)) //well even for less than 4 points we never know ;)
		{
			//not enough memory!
			delete visiblePoints;
			visiblePoints = nullptr;
		}
		return visiblePoints;
	}

	double maxRadius = 0;

	//convert point cloud to an array of double triplets (for qHull)
	coordT* pt_array = new coordT[(nbPoints + 1) * 3];
	{
		coordT* _pt_array = pt_array;

		for (unsigned i = 0; i < nbPoints; ++i)
		{
			CCVector3d P = CCVector3d::fromArray(theCloud->getPoint(i)->u) - viewPoint;
			*_pt_array++ = static_cast<coordT>(P.x);
			*_pt_array++ = static_cast<coordT>(P.y);
			*_pt_array++ = static_cast<coordT>(P.z);

			//we keep track of the highest 'radius'
			double r2 = P.norm2();
			if (maxRadius < r2)
				maxRadius = r2;
		}

		//we add the view point (Cf. HPR)
		*_pt_array++ = 0;
		*_pt_array++ = 0;
		*_pt_array++ = 0;

		maxRadius = sqrt(maxRadius);
	}

	//apply spherical flipping
	{
		maxRadius *= pow(10.0, fParam) * 2;

		coordT* _pt_array = pt_array;
		for (unsigned i = 0; i < nbPoints; ++i)
		{
			CCVector3d P = CCVector3d::fromArray(theCloud->getPoint(i)->u) - viewPoint;

			double r = (maxRadius / P.norm()) - 1.0;
			*_pt_array++ *= r;
			*_pt_array++ *= r;
			*_pt_array++ *= r;
		}
	}

	//array to flag points on the convex hull
	std::vector<bool> pointBelongsToCvxHull;

	static char qHullCommand[] = "qhull QJ Qci";
	if (!qh_new_qhull(3, nbPoints + 1, pt_array, False, qHullCommand, nullptr, stderr))
	{
		try
		{
			pointBelongsToCvxHull.resize(nbPoints + 1, false);
		}
		catch (const std::bad_alloc&)
		{
			//not enough memory!
			delete[] pt_array;
			return nullptr;
		}

		vertexT *vertex = nullptr;
		vertexT **vertexp = nullptr;
		facetT *facet = nullptr;

		FORALLfacets
		{
			//if (!facet->simplicial)
			//	error("convhulln: non-simplicial facet"); // should never happen with QJ

			setT* vertices = qh_facet3vertex(facet);
			FOREACHvertex_(vertices)
			{
				pointBelongsToCvxHull[qh_pointid(vertex->point)] = true;
			}
			qh_settempfree(&vertices);
		}
	}

	delete[] pt_array;
	pt_array = nullptr;

	qh_freeqhull(!qh_ALL);
	//free long memory
	int curlong = 0;
	int totlong = 0;
	//qh_memfreeshort(&curlong, &totlong);
	//free short memory and memory allocator

	if (!pointBelongsToCvxHull.empty())
	{
		//compute the number of points belonging to the convex hull
		unsigned cvxHullSize = 0;
		{
			for (unsigned i = 0; i < nbPoints; ++i)
				if (pointBelongsToCvxHull[i])
					++cvxHullSize;
		}

		CCCoreLib::ReferenceCloud* visiblePoints = new CCCoreLib::ReferenceCloud(theCloud);
		if (cvxHullSize != 0 && visiblePoints->reserve(cvxHullSize))
		{
			for (unsigned i = 0; i < nbPoints; ++i)
				if (pointBelongsToCvxHull[i])
					visiblePoints->addPointIndex(i); //can't fail, see above

			return visiblePoints;

		}
		else //not enough memory
		{
			delete visiblePoints;
			visiblePoints = nullptr;
		}
	}
	return nullptr;
}

}