//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef CC_STICHED_IMAGE_VIEWER
#define CC_STICHED_IMAGE_VIEWER

namespace Ui {
	class ImageStichedViewer;
}
#include <ccHObject.h>
#include "ccPointPickingGenericInterface.h"

//Qt
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <QWheelEvent>
#include <QKeyEvent>
#include <QVector3D>
#include <ccImageDrawer.h>
#include <ccPickingHub.h>
#include <ccPickingListener.h>
#include <ccViewportParameters.h>



// Types of view
enum CAM_VIEW_ORIENTATION {
	CAM_ROOF_VIEW,	/**< Top view (eye: +Z) **/
	CAM_FLOOR_VIEW,	/**< Bottom view **/
	CAM_FRONT_VIEW,	/**< Front view **/
	CAM_BACK_VIEW,	/**< Back view **/
	CAM_LEFT_VIEW,	/**< Left view **/
	CAM_RIGHT_VIEW,	/**< Right view **/
};



class MainWindow;

//! Custom QListWidget to allow for the copy of all selected elements when using CTRL+C

class ccStichedImageViewer : public QWidget, public ccPickingListener
{
	Q_OBJECT

public:

	//~ccImageViewer() override;
	explicit ccStichedImageViewer(QWidget*, ccPickingHub*);
	//! Default destructor
	virtual ~ccStichedImageViewer();

	// add item to combo box
	void addComboBoxItem();

	// Escape key when we want to quit selection
	void keyPressEvent(QKeyEvent*);


	//inherited from 
	bool startPP();
	void stopPP(bool state);
	//! Inherited from ccPickingListener	
	void onItemPicked(const PickedItem& pi) override;
	//!From GL call

	ccImageDrawer* m_imageDrawer;
	Ui::ImageStichedViewer *m_ui;

	ccPointCloud* getCurrentPointCloud() { return m_currentPointCloud; }
	ccPointCloud* getCurrentTrajectory() { return m_currentTrajectory; }
	ccPointCloud* getUnrolledCloud() { return m_unrolledCloud; }
	void setCurrentPointCloud(ccPointCloud* cloud) { m_currentPointCloud = cloud;
													addComboBoxItem();}
	void setCurrentTrajectory(ccPointCloud* cloud) { m_currentTrajectory =cloud ; }
	void setUnrolledCloud(ccPointCloud* cloud) { m_unrolledCloud =cloud ; }

	//! plan call
	int getPlanSize();

	// Enable all tools
	void disableAllWidget();
	// Block all tools
	void enableAllWidget();

	void setCurrentTransform(std::vector<double> t){m_currentTransform = t.size() == 7 ? t: std::vector<double>{ 0, 0, 0, 1, 0, 0, 0 };}
	std::vector<double> getCurrentTransform() { return m_currentTransform; }

	bool m_isAddingCloud;

	//! update stiched image based on pose
	void updateStichedImage(int);
	//! update Node from id 
	void updateNodeId(int);

	//! Change Step Size for node trajectory
	void changeStepNode(int);

	void actionUnroll(ccPointCloud* currentCloud, ccPointCloud* &outCloudUnrolled,
		float radius, CCVector3 center, bool exportDistance,
		CCCoreLib::GenericProgressCallback* progressCb =nullptr);

	void cleanUnrollOctree(int octreeLevel, float radius, ccPointCloud* currentCloud, ccPointCloud* unrolledCloud,
		ccPointCloud* &outCleanUnrolled, ccPointCloud* &outCloudClean, ccPointCloud* &outCloudRemaining,
		CCCoreLib::GenericProgressCallback* progressCb = nullptr
		);

public slots:

	// ChangeNodePosition
	void nodeUp();
	void nodeDown();

protected: 
	//! Picking hub
	ccPickingHub* m_pickingHub;
	
private:

	//Export .dxf file
	void saveDxf();
	
	//  Export to CSV file ( X,Y,Z, dip. dip_dir)
	bool planeToFile();
	// Import form CSV file (X,Y,Z,dip,dip_dir)
	bool planeFromFile();
	// Set the plan size for all plans
	void setSizePlan();
	// Get the file name
	QString getFileCSV();
	// Set the file name
	QString setFileCSV();
	// Activate point picking for GL window on the point  
	void activatePointPicking(); 
	// Deactivate point picking for GL window on the point cloud 
	void deactivatePointPicking();
	// Process picked point
	void processPickedPoint(const PickedItem& picked);
	// Currently in the process of picking
	bool m_currentPicking;
	// Current point cloud address
	ccPointCloud* m_currentPointCloud;
	// Current trajectory address
	ccPointCloud* m_currentTrajectory;

	// Unrolled Cloud
	ccPointCloud* m_unrolledCloud;
	CCVector3 m_center;
	float m_radius;

	// Current viewport
	ccViewportParameters* m_viewport;


	//Current Camera Node Center Position
	CCVector3d m_currentCameraNodeCenterPosition;
	CCVector3d m_currentCameraNodeOrientation;
	CCVector3d m_currentCameraNodeUpOrientation;
	
	//Current Camera Views Far Plane
	float m_currentFar;
	float m_frontFar;
	float m_backFar;
	float m_rightFar;
	float m_leftFar;
	float m_roofFar;
	float m_floorFar;

	CCVector3d m_shiftVector;

	// if its its vertical mode
	bool m_vertical;
	CCVector3d m_up;
	CCVector3d m_forward;


	//Rotations for viewport
	float m_upAxisRotate;
	float m_sideAxisRotate;
	bool m_quat;

	double m_nearCoef;

	//View Shift Transform to apply in side view and top/bottom views
	double m_viewShiftDistance;
	double m_frontShiftDistance;
	double m_backShiftDistance;
	double m_leftShiftDistance;
	double m_roofShiftDistance;
	double m_floorShiftDistance;
	double m_rightShiftDistance;

	std::vector<double> m_currentTransform{ 0,0,0,1,0,0,0 };


	
	//change the view orientation of the following cam
	void changeCamView(CC_VIEW_ORIENTATION);
	
	// Current Orientation
	CC_VIEW_ORIENTATION m_orientation;

	// set spin box
	void setSpinFar();
	void setSpinShift();

	// set mode
	void setShaftMode();

	// set camera node position
	void ccStichedImageViewer::setCameraNodePosition();

	// Get closest point id
	int closestPoseToPoint(const CCVector3* pt);
	void chooseImageAction();
	
	

	// generate image unrolled
	ccImage* generateImageUnroll(float zMax, float zMin, float color);

	// Clean Cloud
	
	CCCoreLib::ReferenceCloud* removeHiddenPoints(CCCoreLib::GenericIndexedCloudPersist * theCloud,CCVector3 bbMin, CCVector3 bbMax, float leafSize);
	void unrollClick();
	void rollingPoint(CCVector3 &point, float radius);

	// Generate linkers
	void generateLinkers(std::vector<ccPolyline*> polylines, std::vector<ccImage*> images, 
		std::vector<std::vector<CCVector3>> &linkersOut, float padding);

};



#endif
