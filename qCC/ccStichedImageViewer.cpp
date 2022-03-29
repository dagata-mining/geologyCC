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

#include "ccStichedImageViewer.h"

//Ui
#include <ui_stichedImageViewerDlg.h>
//Local
#include "ccPersistentSettings.h"
#include "ccPointPickingGenericInterface.h"
#include "mainwindow.h"
#include "CCCoreLib.h"

//qCC_db
#include <ccSingleton.h>
#include <ccDBRoot.h>
#include <ccHObject.h>
#include <ccPlane.h>
#include <ccNormalVectors.h>
#include <ccFileUtils.h>
#include <ccPointCloud.h>
#include <ccImageDrawer.h>
#include <ccScalarField.h>
#include <ccColorScalesManager.h>
//Qt
#include <QApplication>
#include <QClipboard>
#include <QColor>
#include <QKeyEvent>
#include <QMessageBox>
#include <QSettings>
#include <QTextStream>
#include <QThread>
#include <QTime>
#include <QLabel>
#include <Qbitmap>
#include <QWheelEvent>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFileDialog>
#include <QVector3D>
#include <QQuaternion>

//system
#include <cassert>
#ifdef QT_DEBUG
#include <iostream>
#endif

/***************
 *** Globals ***
 ***************/
ccStichedImageViewer::ccStichedImageViewer(QWidget* parent = nullptr, ccPickingHub* pickingHub = nullptr)
	:QWidget(parent)
	, m_imageDrawer(nullptr)
	, m_ui(new Ui::ImageStichedViewer)
	, m_pickingHub(pickingHub)
	, m_currentPicking(false)
	, m_currentPointCloud(nullptr)
	, m_currentTrajectory(nullptr)
	, m_unrolledCloud(nullptr)
	, m_viewport(nullptr)
	, m_currentCameraNodeOrientation(1, 0, 0)
	, m_currentCameraNodeUpOrientation(0, 0, 1)
	, m_sideAxisRotate(0.0)
	, m_upAxisRotate(0.0)
	, m_quat(true)
	, m_currentFar(50.0)
	, m_frontFar(50.0)
	, m_backFar(50.0)
	, m_leftFar(15.0)
	, m_rightFar(15.0)
	, m_roofFar(10.0)
	, m_floorFar(10.0)
	, m_viewShiftDistance(0.0)
	, m_frontShiftDistance(0.0)
	, m_backShiftDistance(0.0)
	, m_leftShiftDistance(7.0)
	, m_rightShiftDistance(7.0)
	, m_floorShiftDistance(20.0)
	, m_roofShiftDistance(20.0)
	, m_orientation(CC_FRONT_VIEW)
	, m_vertical(false)
	, m_forward(1.0,0.0,0.0)
	, m_up(0.0, 0.0, 1.0)
	, m_currentTransform{0,0,0,1,0,0,0} //x,y,z,qw,qx,qy,qz
{
	QWidget* iconOptions;
	m_ui->setupUi(this);

	setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	// Adding vertical Layout
	QHBoxLayout* Hlayout = new QHBoxLayout(parent); 
	
	QVBoxLayout* VlayoutIcon = new QVBoxLayout(parent);
	QVBoxLayout* VlayoutStiched = new QVBoxLayout(parent);

	Hlayout->addLayout(VlayoutIcon,1);
	Hlayout->addLayout(VlayoutStiched,7);

	m_imageDrawer = new ccImageDrawer(parent);
	setLayout(Hlayout);
	VlayoutIcon->addWidget(iconOptions);
	VlayoutStiched->addWidget(m_imageDrawer);
	setShaftMode();

	
	//connect(m_ui->shaftButton, &QAbstractButton::toggled, this, &ccStichedImageViewer::setShaftMode);
	connect(m_ui->chooseImage, &QToolButton::clicked, this, &ccStichedImageViewer::chooseImageAction);
	connect(m_ui->unroll, &QToolButton::clicked, this, &ccStichedImageViewer::unrollClick);
	connect(m_ui->exportCSVPlan, &QToolButton::clicked, this, &ccStichedImageViewer::planeToFile);
	connect(m_ui->importCSVPlan, &QToolButton::clicked, this, &ccStichedImageViewer::planeFromFile);
	connect(m_ui->spinBoxNode, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, &ccStichedImageViewer::updateNodeId);
	connect(m_ui->spinBoxNodeSpeed, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, &ccStichedImageViewer::changeStepNode);
	connect(m_ui->spinBoxPlan, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, &ccStichedImageViewer::setSizePlan);
	connect(m_ui->spinBoxFar, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, &ccStichedImageViewer::setSpinFar);
	connect(m_ui->spinBoxOff, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, &ccStichedImageViewer::setSpinShift);
	//connect(m_ui->comboBoxPointCloud, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &ccStichedImageViewer::setCurrentPointCloudId);

	// change view
	connect(m_ui->poseCenterButtonFront, &QToolButton::clicked, this, [=]() { changeCamView(CC_FRONT_VIEW);});
	connect(m_ui->poseCenterButtonBack, &QToolButton::clicked, this, [=]() { changeCamView(CC_BACK_VIEW);});
	connect(m_ui->poseCenterButtonLeft, &QToolButton::clicked, this, [=]() { changeCamView(CC_LEFT_VIEW);});
	connect(m_ui->poseCenterButtonRight, &QToolButton::clicked, this, [=]() { changeCamView(CC_RIGHT_VIEW);});
	connect(m_ui->poseCenterButtonRoof, &QToolButton::clicked, this, [=]() { changeCamView(CC_TOP_VIEW);});
	connect(m_ui->poseCenterButtonFloor, &QToolButton::clicked, this, [=]() { changeCamView(CC_BOTTOM_VIEW);});

	m_viewport = new ccViewportParameters();
	
}


ccStichedImageViewer::~ccStichedImageViewer()
{
	if (m_imageDrawer)
		delete m_imageDrawer;
	if (m_viewport)
		delete m_viewport;

}




void ccStichedImageViewer::addComboBoxItem()
{
	m_isAddingCloud = true;
	ccHObject* root = MainWindow::TheInstance()->dbRootObject();
	ccHObject::Container pointClouds;
	root->filterChildren(pointClouds, true, CC_TYPES::POINT_CLOUD);
	
	// Verify if there are some pointclouds available
	if (pointClouds.size() == 0)
	{
		m_currentPointCloud = nullptr;
		m_currentTrajectory = nullptr;
		disableAllWidget();
		m_isAddingCloud = false;
		return;
	}


	// Add point clouds to the combo boxes
	m_ui->comboBoxPointCloud->clear();


	for (int i = 0; i < pointClouds.size(); i++)
	{
		ccPointCloud* pCloud = ccHObjectCaster::ToPointCloud(pointClouds[i]);
		QString name = pCloud->getName() + " id:" + QString::number(pCloud->getUniqueID());

		if (pCloud->isImagePointCloud() && pCloud->getTrajectoryCloud())
	
			if (pCloud->getTrajectoryCloud()->isTrajectory())
			{
				m_currentPointCloud = pCloud;

				m_currentTrajectory = pCloud->getTrajectoryCloud();
				m_ui->comboBoxPointCloud->addItem(name, pCloud->getUniqueID());
				enableAllWidget();
			}
		
	}

	if (!m_currentPointCloud || !m_currentTrajectory) 
	{
		disableAllWidget();
		return; }

	m_ui->spinBoxNode->setMaximum(m_currentTrajectory->size() - 1);
	return;
}



void ccStichedImageViewer::disableAllWidget() 
{
	m_ui->chooseImage->setEnabled(false);
	m_ui->unroll->setEnabled(false);
	m_ui->comboBoxPointCloud->setEnabled(false);
	m_ui->exportCSVPlan->setEnabled(false);
	m_ui->importCSVPlan->setEnabled(false);
	m_ui->spinBoxNode->setEnabled(false);
	m_ui->spinBoxPlan->setEnabled(false);
	m_ui->poseCenterButtonFront->setEnabled(false);
	m_ui->poseCenterButtonBack->setEnabled(false);
	m_ui->poseCenterButtonLeft->setEnabled(false);
	m_ui->poseCenterButtonRight->setEnabled(false);
	m_ui->poseCenterButtonFloor->setEnabled(false);
	m_ui->poseCenterButtonRoof->setEnabled(false);
	m_ui->spinBoxOff->setEnabled(false);
	m_ui->spinBoxFar->setEnabled(false);
	m_imageDrawer->disableAllWidget();
	this->setFocus();
}
void ccStichedImageViewer::enableAllWidget()
{
	m_ui->chooseImage->setEnabled(true);
	m_ui->unroll->setEnabled(true);
	m_ui->comboBoxPointCloud->setEnabled(true);
	m_ui->spinBoxNode->setEnabled(true);
	m_ui->spinBoxPlan->setEnabled(true);
	m_ui->exportCSVPlan->setEnabled(true);
	m_ui->importCSVPlan->setEnabled(true);
	m_ui->poseCenterButtonFront->setEnabled(true);
	m_ui->poseCenterButtonBack->setEnabled(true);
	m_ui->poseCenterButtonLeft->setEnabled(true);
	m_ui->poseCenterButtonRight->setEnabled(true);
	m_ui->poseCenterButtonFloor->setEnabled(true);
	m_ui->poseCenterButtonRoof->setEnabled(true);
	m_ui->spinBoxOff->setEnabled(true);
	m_ui->spinBoxFar->setEnabled(true);
	m_imageDrawer->enableAllWidget();
}


void ccStichedImageViewer::updateStichedImage(int id)
{

	// Get imageId
	int imageIdField	= m_currentTrajectory->getScalarFieldIndexByName("imageId");
	if (imageIdField == -1) {return;}
	m_currentTrajectory->setCurrentScalarField(imageIdField);
	int imageIdValue = (int)m_currentTrajectory->getPointScalarValue(id);

	// Get qx
	int qxField = m_currentTrajectory->getScalarFieldIndexByName("qx");
	if (qxField == -1) { return; }
	m_currentTrajectory->setCurrentScalarField(qxField);
	double qx = (double)m_currentTrajectory->getPointScalarValue(id);
	
	// Get qy
	int qyField = m_currentTrajectory->getScalarFieldIndexByName("qy");
	if (qyField == -1) { return; }
	m_currentTrajectory->setCurrentScalarField(qyField);
	double qy = (double)m_currentTrajectory->getPointScalarValue(id);
	
	// Get qz
	int qzField = m_currentTrajectory->getScalarFieldIndexByName("qz");
	if (qzField == -1) { return; }
	m_currentTrajectory->setCurrentScalarField(qzField);
	double qz = (double)m_currentTrajectory->getPointScalarValue(id);
	
	// Get qw
	int qwField = m_currentTrajectory->getScalarFieldIndexByName("qw");
	if (qwField == -1) { return; }
	m_currentTrajectory->setCurrentScalarField(qwField);
	double qw = (double)m_currentTrajectory->getPointScalarValue(id);

	// Get pt
	const CCVector3* pt = m_currentTrajectory->getPoint(id);
	double x = pt->x;
	double y = pt->y;
	double z = pt->z;

	// viewParameter = [x,y,z,qw,qx,qy,qz]
	std::vector<double> viewParameter{ x,y,z,qw,qx,qy,qz };
	ccHObject* imageObject = MainWindow::TheInstance()->dbRootObject()->find(imageIdValue);
	if (!imageObject->isA(CC_TYPES::IMAGE))
	{
		return;
	}
	
	m_imageDrawer->setImageInformation(ccHObjectCaster::ToImage(imageObject), viewParameter);

}




void ccStichedImageViewer::activatePointPicking() 
{
	return;
}
void ccStichedImageViewer::deactivatePointPicking()
{
	return;
}

void ccStichedImageViewer::processPickedPoint(const PickedItem& picked)
{
	return;
}

void ccStichedImageViewer::chooseImageAction()
{
	if (!m_currentPointCloud || !m_currentTrajectory) { return; }
	if (!m_currentPicking)
	{
		startPP();
		disableAllWidget();
		m_currentPicking = true;


	}
	else
	{
		stopPP(true);
		enableAllWidget();
		m_currentPicking = false;

	}
}


bool ccStichedImageViewer::startPP()
{
	if (!m_currentPointCloud || !m_currentTrajectory) { 
		return true; }
	if (!m_pickingHub)
	{
		ccLog::Error("[Point picking] No associated display!");
		return false;
	}

	//activate "point picking mode" in associated GL window
	if (!m_pickingHub->addListener(this, true, true, ccGLWindow::POINT_PICKING))
	{
		ccLog::Error("Picking mechanism already in use. Close the tool using it first.");
		return false;
	}
	MainWindow::TheInstance()->activatePointPickingImg();
	return true;
}

void ccStichedImageViewer::stopPP(bool state)
{
	if (!m_currentPointCloud || !m_currentTrajectory) { return; }
	if (m_pickingHub)
	{
		//deactivate "point picking mode" in all GL windows
		m_pickingHub->removeListener(this);

		MainWindow::TheInstance()->deactivatePointPickingImg(true);
		ccLog::Print("StopPP");
	}

}
void ccStichedImageViewer::onItemPicked(const PickedItem& pi)
{
	
	if (m_currentPicking && pi.entity)
	{
		//Verify if its the associated point cloud in the selected comboBox
		if (pi.entity->isKindOf(CC_TYPES::POINT_CLOUD) && pi.entity == m_currentPointCloud)
		{

			int id = closestPoseToPoint(m_currentPointCloud->getPoint(pi.itemIndex));
			m_ui->spinBoxNode->setValue(id);
			chooseImageAction();
		}
		else if (pi.entity->isKindOf(CC_TYPES::POINT_CLOUD) )
		{
			ccLog::Error("The point you selected is not part of the Point Cloud under analysis. Please select a point in the appropriate point cloud");
		}
	}
	
}

int ccStichedImageViewer::closestPoseToPoint(const CCVector3* pt)
{
	double minDistance = DBL_MAX;
	int closestId = 0;
	for (int i = 0; i < m_currentTrajectory->size(); i++)
	{
		const CCVector3* ptTrj = m_currentTrajectory->getPoint(i);
		CCVector3 distPt(pt->x - ptTrj->x, pt->y - ptTrj->y, pt->z - ptTrj->z);
		double dist = distPt.norm();
		if (dist < minDistance)
		{
			minDistance = dist;
			closestId = i;
		}
	}
	return closestId; 
}


void ccStichedImageViewer::keyPressEvent(QKeyEvent* event)
{
	if (event->key() == Qt::Key_Escape)
	{
		if (m_currentPicking)
		{

			chooseImageAction();
		}
		return;
	}
	
}




QString ccStichedImageViewer::getFileCSV()
{
	//persistent settings
	QSettings settings;
	settings.beginGroup(ccPS::LoadFile());
	QString currentPath = settings.value(ccPS::CurrentPath(), ccFileUtils::defaultDocPath()).toString();
	QString currentOpenDlgFilter = "*.csv";

	//file choosing dialog
	QString selectedFile = QFileDialog::getOpenFileName(this,
		tr("Open file"),
		currentPath,
		"*.csv",
		&currentOpenDlgFilter);
	currentPath = QFileInfo(selectedFile).absolutePath();
	settings.setValue(ccPS::CurrentPath(), currentPath);
	return selectedFile;
}
QString ccStichedImageViewer::setFileCSV()
{
	//persistent settings
	QSettings settings;
	settings.beginGroup(ccPS::LoadFile());
	QString currentPath = settings.value(ccPS::CurrentPath(), ccFileUtils::defaultDocPath()).toString();
	QString currentOpenDlgFilter = "*.csv";
	QString fullPathName =currentPath + QString("/plan");

	//file choosing dialog
	QString selectedFile = QFileDialog::getSaveFileName(this,
		tr("Save"),
		fullPathName,
		"*.csv",
		&currentOpenDlgFilter);
	currentPath = QFileInfo(selectedFile).absolutePath();
	settings.setValue(ccPS::CurrentPath(), currentPath);
	return selectedFile;
}

bool ccStichedImageViewer::planeToFile()
{
	QString path = setFileCSV();
	if (path.isEmpty()) 
	{
		return false;
	}
	ccHObject* root= MainWindow::TheInstance()->dbRootObject();
	ccHObject::Container plans;
	root->filterChildren(plans, true, CC_TYPES::PLANE);
	//other parameters (dataVersion>=20)

	PointCoordinateType dip_deg;
	PointCoordinateType dipDir_deg;
	QFile file(path);
	file.open(QIODevice::WriteOnly | QFile::Truncate | QIODevice::Append);
	QTextStream outStream(&file);
	
	ccPlane* plane;
	outStream << "Xcenter (Easting)," << "Ycenter (Northing),"  << "Zcenter," << "dip," << "dipDirection" << "\n";
	
	for (int i = 0; i<plans.size();i++)
	{

		plane = ccHObjectCaster::ToPlane(plans.at(i));
		if (plane) 
			if (plane->isEnabled())
			{
				{
					ccNormalVectors::ConvertNormalToDipAndDipDir(plane->getNormal(), dip_deg, dipDir_deg);
					outStream << plane->getCenter().x << "," << plane->getCenter().y << "," << plane->getCenter().z << "," << dip_deg << "," << dipDir_deg << "\n";
				}
			}
	}
	return true;
}


bool ccStichedImageViewer::planeFromFile() 
{
	QString path = getFileCSV();
	if (path.isEmpty())
	{
		return false;
	}
	QFile file(path);
	file.open(QIODevice::ReadOnly | QFile::Text);
	QTextStream stringStream(&file);
	QStringList lineList;
	CCVector3 normals;
	CCVector3 tangent0(1,0,0);
	CCVector3 tangent1(0,1,0);
	ccPlane* plane;

	int i = 0;
	while (!stringStream.atEnd()) 
	{
		// prevent the first row
		QString line = file.readLine();
		if (i != 0)
		{



			 // reads line from file
			lineList = line.split(",");
			// verify if at least 5 information
			if (lineList.size() != 5)
			{
				ccLog::Error(QString("Could not import the %1 plane. Make sure you have the right format \n (X,Y,Z,Dip,Dip_dir) ").arg(QString::number(i)));
			}
			else
			{

				normals = ccNormalVectors::ConvertDipAndDipDirToNormal(lineList[3].toFloat(), lineList[4].toFloat());

				tangent0 = CCVector3(1, 0, 0);
				tangent1 = CCVector3(0, 1, 0);

				tangent0 = normals.cross(tangent0);
				if (tangent0.dot(tangent0) < 0.001)
				{
					tangent0 = tangent0 = normals.cross(tangent1);
				}
				tangent0.normalize();
				tangent1 = normals.cross(tangent0);
				tangent1.normalize();
				ccGLMatrix glMat(tangent0, tangent1, normals, CCVector3(lineList[0].toFloat(), lineList[1].toFloat(), lineList[2].toFloat()));
				plane = new ccPlane(m_ui->spinBoxPlan->value(),m_ui->spinBoxPlan->value(), &glMat);
				plane->setName(QString("plane #: %1").arg(QString::number(i)));
				plane->enableStippling(true);
				plane->showNormals(false);
				MainWindow::TheInstance()->addToDB(plane);
			}
		}
		
		i++;
	}
	return true;
}


void ccStichedImageViewer::setSizePlan()
{
	ccHObject* root = MainWindow::TheInstance()->dbRootObject();
	ccHObject::Container plans;
	ccPlane* plane;
	root->filterChildren(plans, true, CC_TYPES::PLANE);
	for (int i = 0; i < plans.size(); i++)
	{

		plane = ccHObjectCaster::ToPlane(plans.at(i));
		if (plane)
			if (plane->isEnabled())
			{
				plane->setXWidth(m_ui->spinBoxPlan->value());
				plane->setYWidth(m_ui->spinBoxPlan->value());
				plane->redrawDisplay();
				
			}
		
	}
	MainWindow::RefreshAllGLWindow(false);
}


int ccStichedImageViewer::getPlanSize() 
{
	return m_ui->spinBoxPlan->value();
}


void ccStichedImageViewer::setShaftMode()
{
		m_vertical = true;
		m_viewShiftDistance = 10.0;
		m_frontShiftDistance = 10.0;
		m_backShiftDistance = 10.0;
		m_leftShiftDistance = 10.0;
		m_rightShiftDistance = 10.0;
		m_floorShiftDistance = 0.0;
		m_roofShiftDistance = 0.0;
		m_currentFar = 20.0;
		m_frontFar = 20.0;
		m_backFar = 20.0;
		m_leftFar = 20.0;
		m_rightFar = 20.0;
		m_roofFar = 50.0;
		m_floorFar = 50.0;
}


void ccStichedImageViewer::updateNodeId(int id)
{
	if (!m_currentPointCloud || !m_currentTrajectory || id + 1 >= m_currentTrajectory->size() || id < 0) { return; }

	setCameraNodePosition();
	// Stiched image
	updateStichedImage(id);
}


void ccStichedImageViewer::nodeUp()
{
	if (!m_currentPointCloud || !m_currentTrajectory || m_ui->spinBoxNode->value() + 1 >= m_currentTrajectory->size()) { return; }
	if (!m_currentPicking && m_currentTrajectory)
	{
		// Prevents node change when picking the image
		m_ui->spinBoxNode->stepUp();
		int id = m_ui->spinBoxNode->value();
		updateNodeId(id);
	}
}


void ccStichedImageViewer::nodeDown()
{
	if (!m_currentPointCloud || !m_currentTrajectory || m_ui->spinBoxNode->value() == 0) { return; }
	if (!m_currentPicking && m_currentTrajectory)
	{
		// Prevents node change when picking the image
		m_ui->spinBoxNode->stepDown();
		int id = m_ui->spinBoxNode->value();
		updateNodeId(id);
	}
}


void ccStichedImageViewer::changeStepNode(int step)
{
	m_ui->spinBoxNode->setSingleStep(step);
}


void ccStichedImageViewer::setSpinFar()
{
	if (!m_currentPointCloud || !m_currentTrajectory) { return; }
	switch (m_orientation)
	{
	case CC_FRONT_VIEW:
	{
		m_frontFar = (float)m_ui->spinBoxFar->value();
		m_currentFar = m_frontFar;
		break;
	}
	case CC_BACK_VIEW:
	{
		m_backFar = (float)m_ui->spinBoxFar->value();
		m_currentFar = m_backFar;
		break;
	}
	case CC_LEFT_VIEW:
	{
		m_leftFar = (float)m_ui->spinBoxFar->value();
		m_currentFar = m_leftFar;
		break;
	}
	case CC_RIGHT_VIEW:
	{
		m_rightFar = (float)m_ui->spinBoxFar->value();
		m_currentFar = m_rightFar;
		break;
	}
	case CC_TOP_VIEW:
	{
		m_roofFar = (float)m_ui->spinBoxFar->value();
		m_currentFar = m_roofFar;
		break;
	}
	case CC_BOTTOM_VIEW:
	{
		m_floorFar = (float)m_ui->spinBoxFar->value();
		m_currentFar = m_floorFar;
		break;
	}
	}
	setCameraNodePosition();
}


void ccStichedImageViewer::setSpinShift()
{
	if (!m_currentPointCloud || !m_currentTrajectory) { return; }
	switch (m_orientation)
	{
	case CC_FRONT_VIEW:
	{
		m_frontShiftDistance = (double)m_ui->spinBoxOff->value();
		m_viewShiftDistance = m_frontShiftDistance;

		break;
	}
	case CC_BACK_VIEW:
	{
		m_backShiftDistance = (double)m_ui->spinBoxOff->value();
		m_viewShiftDistance = m_backShiftDistance;
		break;
	}
	case CC_LEFT_VIEW:
	{
		m_leftShiftDistance = (double)m_ui->spinBoxOff->value();
		m_viewShiftDistance = m_leftShiftDistance;
		break;
	}
	case CC_RIGHT_VIEW:
	{
		m_rightShiftDistance = (double)m_ui->spinBoxOff->value();
		m_viewShiftDistance = m_rightShiftDistance;
		break;
	}
	case CC_TOP_VIEW:
	{
		m_roofShiftDistance = (double)m_ui->spinBoxOff->value();
		m_viewShiftDistance = m_roofShiftDistance;
		break;
	}
	case CC_BOTTOM_VIEW:
	{
		m_floorShiftDistance = (double)m_ui->spinBoxOff->value();
		m_viewShiftDistance = m_floorShiftDistance;
		break;
	}
	}
	setCameraNodePosition();

}

//  Rotate Cam on click change
//void ccStitchedImageViewer::updateViewport()
//{
//	MainWindow* win = MainWindow::TheInstance();
//	win->getActiveGLWindow()->setViewportParameters(m_viewport);
//}	
void ccStichedImageViewer::changeCamView(CC_VIEW_ORIENTATION orientation)
{
	if (!m_currentPointCloud || !m_currentTrajectory) { return; }

	m_orientation = orientation;
	/*MainWindow* win = MainWindow::TheInstance();
	ccGLMatrixd mat = ccGLUtils::GenerateViewMat(orientation);
	win->getActiveGLWindow()->blockSignals(true);
	win->getActiveGLWindow()->setBaseViewMat(mat);
	win->getActiveGLWindow()->blockSignals(false);
	win->getActiveGLWindow()->redraw();*/

	m_viewport->perspectiveView = true;
	m_viewport->objectCenteredView = false;
	m_viewport->cameraView = true;
	m_viewport->useIntrinsic = false;
	m_viewport->zNear = m_viewShiftDistance;
	m_viewport->zNearCoef = 0.1;
	m_viewport->zFar = m_currentFar + m_viewShiftDistance;

	m_ui->spinBoxFar->setValue((int)m_currentFar);
	m_ui->spinBoxOff->setValue((int)m_viewShiftDistance);

	
	switch (m_orientation)
	{
	case CC_FRONT_VIEW:
	{
		//MainWindow::TheInstance()->doActionUnrollClean();
		
		m_shiftVector = { 0, -1, 0 };
		m_up = { 0,0,1 };
		m_forward = { 0,1,0 };
		break;
	}
	case CC_BACK_VIEW:
	{
		generateImageUnroll(0, 0, 0);

		m_shiftVector = { 0, 1, 0 };
		m_up = { 0,0,1 };
		m_forward = { 0,-1,0 };
		break;
	}
	case CC_LEFT_VIEW:
	{
		m_shiftVector = { -1, 0, 0 };
		m_up = { 0,0,1 };
		m_forward = { 1,0,0 };
		break;
	}
	case CC_RIGHT_VIEW:
	{
		m_shiftVector = { 1, 0, 0 };
		m_up = { 0,0,1 };
		m_forward = { -1,0,0 };
		break;
	}
	case CC_TOP_VIEW:
	{
		m_shiftVector = { 0, 0, 1 };
		m_up = { 0,1,0};
		m_forward = { 0,0,-1 };
		break;
	}
	case CC_BOTTOM_VIEW:
	{
		m_shiftVector = { 0, 0, -1 };
		m_up = { 0,1,0 };
		m_forward = { 0,0,1 };
		break;
	}
	}
	setCameraNodePosition();
}


void ccStichedImageViewer::setCameraNodePosition()
{
	if (!m_currentPointCloud || !m_currentTrajectory) { return; }
	// Relative to the node position
	MainWindow* win = MainWindow::TheInstance();

	int pose = m_ui->spinBoxNode->value();
	const CCVector3* point = m_currentTrajectory->getPoint(pose);
	QString message = QString(" X: %1 Y: %2 Z: %3)").arg(point->x).arg(point->y).arg(point->z);
	MainWindow::GetActiveGLWindow()->displayNewMessage(message, ccGLWindow::LOWER_LEFT_MESSAGE, false, 2, ccGLWindow::SCREEN_SIZE_MESSAGE);


		// Add the shift
		

	CCVector3d pointShifted = (m_viewShiftDistance * m_shiftVector) + CCVector3d(point->x, point->y, point->z);
	m_viewport->setCameraCenter(pointShifted,false);
	m_viewport->fov_deg = 100;
	ccGLMatrixd viewMat = ccGLMatrixd::FromViewDirAndUpDir(m_forward, m_up);
	m_viewport->viewMat = viewMat;
	win->getActiveGLWindow()->setViewportParameters(*m_viewport);
	win->getActiveGLWindow()->redraw();
	
	//set custom light postion
	//win->getActiveGLWindow()->setCustomLightPosition((float)pointShifted->x, (float)pointShifted->y, (float)pointShifted->z);
	//win->getActiveGLWindow()->blockSignals(true);
	//win->getActiveGLWindow()->setCameraPos(pointShifted);
	//win->getActiveGLWindow()->blockSignals(false);
	//win->getActiveGLWindow()->redraw();

	//setCameraOrientation();

}


void ccStichedImageViewer::generateImageUnroll(float zMax, float zMin, float color)
{
	if (!m_unrolledCloud) { return; }
	//Generate a new view
	ccViewportParameters imageViewport;
	CCVector3 bbMin;
	CCVector3 bbMax;
	m_unrolledCloud->getBoundingBox(bbMin, bbMax);
	bbMin.y += 15;
	bbMax.y -= 15;

	ccGLWindow* win = MainWindow::GetActiveGLWindow();
	float windowHeight = (float)win->glHeight();
	float windowWidth = (float)win->glWidth();
	//double targetWidth = sqrt(pow((bbMax-bbMin).x,2)+ pow((bbMax - bbMin).y, 2));
	double targetWidth = (bbMax-bbMin).x;
	double targetHeight = (bbMax-bbMin).y;
	double targetRatio = targetHeight/targetWidth;
	double windowRatio = windowHeight / windowWidth;
	ccLog::Error(QString("Target Height: %1 ratioTarget: %2 ratioWindow: %3").arg(targetHeight).arg(targetRatio).arg(windowRatio));

	//Center image
	double focalDistance;
	CCVector3d eye(0, 0, 1);
	CCVector3d center(0, 0, 0);
	CCVector3d top(0, 1, 0);
	imageViewport.viewMat = ccGLMatrixd::FromViewDirAndUpDir(center - eye, top);
	CCVector3d P((bbMax + bbMin).x / 2, (bbMax + bbMin).y / 2, 0);
	imageViewport.setCameraCenter(P, false);
	QImage image;

	if (targetRatio > windowRatio)
	{
		//Optimizing Height
		double zoom = targetHeight * 200 / windowHeight;
		ccLog::Error(QString::number(zoom));

		if (windowHeight < windowWidth)
		{
			targetHeight *= static_cast<double>(windowWidth / windowHeight);
			focalDistance = targetHeight / (imageViewport.computeDistanceToWidthRatio());
		}
		else
		{
			double distance = (windowWidth / 2)/ std::tan(CCCoreLib::DegreesToRadians(imageViewport.fov_deg / 2.0));
			double newFOV = atan((windowHeight / 2) / distance);
			ccLog::Print(QString::number(newFOV));
			//double multipleHeight = targetHeight / windowHeight;
			targetHeight *= static_cast<double>(windowWidth / windowHeight);
			focalDistance = targetHeight / (2*std::tan(newFOV));
		}
	
		imageViewport.setFocalDistance(focalDistance);
		win->setViewportParameters(imageViewport);
		win->redraw();
		
		image = win->renderToImage(zoom, true, false, true);
		//ccLog::Error(QString("Image Height: %1 focal: %2").arg(image.height()).arg(focalDistance));
		// Image should be adapted to the height
		qreal imgHeight = image.height();
		qreal theoricalWidth = imgHeight / targetRatio;
		qreal crop = (image.width() - theoricalWidth) / 2;
		QRect rect(crop, 0, theoricalWidth, imgHeight);
		image = image.copy(rect);
	}
	else
	{
		//Optimizing Width
		double zoom = targetWidth * 200 / windowWidth;
		ccLog::Error(QString::number(zoom));

		if (windowHeight > windowWidth)
		{
			targetWidth *= static_cast<double>(windowWidth / windowHeight);
		}
		
		focalDistance = targetWidth / (imageViewport.computeDistanceToWidthRatio());
		
		imageViewport.setFocalDistance(focalDistance);
		win->setViewportParameters(imageViewport);
		win->redraw();
		
		image = win->renderToImage(zoom, true, false, true);

		// Image should be adapted to the width
		qreal imgWidth = image.width();
		qreal theoricalHeight = imgWidth * targetRatio;
		qreal crop = (image.height() - theoricalHeight) / 2;
		QRect rect(0, crop, imgWidth, theoricalHeight);
		image = image.copy(rect);
	}

	image.save("C:/Users/Asus/Pictures/test.png");
	//win->setCameraPos(P);
	//CCVector3d v(0, 0, focalDistance);
	//win->moveCamera(v);

}

void ccStichedImageViewer::saveDxf(QImage img) 
{

}
void ccStichedImageViewer::unrollClick() 
{
	if (!m_currentPointCloud)
	{
		ccLog::Error("Could not unroll no point cloud is associated");
		return;
	}

	if (m_unrolledCloud)
	{
		MainWindow::TheInstance()->removeFromDB(m_unrolledCloud);
		m_unrolledCloud = nullptr;
	}
	MainWindow::TheInstance()->doActionUnrollClean();
}

void ccStichedImageViewer::actionUnroll(ccPointCloud* currentCloud, ccPointCloud* &outCloudUnrolled,
											float radius, CCVector3 center, bool exportDistance,
											CCCoreLib::GenericProgressCallback* progressCb)
{	
	ccPointCloud::UnrollCylinderParams params;
	params.axisDim = 2;
	params.center = center;
	params.radius = radius;

	outCloudUnrolled = currentCloud->unroll(ccPointCloud::UnrollMode::CYLINDER, &params, exportDistance, 0, 360,progressCb);
	// Apply sf to oldCloud
	if (exportDistance)
	{
		int sfIdxUnrolled = outCloudUnrolled->getScalarFieldIndexByName("Deviation");
		int sfIdxCloud = currentCloud->addScalarField("Deviation");
		if (sfIdxCloud < 0 || sfIdxUnrolled < 0)
		{
			ccLog::Warning("[Unroll] Not enough memory to init the deviation scalar field");
		}
		
		else
		{
			CCCoreLib::ScalarField* deviationSFUnrolled = outCloudUnrolled->getScalarField(sfIdxUnrolled);
			CCCoreLib::ScalarField* deviationSFCloud = currentCloud->getScalarField(sfIdxCloud);
			for (int i = 0; i < currentCloud->size(); i++)
			{
				deviationSFCloud->setValue(i, deviationSFUnrolled->at(i));
				
			}
			deviationSFCloud->computeMinAndMax();
			ccScalarField* sfU = static_cast<ccScalarField*>(deviationSFUnrolled);
			ccScalarField* sfC = static_cast<ccScalarField*>(deviationSFCloud);
			sfU->setColorScale(ccColorScalesManager::GetDefaultScale(ccColorScalesManager::CONVERGENCE));
			sfC->setColorScale(ccColorScalesManager::GetDefaultScale(ccColorScalesManager::CONVERGENCE));
			
		}
	}

}

void ccStichedImageViewer::cleanUnrollOctree(int octreeLevel, float radius, ccPointCloud* currentCloud , ccPointCloud* unrolledCloud,
											ccPointCloud* &outCleanUnrolled, ccPointCloud* &outCloudClean, ccPointCloud* &outCloudRemaining,
											CCCoreLib::GenericProgressCallback* progressCb)
{
	//ccPointCloud* unrolledCloudClean(new ccPointCloud);
	//ccPointCloud* cloudClean(new ccPointCloud);
	//ccPointCloud* cloudRemaining(new ccPointCloud);
	//unique parameter: the octree subdivision level

	assert(octreeLevel >= 0 && octreeLevel <= CCCoreLib::DgmOctree::MAX_OCTREE_LEVEL);
	if (progressCb)
	{
		progressCb->setMethodTitle("Cleaning Cloud");
		progressCb->setInfo(qPrintable(QString("Number of points = %1").arg(octreeLevel)));
	}
	

	ccOctree::Shared theOctree = unrolledCloud->getOctree();
	if (!theOctree)
	{
		theOctree = unrolledCloud->computeOctree();
	}
	
	if (!theOctree)
	{
		ccLog::Error("Couldn't compute octree!");
		return;
	}

	ccOctree::Shared cloudOctree = currentCloud->getOctree();
	if (!cloudOctree)
	{
		cloudOctree = currentCloud->computeOctree();
	}

	if (!cloudOctree)
	{
		ccLog::Error("Couldn't compute octree!");
		return;
	}

	
	CCVector3 bbMin;
	CCVector3 bbMax;
	theOctree.data()->getBoundingBox(bbMin, bbMax);
	
	// New Approach
	
	float leafSize = theOctree.data()->getCellSize(octreeLevel);
	
	CCCoreLib::DgmOctree::cellCodesContainer cellCodes;
	int octreeNumber = theOctree.data()->getCellNumber(octreeLevel);
	try
	{
		cellCodes.reserve(octreeNumber);
	}
	catch (const std::bad_alloc&)
	{
		ccLog::Error("Not enough memory");
		return;
	}
		
	//ccLog::Error(QString("Qty of cells %1").arg(OctreeNumber));
	// Get cell codes
	theOctree.data()->getCellCodes(octreeLevel, cellCodes, false);
	

	std::vector < std::vector<std::pair<unsigned long long, int>>> zDepthMat;
	std::pair<unsigned long long, int> initPair = std::make_pair(0, INT_MAX);
	int zDepthMatWidth = (int)round((bbMax.x - bbMin.x) / leafSize);
	int zDepthMatHeight = (int)round((bbMax.y - bbMin.y) / leafSize);
	zDepthMat.resize(zDepthMatHeight);
	for (int i = 0; i < zDepthMatHeight; i++)
	{
		zDepthMat[i].resize(zDepthMatWidth);
		std::fill(zDepthMat[i].begin(), zDepthMat[i].end(), initPair);
	}

	CCCoreLib::NormalizedProgress nprogress(progressCb, cellCodes.size());
	if (progressCb)
	{
		if (progressCb->textCanBeEdited())
		{
			progressCb->setInfo(qPrintable(QString("Number of cells = %1").arg(cellCodes.size())));
		}
		progressCb->update(0);
		progressCb->start();
	}

	// Keeping the smallestZ cell in the 2D grid
	for (int i = 0; i < cellCodes.size(); i++)
	{
		unsigned long long cellCode = cellCodes[i];
		Tuple3i pos;
		theOctree->getCellPos(cellCode, octreeLevel, pos, false);

		unsigned long long savedCellCode = zDepthMat[pos.y][pos.x].first;
		if (savedCellCode == 0)
		{
			zDepthMat[pos.y][pos.x].first = cellCode;
			zDepthMat[pos.y][pos.x].second = pos.z;
		}
		else
		{
			if (zDepthMat[pos.y][pos.x].second > pos.z)
			{
				zDepthMat[pos.y][pos.x].first = cellCode;
				zDepthMat[pos.y][pos.x].second = pos.z;
			}
		}
		if (progressCb && !nprogress.oneStep())
		{
			ccLog::Warning("Process cancelled by user");
			return;
		}
	}

	
	

	int max = pow(2, octreeLevel);
	std::vector<unsigned long long> cellClean;
	std::vector<unsigned long long> cellHidden;

	nprogress.reset();
	if (progressCb)
	{
		if (progressCb->textCanBeEdited())
		{
			progressCb->setInfo(qPrintable(QString("Number of cells visible = %1").arg(cellCodes.size())));
		}
		progressCb->update(0);
		progressCb->start();
	}
	// Adding surrounding and visible octrees 
	for (int i = 0; i < cellCodes.size(); i++)
	{
		unsigned long long cellCode = cellCodes[i];
		Tuple3i pos;
		theOctree->getCellPos(cellCode, octreeLevel, pos, false);
		
		
		int savedZPos = zDepthMat[pos.y][pos.x].second;

		//getNeighborCellsAround
		if (savedZPos == pos.z ||
			// middle
			zDepthMat[pos.y+1 >= max ? max : pos.y + 1][pos.x].second == pos.z ||
			zDepthMat[pos.y][pos.x + 1 >= max ? max : pos.x + 1 ].second == pos.z ||
			zDepthMat[pos.y + 1 >= max ? max : pos.y + 1][pos.x + 1 >= max ? max : pos.x + 1].second == pos.z ||
			zDepthMat[pos.y - 1 <= 0 ? 0 : pos.y - 1][pos.x].second == pos.z ||
			zDepthMat[pos.y][pos.x - 1 <= 0 ? 0 : pos.x - 1].second == pos.z ||
			zDepthMat[pos.y - 1 <= 0 ? 0 : pos.y - 1][pos.x - 1 <= 0 ? 0 : pos.x - 1].second == pos.z ||
			//back
			savedZPos == pos.z - 1 ||
			savedZPos == pos.z - 2 ||
			zDepthMat[pos.y + 1 >= max ? max : pos.y + 1][pos.x].second == pos.z - 1 ||
			zDepthMat[pos.y][pos.x + 1 >= max ? max : pos.x + 1].second == pos.z - 1 ||
			zDepthMat[pos.y + 1 >= max ? max : pos.y + 1][pos.x + 1 >= max ? max : pos.x + 1].second == pos.z - 1 ||
			zDepthMat[pos.y - 1 <= 0 ? 0 : pos.y - 1][pos.x].second == pos.z - 1 ||
			zDepthMat[pos.y][pos.x - 1 <= 0 ? 0 : pos.x - 1].second == pos.z - 1 ||
			zDepthMat[pos.y - 1 <= 0 ? 0 : pos.y - 1][pos.x - 1 <= 0 ? 0 : pos.x - 1].second == pos.z - 1 ||
			//front
			savedZPos == pos.z + 1 ||
			savedZPos == pos.z + 2 ||
			zDepthMat[pos.y + 1 >= max ? max : pos.y + 1][pos.x].second == pos.z + 1 ||
			zDepthMat[pos.y][pos.x + 1 >= max ? max : pos.x + 1].second == pos.z + 1 ||
			zDepthMat[pos.y + 1 >= max ? max : pos.y + 1][pos.x + 1 >= max ? max : pos.x + 1].second == pos.z + 1 ||
			zDepthMat[pos.y - 1 <= 0 ? 0 : pos.y - 1][pos.x].second == pos.z + 1 ||
			zDepthMat[pos.y][pos.x - 1 <= 0 ? 0 : pos.x - 1].second == pos.z + 1 ||
			zDepthMat[pos.y - 1 <= 0 ? 0 : pos.y - 1][pos.x - 1 <= 0 ? 0 : pos.x - 1].second == pos.z + 1 			
			)
		{
			cellClean.push_back(cellCode);
		}
		else
		{
			cellHidden.push_back(cellCode);	
		}
		if (progressCb && !nprogress.oneStep())
		{
			ccLog::Warning("Process cancelled by user");
			return;
		}
	}
	if (progressCb)
	{
		progressCb->stop();
	}

	CCCoreLib::ReferenceCloud unrolledRefCloud(unrolledCloud);
	CCCoreLib::ReferenceCloud unrolledObstructRefCloud(unrolledCloud);
	CCCoreLib::ReferenceCloud visibleRefCloud(currentCloud);
	CCCoreLib::ReferenceCloud obstructRefCloud(currentCloud);
	
	theOctree->getPointsInCellsWithSortedCellCodes(cellClean, octreeLevel, &unrolledRefCloud);
	for (int i = 0; i < unrolledRefCloud.size(); i++)
	{
		int index = unrolledRefCloud.getPointGlobalIndex(i);
		visibleRefCloud.addPointIndex(index);
	}

	theOctree->getPointsInCellsWithSortedCellCodes(cellHidden, octreeLevel, &unrolledObstructRefCloud);
	for (int i = 0; i < unrolledObstructRefCloud.size(); i++)
	{
		int index = unrolledObstructRefCloud.getPointGlobalIndex(i);
		obstructRefCloud.addPointIndex(index);
	}

		if (visibleRefCloud.size() == unrolledCloud->size())
		{
			ccLog::Error("No points were removed!");
		}
		else
		{
			//create cloud from visibility selection
			outCleanUnrolled = unrolledCloud->partialClone(&unrolledRefCloud);
			unrolledRefCloud.clear();
			unrolledCloud->clear();
			if (!outCleanUnrolled)
			{
				ccLog::Error("Not enough memory!");
				return;
			}
			
			


			QString nameCloud = currentCloud->getName();
			outCloudClean = currentCloud->partialClone(&visibleRefCloud);
			outCloudClean->setImagePointCloud(true);
			outCloudClean->setTrajectoryCloud(m_currentTrajectory);
			outCloudClean->setName(QString("%1.clean").arg(nameCloud));
			visibleRefCloud.clear();

			if (!outCloudClean)
			{
				ccLog::Error("Not enough memory!");
				return;
			}

			outCloudRemaining = currentCloud->partialClone(&obstructRefCloud);
			outCloudRemaining->setImagePointCloud(true);
			outCloudRemaining->setTrajectoryCloud(m_currentTrajectory);
			outCloudRemaining->setName(QString("%1.remaining").arg(nameCloud));
			obstructRefCloud.clear();
			if (!outCloudRemaining)
			{
				ccLog::Error("Not enough memory!");
				return;
			}
			currentCloud->clear();
		}
	
	return;
}

CCCoreLib::ReferenceCloud * ccStichedImageViewer::removeHiddenPoints(CCCoreLib::GenericIndexedCloudPersist * theCloud, CCVector3 bbMin, CCVector3 bbMax, float leafSize)
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

	// Create a 2D matrix

	//unrolledCloud->getBoundingBox(bbMin, bbMax);
	int zDepthMatWidth = (int)ceil((bbMax.x - bbMin.x) / leafSize);
	int zDepthMatHeight = (int)ceil((bbMax.y - bbMin.y) / leafSize);
	std::vector<std::vector<int>> zDepthMat;
	zDepthMat.resize(zDepthMatHeight);
	for (int i = 0; i < zDepthMatHeight; i++)
	{
		zDepthMat[i].resize(zDepthMatWidth);
		std::fill(zDepthMat[i].begin(), zDepthMat[i].end(), -1);
	}

	// Iterate throught all the center points and update if the leaf size is closer 
	int visibleSize = 0;
	for (int i = 0; i < nbPoints; ++i)
	{
		float Px = theCloud->getPoint(i)->x - bbMin.x;
		float Py = theCloud->getPoint(i)->y - bbMin.y;
		float Pz = theCloud->getPoint(i)->z;
		int row = (int)floor(Py / leafSize);
		int column = (int)floor(Px / leafSize);
		int ptIdLocked = zDepthMat[row][column];
		if (ptIdLocked == -1)
		{
			zDepthMat[row][column] = i;
			visibleSize++;
		}
		else
		{
			float lockedZ = theCloud->getPoint(ptIdLocked)->z;
			if (lockedZ < Pz)
			{
				// We update the id because the depth is smaller
				zDepthMat[row][column] = i;
			}
		}
	}
	//ccLog::Error(QString("Visible size %1").arg(visibleSize));
	int gg = 0;
	CCCoreLib::ReferenceCloud* visiblePoints = new CCCoreLib::ReferenceCloud(theCloud);
	if (visibleSize != 0 && visiblePoints->reserve(visibleSize))
	{
		for (int row = 0; row < zDepthMatHeight; ++row)
		{
			for (int col = 0; col < zDepthMatWidth; ++col)
			{
				int visibleIndex = zDepthMat[row][col];
				if (visibleIndex != -1)
				{
					visiblePoints->addPointIndex(visibleIndex);
					gg++;
				}
			}
		}
		//ccLog::Error(QString("Visiblepoints size %1").arg(gg));;
		return visiblePoints;
	}
	else //not enough memory
	{
		delete visiblePoints;
		visiblePoints = nullptr;
	}
}

void ccStichedImageViewer::rollingPoint(CCVector3 &point, float radius)
{
	float circumference = 2 * radius * M_PI;
	float angle = point.x/(circumference/2) ;
	float z = point.y;

	point.x = -point.z * sin(angle);
	point.y = -point.z * cos(angle);
	point.z = z; 
}
