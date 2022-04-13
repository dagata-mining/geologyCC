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

#include "DxfFilter.h"

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
#include <QJsonObject.h>
#include <QJsonArray.h>
#include <QJsonDocument.h>


//system
#include <cassert>
#include <fstream>
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
	, m_radius(1.0)
	, m_center(0,0,0)
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
	//DxfFilter dxf;
	//dxf.saveToFileShaftTest();
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
		saveDxf();
		
		m_shiftVector = { 0, -1, 0 };
		m_up = { 0,0,1 };
		m_forward = { 0,1,0 };
		break;
	}
	case CC_BACK_VIEW:
	{
		

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


ccImage* ccStichedImageViewer::generateImageUnroll(float zMax, float zMin)
{

	if (!m_unrolledCloud) { return nullptr; }
	//Generate a new view
	ccViewportParameters imageViewport;
	CCVector3 bbMin;
	CCVector3 bbMax;
	ccImage* img(new ccImage);
	QImage image;
	m_unrolledCloud->getBoundingBox(bbMin, bbMax);

	ccGLWindow* win = MainWindow::GetActiveGLWindow();
	float windowHeight = (float)win->glHeight();
	float windowWidth = (float)win->glWidth();
	//double targetWidth = sqrt(pow((bbMax-bbMin).x,2)+ pow((bbMax - bbMin).y, 2));
	double targetWidth = (bbMax-bbMin).x;
	double targetHeight = (bbMax-bbMin).y;
	double targetRatio = targetHeight/targetWidth;
	double windowRatio = windowHeight / windowWidth;
	//ccLog::Error(QString("Target Height: %1 ratioTarget: %2 ratioWindow: %3").arg(targetHeight).arg(targetRatio).arg(windowRatio));
	
	//Center image
	double focalDistance;
	CCVector3d eye(0, 0, 1);
	CCVector3d center(0, 0, 0);
	CCVector3d top(0, 1, 0);
	imageViewport.viewMat = ccGLMatrixd::FromViewDirAndUpDir(center - eye, top);
	CCVector3d P((bbMax + bbMin).x / 2, (bbMax + bbMin).y / 2, 0);
	ccLog::Print(QString("Center Camera %1, %2, %3").arg(P.x).arg(P.y).arg(P.z));
	imageViewport.setCameraCenter(P, false);


	if (targetRatio > windowRatio)
	{
		//Optimizing Height
		double zoom = targetHeight * 200 / windowHeight;

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
		
		while (image.isNull())
		{
			image = win->renderToImage(zoom, true, false, true);
			if (zoom < 1)
			{
				ccLog::Error("Could not export Bg Image, due to momory shortage");
				return nullptr;
			}
			if (image.isNull())
			{
				zoom -= 1;
			}
		}
		ccLog::Print(QString("Image created at zoom of %1").arg(zoom));

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

		if (windowHeight > windowWidth)
		{
			targetWidth *= static_cast<double>(windowWidth / windowHeight);
		}
		
		focalDistance = targetWidth / (imageViewport.computeDistanceToWidthRatio());
		
		imageViewport.setFocalDistance(focalDistance);
		win->setViewportParameters(imageViewport);
		win->redraw();
		
		while (image.isNull())
		{
			image = win->renderToImage(zoom, true, false, true);
			zoom -= 1;
			if (zoom < 1)
			{
				ccLog::Error("Could not export Bg Image, due to momory shortage");
				return nullptr;
			}
			if (image.isNull())
			{
				zoom -= 1;
			}
		}
		ccLog::Print(QString("Image created at zoom of %1").arg(zoom));
		
		// Image should be adapted to the width
		qreal imgWidth = image.width();
		qreal theoricalHeight = imgWidth * targetRatio;
		qreal crop = (image.height() - theoricalHeight) / 2;
		QRect rect(0, crop, imgWidth, theoricalHeight);
		image = image.copy(rect);
	}
	img->setData(image);
	bbMin = CCVector3(bbMin.x, bbMin.y, 0);
	bbMax = CCVector3(bbMax.x, bbMax.y, 0);
	ccBBox box(bbMin,bbMax);
	img->setPositionBox(box);
	return img;

	//win->setCameraPos(P);
	//CCVector3d v(0, 0, focalDistance);
	//win->moveCamera(v);

}

std::vector<ccImage*> ccStichedImageViewer::generateImagesUnroll(float zMax, float zMin, float slice)
{
	std::vector<ccImage*> imagesOutput;
	if (!m_unrolledCloud) { return imagesOutput; }
	
	//get init slice
	float initY = floor(zMin / slice) * slice;
	while (initY < zMax)
	{
		float bottomY = initY;
		float topY = initY + slice;
		//Generate a new view
		ccViewportParameters imageViewport;
		CCVector3 bbMin;
		CCVector3 bbMax;
		ccImage* img(new ccImage);
		QImage image;
		m_unrolledCloud->getBoundingBox(bbMin, bbMax);
		bbMin.y = bottomY;
		bbMax.y = topY;

		ccGLWindow* win = MainWindow::GetActiveGLWindow();
		float windowHeight = (float)win->glHeight();
		float windowWidth = (float)win->glWidth();
		//double targetWidth = sqrt(pow((bbMax-bbMin).x,2)+ pow((bbMax - bbMin).y, 2));
		double targetWidth = (bbMax - bbMin).x;
		double targetHeight = (bbMax - bbMin).y;
		double targetRatio = targetHeight / targetWidth;
		double windowRatio = windowHeight / windowWidth;
		//ccLog::Error(QString("Target Height: %1 ratioTarget: %2 ratioWindow: %3").arg(targetHeight).arg(targetRatio).arg(windowRatio));

		//Center image
		double focalDistance;
		CCVector3d eye(0, 0, 1);
		CCVector3d center(0, 0, 0);
		CCVector3d top(0, 1, 0);
		imageViewport.viewMat = ccGLMatrixd::FromViewDirAndUpDir(center - eye, top);
		CCVector3d P((bbMax + bbMin).x / 2, (bbMax + bbMin).y / 2, 0);
		ccLog::Print(QString("Center Camera %1, %2, %3").arg(P.x).arg(P.y).arg(P.z));
		imageViewport.setCameraCenter(P, false);
		
		if (targetRatio > windowRatio)
		{
			//Optimizing Height
			double zoom = targetHeight * 200 / windowHeight;

			if (windowHeight < windowWidth)
			{
				targetHeight *= static_cast<double>(windowWidth / windowHeight);
				focalDistance = targetHeight / (imageViewport.computeDistanceToWidthRatio());
			}
			else
			{
				double distance = (windowWidth / 2) / std::tan(CCCoreLib::DegreesToRadians(imageViewport.fov_deg / 2.0));
				double newFOV = atan((windowHeight / 2) / distance);
				ccLog::Print(QString::number(newFOV));
				//double multipleHeight = targetHeight / windowHeight;
				targetHeight *= static_cast<double>(windowWidth / windowHeight);
				focalDistance = targetHeight / (2 * std::tan(newFOV));
			}

			imageViewport.setFocalDistance(focalDistance);
			win->setViewportParameters(imageViewport);
			win->setPointSize(2);
			win->redraw();

			while (image.isNull())
			{
				image = win->renderToImage(zoom, true, false, true);
				if (zoom < 1)
				{
					ccLog::Error("Could not export Bg Image, due to momory shortage");
					return imagesOutput;
				}
				if (image.isNull())
				{
					zoom -= 1;
				}
			}
			ccLog::Print(QString("Image created at zoom of %1").arg(zoom));

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

			if (windowHeight > windowWidth)
			{
				targetWidth *= static_cast<double>(windowWidth / windowHeight);
			}

			focalDistance = targetWidth / (imageViewport.computeDistanceToWidthRatio());

			imageViewport.setFocalDistance(focalDistance);
			win->setViewportParameters(imageViewport);
			win->setPointSize(2);
			win->redraw();

			while (image.isNull())
			{
				image = win->renderToImage(zoom, true, false, true);
				zoom -= 1;
				if (zoom < 1)
				{
					ccLog::Error("Could not export Bg Image, due to momory shortage");
					return imagesOutput;
				}
				if (image.isNull())
				{
					zoom -= 1;
				}
			}
			ccLog::Print(QString("Image created at zoom of %1").arg(zoom));

			// Image should be adapted to the width
			qreal imgWidth = image.width();
			qreal theoricalHeight = imgWidth * targetRatio;
			qreal crop = (image.height() - theoricalHeight) / 2;
			QRect rect(0, crop, imgWidth, theoricalHeight);
			image = image.copy(rect);
		}
		img->setData(image);
		bbMin = CCVector3(bbMin.x, bbMin.y, 0);
		bbMax = CCVector3(bbMax.x, bbMax.y, 0);
		ccBBox box(bbMin, bbMax);
		img->setPositionBox(box);
		imagesOutput.push_back(img);
		
		// Iterate
		initY += slice;
	}
	
	return imagesOutput;

	//win->setCameraPos(P);
	//CCVector3d v(0, 0, focalDistance);
	//win->moveCamera(v);

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
	return;
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

void ccStichedImageViewer::saveDxf()
{
	if (!m_unrolledCloud)
	{
		ccLog::Error("Could not be save because there are no unrolled cloud available");
		return;
	}
	//persistent settings
	QSettings settings;
	settings.beginGroup(ccPS::LoadFile());
	QString currentPath = settings.value(ccPS::CurrentPath(), ccFileUtils::defaultDocPath()).toString();
	QString currentOpenDlgFilter = "*.dxf";
	QString fullPathName = currentPath + QString("/shaft");

	//file choosing dialog
	QString selectedFile = QFileDialog::getSaveFileName(this,
		tr("Save"),
		fullPathName,
		"*.dxf",
		&currentOpenDlgFilter);

	currentPath = QFileInfo(selectedFile).absolutePath();
	QString currentName = QFileInfo(selectedFile).baseName();
	QString jsonFilePath = currentPath + QString("/") + currentName + QString(".json");

	settings.setValue(ccPS::CurrentPath(), currentPath);

	// Output file
	//selectedFile;

	// Generate images unrolled
	// hide everthing else first
	ccHObject* root = MainWindow::TheInstance()->dbRootObject();
	root->toggleVisibility_recursive();
	m_unrolledCloud->setEnabled(true);
	m_unrolledCloud->setVisible(true);
	
	CCVector3 bbMin, bbMax;
	m_unrolledCloud->getBoundingBox(bbMin, bbMax);
	std::vector<std::vector<ccImage*>> bgImages;
	for (int i = 0; i < m_unrolledCloud->getNumberOfScalarFields(); i++)
	{
		m_unrolledCloud->setCurrentDisplayedScalarField(i);
		
		std::vector<ccImage*> images = generateImagesUnroll(bbMax.y, bbMin.y,10);
		if (!images.empty())
		{
			bgImages.push_back(images);
		}
	}
	m_unrolledCloud->setEnabled(false);
	m_unrolledCloud->setVisible(false);
	root->toggleVisibility_recursive();

	// Query to get all crack images
	ccHObject::Container filteredChildrenPolyline, filteredChildrenImage;
	int polyObjSize = m_imageDrawer->getPolylineObject()->filterChildren(filteredChildrenPolyline, true, CC_TYPES::POLY_LINE, true);
	int imageObjSize = m_imageDrawer->getPolylineObject()->filterChildren(filteredChildrenImage, true, CC_TYPES::IMAGE, true);
	std::vector<std::pair<ccPolyline*, ccImage*>> crackPairs;

	for (int i = 0; i < polyObjSize; i++)
	{
		ccImage* img = nullptr;
		ccPolyline* poly = static_cast<ccPolyline*>(filteredChildrenPolyline[i]);
		QString polyName = poly->getName();
		QString polyId = polyName.split("_").takeLast();
		bool polyHasImg = false;
		
		for (int j = 0; j < imageObjSize; j++)
		{
			QString imgName = filteredChildrenImage[j]->getName();
			QString imgId = imgName.split("_").takeLast();
			if (imgId == polyId)
			{
				polyHasImg = true;
				img = static_cast<ccImage*>(filteredChildrenImage[j]);
				break;
			}
		}
		crackPairs.push_back(std::make_pair(poly, img));
	}
	if (crackPairs.empty()) 
	{
		ccLog::Error("No cracks available, saving to .dxf cancelled");
		return;
	}



	// Unroll cracks
	float radius = m_radius;
	float circumference = 2 * M_PI * radius;
	CCVector3 center = m_center;;
	
	for (int i = 0; i < crackPairs.size(); i++)
	{
		
		ccPointCloud* crackCloud(new ccPointCloud); 
		crackCloud->reserve(crackPairs[i].first->size());
		ccPolyline* unrolledCrack(new ccPolyline(crackCloud));
		for (int j = 0; j < crackPairs[i].first->size(); j++)
		{
			const CCVector3 *pt = crackPairs[i].first->getPoint(j);
			CCVector3 transPt = *pt - center;
			float depth = sqrt(transPt.x * transPt.x + transPt.y *transPt.y);
			//ProjectOnCylinder(AP, dim, params->radius, delta, longitude_rad);
			float angle = atan2(transPt.x, transPt.y);
			float xCylinder = (angle / M_PI) * (circumference / 2);
			
			transPt.x = xCylinder; //x
			transPt.y = transPt.z;//y
			transPt.z = -depth;//z
			crackCloud->addPoint(transPt);
			unrolledCrack->addPointIndex(j);
			//ccLog::Print(QString("unrolled crack: %1, %2, %3").arg(transPt.x).arg(transPt.y).arg(transPt.z));
		}
		
		crackPairs[i].first = unrolledCrack;
	}



	// sorting the vectors
	std::vector<float> depthSort;
	for (int i = 0; i < crackPairs.size(); i++)
	{
		CCVector3 ccMax = crackPairs[i].first->getBB_recursive().maxCorner();
		depthSort.push_back(ccMax.y);
		
	}
	auto p = sort_permutation(depthSort,
		[](float const& a, float const& b) { return a > b; });


	//Sort cracks and crack images
	crackPairs = apply_permutation(crackPairs, p);
	depthSort = apply_permutation(depthSort, p);

	// Creating linkers and positionning crack images
	std::vector<std::vector<CCVector3>> linkers;
	ccStichedImageViewer::generateLinkers(crackPairs, linkers, 1.0, 0.1);

	// Creating Grids
	std::vector<std::pair<CCVector3, CCVector3>> lines;
	std::vector<std::pair<CCVector3, QString>> texts;
	annotation(bbMin.y, bbMax.y, m_radius, 0.5, 10, lines, texts);
		
	//Layers
	QJsonObject layersNode;
	
	// Add both the bg images and the cracks as layers
	QStringList layersBgImages;
	for (int i = 0; i < bgImages.size(); i++)
	{
		QString  layerName = QString("shaftImage_%1").arg(i);
		layersBgImages.push_back(layerName);
		QJsonObject layer;
		layer["color"] = "";
		layer["type"] = "";
		layer["width"] = "";
		layer["locked"] = "1";
		layersNode[layerName] = layer;
	}
	
	QStringList layersCrack;
	for (int i = 0; i < crackPairs.size(); i++)
	{
		QString layerName = QString("cracks_%1").arg(i);
		layersCrack.push_back(layerName);
		QJsonObject layer;
		layer["color"] = "red";
		layer["type"] = "";
		layer["width"] = "";
		layer["locked"] = "0";
		layersNode[layerName] = layer;
		
	}
	// Grid Layer
	{
		QString layerName = QString("Grid");
		QJsonObject layer;
		layer["color"] = "light_grey";
		layer["type"] = "";
		layer["width"] = "";
		layer["locked"] = "1";
		layersNode[layerName] = layer;
	}

	// Add the images to yaml and write them also 
	QString folderImageAbsolutePath = currentPath + QString("/") + currentName;
	QDir dir(folderImageAbsolutePath);
	dir.mkdir(folderImageAbsolutePath);

	QJsonObject imagesNode;
	for (int i = 0; i < bgImages.size(); i++)
	{
		QString layerName = layersBgImages[i];
		std::vector<ccImage*> croppedImages = bgImages[i];
		for (int j = 0; j < croppedImages.size(); j++)
		{
			QJsonObject image;
			QString imageName = QString("bgImage_%1_%2").arg(i).arg(j);
			ccImage* img = croppedImages[j];
			CCVector3 minC;
			CCVector3 maxC;
			minC = img->getPositionBox().minCorner();
			maxC = img->getPositionBox().maxCorner();

			image["layerName"] = layerName;
			image["widthPx"] = QString::number(img->getImage().width());
			image["heightPx"] = QString::number(img->getImage().height());
			image["widthM"] = QString::number(maxC.x - minC.x);
			image["heightM"] = QString::number(maxC.y - minC.y);
			// position left bottom
			image["xlb"] = QString::number(minC.x);
			image["ylb"] = QString::number(minC.y);
			image["zlb"] = QString::number(0);
			image["relPath"] = QString("./" + currentName + "/" + imageName + ".png");

			
			imagesNode[imageName] = image;
			// Export the image in a file
			//ccLog::Error(QString("Saving Img to:%1").arg(folderImageAbsolutePath + QString("/") + imageName + QString(".png")));
			img->getImage().save(folderImageAbsolutePath + QString("/") + imageName + QString(".png"));
		}
		
	}

	for (int i = 0; i < crackPairs.size(); i++)
	{
		if (crackPairs[i].second)
		{
			QString  layerName = layersCrack[i];
			QJsonObject image;
			ccImage* img = crackPairs[i].second;
			CCVector3 minC;
			CCVector3 maxC;
			minC = img->getPositionBox().minCorner();
			maxC = img->getPositionBox().maxCorner();

			image["layerName"] = layerName;
			image["widthPx"] = QString::number(img->getImage().width());
			image["heightPx"] = QString::number(img->getImage().height());
			image["widthM"] = QString::number(maxC.x - minC.x);
			image["heightM"] = QString::number(maxC.y - minC.y);
			// position left bottom
			image["xlb"] = QString::number(minC.x);
			image["ylb"] = QString::number(minC.y);
			image["zlb"] = QString::number(0);
			image["relPath"] = QString("./" + currentName + "/" + layerName + ".png");

			QString imageName = QString("crackImage_%1").arg(i);
			imagesNode[imageName] = image;
			// Export the image in a file
			img->getImage().save(folderImageAbsolutePath + QString("/") + layerName + QString(".png"));
		}
	}
	
	// Add cracks to yaml

	QJsonObject polylinesNode;

	for (int i = 0; i < crackPairs.size(); i++)
	{
		QString  layerName = layersCrack[i];
		QJsonObject polylineNode;
		
		QString polyName = QString("crack_%1").arg(i);
		ccPolyline* poly = crackPairs[i].first;
	
		// Transform polyline to vector of points unrolled
		QJsonArray unrolledNode;
		for (int j = 0; j < poly->size(); j++)
		{ 
			QJsonArray pointNode;
			pointNode.push_back(poly->getPoint(j)->x); //x
			pointNode.push_back(poly->getPoint(j)->y);//y
			pointNode.push_back(poly->getPoint(j)->z);//z
			unrolledNode.push_back(pointNode);
		}
		// Generate node from pts
		polylineNode["layerName"] = layerName;
		polylineNode["pts"] = unrolledNode;
		polylineNode["color"] = "";
		polylineNode["width"] = "0.3";
		polylineNode["type"] = "";
		polylinesNode[polyName] = polylineNode;
	}

	for (int i = 0; i < linkers.size(); i++)
	{
		QString  layerName = layersCrack[i];
		QJsonObject polylineNode;

		QString polyName = QString("linker_%1").arg(i);
		std::vector<CCVector3> linker = linkers[i];

		// Transform polyline to vector of points unrolled
		if (!linker.empty())
		{
			QJsonArray linkerNode;
				for (int j = 0; j < linker.size(); j++)
				{
					QJsonArray pointNode;
					pointNode.push_back(linker[j].x); //x
					pointNode.push_back(linker[j].y);//y
					pointNode.push_back(linker[j].z);//z
					linkerNode.push_back(pointNode);
				}
			// Generate node from pts
			polylineNode["layerName"] = layerName;
			polylineNode["pts"] = linkerNode;
			polylineNode["color"] = "black";
			polylineNode["width"] = "0.25";
			polylineNode["type"] = "dashed";
			polylinesNode[polyName] = polylineNode;
		}
	}
	// Add grid
	QJsonObject linesNode;
	for (int i = 0; i < lines.size(); i++)
	{
		QJsonObject lineNode;
		// Generate node from pts
		lineNode["layerName"] = QString("Grid");
		QJsonArray pt1;
		pt1.push_back(lines[i].first.x); //x
		pt1.push_back(lines[i].first.y);//y
		pt1.push_back(lines[i].first.z);//z
		QJsonArray pt2;
		pt2.push_back(lines[i].second.x); //x
		pt2.push_back(lines[i].second.y);//y
		pt2.push_back(lines[i].second.z);//z
		
		lineNode["pt1"] = pt1;
		lineNode["pt2"] = pt2;
		lineNode["color"] = "";
		lineNode["width"] = "";
		lineNode["type"] = "";
		linesNode[QString("gridline_%1").arg(i)] = lineNode;
	}

	// Add text
	QJsonObject textsNode;
	for (int i = 0; i < texts.size(); i++)
	{
		QJsonObject textNode;
		// Generate node from pts
		textNode["layerName"] = QString("Grid");

		textNode["xlb"] = texts[i].first.x; //x
		textNode["ylb"] = texts[i].first.y; //x
		textNode["zlb"] = texts[i].first.z; //x

		textNode["text"] = texts[i].second;
		textNode["color"] = "";
		textNode["size"] = "0.2";
		textNode["type"] = "";
		textsNode[QString("gridtext_%1").arg(i)] = textNode;
	}

	QJsonObject json;
	json["layers"] = layersNode;
	json["images"] = imagesNode;
	json["polylines"] = polylinesNode;
	json["lines"] = linesNode;
	json["texts"] = textsNode;
	QJsonDocument out;
	out.setObject(json);
	QByteArray bytes = out.toJson(QJsonDocument::Indented);
	QFile file(jsonFilePath);
	if (file.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate))
	{
		QTextStream iStream(&file);
		iStream.setCodec("utf-8");
		iStream << bytes;
		file.close();
	}
	else
	{
		ccLog::Error(QString("Failed to export the conversion file .yaml for DXF export. Make sure the document is closed"));
	}


	//Deleting unwanted unrolled cracks
	for (int i = 0; i < crackPairs.size(); i++)
	{
		crackPairs[i].first->clear();
	}

}

// Ouputs the linkers for the crack images and updates the image position
void ccStichedImageViewer::generateLinkers(std::vector<std::pair<ccPolyline*,ccImage*>> &crackPairs,std::vector<std::vector<CCVector3>> &linkersOut, float paddingShaft, float paddingBetween) 
{
	
	
	// Polylines and images should be already sorted
	// Image should be already positionned accordingly to the crack just need a horizontal shift
	// Shift align all the boxed to right and scale down image
	// crackPairs first = crackPolyline second= crackImage
	for (int i = 0; i < crackPairs.size(); i++)
	{
		if (crackPairs[i].second)
		{
			ccPolyline* polyline = crackPairs[i].first;
			CCVector3 bbMin, bbMax;
			polyline->getBoundingBox(bbMin, bbMax);
			float pxHeight = (float)crackPairs[i].second->getImage().height();
			float pxWidth = (float)crackPairs[i].second->getImage().width();
			float mWidth = bbMax.x - bbMin.x;
			float mHeight = bbMax.y - bbMin.y;

			float ratioH = pxHeight / mHeight;
			float ratioW = pxWidth / mWidth;
			if (ratioH < ratioW) { ratioW = ratioH; }
			// image size
			mWidth = pxWidth / ratioW;
			mHeight = pxHeight / ratioW;
			// image position aligning and center
			CCVector3 bbMinImg, bbMaxImg;
			bbMinImg.x = m_radius * M_PI + paddingShaft;
			bbMaxImg.x = bbMinImg.x + mWidth;
			bbMinImg.y = (bbMin.y + bbMax.y)/2 - (mHeight/2);
			bbMaxImg.y = (bbMin.y + bbMax.y) / 2 + (mHeight / 2);
			// updating image
			ccBBox newPos(bbMinImg, bbMaxImg);
			crackPairs[i].second->setPositionBox(newPos);
		}
	}

	// Image should now shifting until they are cleared from overlapp
	std::vector<ccBBox> placedBBox; 
	for (int i = 0; i < crackPairs.size(); i++)
	{
		if (crackPairs[i].second)
		{
			ccBBox currentBBox = crackPairs[i].second->getPositionBox();
			for (int j = 0; j < placedBBox.size(); j++)
			{
				if (isBoxCrossing(placedBBox[j], currentBBox, paddingBetween))
				{
					// find shift
					//ccLog::Error("box crossed");
					CCVector3 bbMin = currentBBox.minCorner();
					CCVector3 bbMax = currentBBox.maxCorner();
					float xShift = (placedBBox[j].maxCorner().x - placedBBox[j].minCorner().x) + paddingBetween;
					bbMin.x += xShift;
					bbMax.x += xShift;
					currentBBox = ccBBox(bbMin, bbMax);
				}
			}
			placedBBox.push_back(currentBBox);
			crackPairs[i].second->setPositionBox(currentBBox);
		}
	}
	// Creating linkers
	/**                                    ________
			/\             |              |        |
		   /  \/\Crack\  / |----linker----| IMAGE  |
					   \/  |              |________|
	**/
	for (int i = 0; i < crackPairs.size(); i++)
	{
		std::vector<CCVector3> pts;
		if (crackPairs[i].second)
		{
			CCVector3 bbMinImg, bbMaxImg, bbMinPoly, bbMaxPoly;
			crackPairs[i].first->getBoundingBox(bbMinPoly, bbMaxPoly);
			bbMinImg = crackPairs[i].second->getPositionBox().minCorner();
			bbMaxImg = crackPairs[i].second->getPositionBox().maxCorner();

			pts.push_back(CCVector3(bbMaxPoly.x + paddingBetween, bbMaxPoly.y, 0));
			pts.push_back(CCVector3(bbMaxPoly.x + paddingBetween, bbMinPoly.y, 0));
			pts.push_back(CCVector3(bbMaxPoly.x + paddingBetween, (bbMaxPoly.y + bbMinPoly.y) / 2, 0));
			pts.push_back(CCVector3(bbMinImg.x, (bbMaxImg.y + bbMinImg.y) / 2, 0));
		}
		linkersOut.push_back(pts);
	}
	return;
}

void  ccStichedImageViewer::annotation(float yMin, float yMax, float radius, float padding, float slice,
	std::vector<std::pair<CCVector3, CCVector3>> &lines, std::vector<std::pair<CCVector3, QString>> &text)
{
	
	CCVector3 pt1;
	CCVector3 pt2;
	// Horizontal lines
	float initY = floor(yMin / slice) * slice;
	float savedY = initY;
	while (initY < yMax)
	{
		pt1.x = -radius * M_PI - padding;
		pt2.x = radius * M_PI + padding;
		pt1.y = initY;
		pt2.y = initY;
		lines.push_back(std::make_pair(pt1, pt2));
		pt1.x -= 0.6;
		pt1.y -= 0.3;
		text.push_back(std::make_pair(pt1, QString("%1m").arg(initY)));
		pt2.x += 0.2;
		pt2.y -= 0.3;
		text.push_back(std::make_pair(pt2, QString("%1m").arg(initY)));
		initY += slice;
	}

	// Generate orientations axis full length
	// South left
	yMin = savedY;
	yMax = initY;
	pt1.x = -radius * M_PI;
	pt2.x = -radius * M_PI;
	pt1.y = yMax + padding;
	pt2.y = yMin - padding;
	lines.push_back(std::make_pair(pt1, pt2));
	pt1.y += 0.2;
	pt1.x -= 0.1;
	text.push_back(std::make_pair(pt1, QString("S")));
	// West
	pt1.x = -radius * M_PI_2;
	pt2.x = -radius * M_PI_2;
	pt1.y = yMax + padding;
	pt2.y = yMin - padding;
	lines.push_back(std::make_pair(pt1, pt2));
	pt1.y += 0.2;
	pt1.x -= 0.1;
	text.push_back(std::make_pair(pt1, QString("W")));
	// North
	pt1.x = 0;
	pt2.x = 0;
	pt1.y = yMax + padding;
	pt2.y = yMin - padding;
	lines.push_back(std::make_pair(pt1, pt2));
	pt1.y += 0.2;
	pt1.x -= 0.1;
	text.push_back(std::make_pair(pt1, QString("N")));
	// East
	pt1.x = radius * M_PI_2;
	pt2.x = radius * M_PI_2;
	pt1.y = yMax + padding;
	pt2.y = yMin - padding;
	lines.push_back(std::make_pair(pt1, pt2));
	pt1.y += 0.2;
	pt1.x -= 0.1;
	text.push_back(std::make_pair(pt1, QString("E")));
	// South right
	pt1.x = radius * M_PI;
	pt2.x = radius * M_PI;
	pt1.y = yMax + padding;
	pt2.y = yMin - padding;
	lines.push_back(std::make_pair(pt1, pt2));
	pt1.y += 0.2;
	pt1.x -= 0.1;
	text.push_back(std::make_pair(pt1, QString("S")));

	return; 

}

bool ccStichedImageViewer::isBoxCrossing(ccBBox a, ccBBox b, float padding )
{

	return !(b.minCorner().x - padding > a.maxCorner().x + padding
		|| b.maxCorner().x + padding < a.minCorner().x - padding
		|| b.maxCorner().y + padding < a.minCorner().y  - padding
		|| b.minCorner().y - padding > a.maxCorner().y + padding);
}

//
template <typename T, typename Compare>
std::vector<std::size_t> ccStichedImageViewer::sort_permutation(
	const std::vector<T>& vec,
	Compare& compare)
{
	std::vector<std::size_t> p(vec.size());
	std::iota(p.begin(), p.end(), 0);
	std::sort(p.begin(), p.end(),
		[&](std::size_t i, std::size_t j) { return compare(vec[i], vec[j]); });
	return p;
}

template <typename T>
std::vector<T> ccStichedImageViewer::apply_permutation(
	const std::vector<T>& vec,
	const std::vector<std::size_t>& p)
{
	std::vector<T> sorted_vec(vec.size());
	std::transform(p.begin(), p.end(), sorted_vec.begin(),
		[&](std::size_t i) { return vec[i]; });
	return sorted_vec;
}



