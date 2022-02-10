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

#include "ccImageDrawer.h"

#include <ManualSegmentationTools.h>
#include <SquareMatrix.h>
#include "ccReservedIDs.h"

//qCC_db
#include <ccLog.h>
#include <ccPolyline.h>
#include <ccGenericPointCloud.h>
#include <ccPointCloud.h>
#include <ccMesh.h>
#include <ccHObjectCaster.h>
#include <cc2DViewportObject.h>
#include <ccPlane.h>
#include <ccImage.h>



//Local
#include "ccPersistentSettings.h"
#include "mainwindow.h"
#include "CCCoreLib.h"
#include <ccDBRoot.h>

//qCC_db
#include <ccSingleton.h>

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
#include <QQuaternion>
#include <QtMath>



//system
#include <cassert>
#ifdef QT_DEBUG
#include <iostream>
#endif

/***************
 *** Globals ***
 ***************/
ccImageDrawer::ccImageDrawer(QWidget *parent) : QWidget(parent)
, m_paused(false)
, m_polyVertices(nullptr)
, m_segmentationPoly(nullptr)
, m_locked(false)
, m_pointCloudIsHidden(false)
, m_parent(parent)
, m_zNear(0.001)
, m_zFar(10)
, m_bright(0)
, m_contrast(0)
, m_nodeId(0)
, m_tf_rotate(0)
, m_tf_scale()
, m_tf_trans()
{
	m_button_ok.setText("Create Plane");
	m_button_ok.setParent(this);
	m_button_ok.setCursor(Qt::ArrowCursor);
	m_button_ok.setEnabled(false);
	connect(&m_button_ok, &QPushButton::clicked, this, &ccImageDrawer::callbackOk);

	m_button_expand.setText("Expand");
	m_button_expand.setParent(this);
	m_button_expand.setCursor(Qt::ArrowCursor);
	m_button_expand.setEnabled(false);
	connect(&m_button_expand, &QPushButton::clicked, this, &ccImageDrawer::callbackExpand);

	m_button_pause.setText("Start Drawing");
	m_button_pause.setParent(this);
	m_button_pause.setCursor(Qt::ArrowCursor);
	connect(&m_button_pause, &QPushButton::clicked, this, &ccImageDrawer::callbackPause);

	m_button_cancel.setText("Refresh");
	m_button_cancel.setParent(this);
	m_button_cancel.setCursor(Qt::ArrowCursor);
	connect(&m_button_cancel, &QPushButton::clicked, this, &ccImageDrawer::callbackCancel);

	m_button_addPolygon.setText("Add Geology");
	m_button_addPolygon.setParent(this);
	m_button_addPolygon.setCursor(Qt::ArrowCursor);
	m_button_addPolygon.setEnabled(false);
	connect(&m_button_addPolygon, &QPushButton::clicked, this, &ccImageDrawer::addPolygon);

	m_button_clear.setText("Clear All");
	m_button_clear.setParent(this);
	m_button_clear.setCursor(Qt::ArrowCursor);
	connect(&m_button_clear, &QPushButton::clicked, this, &ccImageDrawer::callbackClear);

	m_counter.setValue(0);
	m_counter.setReadOnly(true);
	m_counter.setMaximumWidth(35);
	m_counter.setCursor(Qt::ForbiddenCursor);

	m_far.setValue(10);
	m_far.setMaximum(30);
	m_far.setMinimum(1);
	m_far.setPrefix("Max Dist:"); 
	m_far.setSuffix("m");
	m_far.setMaximumWidth(100);
	m_far.setCursor(Qt::ArrowCursor);

	m_bright.setValue(0);
	m_bright.setMaximum(255);
	m_bright.setMinimum(-255);
	m_bright.setSingleStep(5);
	m_bright.setPrefix("Bright:");
	m_bright.setMaximumWidth(80);
	m_bright.setCursor(Qt::ArrowCursor);
	connect(&m_bright, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, &ccImageDrawer::bright);

	m_contrast.setValue(0);
	m_contrast.setMaximum(50);
	m_contrast.setMinimum(-50);
	m_contrast.setSingleStep(5);
	m_contrast.setPrefix("Contrast:");
	m_contrast.setMaximumWidth(85);
	m_contrast.setCursor(Qt::ArrowCursor);
	connect(&m_contrast, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, &ccImageDrawer::contrast);
	

	m_brush.setColor(QColor(150, 0, 0));
	m_brush.setStyle(Qt::SolidPattern);
	m_pen.setColor(QColor(255, 0, 0));

	m_opacity = 0.5;


	setCursor(Qt::OpenHandCursor);
	setFocusPolicy(Qt::StrongFocus);
	initLayout();
	update();

	m_polyVertices = new ccPointCloud("vertices", static_cast<unsigned>(ReservedIDs::INTERACTIVE_SEGMENTATION_TOOL_POLYLINE_VERTICES));
	m_segmentationPoly = new ccPolyline(m_polyVertices, static_cast<unsigned>(ReservedIDs::INTERACTIVE_SEGMENTATION_TOOL_POLYLINE));

}

ccImageDrawer::~ccImageDrawer()
{
	if (m_segmentationPoly)
		delete m_segmentationPoly;
	m_segmentationPoly = nullptr;

	if (m_polyVertices)
		delete m_polyVertices;
	m_polyVertices = nullptr;
}

void ccImageDrawer::initLayout() {
	QHBoxLayout *button_bar = new QHBoxLayout();
	button_bar->addWidget(&m_button_expand);
	button_bar->addWidget(&m_button_pause);
	button_bar->addWidget(&m_button_cancel);
	button_bar->addWidget(&m_far);
	button_bar->addWidget(&m_button_addPolygon);
	button_bar->addWidget(&m_counter);
	button_bar->addWidget(&m_button_ok);
	button_bar->addWidget(&m_button_clear);
	button_bar->addWidget(&m_contrast);
	button_bar->addWidget(&m_bright);
	button_bar->setAlignment(Qt::AlignBottom | Qt::AlignRight);
	this->setLayout(button_bar);
	QMessageBox(QMessageBox::NoIcon, QString("Here"), QString("Init"));
}
void ccImageDrawer::disableAllWidget() 
{

	m_button_pause.setEnabled(false);
	m_button_expand.setEnabled(false);
	m_button_cancel.setEnabled(false);
	m_button_addPolygon.setEnabled(false);
	m_counter.setEnabled(false);
	m_far.setEnabled(false);
	m_button_ok.setEnabled(false);
	m_button_clear.setEnabled(false);
	m_bright.setEnabled(false);
	m_contrast.setEnabled(false);
	m_locked = true;
	return;
	
}
void ccImageDrawer::enableAllWidget()
{
	m_button_pause.setEnabled(true);
	m_button_expand.setEnabled(true);
	m_button_cancel.setEnabled(false);
	m_button_addPolygon.setEnabled(false);
	m_counter.setEnabled(true);
	m_far.setEnabled(true);
	m_button_ok.setEnabled(false);
	m_button_clear.setEnabled(true);
	m_locked = false;
	m_bright.setEnabled(true);
	m_contrast.setEnabled(true);
	return;
	
}

bool ccImageDrawer::setImage(QPixmap &image) {
	if (image.width()<=0 || image.height() <= 0)
	{
		return false;
	}
	qreal rw = static_cast<qreal>(width()) / static_cast<qreal>(image.width());
	qreal rh = static_cast<qreal>(height()) / static_cast<qreal>(image.height()/2);// height is /2 because we dont need top and bottom
	if (rh < rw) { rw = rh; }

	if(m_tf_scale.isIdentity() && m_tf_trans.isIdentity())
	{
		m_tf_rotate = 0;
		m_tf_scale.scale(rw, rw);
		m_tf_trans.translate(0, -(image.height() / 4));
	}
	m_image = QPixmap(image);
	m_image_backup = QPixmap(image);
	bright(m_bright.value());
	contrast(m_contrast.value());
	update();
	return true;
}

void ccImageDrawer::setBrushParameters(qreal opacity, QColor color) {
	m_opacity = opacity;
	m_brush_color = color;
}

QPolygon ccImageDrawer::getPolylines() {
	return QPolygon(m_polygon);
}

// ************ Painters ************
void ccImageDrawer::paintEvent(QPaintEvent *event) {
	QPainter painter;
	painter.begin(this);

	QTransform rotating;
	rotating.translate(m_image.width() / 2, m_image.height() / 2);
	rotating.rotate(m_tf_rotate);
	rotating.translate(-m_image.width() / 2, -m_image.height() / 2);

	painter.setWorldTransform(rotating * m_tf_trans * m_tf_scale, true);
	painter.drawPixmap(0, 0, m_image);
	painter.setBrush(m_brush);
	painter.setPen(m_pen);
	painter.setOpacity(m_opacity);
	painter.drawPolygon(m_polygon);
	painter.end();

}

// ************ Signals/Slots logic ************
void ccImageDrawer::callbackOk() {
	ccPointCloud* cloud = createNewPointCloudFromHighlighted();
	if (cloud->size() < 10)
	{
		ccLog::Error("Not engough points highlighted to create a proper plan");
		return;
	}
	createPlan(cloud);
	callbackClear();
	update();
	emit actionOkDrawer();
}

void ccImageDrawer::callbackPause() {
	togglePause();
	emit actionPauseDrawer();
}
void ccImageDrawer::callbackCancel() {
	m_polygon.clear();
	m_segmentationPoly->clear();
	m_polyVertices->clear();
	m_button_addPolygon.setEnabled(false);
	update();
	emit actionCancelDrawer();

}
void ccImageDrawer::callbackClear() {
	
	m_polygon.clear();
	m_polygonList.clear();
	m_button_addPolygon.setEnabled(false);
	m_counter.setValue(0);
	m_button_ok.setEnabled(false);
	update();
	emit actionClearDrawer();
	resetCloudVisibility();
}

void ccImageDrawer::callbackExpand() {

	if (m_image.width() <= 0 || m_image.height() <= 0)
	{
		return;
	}
	qreal rw = static_cast<qreal>(width()) / static_cast<qreal>(m_image.width());
	qreal rh = static_cast<qreal>(height()) / static_cast<qreal>(m_image.height() / 2);// height is /2 because we dont need top and bottom
	if (rh < rw) { rw = rh; }

	m_tf_scale = QTransform();
	m_tf_trans = QTransform();

	m_tf_rotate = 0;
	m_tf_scale.scale(rw, rw);
	m_tf_trans.translate(0, -(m_image.height() / 4));
	update();
}


void ccImageDrawer::emitPaintEvent() {
	update();
}

void ccImageDrawer::addPolygon() {
	if (m_polygon.size() >= 3)
	{
		
		
		m_button_addPolygon.setEnabled(false);
		
		transformPolygonToGL();
		bool points = segmentSpherical(true);
		if (points) 
		{
			m_counter.stepUp();
			m_polygonList.push_back(m_polygon);
		}


		if (m_polygonList.size() >= 2)
		{
			m_button_ok.setEnabled(true);

		}
		m_polygon.clear();
		update();
		
	}

}

void ccImageDrawer::togglePause() 
{
	
	if (m_paused) 
	{
		m_paused = false;
		m_button_pause.setText("Start Drawing");
		setCursor(Qt::OpenHandCursor);
		if (m_polygon.size() > 0)
		{
			m_button_cancel.setEnabled(true);
		}
		m_button_cancel.setEnabled(true);

		if (m_polygon.size() >= 3) 
		{
			m_button_addPolygon.setEnabled(true);
		}
		else 
		{
			m_button_addPolygon.setEnabled(false);
		}
		//MainWindow::TheInstance()->ccGetStichedImageViewer()->enableAllWidget();
	}
	else
	{
		m_paused = true;
		setCursor(Qt::CrossCursor);
		m_button_pause.setText("Stop Drawing");
		m_button_addPolygon.setEnabled(false);
		//MainWindow::TheInstance()->ccGetStichedImageViewer()->disableAllWidget();
	}

}
bool ccImageDrawer::isInsideImage(QPoint point)
{
	QSize imageSize = m_image.size();
	if (point.x() >= 0 && point.x() <= imageSize.width() && point.y() >= 0 && point.y() <= imageSize.height())
	{
		return true;
	}
	else
	{
		return false;
	}
}

// ************ Mouse event logic ************
void ccImageDrawer::mousePressEvent(QMouseEvent *event) {
	if (m_locked) { return; }
	m_button_value = event->button();
	
	// verify if selection is activated
	if (m_paused) {
		if (event->button() == Qt::RightButton) {
			togglePause();
		}
		else if (event->button() == Qt::LeftButton) {
			
			
			QTransform rotating;
			rotating.translate(m_image.width() / 2, m_image.height() / 2);
			rotating.rotate(m_tf_rotate);
			rotating.translate(-m_image.width() / 2, -m_image.height() / 2);
			QTransform tf = rotating * m_tf_trans * m_tf_scale;

			if (isInsideImage(tf.inverted().map(event->pos())))
			{
				m_polygon << tf.inverted().map(event->pos());
				
				update();
			}
		}
	}
	else 
	{
		if (event->button() == Qt::RightButton) {
			togglePause();
		}
		else if (event->button() == Qt::LeftButton) {
			m_zoom_anchor = event->pos();
			setCursor(Qt::ClosedHandCursor);
			update();
		}
	}
}

void ccImageDrawer::mouseReleaseEvent(QMouseEvent *event) {
	if (m_locked) { return; }
	if (!m_paused) 
	{
		if (event->button() == Qt::LeftButton) 
		{
			setCursor(Qt::OpenHandCursor);
		}
	}
	update();
}

void ccImageDrawer::mouseMoveEvent(QMouseEvent *event) {
	if (m_locked) { return; }
	if (!m_paused) 
	{
		if (m_button_value == Qt::LeftButton) {
			QPoint dPose = event->pos() - m_zoom_anchor;
			m_zoom_anchor = event->pos();
			m_tf_trans.translate(dPose.x(), dPose.y());
			update();
		}
	
	}
	/*else 
	{
		if (m_button_value == Qt::LeftButton) {
			QTransform tf = m_tf_trans * m_tf_scale;
			m_polygon << tf.inverted().map(event->pos());
			update();
		}
	}*/
}

void ccImageDrawer::wheelEvent(QWheelEvent * event) {
	if (m_locked) { return; }
	if (!m_paused) {
		if (event->delta() > 0) {
			m_tf_scale.scale(1.1, 1.1);
		}
		else {
			m_tf_scale.scale(0.9, 0.9);
		}
	}
	update();
}

void ccImageDrawer::keyPressEvent(QKeyEvent * event) {

	if (m_locked) { return; }
	if (!m_paused) {
		if (event->key() == Qt::Key_Left) {
			if (m_tf_rotate == 0) 
			{
				m_tf_rotate = 270;
			}
			else
			{
				m_tf_rotate = m_tf_rotate - 90;

			}

		}
		else if (event->key() == Qt::Key_Right)
		{
			if (m_tf_rotate == 270)
			{
				m_tf_rotate = 0;
			}
			else
			{
				m_tf_rotate = m_tf_rotate + 90;
			}
		}
	}
	update();
}


void ccImageDrawer::transformPolygonToGL()
{
	// Get GL Window
	ccGLWindow* glWindow = MainWindow::GetActiveGLWindow();
	if (!glWindow)
	{
		return;
	}
	// Calculate the ratio between the image and the glWindow
	QSize windowSize = glWindow->getScreenSize();
	QSize imageSize = m_image.size();
	float windowHeight = (float)windowSize.height();
	float windowWidth = (float)windowSize.width();
	float heightRatio = windowHeight / (float)imageSize.height();
	float widthRatio = windowWidth / (float)imageSize.width();
	// Transform QPolygon to 
	m_polyVertices->clear();
	m_segmentationPoly->clear();
	for (int i = 0; i < m_polygon.count(); i++)
	{
		CCVector3 P(static_cast<PointCoordinateType>(m_polygon.point(i).x()),
			static_cast<PointCoordinateType>(m_polygon.point(i).y()),
			0);
		m_polyVertices->addPoint(P);
		m_segmentationPoly->addPointIndex(i);
	}
	CCVector3 P(static_cast<PointCoordinateType>(m_polygon.point(0).x()),
		static_cast<PointCoordinateType>(m_polygon.point(0).y()),
		0);
	m_polyVertices->addPoint(P);
	m_segmentationPoly->addPointIndex(0);

	//DEBUG
	for (int i = 0; i < m_polyVertices->size(); i++)
	{
		CCVector3d P3D(m_polyVertices->getPoint(i)->x, m_polyVertices->getPoint(i)->y, m_polyVertices->getPoint(i)->z);
		ccLog::Print(QString("Segment Point x: %1px y: %2px z: %3").arg(P3D.x).arg(P3D.y).arg(P3D.z));
	}
	//

	/*
	ccLog::Error(QString("windowHeight : %1 windowWidth : %2 imageHeight : %3 imageWidth: %4 ")
		.arg(QString::number(windowSize.height()))
			.arg(QString::number(windowSize.width()))
				.arg(QString::number(imageSize.height()))
					.arg(QString::number(imageSize.width()))	
	);

	ccLog::Error(QString("ccGLConvertX : %1 ccGLConvertY : %2 ")
		.arg(QString::number(m_polygon.point(0).x()*widthRatio))
		.arg(QString::number(m_polygon.point(0).y()*heightRatio))
	);
	ccLog::Error(QString("ImagePositionx : %1 ImagePositiony : %2 ")
		.arg(QString::number(m_polygon.point(0).x()*widthRatio))
		.arg(QString::number(m_polygon.point(0).y()*heightRatio))
	);
	*/

	if (m_polyVertices->size() <= 3) {
		ccLog::Error("Your polyline has not engough points needs at least 4");
		m_polyVertices->clear();
		m_segmentationPoly->clear();
		return;
	}
	if (m_polyVertices->size() >= 256 ) {
		ccLog::Error("Your polyline has too many points please limit to 256 points");
		m_polyVertices->clear();
		m_segmentationPoly->clear();
		return;
	}
	
	m_segmentationPoly->setClosed(true);

}


void ccImageDrawer::resetCloudVisibility() 
{
	ccGLWindow* glWindow = MainWindow::GetActiveGLWindow();
	ccPointCloud* pointCloud = MainWindow::TheInstance()->ccGetStichedImageViewer()->getCurrentPointCloud();

	if (!glWindow || !pointCloud)
	{
		return;
	}
	ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(pointCloud);
	cloud->resetVisibilityArray();

	m_segmentationPoly->clear();
	m_polyVertices->clear();
	glWindow->redraw(false, false);
	glWindow->refresh();
}




bool ccImageDrawer::segmentSpherical(bool keepPointsInside)
{
	ccGLWindow* glWindow = MainWindow::GetActiveGLWindow();
	ccPointCloud* pointCloud = MainWindow::TheInstance()->ccGetStichedImageViewer()->getCurrentPointCloud();
	std::vector<double> transform = m_viewParameters;


	double maxDist = m_zFar;

	ccGLMatrixd poseMat;
	CCCoreLib::SquareMatrixd rotMat(3);
	double quaternion[4];
	quaternion[0] = transform[3];
	quaternion[1] = transform[4];
	quaternion[2] = transform[5];
	quaternion[3] = transform[6];
	rotMat.initFromQuaternion(quaternion);
	rotMat.toGlMatrix(poseMat.data());
	poseMat.getTranslation()[0] = transform[0];
	poseMat.getTranslation()[1] = transform[1];
	poseMat.getTranslation()[2] = transform[2];
	// Inverting mat for points
	poseMat.invert();

	if (!glWindow || !pointCloud)
	{
		return false;
	}

	m_pointCloudIsHidden = !pointCloud->isEnabled();
	if (m_pointCloudIsHidden)
	{
		pointCloud->setEnabled(true);
	}

	if (!m_segmentationPoly)
	{
		ccLog::Error("No polyline defined!");
		return false;
	}

	if (!m_segmentationPoly->isClosed())
	{
		ccLog::Error("Define and/or close the segmentation polygon first! (right click to close)");
		return false;
	}

	//Prepare CLoud

	ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(pointCloud);
	if (!cloud) { return false; };

	ccGenericPointCloud::VisibilityTableType& visibilityArray = cloud->getTheVisibilityArray();
	if (visibilityArray.empty()) {
		cloud->resetVisibilityArray();
	};
	int cloudSize = static_cast<int>(cloud->size());
	//we project each point and we check if it falls inside the segmentation polyline
	bool atLeast1point = false;
	
#if defined(_OPENMP)
#pragma omp parallel for
#endif
	
	//for (int i = 0; i < pointIndexInsideRadius.size(); ++i)
	

	for (int i = 0; i < cloudSize; ++i)
	{
		//int idxPt = pointIndexInsideRadius[i].pointIndex;
		//if (visibilityArray[idxPt] == CCCoreLib::POINT_VISIBLE)
		if (visibilityArray[i] == CCCoreLib::POINT_VISIBLE)
		{
			
			CCVector3d P3D(cloud->getPoint(i)->x, cloud->getPoint(i)->y, cloud->getPoint(i)->z);
			//CCVector3d P3D(cloud->getPoint(idxPt)->x, cloud->getPoint(idxPt)->y, cloud->getPoint(idxPt)->z);
			//const CCVector3* P3Dinfo = pointIndexInsideRadius[i].point;

			CCVector3d Q2D;
			//if (i % 1000000 == 0) { ccLog::Print(QString("Points before projection x: %1m y: %2m z: %3m").arg(P3D.x).arg(P3D.y).arg(P3D.z)); }
			projectSpherical(P3D, Q2D, poseMat);
			//if (i % 1000000 == 0) { ccLog::Print(QString("Points value from projection x: %1px y: %2px z: %3m").arg(Q2D.x).arg(Q2D.y).arg(Q2D.z)); }
			bool pointInside = false;

			if (Q2D.z <= maxDist)
			{
				CCVector2 P2D(static_cast<PointCoordinateType>(Q2D.x),
					static_cast<PointCoordinateType>(Q2D.y));

				pointInside = CCCoreLib::ManualSegmentationTools::isPointInsidePoly(P2D, m_segmentationPoly);

			}
			if (pointInside) { atLeast1point = true; }
			//visibilityArray[idxPt] = (keepPointsInside != pointInside ? CCCoreLib::POINT_VISIBLE : CCCoreLib::POINT_HIGHLIGHTED);
			visibilityArray[i] = (keepPointsInside != pointInside ? CCCoreLib::POINT_VISIBLE : CCCoreLib::POINT_HIGHLIGHTED);
		}
	}
	if (m_pointCloudIsHidden)
	{
		pointCloud->setEnabled(false);
	}
	glWindow->redraw(false, false);
	glWindow->refresh();


	return atLeast1point;
}

void ccImageDrawer::projectSpherical(CCVector3d P3D, CCVector3d &Q2D, ccGLMatrixd poseMat)
{
	poseMat.apply(P3D);
	// Transform to spherical
	double radius = P3D.norm(); // radius
	double theta = atan2(P3D.x, P3D.y); // longitude
	double phi = asin(P3D.z / radius); // latitude
	// Convert to the size of the image 
	float u = ((float)m_image.width() / 2) * ( 1 + theta / M_PI);
	float v = ((float)m_image.height() / 2) * (1 - phi / M_PI_2);

	Q2D = CCVector3d(u, v, radius);
	return;
}


ccPointCloud* ccImageDrawer::createNewPointCloudFromHighlighted()
{
	ccGLWindow* glWindow = MainWindow::GetActiveGLWindow();
	ccPointCloud* pointCloud = MainWindow::TheInstance()->ccGetStichedImageViewer()->getCurrentPointCloud();


	if (!glWindow || !pointCloud)
	{
		return nullptr;
	}


	ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(pointCloud);

	if (!cloud->isVisibilityTableInstantiated())
	{
		ccLog::Error(QString(" Visibility table not instantiated!"));
		return nullptr;
	}
	ccGenericPointCloud::VisibilityTableType* visTable = &cloud->getTheVisibilityArray();

	// Create a new point cloud 
	ccPointCloud* result = nullptr;
	//we create a temporary entity with the visible points only
	CCCoreLib::ReferenceCloud* rc = pointCloud->getTheHighlightedPoints(visTable, true);
	if (!rc)
	{
		//a warning message has already been issued by getTheVisiblePoints!
		//ccLog::Warning("[ccPointCloud] An error occurred during points selection!");
		return nullptr;
	}

	//convert selection to cloud
	result = pointCloud->partialClone(rc);

	//don't need this one anymore
	delete rc;
	rc = nullptr;

	return result;
}

void ccImageDrawer::createPlan(ccPointCloud* pointCloud) 
{
		ccShiftedObject* shifted = nullptr;
		CCCoreLib::GenericIndexedCloudPersist* cloud = nullptr;

		ccGenericPointCloud* gencloud = ccHObjectCaster::ToGenericPointCloud(pointCloud);
		if (gencloud)
		{
			cloud = static_cast<CCCoreLib::GenericIndexedCloudPersist*>(gencloud);
			shifted = gencloud;
		}

		if (cloud)
		{
			double rms = 0.0;
			CCVector3 C;
			CCVector3 N;

			ccHObject* plane = nullptr;
			

			ccPlane* pPlane = ccPlane::Fit(cloud, &rms);
			int sizePlan = MainWindow::TheInstance()->ccGetStichedImageViewer()->getPlanSize();
			if (sizePlan <= 0)
			{
				sizePlan = 10;
			}
			pPlane->setXWidth(sizePlan);
			pPlane->setYWidth(sizePlan);
			
			if (pPlane)
			{
				plane = static_cast<ccHObject*>(pPlane);
				N = pPlane->getNormal();
				C = *CCCoreLib::Neighbourhood(cloud).getGravityCenter();
				pPlane->enableStippling(true);

				if (shifted)
				{
					pPlane->copyGlobalShiftAndScale(*shifted);
				}
			}

			if (plane)
			{
				ccLog::Print("\t- plane fitting RMS: %f", rms);

				//We always consider the normal with a positive 'Z' by default!
				if (N.z < 0.0)
					N *= -1.0;
				ccLog::Print("\t- normal: (%f,%f,%f)", N.x, N.y, N.z);

				//we compute strike & dip by the way
				PointCoordinateType dip = 0.0f;
				PointCoordinateType dipDir = 0.0f;
				ccNormalVectors::ConvertNormalToDipAndDipDir(N, dip, dipDir);
				QString dipAndDipDirStr = ccNormalVectors::ConvertDipAndDipDirToString(dip, dipDir);
				ccLog::Print(QString("\t- %1").arg(dipAndDipDirStr));

				//hack: output the transformation matrix that would make this normal points towards +Z
				ccGLMatrix makeZPosMatrix = ccGLMatrix::FromToRotation(N, CCVector3(0, 0, CCCoreLib::PC_ONE));
				CCVector3 Gt = C;
				makeZPosMatrix.applyRotation(Gt);
				makeZPosMatrix.setTranslation(C - Gt);

				plane->setName(dipAndDipDirStr);
				plane->applyGLTransformation_recursive(); //not yet in DB
				plane->setVisible(true);
				plane->showNormals(false);
				plane->setSelectionBehavior(ccHObject::SELECTION_FIT_BBOX);

				
				MainWindow::TheInstance()->addToDB(plane);
			}
		}
	//delete the cloud
	delete cloud;
	cloud = nullptr;
}

bool ccImageDrawer::setImageInformation(ccImage* imageCC, std::vector<double> viewParameters)
{
	
	if (!setImage(QPixmap::fromImage(imageCC->data())))
	{
		ccLog::Error("Could not load the image associated to the node");
		return false;
	}
	
	// viewParameter = [x,y,z,qw,qx,qy,qz]
	m_viewParameters = viewParameters;

	
	return true;

}






/*void ccImageDrawer::setRotationVectors()
{
	CCVector3d up(0.0, -1.0, 0.0);
	CCVector3d forward(0.0, 0.0, 1.0);
	// rotating the up vector
		QQuaternion quat(m_viewParameters[3], m_viewParameters[4], m_viewParameters[5], m_viewParameters[6]);
		QVector3D upV(up[0], up[1], up[2]);
		QVector3D rotatedUp = quat.rotatedVector(upV);
		CCVector3d rotateUp(rotatedUp[0], rotatedUp[1], rotatedUp[2]);
		m_upVector = rotateUp;

	// rotating the forward vector
		QVector3D forwardV(forward[0], forward[1], forward[2]);
		QVector3D rotatedForward = quat.rotatedVector(forwardV);
		CCVector3d rotateForward(rotatedForward[0], rotatedForward[1], rotatedForward[2]);
		m_forwardVector = rotateForward;

}
*/
void ccImageDrawer::highlightPoints() 
{
	 
}

void ccImageDrawer::bright( int brightness)
{
	QImage image = m_image_backup.toImage();
	uchar *line = image.scanLine(0);
	uchar *pixel = line;

	for (int y = 0; y < image.height(); ++y)
	{
		pixel = line;
		for (int x = 0; x < image.width(); ++x)
		{
			*pixel = qBound(0, *pixel + brightness, 255);
			*(pixel + 1) = qBound(0, *(pixel + 1) + brightness, 255);
			*(pixel + 2) = qBound(0, *(pixel + 2) + brightness, 255);
			pixel += 4;
		}

		line += image.bytesPerLine();
	}

	m_image= QPixmap::fromImage(image);
	update();

}

void ccImageDrawer::contrast(int contrast)
{
	if (contrast < -50) return;
	if (contrast > 50) return;
	float factor = (100.0 + contrast) / 100.0;
	QImage image = m_image_backup.toImage();
	uchar *line = image.scanLine(0);
	uchar *pixel = line;

	for (int y = 0; y < image.height(); ++y)
	{
		pixel = line;
		for (int x = 0; x < image.width(); ++x)
		{
			*pixel = qBound(0, (int)(128 + factor * (*pixel -128)), 255);
			*(pixel + 1) = qBound(0, (int)(128 + factor * (*(pixel + 1) - 128)), 255);
			*(pixel + 2) = qBound(0, (int)(128 + factor * (*(pixel + 2) - 128)), 255);
			pixel += 4;
		}

		line += image.bytesPerLine();
	}
	m_image = QPixmap::fromImage(image);
	update();
}