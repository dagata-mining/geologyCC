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
#include <ReferenceCloud.h>
#include <ccPointCloud.h>
#include <CCCoreLib.h>


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



extern "C"
{
#include <qhull_a.h>
}

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
, m_draw_type(DRAW_PLANE)
, m_radio_plane("Plan")
, m_radio_poly("Polyline")
, m_radio_cross("Cross Section")
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

	m_radio_plane.setParent(this);
	m_radio_plane.setCursor(Qt::ArrowCursor);
	m_radio_plane.toggle();
	connect(&m_radio_plane, &QRadioButton::toggled, this, &ccImageDrawer::toggleMode);
	

	m_radio_poly.setParent(this);
	m_radio_poly.setCursor(Qt::ArrowCursor);
	connect(&m_radio_poly, &QRadioButton::toggled, this, &ccImageDrawer::toggleMode);

	m_radio_cross.setParent(this);
	m_radio_cross.setCursor(Qt::ArrowCursor);
	connect(&m_radio_cross, &QRadioButton::toggled, this, &ccImageDrawer::toggleMode);

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
	button_bar->addWidget(&m_radio_poly);
	button_bar->addWidget(&m_radio_plane);
	button_bar->addWidget(&m_radio_cross);
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
}
void ccImageDrawer::disableAllWidget() 
{

	m_button_pause.setEnabled(false);
	m_radio_plane.setEnabled(false);
	m_radio_poly.setEnabled(false);
	m_radio_cross.setEnabled(false);

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
	m_radio_plane.setEnabled(true);
	m_radio_poly.setEnabled(true);
	m_radio_cross.setEnabled(true);
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

	switch (m_draw_type)
	{
		case DRAW_PLANE:
			painter.drawPixmap(0, 0, m_image);
			m_brush.setColor(QColor(150, 0, 0));
			m_brush.setStyle(Qt::SolidPattern);
			m_pen.setColor(QColor(255, 0, 0));
			m_pen.setWidth(2);
			painter.setBrush(m_brush);
			painter.setPen(m_pen);
			painter.setOpacity(m_opacity);
			painter.drawPolygon(m_polygon);
			break;
		case DRAW_POLYLINE:
			painter.drawPixmap(0, 0, m_image);
			m_pen.setColor(QColor(255, 0, 0));
			m_pen.setWidth(20);
			painter.setBrush(m_brush);
			painter.setPen(m_pen);
			painter.setOpacity(1);
			painter.drawPolyline(m_polygon);
			break;
		case DRAW_SECTION:
			painter.drawPixmap(0, 0, m_image);
			m_pen.setColor(QColor(255, 0, 0));
			m_pen.setWidth(20);
			painter.setBrush(m_brush);
			painter.setPen(m_pen);
			painter.setOpacity(1);
			if (m_polygon.size() == 1)
			{
				QPoint pt1(0, m_polygon[0].y());
				QPoint pt2(m_image.width() - 1, m_polygon[0].y());
				QPolygon tempPoly;
				tempPoly.push_back(pt1);
				tempPoly.push_back(pt2);
				painter.drawPolyline(tempPoly);
			}
			else
			{
				m_polygon.clear();
				painter.drawPolyline(m_polygon);
			}
			break;
	}

	painter.end();

}

// ************ Signals/Slots logic ************
void ccImageDrawer::toggleMode() {

	if (m_radio_plane.isChecked())
	{
		m_draw_type = DRAW_PLANE;
		m_button_addPolygon.setEnabled(true);
		m_button_ok.setText("Create Plane");
		m_button_addPolygon.setText("Add Geology");
		callbackClear();
	}
	else if(m_radio_poly.isChecked())
	{
		m_draw_type = DRAW_POLYLINE;
		m_button_addPolygon.setEnabled(false);
		m_button_ok.setText("N/A");
		m_button_addPolygon.setText("Draw Polyline");
		callbackClear();
	}
	else 
	{
		m_draw_type = DRAW_SECTION;
		m_button_addPolygon.setEnabled(false);
		m_button_ok.setText("N/A");
		m_button_addPolygon.setText("Cross Section");
		callbackClear();
	}

}

void ccImageDrawer::callbackOk() {
	if (m_draw_type == DRAW_PLANE)
	{
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
	
	emit actionCancelDrawer();
	update();


}
void ccImageDrawer::callbackClear() {
	m_polygon.clear();
	m_polygonList.clear();
	m_button_addPolygon.setEnabled(false);
	m_counter.setValue(0);
	m_button_ok.setEnabled(false);
	emit actionClearDrawer();
	resetCloudVisibility();
	update();
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
	if (m_polygon.size() >= 3  && m_draw_type == DRAW_PLANE)
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
	if (m_draw_type == DRAW_POLYLINE && m_polygon.size() > 1)
	{
		segmentToPoly(0.5);
		callbackClear();
		update();
	}
	if (m_draw_type == DRAW_SECTION && m_polygon.size() == 1)
	{
		sectionToPoly(0.5);
		callbackClear();
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

		if (m_polygon.size() >= 3 && m_draw_type == DRAW_PLANE) 
		{
			m_button_addPolygon.setEnabled(true);
		}
		else if (m_polygon.size() >= 2 && m_draw_type == DRAW_POLYLINE)
		{
			m_button_addPolygon.setEnabled(true);
		}
		else if (m_polygon.size() == 1 && m_draw_type == DRAW_SECTION)
		{
			m_button_addPolygon.setEnabled(true);
		}
		else 
		{
			m_button_addPolygon.setEnabled(false);
		}
		m_radio_plane.setEnabled(true);
		m_radio_poly.setEnabled(true);
		m_radio_cross.setEnabled(true);
		//MainWindow::TheInstance()->ccGetStichedImageViewer()->enableAllWidget();
	}
	else
	{
		m_paused = true;
		setCursor(Qt::CrossCursor);
		m_button_pause.setText("Stop Drawing");
		m_button_addPolygon.setEnabled(false);
		m_radio_plane.setEnabled(false);
		m_radio_poly.setEnabled(false);
		m_radio_cross.setEnabled(false);
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
				if (m_draw_type == DRAW_SECTION)
				{
					m_polygon.clear();
					m_polygon << tf.inverted().map(event->pos());
				}
				else
				{
					m_polygon << tf.inverted().map(event->pos());
				}
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

void ccImageDrawer::generateSquarePolygonFromPoint(QPoint pt, float squareWidth)
{
	// topLeft
	CCVector3 topLeft(pt.x() - squareWidth > 0 ? pt.x() - squareWidth : 0,
		pt.y() - squareWidth > 0 ? pt.y() - squareWidth : 0,
		0);
	// topRight
	CCVector3 topRight(pt.x() + squareWidth > (float)m_image.width() ? pt.x() + squareWidth : (float)m_image.width(),
		pt.y() - squareWidth > 0 ? pt.y() - squareWidth : 0,
		0);
	// bottomRight
	CCVector3 bottomRight(pt.x() + squareWidth > (float)m_image.width() ? pt.x() + squareWidth : (float)m_image.width(),
		pt.y() + squareWidth > (float)m_image.height() ? pt.y() + squareWidth : (float)m_image.height(),
		0);
	// bottomLeft
	CCVector3 bottomLeft(pt.x() - squareWidth > 0 ? pt.x() - squareWidth : 0,
		pt.y() + squareWidth > (float)m_image.height() ? pt.y() + squareWidth : (float)m_image.height(),
		0);



	// creating polyline 
	m_polyVertices->clear();
	m_segmentationPoly->clear();

	// Top left
	m_polyVertices->addPoint(topLeft);
	m_segmentationPoly->addPointIndex(0);
	// Top right
	m_polyVertices->addPoint(topRight);
	m_segmentationPoly->addPointIndex(1);
	// Bottom right
	m_polyVertices->addPoint(bottomRight);
	m_segmentationPoly->addPointIndex(2);
	// BottomLeft
	m_polyVertices->addPoint(bottomLeft);
	m_segmentationPoly->addPointIndex(3);
	// Top left closing
	m_polyVertices->addPoint(topLeft);
	m_segmentationPoly->addPointIndex(0);
	// Closing Polyline 
	m_segmentationPoly->setClosed(true);

	return;
}

std::vector<CCVector2> ccImageDrawer::smoothPolygonLine( float pxPrecision)
{
	std::vector<CCVector2> smoothPoints;
	if (m_polygon.size() < 2) { return smoothPoints; }
	

	for (int i = 0; i + 1 < m_polygon.size(); i++)
	{
		QPoint pt1 = m_polygon[i];
		QPoint pt2 = m_polygon[i+1];
		QPoint delta = pt2 - pt1; 
		ccLog::Print(QString("pt1(%1, %2) pt2(%3, %4) delta(%5, %6)").arg(pt1.x()).arg(pt1.y()).arg(pt2.x()).arg(pt2.y()).arg(delta.x()).arg(delta.y()));
		float a = (float)delta.y() / (float)delta.x();
		float xSteps = sqrt(pow(pxPrecision, 2) / (1 + pow(a, 2)));
		ccLog::Print(QString("xSteps: %1 a: %2").arg(xSteps).arg(a));
		float accumulatedX = xSteps;
		CCVector2 ptToAdd;

		// add first point 
		smoothPoints.push_back(CCVector2((float)pt1.x(),(float)pt1.y()));

		while (accumulatedX < (float)delta.x())
		{
			ptToAdd = CCVector2(accumulatedX + (float)pt1.x(), accumulatedX*a + (float)pt1.y());
			smoothPoints.push_back(ptToAdd);
			accumulatedX += xSteps;
		}
	}
	// add last point 
	smoothPoints.push_back(CCVector2((float)m_polygon.back().x(), (float)m_polygon.back().y()));
	
	for (int i = 0; i < smoothPoints.size(); i++)
	{
		ccLog::Print(QString("Pt smoothed are (%1, %2,)").arg(smoothPoints[i][0]).arg(smoothPoints[i][1]));
	}
	return smoothPoints;
}


void ccImageDrawer::sectionToPoly(float degPrecision)
{
	// Get the point cloud of the biggest rectangle based on the polyline
	ccGLWindow* glWindow = MainWindow::GetActiveGLWindow();
	ccPointCloud* pointCloud = MainWindow::TheInstance()->ccGetStichedImageViewer()->getCurrentPointCloud();
	std::vector<double> transform = m_viewParameters;
	double maxDist = m_zFar;
	float pxPrecision = degPrecision / 360 * m_image.width();


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
		return;
	}

	m_pointCloudIsHidden = !pointCloud->isEnabled();
	if (m_pointCloudIsHidden)
	{
		pointCloud->setEnabled(true);
	}

	if (!m_polygon.size() == 1)
	{
		ccLog::Error("Section should only have one point Cancelling");
		return;
	}

	ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(pointCloud);
	if (!cloud) { return; };

	int cloudSize = static_cast<int>(cloud->size());
	//we project each point and we check if it falls inside the segmentation polyline
	bool atLeast1point = false;
	// We create a temporary pair of xyz and uvz in order accelerate speed (act as the points kept in the whole cloud)


	// we create the segmentation polyline  (the big big retangle)

	// Get min and max
	float maxX = m_image.width();
	float maxY = m_polygon[0].y() + 2*pxPrecision > m_image.height()? m_image.height():m_polygon[0].y() + 2*pxPrecision;
	float minX = 0;
	float minY =  m_polygon[0].y() - 2*pxPrecision < 0 ? 0 : m_polygon[0].y() - 2*pxPrecision;

	m_polyVertices->clear();
	m_segmentationPoly->clear();

	// Top left
	m_polyVertices->addPoint(CCVector3(minX, minY, 0));
	m_segmentationPoly->addPointIndex(0);
	// Top right
	m_polyVertices->addPoint(CCVector3(maxX, minY, 0));
	m_segmentationPoly->addPointIndex(1);
	// Bottom right
	m_polyVertices->addPoint(CCVector3(maxX, maxY, 0));
	m_segmentationPoly->addPointIndex(2);
	// BottomLeft
	m_polyVertices->addPoint(CCVector3(minX, maxY, 0));
	m_segmentationPoly->addPointIndex(3);
	// Top left closing
	m_polyVertices->addPoint(CCVector3(minX, minY, 0));
	m_segmentationPoly->addPointIndex(0);
	// Closing Polyline 
	m_segmentationPoly->setClosed(true);

	ccPointCloud* cloudSegmented = new ccPointCloud();
	ccPointCloud* cloudHpr = new ccPointCloud();
	cloudSegmented->reserve(static_cast<unsigned>(cloudSize));
	for (int i = 0; i < cloudSize; ++i)
	{
		CCVector3d P3D(cloud->getPoint(i)->x, cloud->getPoint(i)->y, cloud->getPoint(i)->z);

		CCVector3d Q2D;
		projectSpherical(P3D, Q2D, poseMat);
		bool pointInside = false;

		if (Q2D.z <= maxDist)
		{
			CCVector2 P2D(static_cast<PointCoordinateType>(Q2D.x),
				static_cast<PointCoordinateType>(Q2D.y));

			pointInside = CCCoreLib::ManualSegmentationTools::isPointInsidePoly(P2D, m_segmentationPoly);

		}
		if (pointInside)
		{
			atLeast1point = true;
			// add to temp point cloud
			cloudSegmented->addPoint(*cloud->getPoint(i));
		}
	}
	
	//cloudHpr = doAction(cloudSegmented, CCVector3d(transform[0], transform[1], transform[2]));
	*cloudHpr = *cloudSegmented;
	delete cloudSegmented;
	
	//get all the points to a pair
	std::vector<std::pair<CCVector3d, CCVector3d>> tempCloud;
	for (int i = 0; i < cloudHpr->size(); i++)
	{
		CCVector3d P3D(cloudHpr->getPoint(i)->x, cloudHpr->getPoint(i)->y, cloudHpr->getPoint(i)->z);
		CCVector3d Q2D;
		projectSpherical(P3D, Q2D, poseMat);
		tempCloud.push_back(std::make_pair(P3D, Q2D));
	}
	delete cloudHpr;
	

	// generate smoother polygon line
	float xAccumulated = 0;
	std::vector<CCVector2> ptList;
	while (xAccumulated < m_image.width())
	{
		CCVector2 P2D(xAccumulated, m_polygon[0].y());
		ptList.push_back(P2D);
		xAccumulated += pxPrecision;
	}


	// now that we have our smaller cloud we can process to calculate points of the polyline
	ccPointCloud* finalVertices = new ccPointCloud("vertices", static_cast<unsigned>(ReservedIDs::INTERACTIVE_SEGMENTATION_TOOL_POLYLINE_VERTICES));
	ccPolyline* finalPolyline = new ccPolyline(finalVertices, static_cast<unsigned>(ReservedIDs::INTERACTIVE_SEGMENTATION_TOOL_POLYLINE));
	finalPolyline->setColor(ccColor::Rgb(255, 0, 0));
	finalPolyline->setWidth(2);
	int polylineId = 0;
	for (int j = 0; j < ptList.size(); j++)
	{
		float u = ptList[j][0];
		float v = ptList[j][1];
		int ptsFound = 0;
		CCVector3d P3D(0, 0, 0);
		float xSum = 0;
		float ySum = 0;
		float zSum = 0;
		for (int i = 0; i < tempCloud.size(); ++i)
		{
			// Findind points
			CCVector3d P2D = tempCloud[i].second;
			float ptDistance = sqrt(pow(u - P2D[0], 2) + pow(v - P2D[1], 2));
			if (ptDistance < pxPrecision)
			{
				P3D += tempCloud[i].first;
				ptsFound++;
			}
		}

		if (ptsFound > 1)
		{
			// Averaging result
			P3D /= ptsFound;
			ccLog::Print(QString("Points found added to polyline pt:%1").arg(polylineId));
			// New polyline
			finalVertices->addPoint(CCVector3(P3D.x, P3D.y, P3D.z));
			finalPolyline->addPointIndex(polylineId);
			polylineId++;
		}

		if (ptsFound <= 1)
		{
			ccLog::Warning(QString("No 3D Points found at pt:%1, not adding to polyline").arg(polylineId));
		}
	}

	finalPolyline->setClosed(true);

	// level the z values
	CCVector3 bbMax;
	CCVector3 bbMin;
	finalVertices->getBoundingBox(bbMin, bbMax);
	float zAvg = (bbMin.z + bbMax.z) / 2;
	ccPointCloud tempVertices = *finalVertices;
	finalVertices->clear();

	for (int i = 0; i < tempVertices.size(); i++)
	{
		CCVector3 pt(*tempVertices.getPoint(i));
		
		finalVertices->addPoint(CCVector3(pt.x, pt.y, zAvg));
	}

	if (m_pointCloudIsHidden)
	{
		pointCloud->setEnabled(false);
	}


	//Adding polyline to file
	ccHObject* finalObject = static_cast<ccHObject*>(finalPolyline);
	finalObject->setEnabled(true);
	MainWindow::TheInstance()->addToDB(finalObject);

	glWindow->redraw();
	glWindow->refresh();


	return;

}

void ccImageDrawer::segmentToPoly(float degPrecision)
{
	// Get the point cloud of the biggest rectangle based on the polyline
	ccGLWindow* glWindow = MainWindow::GetActiveGLWindow();
	ccPointCloud* pointCloud = MainWindow::TheInstance()->ccGetStichedImageViewer()->getCurrentPointCloud();
	std::vector<double> transform = m_viewParameters;
	double maxDist = m_zFar;
	float pxPrecision = degPrecision / 360 * m_image.width();


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
		return;
	}

	m_pointCloudIsHidden = !pointCloud->isEnabled();
	if (m_pointCloudIsHidden)
	{
		pointCloud->setEnabled(true);
	}

	if (!m_polygon.size() > 2)
	{
		ccLog::Error("No polyline defined!");
		return;
	}

	ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(pointCloud);
	if (!cloud) { return; };

	int cloudSize = static_cast<int>(cloud->size());
	//we project each point and we check if it falls inside the segmentation polyline
	bool atLeast1point = false;

	// We create a temporary pair of xyz and uvz in order accelerate speed (act as the points kept in the whole cloud)
	

	// we create the segmentation polyline  (the big big retangle)

	// Get min and max
	float maxX = 0;
	float maxY = 0;
	float minX = m_image.width();
	float minY = m_image.height();

	for (int i = 0; i < m_polygon.count(); i++)
	{

		if (m_polygon.point(i).x() > maxX)
		{
			maxX = m_polygon.point(i).x();
		}
		if (m_polygon.point(i).y() > maxY)
		{
			maxY = m_polygon.point(i).y();
		}
		if (m_polygon.point(i).x() < minX)
		{
			minX = m_polygon.point(i).x();
		}
		if (m_polygon.point(i).y() < minY)
		{
			minY = m_polygon.point(i).y();
		}
	}
	maxX = maxX + pxPrecision > (float)m_image.width() ? (float)m_image.width() : maxX + pxPrecision;
	maxY = maxY + pxPrecision > (float)m_image.height() ? (float)m_image.height() : maxY + pxPrecision;
	minX = minX - pxPrecision < 0 ? 0 : minX - pxPrecision;
	minY = minY - pxPrecision < 0 ? 0 : minY - pxPrecision;

	m_polyVertices->clear();
	m_segmentationPoly->clear();

	// Top left
	m_polyVertices->addPoint(CCVector3(minX, minY, 0));
	m_segmentationPoly->addPointIndex(0);
	// Top right
	m_polyVertices->addPoint(CCVector3(maxX, minY, 0));
	m_segmentationPoly->addPointIndex(1);
	// Bottom right
	m_polyVertices->addPoint(CCVector3(maxX, maxY, 0));
	m_segmentationPoly->addPointIndex(2);
	// BottomLeft
	m_polyVertices->addPoint(CCVector3(minX, maxY, 0));
	m_segmentationPoly->addPointIndex(3);
	// Top left closing
	m_polyVertices->addPoint(CCVector3(minX, minY, 0));
	m_segmentationPoly->addPointIndex(0);
	// Closing Polyline 
	m_segmentationPoly->setClosed(true);

	ccPointCloud* cloudSegmented = new ccPointCloud;
	ccPointCloud* cloudHpr = new ccPointCloud;

	for (int i = 0; i < cloudSize; ++i)
	{
		CCVector3d P3D(cloud->getPoint(i)->x, cloud->getPoint(i)->y, cloud->getPoint(i)->z);

		CCVector3d Q2D;
		projectSpherical(P3D, Q2D, poseMat);
		bool pointInside = false;

		if (Q2D.z <= maxDist)
		{
			CCVector2 P2D(static_cast<PointCoordinateType>(Q2D.x),
				static_cast<PointCoordinateType>(Q2D.y));

			pointInside = CCCoreLib::ManualSegmentationTools::isPointInsidePoly(P2D, m_segmentationPoly);

		}
		if (pointInside)
		{
			atLeast1point = true;
			// add to temp point cloud
			cloudSegmented->addPoint(*cloud->getPoint(i));
		}
	}

	cloudHpr = doAction(cloudSegmented, CCVector3d(transform[0], transform[1], transform[2]));
	//*cloudHpr = *cloudSegmented;
	delete cloudSegmented;

	//get all the points to a pair
	std::vector<std::pair<CCVector3d, CCVector3d>> tempCloud;
	for (int i = 0; i < cloudHpr->size(); i++)
	{
		CCVector3d P3D(cloudHpr->getPoint(i)->x, cloudHpr->getPoint(i)->y, cloudHpr->getPoint(i)->z);
		CCVector3d Q2D;
		projectSpherical(P3D, Q2D, poseMat);
		tempCloud.push_back(std::make_pair(P3D, Q2D));
	}
	delete cloudHpr;
	

	// generate smoother polygon line
	std::vector<CCVector2> ptList = smoothPolygonLine(pxPrecision);

	// now that we have our smaller cloud we can process to calculate points of the polyline
	ccPointCloud* finalVertices = new ccPointCloud("vertices", static_cast<unsigned>(ReservedIDs::INTERACTIVE_SEGMENTATION_TOOL_POLYLINE_VERTICES));
	ccPolyline* finalPolyline = new ccPolyline(finalVertices, static_cast<unsigned>(ReservedIDs::INTERACTIVE_SEGMENTATION_TOOL_POLYLINE));
	finalPolyline->setColor(ccColor::Rgb(255, 0, 0));
	finalPolyline->setWidth(2);
	int polylineId = 0;

	for (int j = 0; j < ptList.size(); j++)
	{
		float u = ptList[j][0];
		float v = ptList[j][1];
		int ptsFound = 0;
		CCVector3d P3D(0, 0, 0);
		float xSum = 0;
		float ySum = 0;
		float zSum = 0;
		for (int i = 0; i < tempCloud.size(); ++i)
		{
			// Findind points
			CCVector3d P2D = tempCloud[i].second;
			float ptDistance = sqrt(pow(u - P2D[0], 2) + pow(v - P2D[1], 2));
			if (ptDistance < pxPrecision)
			{
				P3D += tempCloud[i].first;
				ptsFound++;
			}
		}

		if (ptsFound > 1)
		{
			// Averaging result
			P3D /= ptsFound;
			ccLog::Print(QString("Points found added to polyline pt:%1").arg(polylineId));
			// New polyline
			finalVertices->addPoint(CCVector3(P3D.x,P3D.y,P3D.z));
			finalPolyline->addPointIndex(polylineId);
			polylineId++;
		}

		if (ptsFound <= 1)
		{
			ccLog::Warning(QString("No 3D Points found at pt:%1, not adding to polyline").arg(polylineId));
		}
	}
	if (m_pointCloudIsHidden)
	{
		pointCloud->setEnabled(false);
	}


	//Adding polyline to file
	ccHObject* finalObject = static_cast<ccHObject*>(finalPolyline);
	finalObject->setEnabled(true);
	MainWindow::TheInstance()->addToDB(finalObject);

	glWindow->redraw();
	glWindow->refresh();


	return;

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
	glWindow->redraw();
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
	delete rc;
	rc = nullptr;

	//CCCoreLib::GenericIndexedCloudPersist* hprCloud = ccHObjectCaster::ToGenericPointCloud(result);
	//ccStdPluginInterface:: 
	
	CCVector3d viewPoint(m_viewParameters[0], m_viewParameters[1], m_viewParameters[2]);
	ccPointCloud* hprCloud = doAction(result,viewPoint);
	delete result;
	
	ccLog::Print(QString("Amount of points: %1").arg(hprCloud->size()));
	glWindow->redraw();
	glWindow->refresh();
	//CCCoreLib::ReferenceCloud* hprRc= hpr1->removeHiddenPoints(static_cast<CCCoreLib::GenericIndexedCloudPersist*>(result), viewPoint, 0.01);

	//don't need this one anymore
	//result = result->partialClone(hprRc);

	//Degug purposes


	return hprCloud;
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

ccPointCloud* ccImageDrawer::doAction(ccPointCloud* cloud, CCVector3d viewPoint)
{
	ccPointCloud* resultCloud = nullptr;
	//unique parameter: the octree subdivision level
	int octreeLevel = 10;
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
		return resultCloud;
	}


	//HPR
	QScopedPointer<CCCoreLib::ReferenceCloud> visibleCells;
	{
		QScopedPointer<CCCoreLib::ReferenceCloud> theCellCenters(CCCoreLib::CloudSamplingTools::subsampleCloudWithOctreeAtLevel(cloud,
			static_cast<unsigned char>(octreeLevel),
			CCCoreLib::CloudSamplingTools::NEAREST_POINT_TO_CELL_CENTER,
			0,
			theOctree.data()));
		if (!theCellCenters)
		{
			ccLog::Error("Error while simplifying point cloud with octree!");
			return resultCloud;
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
			return resultCloud;
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
				return resultCloud;
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
			resultCloud = cloud->partialClone(&visiblePoints);
			if (!resultCloud)
			{
				ccLog::Error("Not enough memory!");
			}

		}
	}
	return resultCloud;
	//currently selected entities appearance may have changed!
	//m_app->refreshAll();
}

CCCoreLib::ReferenceCloud * ccImageDrawer::removeHiddenPoints(CCCoreLib::GenericIndexedCloudPersist * theCloud, CCVector3d viewPoint, double fParam)
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
