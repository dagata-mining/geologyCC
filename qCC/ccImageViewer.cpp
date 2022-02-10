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

#include "ccImageViewer.h"


//Local
#include "ccPersistentSettings.h"
#include "mainwindow.h"

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
#include <QFileInfo>


//system
#include <cassert>
#ifdef QT_DEBUG
#include <iostream>
#endif

/***************
 *** Globals ***
 ***************/
ccImageViewer::ccImageViewer(QWidget* parent = nullptr)
	:QGraphicsView(parent)
	, m_parent(parent)
	, m_chooseImage(false)
	, m_scene(nullptr)
	, m_pixmapItem(nullptr)
	, m_width(0)
{
	//connect(parent->imageViewer, &QAction::wheelEvent, this, &ccImageViewer::wheelEvent);

	setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	setDragMode(QGraphicsView::ScrollHandDrag);

	m_pixmapItem = new QGraphicsPixmapItem(QPixmap(":/CC/images/ccFullScreen.png"));
	m_pixmapItem->setTransformationMode(Qt::SmoothTransformation);
	
	m_scene = new QGraphicsScene();
	m_scene->setSceneRect(50,50,100,100);
	setScene(m_scene);

}

ccImageViewer::~ccImageViewer()
{
	if (m_pixmapItem)
		delete m_pixmapItem;
	m_pixmapItem = nullptr;

	if (m_scene)
		delete m_scene;
	m_scene = nullptr;
}

void ccImageViewer::changeImage(QString path)
{
	QPixmap img = QPixmap(path);
	QFileInfo check_file(path);
	if (check_file.exists() && check_file.isFile())
	{
		m_pixmapItem->setPixmap(img);
		m_scene->setSceneRect(0, 0, img.width(), img.height());
		fitInView(scene()->sceneRect(), Qt::KeepAspectRatio);
	}


}
void ccImageViewer::setActivateChooseImage() {
	setDragMode(QGraphicsView::NoDrag);
	setCursor(Qt::PointingHandCursor);
	m_chooseImage = true;
}
void ccImageViewer::setDeactivateChooseImage() {
	setDragMode(QGraphicsView::ScrollHandDrag);
	m_chooseImage = false;
}


void ccImageViewer::mousePressEvent(QMouseEvent *event)
{

	if (m_chooseImage) {
		
		QPointF pixelValue = m_pixmapItem->mapFromScene(mapToScene(event->pos()));
		int height = m_pixmapItem->pixmap().height();
		int width = m_pixmapItem->pixmap().width();
		int cameraId;
		if ((int)pixelValue.x() < 0) 
		{

			emit imageChosen(-1,-1);
			return;
		}
		else if (static_cast<int>(pixelValue.x()) > width)
		{

			emit imageChosen(-1,-1);
			return;
		}
		else 
		{
			cameraId =static_cast<int>(static_cast<int>(pixelValue.x()) / (width / 5));
		}
		emit imageChosen(-1,cameraId);
		return;
	}
	QGraphicsView::mousePressEvent(event);
	
}

void ccImageViewer::wheelEvent(QWheelEvent *event)
{

	if (event->delta() > 0)
		scale(1.15, 1.15);
	else
		scale(0.9, 0.9);
}
void ccImageViewer::resizeEvent(QResizeEvent *event)
{
	double w = width();
	if (m_width!=w)
	{
		resetMatrix();
		double iw = m_pixmapItem->pixmap().width();
		scale(w / iw, w / iw);
		m_width = w;
	}

}

void ccImageViewer::keyPressEvent(QKeyEvent* event)
{
	if (event->key() == Qt::Key_Left)
		rotate(-90);
	else if (event->key() == Qt::Key_Right)
		rotate(90);
	else if (event->key() == Qt::Key_Escape)
	{
		if (m_chooseImage) {
			emit imageChosen(-1,-1);
		}

	}
}
