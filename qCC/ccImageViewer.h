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

#ifndef CC_IMAGE_VIEWER
#define CC_IMAGE_VIEWER




//Qt
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <QWheelEvent>
#include <QKeyEvent>

class MainWindow;

//! Custom QListWidget to allow for the copy of all selected elements when using CTRL+C

class ccImageViewer : public QGraphicsView
{
	Q_OBJECT

public:

	//~ccImageViewer() override;
	explicit ccImageViewer(QWidget*);
	//! Default destructor
	virtual ~ccImageViewer();


	void changeImage(QString);
	void ccImageViewer::setActivateChooseImage();
	void ccImageViewer::setDeactivateChooseImage();
	bool ccImageViewer::getChooseImageActivated() { return m_chooseImage; }
signals:
	void imageChosen(int,int);
	//void nodeUp();
	//void nodeDown();

private:
	QGraphicsPixmapItem* m_pixmapItem;
	QGraphicsScene* m_scene;
	QWidget* m_parent;
	bool m_chooseImage;
	double m_width;

	void wheelEvent(QWheelEvent*);
	void keyPressEvent(QKeyEvent*);
	void resizeEvent(QResizeEvent*);
	
	void mousePressEvent(QMouseEvent*);
};

#endif
