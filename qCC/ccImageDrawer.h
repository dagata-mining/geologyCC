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

#ifndef CC_IMAGE_DRAWER
#define CC_IMAGE_DRAWER

#include <QtGlobal>
#include <QWidget>
#include <QPushButton>
#include <QRadioButton>
#include <QTimer>
#include <QPainter>
#include <QHBoxLayout>
#include <QWheelEvent>
#include <QKeyEvent>
#include <QGenericMatrix>
#include <QSpinBox.h>
#include <ccPointCloud.h>
#include <ccPolyline.h>
#include <ccImage.h>
#include <DgmOctreeReferenceCloud.h>
#include <DgmOctree.h>
#include <ccOctree.h>
#include <ccOctreeProxy.h>
#include <ccProgressDialog.h>

#include <CloudSamplingTools.h>

enum DRAW_TYPE {
	DRAW_PLANE,	/**< Plane **/
	DRAW_POLYLINE,	/**< Polyline **/
	DRAW_SECTION,	/**< Cross section**/
};

class ccImageDrawer : public QWidget
{
	Q_OBJECT
private:
	QPushButton m_button_cancel;
	QPushButton m_button_ok;
	QPushButton m_button_expand;
	QPushButton m_button_clear;
	QPushButton m_button_pause;
	QPushButton m_button_addPolygon;

	QSpinBox m_counter;
	QSpinBox m_far;
	QDoubleSpinBox m_precision;

	QRadioButton m_radio_plane;
	QRadioButton m_radio_poly;
	QRadioButton m_radio_cross;
	DRAW_TYPE m_draw_type;

	QPixmap m_image;
	QPixmap m_image_backup;
	QColor m_brush_color;
	QPolygon m_polygon;
	
	ccPolyline* m_polylineObject;
	ccPointCloud* m_polyVertices;
	ccPolyline*	m_segmentationPoly;

	QPen m_pen;
	QBrush m_brush;
	qreal m_opacity = 1.;

	QTransform m_tf_trans;
	QTransform m_tf_scale;
	qreal m_tf_rotate;
	qreal m_scale = 1.;
	QPoint m_zoom_anchor;
	int m_button_value;
	QSpinBox m_bright;
	QSpinBox m_contrast;
	int m_nodeId;
	

	// Stop or start drawing 	
	bool m_paused = true;

	// Polygon List
	std::vector<QPolygon> m_polygonList;

	void initPolyObject();
	void initLayout(void);
	//QPoint imageToScreen(QPoint & point);
	//QPoint screenToImage(QPoint & point)

	bool m_pointCloudIsHidden;

public:
	explicit ccImageDrawer(QWidget *parent = nullptr);
	//! Destructor
	virtual ~ccImageDrawer();

	bool setImage(QPixmap & image);
	void setBrushParameters(qreal opacity, QColor color);
	QPolygon getPolylines(void);

	// Stop Start Drawing
	void togglePause();

	// Add a polygon to the polygon list
	void addPolygon();

	//Higlight points
	void highlightPoints();

	//Create Polyline
	void transformPolygonToGL();

	// Segmentation Process
	bool segmentSpherical(bool keepPointsInside);
	void resetCloudVisibility();
	ccPointCloud* createNewPointCloudFromHighlighted();
	//Create the best fit plan
	void createPlan(ccPointCloud* cloud);

	//Image
	bool isInsideImage(QPoint);

	//disable - enbable widget
	void disableAllWidget();
	void enableAllWidget();

	// set the pose, position true= successful false = error  block drawing
	bool setImageInformation(ccImage* imageCC, std::vector<double> viewParameters);

	// Return float x,y,z,qw,qx,qy,qz
	std::vector<double> m_viewParameters;

	//intinsic values fx,fy,cx,cy
	std::vector<double> m_intrinsic;

	// zPlan
	float m_zNear;
	float m_zFar;

	// 
	int getNodeId() { return m_nodeId; }
	void setNodeId(int nodeId) { m_nodeId = nodeId; }

	// Polyline Object
	ccHObject* getPolylineObject() { return m_polylineObject; }

	// Increase Image Brightness
	void bright(int);
	// Increase Image Contrast
	void contrast(int);

	// Project spherical
	void projectSpherical(CCVector3d P3D, CCVector3d &Q2D, ccGLMatrixd poseMat);
	// creates a small square for polyline search
	void generateSquarePolygonFromPoint(QPoint pt, float squareWidth);
	
	// creates a list of points for the polyline with precision in degrees
	void segmentToPoly(float degPrecision);
	
	// Create section polylines 
	void sectionToPoly(float degPrecision);

	
	// smooth the polygon line, precision is in px
	std::vector<CCVector2> ccImageDrawer::smoothPolygonLine(float pxPrecision);


	CCCoreLib::ReferenceCloud* removeHiddenPoints(CCCoreLib::GenericIndexedCloudPersist * theCloud, CCVector3d viewPoint, double fParam);
	ccPointCloud* doAction(ccPointCloud* cloud, CCVector3d viewPoint);



protected:
	void mouseMoveEvent(QMouseEvent *event);
	//void mouseHoverEvent(QHoverEvent *event);
	void mousePressEvent(QMouseEvent *event);
	void mouseReleaseEvent(QMouseEvent *event);
	void wheelEvent(QWheelEvent *event);
	void keyPressEvent(QKeyEvent * event);
	void paintEvent(QPaintEvent *event);
	bool m_locked;

	QWidget* m_parent;

signals:

	void actionClearDrawer();
	
	void actionCancelDrawer();
	void actionPauseDrawer();
	void actionOkDrawer();

public slots:
	void toggleMode(void);
	void callbackClear(void);
	void callbackPause(void);
	void callbackOk(void);
	void callbackCancel(void);
	void callbackExpand(void);
	void emitPaintEvent(void);
};

#endif