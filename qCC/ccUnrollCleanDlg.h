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

#ifndef CC_UNROLL_CLEAN_DLG_HEADER
#define CC_UNROLL_CLEAN_DLG_HEADER

#include <QDialog>

//qCC_db
#include <ccPointCloud.h>

namespace Ui {
	class UnrollCleanDialog;
}

//! Dialog: unroll clould on a cylinder or a cone
class ccUnrollCleanDlg : public QDialog
{
	Q_OBJECT

public:

	//! Default constructor
	explicit ccUnrollCleanDlg(float biggestSize, QWidget* parent = nullptr);
	~ccUnrollCleanDlg();
	
	bool isCleanEnabled() const;
	bool isDistanceEnabled() const;
	CCVector3 getAxisPosition() const;
	double getRadius() const;
	int getOctreeLevel() const;
	bool exportDistance() const;

	void toPersistentSettings() const;
	void fromPersistentSettings();

protected:
	void cleanStateChanged(int checkState);
	void setOctreeLevel(int octreeLevel);

protected:
	bool coneMode;

	Ui::UnrollCleanDialog* m_ui;
	float m_biggestSize;
};

#endif
