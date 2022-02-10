//##########################################################################
//#                                                                        #
//#            CLOUDCOMPARE PLUGIN: ColorimetricSegmenter                  #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#    COPYRIGHT:	Tri-Thien TRUONG, Ronan COLLIER, Mathieu LETRONE       #
//#                                                                        #
//##########################################################################

#include "RgbDialog.h"

//common
#include <ccPickingHub.h>
#include <ccGenericPointCloud.h>

//qCC_gl
#include <ccGLWidget.h>
#include <ccGLWindow.h>

//Qt
#include <QCheckBox>

/*
	Constructor
*/
RgbDialog::RgbDialog(ccPickingHub* pickingHub, QWidget* parent)
	: QDialog(parent)
	, Ui::RgbDialog()
	, m_pickingWin(nullptr)
	, m_pickingHub(pickingHub)
{
	assert(pickingHub);

	setModal(false);
	setupUi(this);

	//Link between Ui and actions
	connect(pointPickingButton_first, &QCheckBox::toggled, this, &RgbDialog::pickPoint_first);
	connect(pointPickingButton_second, &QCheckBox::toggled, this, &RgbDialog::pickPoint_second);

	//auto disable picking mode on quit
	connect(this, &QDialog::finished, [&]()
	{
		if (pointPickingButton_first->isChecked()) pointPickingButton_first->setChecked(false);
		if (pointPickingButton_second->isChecked()) pointPickingButton_second->setChecked(false);
	}
	);
}

/*
	Method for the first picking point functionnality
*/
void RgbDialog::pickPoint_first(bool state)
{
	if (!m_pickingHub)
	{
		return;
	}
	if (state)
	{
		if (!m_pickingHub->addListener(this, true))
		{
			ccLog::Error("Can't start the picking process (another tool is using it)");
			state = false;
		}
	}
	else
	{
		m_pickingHub->removeListener(this);
	}
	pointPickingButton_first->blockSignals(true);
	pointPickingButton_first->setChecked(state);
	pointPickingButton_first->blockSignals(false);
}

/*
	Method for the second picking point functionnality
*/
void RgbDialog::pickPoint_second(bool state)
{
	if (!m_pickingHub)
	{
		return;
	}
	if (state)
	{
		if (!m_pickingHub->addListener(this, true))
		{
			ccLog::Error("Can't start the picking process (another tool is using it)");
			state = false;
		}
	}
	else
	{
		m_pickingHub->removeListener(this);
	}
	pointPickingButton_second->blockSignals(true);
	pointPickingButton_second->setChecked(state);
	pointPickingButton_second->blockSignals(false);
}

/*
	Method applied after a point is picked by picking point functionnality
*/
void RgbDialog::onItemPicked(const PickedItem& pi)
{
	assert(pi.entity);
	m_pickingWin = m_pickingHub->activeWindow();

	if (pi.entity->isKindOf(CC_TYPES::POINT_CLOUD))
	{
		if (static_cast<ccGenericPointCloud*>(pi.entity)->hasColors()) {
				//Get RGB values of the picked point
				ccGenericPointCloud* cloud = static_cast<ccGenericPointCloud*>(pi.entity);
				const ccColor::Rgb& rgb = cloud->getPointColor(pi.itemIndex);
				if (pointPickingButton_first->isChecked()) {
					ccLog::Print("Point picked from first point picker");

					red_first->setValue(rgb.r);
					green_first->setValue(rgb.g);
					blue_first->setValue(rgb.b);

					pointPickingButton_first->setChecked(false);
				}
				else {
					ccLog::Print("Point picked from second point picker");
					red_second->setValue(rgb.r);
					green_second->setValue(rgb.g);
					blue_second->setValue(rgb.b);

					pointPickingButton_second->setChecked(false);
				}
		}
		else {
			ccLog::Print("The point cloud is not with RGB values.");
		}

	}
}
