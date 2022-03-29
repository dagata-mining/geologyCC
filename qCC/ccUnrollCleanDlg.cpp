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

#include "ccUnrollCleanDlg.h"
#include "ui_unrollCleanDlg.h"

//Qt
#include <QSettings>

ccUnrollCleanDlg::ccUnrollCleanDlg(float biggestSize, QWidget* parent/*=0*/)
	: QDialog(parent)
	, m_ui(new Ui::UnrollCleanDialog)
	, m_biggestSize(0.0)
{
	m_ui->setupUi(this);
	m_biggestSize = biggestSize;

	connect(m_ui->checkBoxClean, &QCheckBox::stateChanged, this, &ccUnrollCleanDlg::cleanStateChanged);
	connect(m_ui->spinBoxOctreeLevel, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, &ccUnrollCleanDlg::setOctreeLevel);

	m_ui->checkBoxClean->setChecked(true);
	m_ui->spinBoxOctreeLevel->setEnabled(true);
	m_ui->spinBoxOctreeLevel->setMaximum(CCCoreLib::DgmOctree::MAX_OCTREE_LEVEL);
	setOctreeLevel(m_ui->spinBoxOctreeLevel->value());
}

ccUnrollCleanDlg::~ccUnrollCleanDlg()
{
	delete m_ui;
}


bool ccUnrollCleanDlg::isCleanEnabled() const
{
	return (m_ui->checkBoxClean->checkState() == Qt::Checked);
}

bool ccUnrollCleanDlg::isDistanceEnabled() const
{
	return (m_ui->checkBoxDistance->checkState() == Qt::Checked);
}


CCVector3 ccUnrollCleanDlg::getAxisPosition() const
{
	return CCVector3(static_cast<PointCoordinateType>(m_ui->doubleSpinBoxAxisX->value()),
		static_cast<PointCoordinateType>(m_ui->doubleSpinBoxAxisY->value()),0);
}

double ccUnrollCleanDlg::getRadius() const
{
	return m_ui->radiusDoubleSpinBox->value();
}

int ccUnrollCleanDlg::getOctreeLevel() const
{
	return m_ui->spinBoxOctreeLevel->value();
}


void ccUnrollCleanDlg::cleanStateChanged(int checkState)
{
	if (checkState == Qt::Checked)
	{
		m_ui->spinBoxOctreeLevel->setEnabled(true);
	}
	else
	{
		m_ui->spinBoxOctreeLevel->setEnabled(false);
	}
}

void ccUnrollCleanDlg::setOctreeLevel(int octreeLevel)
{
	float octreeSize = m_biggestSize / pow(2, octreeLevel);
	m_ui->spinBoxOctreeSize->setValue(octreeSize);
}

//semi-persistent settings
//static CCVector3d s_axisCenter(0, 0, 0);


void ccUnrollCleanDlg::toPersistentSettings() const
{
	QSettings settings;
	settings.beginGroup("UnrollClean");
	{

		settings.setValue("radius", m_ui->radiusDoubleSpinBox->value());
		settings.setValue("octreeLevel", m_ui->spinBoxOctreeLevel->value());
		settings.setValue("clean", m_ui->checkBoxClean->isChecked());
		settings.setValue("exportDistance", m_ui->checkBoxDistance->isChecked());
		settings.setValue("x", m_ui->doubleSpinBoxAxisX->value());
		settings.setValue("y", m_ui->doubleSpinBoxAxisY->value());
		//save the axis center as semi-persistent only

	}
	settings.endGroup();
}

void ccUnrollCleanDlg::fromPersistentSettings()
{
	QSettings settings;
	settings.beginGroup("UnrollClean");
	{
		double radius = settings.value("radius", m_ui->radiusDoubleSpinBox->value()).toDouble();
		int octreeLevel = settings.value("octreeLevel", m_ui->spinBoxOctreeLevel->value()).toInt();
		double x = settings.value("x", m_ui->doubleSpinBoxAxisX->value()).toDouble();
		double y = settings.value("y", m_ui->doubleSpinBoxAxisY->value()).toDouble();
		bool clean = settings.value("clean", m_ui->checkBoxClean->isChecked()).toBool();
		bool exportDistance = settings.value("exportDistance", m_ui->checkBoxDistance->isChecked()).toBool();


		m_ui->radiusDoubleSpinBox->setValue(radius);
		m_ui->spinBoxOctreeLevel->setValue(octreeLevel);
		setOctreeLevel(octreeLevel);
		m_ui->checkBoxClean->setChecked(clean);
		if (!clean) { m_ui->spinBoxOctreeLevel->setEnabled(false);}
		m_ui->checkBoxDistance->setChecked(exportDistance);

		m_ui->doubleSpinBoxAxisX->setValue(x);
		m_ui->doubleSpinBoxAxisY->setValue(y);

	}
	settings.endGroup();
}


