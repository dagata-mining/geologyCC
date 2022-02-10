//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qM3C2                       #
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
//#            COPYRIGHT: UNIVERSITE EUROPEENNE DE BRETAGNE                #
//#                                                                        #
//##########################################################################

#include "LibpointMatcherDialog.h"

//qCC
#include "ccMainAppInterface.h"

//qCC_db
#include <ccFileUtils.h>
#include <ccPointCloud.h>

//Qt
#include <QMainWindow>
#include <QComboBox>
#include <QFileInfo>
#include <QFileDialog>
#include <QMessageBox>
#include <QThread>
#include <QstackedWidget>
#include <Qstring>

static bool s_firstTimeInit = true;

/*** HELPERS ***/
static QString GetEntityName(ccHObject* obj)
{
	if (!obj)
	{
		assert(false);
		return QString();
	}

	QString name = obj->getName();
	if (name.isEmpty())
		name = "unnamed";
	name += QString(" [ID %1]").arg(obj->getUniqueID());

	return name;
}



LibpointmatcherDialog::LibpointmatcherDialog(ccMainAppInterface* app)
	: QDialog(app ? app->getMainWindow() : nullptr)
	, Ui::LibpointmatcherDialog()
	, m_app(app)
	, m_corePointsCloud(nullptr)
	, m_normalParams(nullptr)
	, m_currentFilterName("")
	, m_filterItem(0)
{

	setupUi(this);
	connect(addFilterButton, &QAbstractButton::clicked, this, &LibpointmatcherDialog::addToFilterList);
	connect(listFilters, &QListWidget::currentItemChanged, this, &LibpointmatcherDialog::selectingFilterItem);
	connect(listFilters, &QListWidget::itemClicked, this, &LibpointmatcherDialog::selectingFilterItem);
	connect(switchDownFilter, &QToolButton::clicked, this, &LibpointmatcherDialog::changeFilterPositionDown);
	connect(switchUpFilter, &QToolButton::clicked, this, &LibpointmatcherDialog::changeFilterPositionUp);
	connect(deleteOneFilter, &QToolButton::clicked, this, &LibpointmatcherDialog::removeFromFilterList);

	// Set up on initialization 
	listFilters->setCurrentRow(0);
	selectingFilterItem();
}

void LibpointmatcherDialog::disableFilterListButtons()
{

	switchDownFilter->setEnabled(false);
	switchUpFilter->setEnabled(false);
	deleteOneFilter->setEnabled(false);
}

void LibpointmatcherDialog::addToFilterList() {
	if (filterTabWidget->currentIndex() != 0)
	{
		return;
	};

	acceptFilterOptions();
	listFilters->addItem(m_currentFilterName);


};

void LibpointmatcherDialog::selectingFilterItem()
{
	m_filterItem = listFilters->currentRow();
	switchDownFilter->setEnabled(true);
	switchUpFilter->setEnabled(true);
	if (m_filterItem == 0)
	{
		switchUpFilter->setEnabled(false);
	}
	if (m_filterItem+1 == m_filters.size())
	{
		switchDownFilter->setEnabled(false);
	}
	deleteOneFilter->setEnabled(true);

}

void LibpointmatcherDialog::changeFilterPositionUp()
{

	if (m_filterItem == 0) {
		return;
	}
	std::iter_swap(m_filters.begin() + m_filterItem, m_filters.begin() + m_filterItem - 1);
	std::iter_swap(m_needNormals.begin() + m_filterItem, m_needNormals.begin() + m_filterItem - 1);
	std::iter_swap(m_useExistingNormals.begin() + m_filterItem, m_useExistingNormals.begin() + m_filterItem - 1);

	QListWidgetItem* currentItem = listFilters->takeItem(m_filterItem-1);
	listFilters->insertItem(m_filterItem, currentItem);


	selectingFilterItem();
};

void LibpointmatcherDialog::changeFilterPositionDown()
{

	if (m_filterItem + 1 == m_filters.size()) {
		return;
	}
	std::iter_swap(m_filters.begin() + m_filterItem, m_filters.begin() + m_filterItem + 1);
	std::iter_swap(m_needNormals.begin() + m_filterItem, m_needNormals.begin() + m_filterItem + 1);
	std::iter_swap(m_useExistingNormals.begin() + m_filterItem, m_useExistingNormals.begin() + m_filterItem + 1);

	QListWidgetItem* currentItem = listFilters->takeItem(m_filterItem+1);
	listFilters->insertItem(m_filterItem, currentItem);


	selectingFilterItem();

};

void LibpointmatcherDialog::removeFromFilterList() {

	if (m_filters.size() == 0) {
		return;
	}
	m_filters.erase(m_filters.begin() + m_filterItem);
	m_needNormals.erase(m_needNormals.begin() + m_filterItem);
	m_useExistingNormals.erase(m_useExistingNormals.begin() + m_filterItem);

	qDeleteAll(listFilters->selectedItems());
	listFilters->setCurrentRow(0);

	if (m_filters.size() == 0)
	{
		disableFilterListButtons();
		
	}
	else 
	{
		selectingFilterItem();
	}


};

void LibpointmatcherDialog::acceptNormalOptions()
{

	//SurfaceNormalFilter
	std::string knnValue = std::to_string((int)round(normalsKnn->value()));
	std::string epsilonValue = std::to_string(normalsEpsilon->value());
	m_normalParams = PM::get().DataPointsFilterRegistrar.create(
		"SurfaceNormalDataPointsFilter",
		{
			{"knn", knnValue},
			{"epsilon", epsilonValue},
			{"keepNormals", "1"},
			{"keepDensities", "1"},
			{"keepEigenValues", "0"},
			{"keepEigenVectors", "0"},
			{"keepMatchedIds", "0"},
		}
	);
}



void LibpointmatcherDialog::acceptFilterOptions() {
	int indexFilter = Options->currentIndex();
	bool useExistingNormals = true;
	bool needNormals = false;
	PM::Parameters params;

	std::string filterName;
	std::shared_ptr<PM::DataPointsFilter> filterParams;

	switch (indexFilter) {


	case 0:
	{
		//MaximumDensityFilter
		std::string maxDensityValue = std::to_string(maxDensity->value() / 10000.00);
		filterParams = PM::get().DataPointsFilterRegistrar.create(
			"MaxDensityDataPointsFilter",
			{
				{"maxDensity", maxDensityValue},
			}
		);

		useExistingNormals = false;
		needNormals = true;
		filterName = "MaxDensity: " + maxDensityValue + " MaxDensityDataPointsFilter";
		m_currentFilterName = QString(filterName.c_str());
		break;
	}
	case 1:
	{
		//DistanceLimitFilter
		std::string distValue = std::to_string(distanceThreshold->value());

		std::string dimValue;
		//Radial does not seem to work
		if (dimX->isChecked()) { dimValue = "0"; }
		else if (dimY->isChecked()) { dimValue = "1"; }
		else if (dimRadial->isChecked()) { dimValue = "-1"; }
		else { dimValue = "2"; }

		std::string removeInsideValue = "0";
		if (keepDimOuside->isChecked()) { removeInsideValue = "1"; }

		filterParams = PM::get().DataPointsFilterRegistrar.create(
			"DistanceLimitDataPointsFilter",
			{
				{"dim",dimValue},
				{"dist", distValue},
				{"removeInside",removeInsideValue}
			}
		);
		filterName = "MaxDistance: " + distValue + " DistanceLimitDataPointsFilter";
		m_currentFilterName = QString(filterName.c_str());
		break;

	}
	case 2:
	{
		//MaximumPointCountFilter
		std::string seedValue = std::to_string((int)round(srandSeed->value()));
		std::string maxCountValue = std::to_string((int)round(maxPointCount->value()));
		filterParams = PM::get().DataPointsFilterRegistrar.create(
			"MaxPointCountDataPointsFilter",
			{
				{"seed",seedValue},
				{"maxCount", maxCountValue},
			}
		);
		filterName = "MaxPointCount: " + maxCountValue + " MaxPointCountDataPointsFilter";
		m_currentFilterName = QString(filterName.c_str());
		break;
	}
	case 3:
	{
		//MaximumQuantileAxisFilter
		std::string ratioValue = std::to_string(ratioQuantile->value());
		std::string dimValue;
		std::string removeBeyondValue = "1";
		if (underQuantile->isChecked()) { removeBeyondValue = "0"; }
		if (dimX_Quantile->isChecked()) { dimValue = "0"; }
		else if (dimY_Quantile->isChecked()) { dimValue = "1"; }
		else { dimValue = "2"; }

		filterParams = PM::get().DataPointsFilterRegistrar.create(
			"MaxQuantileOnAxisDataPointsFilter",
			{
				{"dim", dimValue},
				{"ratio", ratioValue},
				{"removeBeyond", removeBeyondValue},
			}
		);
		filterName = "QuantileRatio: " + ratioValue + " MaxQuantileOnAxisDataPointsFilter";
		m_currentFilterName = QString(filterName.c_str());
		break;
	}
	case 4:
	{
		//RandomSamplingFilter
		std::string probValue = std::to_string(randomRatio->value());
		filterParams = PM::get().DataPointsFilterRegistrar.create(
			"RandomSamplingDataPointsFilter",
			{
				{"prob", probValue},

			}
		);
		filterName = "Probability: " + probValue + " RandomSamplingDataPointsFilter";
		m_currentFilterName = QString(filterName.c_str());
		break;
	}
	case 5:
	{
		//ShadowPointFilter
		filterParams = PM::get().DataPointsFilterRegistrar.create(
			"ShadowDataPointsFilter"
		);
		if (!shadowNormals->isChecked()) { useExistingNormals = false; }
		needNormals = true;
		filterName = "ShadowDataPointsFilter";
		m_currentFilterName = QString(filterName.c_str());
		break;
	}
	case 6:
	{
		//VoxelGridFilter
		std::string vSizeXValue = std::to_string(voxelSizeX->value());
		std::string vSizeYValue = std::to_string(voxelSizeY->value());
		std::string vSizeZValue = std::to_string(voxelSizeZ->value());
		std::string useCentroidValue = "1";
		if (voxelCenter->isChecked()) { useCentroidValue = "0"; }
		filterParams = PM::get().DataPointsFilterRegistrar.create(
			"VoxelGridDataPointsFilter",
			{
				{"vSizeX", vSizeXValue},
				{"vSizeY", vSizeYValue },
				{"vSizeZ", vSizeZValue},
				{"useCentroid", useCentroidValue},
				{"averageExistingDescriptors","0"}
			}
		);
		filterName = "Voxels x: " + vSizeXValue + " y: " + vSizeYValue + " z: " + vSizeZValue + " VoxelGridDataPointsFilter";
		m_currentFilterName = QString(filterName.c_str());
		break;
	}
	case 7:
	{
		//OctreeGridFilter
		std::string maxPointByNodeValue = std::to_string((int)round(stopPointOctree->value()));
		std::string maxSizeByNodeValue = std::to_string(octreeSize->value());
		std::string samplingMethodValue;
		if (octreeSampleFirst->isChecked()) { samplingMethodValue = "0"; }
		else if (octreeSampleRandom->isChecked()) { samplingMethodValue = "1"; }
		else if (octreeSampleCentroid->isChecked()) { samplingMethodValue = "2"; }
		else { samplingMethodValue = "3"; }


		filterParams = PM::get().DataPointsFilterRegistrar.create(
			"OctreeGridDataPointsFilter",
			{
				{"maxPointByNode",maxPointByNodeValue},
				{"maxSizeByNode",maxSizeByNodeValue},
				{"samplingMethod", samplingMethodValue},
				{"buildParallel","1"}
			}
		);

		filterName = "MaxPointByNode: " + maxPointByNodeValue + " maxSizeByNode: " + maxSizeByNodeValue + " OctreeGridDataPointsFilter";
		m_currentFilterName = QString(filterName.c_str());

		break;
	}
	case 8:
	{
		//NormalSpaceSamplingFilter
		const double halfC = M_PI / 180;
		std::string epsilonValue = std::to_string(epsilonNormal->value()*halfC);
		std::string seedValue = std::to_string((int)round(srandNormal->value()));
		std::string nbSampleValue = std::to_string((int)round(maxPointCountNormal->value()));
		filterParams = PM::get().DataPointsFilterRegistrar.create(
			"NormalSpaceDataPointsFilter",
			{
				{"nbSample", nbSampleValue},
				{"seed", seedValue},
				{"epsilon", epsilonValue}
			}
		);

		if (!normalNormals_2->isChecked()) { useExistingNormals = false; }
		needNormals = true;

		filterName = "Sample Number: " + nbSampleValue + " NormalSpaceDataPointsFilter";
		m_currentFilterName = QString(filterName.c_str());
		break;
	}
	case 9:
	{
		//CovarianceSamplingFilter
		std::string torqueNormValue;
		std::string nbSampleValue = std::to_string((int)round(maxPointCountCov->value()));

		if (torqueNo->isChecked()) { torqueNormValue = "0"; }
		else if (torqueAvg->isChecked()) { torqueNormValue = "1"; }
		else { torqueNormValue = "2"; }

		filterParams = PM::get().DataPointsFilterRegistrar.create(
			"CovarianceSamplingDataPointsFilter",
			{
				{"nbSample", nbSampleValue},
				{"torqueNorm", torqueNormValue},
			}
		);
		if (!covNormals->isChecked()) { useExistingNormals = false; }
		needNormals = true;
		filterName = "Sample Number: " + nbSampleValue + " CovarianceSamplingDataPointsFilter";
		m_currentFilterName = QString(filterName.c_str());
		break;
	}
	case 10:
	{
		//SpectralDecompositionFilter
		std::string kValue = std::to_string((int)round(spdfKnn->value()));
		std::string sigmaValue = std::to_string(spdfSigma->value());
		std::string radiusValue = std::to_string(spdfRadius->value());
		std::string itMaxValue = std::to_string((int)round(spdfIter->value()));

		filterParams = PM::get().DataPointsFilterRegistrar.create(
			"SpectralDecompositionDataPointsFilter",
			{
				{"k", kValue},
				{"sigma", sigmaValue},
				{"radius", radiusValue},
				{"itMax", itMaxValue},
				{"keepNormals", "0"},
				{"keepLabels", "0"},
				{"keepLambdas", "0"},
				{"keepTensors", "0"},
			}
		);

		if (!spdfNormals->isChecked()) { useExistingNormals = false; }
		needNormals = true;
		filterName = "Knn: " + kValue + " SpectralDecompositionDataPointsFilter";
		m_currentFilterName = QString(filterName.c_str());
		break;
	}
	}
	//Push into the fitler params vector
	m_filters.push_back(filterParams);
	m_needNormals.push_back(needNormals);
	m_useExistingNormals.push_back(useExistingNormals);
}

int LibpointmatcherDialog::getCurrentFilterTabWidget()
{
	return filterTabWidget->currentIndex();
}

void LibpointmatcherDialog::setCloud1Visibility(bool state)
{
	if (m_cloud1)
	{
		m_cloud1->setVisible(state);
		m_cloud1->prepareDisplayForRefresh();
	}
	if (m_app)
	{
		m_app->refreshAll();
		m_app->updateUI();
	}
}

void LibpointmatcherDialog::setCloud2Visibility(bool state)
{
	if (m_cloud2)
	{
		m_cloud2->setVisible(state);
		m_cloud2->prepareDisplayForRefresh();
	}
	if (m_app)
	{
		m_app->refreshAll();
		m_app->updateUI();
	}
}





