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

#include "LibpointMatcherOutlierDialog.h"

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



LibpointmatcherOutlierDialog::LibpointmatcherOutlierDialog(ccPointCloud* cloud1, ccPointCloud* cloud2, ccMainAppInterface* app)
	: QDialog(app ? app->getMainWindow() : nullptr)
	, Ui::LibpointmatcherOutlierDialog()
	, m_app(app)
	, m_corePointsCloud(nullptr)
	, m_normalParams(nullptr)
	, m_currentFilterName("")
	, m_filterItemRef(0)
	, m_filterItemRead(0)
	, m_cloudRef(nullptr)
	, m_cloudRead(nullptr)
	, m_refFilterInit(false)
	, m_readFilterInit(false)
	, m_noFilter(false)
	, m_refNeedsNormalICP(false)
	, m_kdTree(nullptr)
	, m_currentReadEntityIndex(1)
{

	setupUi(this);
	//Ref
	connect(AddRef, &QAbstractButton::clicked, this, &LibpointmatcherOutlierDialog::addToFilterListRef);
	connect(listFiltersRef, &QListWidget::currentItemChanged, this, &LibpointmatcherOutlierDialog::selectingFilterItemRef);
	connect(listFiltersRef, &QListWidget::itemClicked, this, &LibpointmatcherOutlierDialog::selectingFilterItemRef);
	connect(switchDownFilterRef, &QToolButton::clicked, this, &LibpointmatcherOutlierDialog::changeFilterPositionDownRef);
	connect(switchUpFilterRef, &QToolButton::clicked, this, &LibpointmatcherOutlierDialog::changeFilterPositionUpRef);
	connect(deleteOneFilterRef, &QToolButton::clicked, this, &LibpointmatcherOutlierDialog::removeFromFilterListRef);
	// read
	connect(AddRead, &QAbstractButton::clicked, this, &LibpointmatcherOutlierDialog::addToFilterListRead);
	connect(listFiltersRead, &QListWidget::currentItemChanged, this, &LibpointmatcherOutlierDialog::selectingFilterItemRead);
	connect(listFiltersRead, &QListWidget::itemClicked, this, &LibpointmatcherOutlierDialog::selectingFilterItemRead);
	connect(switchDownFilterRead, &QToolButton::clicked, this, &LibpointmatcherOutlierDialog::changeFilterPositionDownRead);
	connect(switchUpFilterRead, &QToolButton::clicked, this, &LibpointmatcherOutlierDialog::changeFilterPositionUpRead);
	connect(deleteOneFilterRead, &QToolButton::clicked, this, &LibpointmatcherOutlierDialog::removeFromFilterListRead);
	// Clouds
	connect(swapCloudsButton, &QToolButton::clicked, this, &LibpointmatcherOutlierDialog::swapClouds);
	connect(noFilterSubsampling, &QCheckBox::clicked, this, &LibpointmatcherOutlierDialog::noFilter);

	// Set up on initialization 
	listFiltersRef->setCurrentRow(0);
	selectingFilterItemRef();
	listFiltersRead->setCurrentRow(0);
	selectingFilterItemRead();
	setClouds(cloud1, cloud2);
}

void LibpointmatcherOutlierDialog::disableFilterListButtonsRef()
{

	switchDownFilterRef->setEnabled(false);
	switchUpFilterRef->setEnabled(false);
	deleteOneFilterRef->setEnabled(false);
}
void LibpointmatcherOutlierDialog::disableFilterListButtonsRead()
{
	switchDownFilterRead->setEnabled(false);
	switchUpFilterRead->setEnabled(false);
	deleteOneFilterRead->setEnabled(false);
}

void LibpointmatcherOutlierDialog::addToFilterListRef() {
	if (filterTabWidget->currentIndex() != 0)
	{
		return;
	};

	acceptFilterOptions(true);
	listFiltersRef->addItem(m_currentFilterName);
	m_refFilterInit = true;


};
void LibpointmatcherOutlierDialog::noFilter()
{
	m_noFilter = noFilterSubsampling->isChecked();
}

void LibpointmatcherOutlierDialog::addToFilterListRead() {
	if (filterTabWidget->currentIndex() != 0)
	{
		return;
	}

	acceptFilterOptions(false);
	listFiltersRead->addItem(m_currentFilterName);
	m_readFilterInit = true;


};

void LibpointmatcherOutlierDialog::selectingFilterItemRef()
{
	m_filterItemRef = listFiltersRef->currentRow();
	switchDownFilterRef->setEnabled(true);
	switchUpFilterRef->setEnabled(true);
	if (m_filterItemRef == 0)
	{
		switchUpFilterRef->setEnabled(false);
	}
	if (m_filterItemRef+1 == m_filtersRef.size())
	{
		switchDownFilterRef->setEnabled(false);
	}
	deleteOneFilterRef->setEnabled(true);

}

void LibpointmatcherOutlierDialog::selectingFilterItemRead()
{
	m_filterItemRead = listFiltersRead->currentRow();
	switchDownFilterRead->setEnabled(true);
	switchUpFilterRead->setEnabled(true);
	if (m_filterItemRead == 0)
	{
		switchUpFilterRead->setEnabled(false);
	}
	if (m_filterItemRead + 1 == m_filtersRead.size())
	{
		switchDownFilterRead->setEnabled(false);
	}
	deleteOneFilterRead->setEnabled(true);

}


void LibpointmatcherOutlierDialog::changeFilterPositionUpRef()
{

	if (m_filterItemRef == 0) {
		return;
	}
	std::iter_swap(m_filtersRef.begin() + m_filterItemRef, m_filtersRef.begin() + m_filterItemRef - 1);
	std::iter_swap(m_needNormalsRef.begin() + m_filterItemRef, m_needNormalsRef.begin() + m_filterItemRef - 1);
	std::iter_swap(m_useExistingNormalsRef.begin() + m_filterItemRef, m_useExistingNormalsRef.begin() + m_filterItemRef - 1);

	QListWidgetItem* currentItem = listFiltersRef->takeItem(m_filterItemRef-1);
	listFiltersRef->insertItem(m_filterItemRef, currentItem);


	selectingFilterItemRef();
};
void LibpointmatcherOutlierDialog::changeFilterPositionUpRead()
{

	if (m_filterItemRead == 0) {
		return;
	}
	std::iter_swap(m_filtersRead.begin() + m_filterItemRead, m_filtersRead.begin() + m_filterItemRead - 1);
	std::iter_swap(m_needNormalsRead.begin() + m_filterItemRead, m_needNormalsRead.begin() + m_filterItemRead - 1);
	std::iter_swap(m_useExistingNormalsRead.begin() + m_filterItemRead, m_useExistingNormalsRead.begin() + m_filterItemRead - 1);

	QListWidgetItem* currentItem = listFiltersRead->takeItem(m_filterItemRead - 1);
	listFiltersRead->insertItem(m_filterItemRead, currentItem);


	selectingFilterItemRead();
};

void LibpointmatcherOutlierDialog::changeFilterPositionDownRef()
{

	if (m_filterItemRef + 1 == m_filtersRef.size()) {
		return;
	}
	std::iter_swap(m_filtersRef.begin() + m_filterItemRef, m_filtersRef.begin() + m_filterItemRef + 1);
	std::iter_swap(m_needNormalsRef.begin() + m_filterItemRef, m_needNormalsRef.begin() + m_filterItemRef + 1);
	std::iter_swap(m_useExistingNormalsRef.begin() + m_filterItemRef, m_useExistingNormalsRef.begin() + m_filterItemRef + 1);

	QListWidgetItem* currentItem = listFiltersRef->takeItem(m_filterItemRef+1);
	listFiltersRef->insertItem(m_filterItemRef, currentItem);


	selectingFilterItemRef();

};

void LibpointmatcherOutlierDialog::changeFilterPositionDownRead()
{

	if (m_filterItemRead + 1 == m_filtersRead.size()) {
		return;
	}
	std::iter_swap(m_filtersRead.begin() + m_filterItemRead, m_filtersRead.begin() + m_filterItemRead + 1);
	std::iter_swap(m_needNormalsRead.begin() + m_filterItemRead, m_needNormalsRead.begin() + m_filterItemRead + 1);
	std::iter_swap(m_useExistingNormalsRead.begin() + m_filterItemRead, m_useExistingNormalsRead.begin() + m_filterItemRead + 1);

	QListWidgetItem* currentItem = listFiltersRead->takeItem(m_filterItemRead + 1);
	listFiltersRead->insertItem(m_filterItemRead, currentItem);


	selectingFilterItemRead();

};

void LibpointmatcherOutlierDialog::removeFromFilterListRef() {

	if (m_filtersRef.size() == 0) {
		return;
	}
	m_filtersRef.erase(m_filtersRef.begin() + m_filterItemRef);
	m_needNormalsRef.erase(m_needNormalsRef.begin() + m_filterItemRef);
	m_useExistingNormalsRef.erase(m_useExistingNormalsRef.begin() + m_filterItemRef);

	qDeleteAll(listFiltersRef->selectedItems());
	listFiltersRef->setCurrentRow(0);

	if (m_filtersRef.size() == 0)
	{
		disableFilterListButtonsRef();
		
	}
	else 
	{
		selectingFilterItemRef();
	}


};
void LibpointmatcherOutlierDialog::removeFromFilterListRead() {

	if (m_filtersRead.size() == 0) {
		return;
	}
	m_filtersRead.erase(m_filtersRead.begin() + m_filterItemRead);
	m_needNormalsRead.erase(m_needNormalsRead.begin() + m_filterItemRead);
	m_useExistingNormalsRead.erase(m_useExistingNormalsRead.begin() + m_filterItemRead);

	qDeleteAll(listFiltersRead->selectedItems());
	listFiltersRead->setCurrentRow(0);

	if (m_filtersRead.size() == 0)
	{
		disableFilterListButtonsRead();

	}
	else
	{
		selectingFilterItemRead();
	}


};

void LibpointmatcherOutlierDialog::acceptNormalOptions()
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

void LibpointmatcherOutlierDialog::acceptOutlierOption() 
{
	
	int indexOutlier = outlierType->currentIndex();


	switch (indexOutlier) {
	
	case 0:
	{
		std::string maxDistValue = std::to_string(maxDistOutlier->value());
		m_outlierFilter = PM::get().OutlierFilterRegistrar.create("MaxDistOutlierFilter",
			{
				{"maxDist",maxDistValue},
			}
		);
		break;
	}
	case 1:
	{
		std::string minDistValue = std::to_string(minDistOutlier->value());
		m_outlierFilter = PM::get().OutlierFilterRegistrar.create("MinDistOutlierFilter",
			{
				{"minDist",minDistValue},
			}
		);
		break;
	}
	case 2:
	{
		std::string factorValue = std::to_string(medianDistOutlier->value());
		m_outlierFilter = PM::get().OutlierFilterRegistrar.create("MedianDistOutlierFilter",
			{
				{"factor",factorValue},
			}
		);
		break;
	}
	case 3:
	{
		std::string ratioValue = std::to_string(trimmedOutlierRatio->value());
		m_outlierFilter = PM::get().OutlierFilterRegistrar.create("TrimmedDistOutlierFilter",
			{
				{"ratio",ratioValue},
			}
		);
		break;
	}
	case 4:
	{
		std::string minRatioValue = std::to_string(varTrimmedOutlierMin->value());
		std::string maxRatioValue = std::to_string(varTrimmedOutlierMax->value());
		std::string lamdaValue = std::to_string(varTrimmedOutlierLambda->value());
		m_outlierFilter = PM::get().OutlierFilterRegistrar.create("VarTrimmedDistOutlierFilter",
			{
				{"minRatio",minRatioValue},
				{"maxRatio",maxRatioValue},
				{"lambda",lamdaValue}
			}
		);
		break;
	}
	case 5:
	{
		const double halfC = M_PI / 180;
		std::string maxAngleValue = std::to_string(surfaceOutlierAngle->value()*halfC);
		m_outlierFilter = PM::get().OutlierFilterRegistrar.create("SurfaceNormalOutlierFilter",
			{
				{"maxAngle",maxAngleValue},	
			}
		);
		m_refNeedsNormalICP = true;
		m_readNeedsNormalICP = true;
		break;
	}
	case 6:
	{
		
		int indexRobust = optionRobust->currentIndex();
		std::string robustFctValue = 0;
		switch (indexRobust)
			{
			case 0:
				robustFctValue = "cauchy";
				break;
			case 1:
				robustFctValue = "welsch";
				break;
			case 2:
				robustFctValue = "sc";
				break;
			case 3:
				robustFctValue = "gm";
				break;
			case 4:
				robustFctValue = "tukey";
				break;
			case 5:
				robustFctValue = "huber";
				break;
			case 6:
				robustFctValue = "L1";
				break;
			}

		std::string tuningValue = std::to_string(robustOutlierTuning->value());
		std::string iterationValue = std::to_string((int)round(robustOutlierIteration->value()));

		std::string scaleValue = "none";
		if (robustOutlierScaleMad->isChecked()) { scaleValue = "mad"; }
		if (robustOutlierScaleBerg->isChecked()) { scaleValue = "berg"; }
		
		std::string distanceTypeValue = "point2point";
		if (robustOutlierDistancePlane->isChecked()) 
		{ 
			distanceTypeValue = "point2plane"; 
			m_refNeedsNormalICP = true;
		}

		m_outlierFilter = PM::get().OutlierFilterRegistrar.create("RobustOutlierFilter",
			{
				{"robustFct",robustFctValue},
				{"tuning",tuningValue},
				{"scaleEstimator",scaleValue},
				{"nbIterationForScale",iterationValue},
				{"distanceType",distanceTypeValue}
			}
		);
		break;
	}
	default: 
	{
		m_outlierFilter = PM::get().OutlierFilterRegistrar.create("NullOutlierFilter");
		break;
	}
	}
}

void LibpointmatcherOutlierDialog::acceptCheckerOption() 
{
	std::shared_ptr<PM::TransformationChecker> checker;
	if (enableIterationChecker->isChecked())
	{
		std::string iterationValue = std::to_string((int)round(iterationCheckerIter->value()));
		checker = PM::get().TransformationCheckerRegistrar.create("CounterTransformationChecker", {
			{"maxIterationCount",iterationValue}
			});
		m_checkers.push_back(checker);
		ccLog::Print(QString("After Iteration checker"));
	}
	if (enableDifferentialChecker->isChecked())
	{
		std::string minDiffRotErrValue = std::to_string(differentialCheckerRot->value());
		std::string minDiffTransErrValue = std::to_string(differentialCheckerTrans->value());
		std::string smoothLengthValue = std::to_string((int)round(differentialCheckerIter->value()));
		checker = PM::get().TransformationCheckerRegistrar.create("DifferentialTransformationChecker", {
			{"minDiffRotErr",minDiffRotErrValue},
			{"minDiffTransErr",minDiffTransErrValue},
			{"smoothLength",smoothLengthValue}
			});
		m_checkers.push_back(checker);
		ccLog::Print(QString("After Differential checker"));
	}
	if (enableMaxBoundChecker->isChecked())
	{
		std::string maxRotationNormValue = std::to_string(maxBoundCheckerRot->value());
		std::string maxTranslationNormValue = std::to_string(maxBoundCheckerTrans->value());
		checker = PM::get().TransformationCheckerRegistrar.create("DifferentialTransformationChecker", {
			{"maxRotationNorm",maxRotationNormValue},
			{"maxTranslationNorm",maxTranslationNormValue},
			});
		m_checkers.push_back(checker);
	}
	// Fall back to this if nothing is selected (40 iter default)
	if (!enableMaxBoundChecker->isChecked() && !enableDifferentialChecker->isChecked() && !enableIterationChecker->isChecked())
	{
		checker = PM::get().TransformationCheckerRegistrar.create("CounterTransformationChecker");
		m_checkers.push_back(checker);
	}
}

void LibpointmatcherOutlierDialog::acceptFilterOptions(bool ref) 
{
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
	if (ref) 
	{
		//Push into the fitler params vector
		m_filtersRef.push_back(filterParams);
		m_needNormalsRef.push_back(needNormals);
		m_useExistingNormalsRef.push_back(useExistingNormals);
	}
	else
	{
		m_filtersRead.push_back(filterParams);
		m_needNormalsRead.push_back(needNormals);
		m_useExistingNormalsRead.push_back(useExistingNormals);
	}
}
void LibpointmatcherOutlierDialog::acceptKdTreeOption()
{

	std::string epsilonValue = std::to_string((int)round(kdTreeEpsilon->value()));
	std::string knnValue = std::to_string((int)round(kdTreeKnn->value()));
	std::string maxDistValue = std::to_string(kdTreeMaxDist->value());
	std::string searchTypeValue = "1";
	if (kdTreeBrute->isChecked()) { searchTypeValue = "0"; }
	if (kdTreeTree->isChecked()) { searchTypeValue = "1"; }
	if (kdTreeMaxDist->value() < 0.001) { maxDistValue = "inf"; }

	m_kdTree = PM::get().MatcherRegistrar.create(
		"KDTreeMatcher",
		{
			{"knn", knnValue},
			{"epsilon", epsilonValue},
			{"searchType",searchTypeValue},
			{"maxDist",maxDistValue}
		}
	);
};
void LibpointmatcherOutlierDialog::acceptMinimizerOption()
{
	int indexMinimizer = minimizerType->currentIndex();
	bool useExistingNormals=true;
	bool needNormals=false;

	switch (indexMinimizer) 
	{
	case 0: 
	{
		std::string pplaneforce2DValue = "0";
		std::string pplaneforce4DOFValue = "0";
		if (pplaneforce4DOF->isChecked()){ pplaneforce4DOFValue = "1"; }
		if (pplaneforce2D->isChecked()) { pplaneforce2DValue = "1"; }
		m_errorMinimizer =
			PM::get().ErrorMinimizerRegistrar.create("PointToPlaneErrorMinimizer", {
				{"force2D",pplaneforce2DValue},
				{"force4DOF",pplaneforce4DOFValue} 
				}
				);
		// Reference Cloud will need normals
		if (!pplaneNormals->isChecked()) { useExistingNormals = false; }
		m_refNeedsNormalICP = true;
		break;
	}
	case 1:
	{
		std::string pplaneforce2DValue = "0";
		std::string pplaneforce4DOFValue = "0";
		if (pplaneforce4DOFCov->isChecked()) { pplaneforce4DOFValue = "1"; }
		if (pplaneforce2DCov->isChecked()) { pplaneforce2DValue = "1"; }
		std::string sensorStdDevValue = std::to_string(pplaneStdDevCov->value());

		m_errorMinimizer =
			PM::get().ErrorMinimizerRegistrar.create("PointToPlaneWithCovErrorMinimizer", {
				{"force2D",pplaneforce2DValue},
				{"force4DOF",pplaneforce4DOFValue},
				{"sensorStdDev",sensorStdDevValue}
				}
		);
		// Reference Cloud will need normals
		if (!pplaneNormalsCov->isChecked()) { useExistingNormals = false; }
		m_refNeedsNormalICP = true;
		break;
	}
	case 2:
	{

		m_errorMinimizer =
			PM::get().ErrorMinimizerRegistrar.create("PointToPointErrorMinimizer");
		break;
	}
	case 3:
	{
		std::string sensorStdDevValue = std::to_string(ppointStdDevCov->value());
		m_errorMinimizer =
			PM::get().ErrorMinimizerRegistrar.create("PointToPointWithCovErrorMinimizer", {
				{"sensorStdDev",sensorStdDevValue}
				}
		);
		break;
	}
	case 4:
	{

		m_errorMinimizer =
			PM::get().ErrorMinimizerRegistrar.create("PointToPointSimilarityErrorMinimizer");
		// Reference Cloud will need normals, the cloud will scaled
		if (!pSimilarityNormals->isChecked()) { useExistingNormals = false; }
		m_refNeedsNormalICP = true;
		break;
	}

	}

}
void LibpointmatcherOutlierDialog::setClouds(ccPointCloud* cloud1, ccPointCloud* cloud2)
{
	if (!cloud1 || !cloud2)
	{
		assert(false);
		return;
	}
	m_cloudRef = cloud1;
	m_cloudRead = cloud2;
	m_currentReadEntityIndex = 1;

	refCloudName->setText(GetEntityName(cloud1));
	readCloudName->setText(GetEntityName(cloud2));
}

void LibpointmatcherOutlierDialog::swapClouds()
{
	if (!m_cloudRef || !m_cloudRead)
	{
		assert(false);
		return;
	}
	ccPointCloud* temp_ref = m_cloudRef;
	ccPointCloud* temp_read = m_cloudRead;
	m_cloudRef = temp_read;
	m_cloudRead = temp_ref;

	refCloudName->setText(GetEntityName(m_cloudRef));
	readCloudName->setText(GetEntityName(m_cloudRead));

	if (m_currentReadEntityIndex == 0) 
	{
		m_currentReadEntityIndex = 1;
	}
	else
	{
		m_currentReadEntityIndex = 0; 
	}
}


int LibpointmatcherOutlierDialog::getCurrentFilterTabWidget()
{
	return filterTabWidget->currentIndex();
}

void LibpointmatcherOutlierDialog::setCloud1Visibility(bool state)
{
	if (m_cloudRef)
	{
		m_cloudRef->setVisible(state);
		m_cloudRef->prepareDisplayForRefresh();
	}
	if (m_app)
	{
		m_app->refreshAll();
		m_app->updateUI();
	}
}

void LibpointmatcherOutlierDialog::setCloud2Visibility(bool state)
{
	if (m_cloudRead)
	{
		m_cloudRead->setVisible(state);
		m_cloudRead->prepareDisplayForRefresh();
	}
	if (m_app)
	{
		m_app->refreshAll();
		m_app->updateUI();
	}
}





