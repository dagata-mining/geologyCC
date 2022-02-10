//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: Libpointmatcher             #
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

#ifndef LIBPOINTMATCHER_CONVERGENCE_DIALOG_HEADER
#define LIBPOINTMATCHER_CONVERGENCE_DIALOG_HEADER

#include <ui_LibpointmatcherConvergenceDialog.h>

//Local
#include <LibpointmatcherTools.h>
#include <Libpointmatcher.h>

//Qt
#include <QSettings>

class ccMainAppInterface;
class ccPointCloud;
class ccHObject;

//! Libpointmatcher plugin's main dialog
class LibpointmatcherConvergenceDialog : public QDialog, public Ui::LibpointmatcherConvergenceDialog
{
	Q_OBJECT

public:

	//! Default constructor
	LibpointmatcherConvergenceDialog(std::vector<ccHObject*> entities, ccMainAppInterface* app);

	//! Returns cloud #1
	ccPointCloud* getCloudRefConvergence() const { return m_cloudRefConvergence; }
	//! Returns cloud #2
	ccPointCloud* getCloudRead() const { return m_cloudRead; }


	//! Do we use existing normals will tell to convert to DP with normals descriptors 
	std::vector<bool> useExistingNormalsRef() const { return m_useExistingNormalsRef; }

	//! Do we need normals to be calculated
	std::vector<bool> needNormalsRef() const { return m_needNormalsRef; }

	//! Do we need normals to be calculated at index
	bool getNeedNormalsRef(int i) const { return m_needNormalsRef[i]; }

	//! Do we need normals to be calculated at index
	bool getNeedNormalsRead(int i) const { return m_needNormalsRead[i]; }

	//! No filter subsampling allowed
	bool nofilterAllowed() const { return m_noFilter; }

	//! Verify is we need at least normals one time
	bool needAtLeastOneNormalRef() const { return std::all_of(m_needNormalsRef.begin(), m_needNormalsRef.end(), [](bool v) { return v; }); }

	//! Verify is we import normals one time
	bool useAtLeastOneNormalRef() const { return std::all_of(m_useExistingNormalsRef.begin(), m_useExistingNormalsRef.end(), [](bool v) { return v; }); }

	//! Verify is we need at least normals one time
	bool needAtLeastOneNormalRead() const { return std::all_of(m_needNormalsRead.begin(), m_needNormalsRead.end(), [](bool v) { return v; }); };

	//! Verify is we import normals one time
	bool useAtLeastOneNormalRead() const { return std::all_of(m_useExistingNormalsRead.begin(), m_useExistingNormalsRead.end(), [](bool v) { return v; }); }

	//! Verify if ref cloud need normals for minimizers or outliers 
	bool refCloudNeedNormalsICP() const { return m_refNeedsNormalICP; }

	//! Verify if read cloud need normals for minimizers or outliers 
	bool readCloudNeedNormalsICP() const { return m_readNeedsNormalICP; }

	//! Returns vector of parameters
	std::vector< std::shared_ptr<PM::DataPointsFilter>> getFiltersRef() const { return m_filtersRef; }

	//! Returns vector of parameters
	std::vector< std::shared_ptr<PM::DataPointsFilter>> getFiltersRead() const { return m_filtersRead; }

	//! Return normal Parameters
	std::shared_ptr<PM::DataPointsFilter> getNormalParams() const { return m_normalParams; }

	//! Return kdTreeParams
	std::shared_ptr<PM::Matcher> getKdTree() const { return m_kdTree; }

	//! Return outlierFilter
	std::shared_ptr<PM::OutlierFilter> getOutlierFilter() const { return m_outlierFilter; }


	//! Return errorMinimizer
	std::shared_ptr<PM::ErrorMinimizer> getErrorMinimizer() const { return m_errorMinimizer; }

	//! Return Transformation Checkers
	std::vector< std::shared_ptr<PM::TransformationChecker>> getCheckers() const { return m_checkers; }

	//! Return The cloud index to be transformed
	int getCurrentreadIndexEntity() const { return m_currentReadEntityIndex; }

	//! Return the slice list
	std::vector<ccPointCloud*> getSliceList() const { return m_sliceList; }

	//! Return the slice list Indexes
	std::vector<int> getSliceListIndexes() const { return m_sliceListIndex; }

	//! change the filter options
	void acceptFilterOptions(bool);
	//! changing the selected item on the filters list disabling and enabling position change and deleting filter
	void selectingFilterItemRef();
	void selectingFilterItemRead();
	//! accept Normals Options
	void acceptNormalOptions();
	//! add to filter list
	void addToFilterListRef();
	void addToFilterListRead();
	//! changeFilterPositionUp
	void changeFilterPositionUpRef();
	void changeFilterPositionUpRead();
	//! changeFilterPositionDown
	void changeFilterPositionDownRef();
	void changeFilterPositionDownRead();
	//! remove a Filter to the filter List
	void removeFromFilterListRef();
	void removeFromFilterListRead();
	//! disable list filter buttons;
	void disableFilterListButtonsRef();
	void disableFilterListButtonsRead();
	//! return current filter tab widget
	int getCurrentFilterTabWidget();

	//! Outlier filters
	void acceptOutlierOption();
	//! KD tree filters
	void acceptKdTreeOption();
	//! Minimizer Option
	void acceptMinimizerOption();
	//! Checker Option
	void acceptCheckerOption();


	//! Swap clouds
	void swapClouds();

	//! No Filter
	void noFilter();

	//! Init Slice list
	void initSliceList(std::vector<ccHObject*> entities);

protected:

	void setCloud1Visibility(bool);
	void setCloud2Visibility(bool);
	void verifySliceEnbaling();
	void changeSlicePositionUp();
	void changeSlicePositionDown();
	void removeSlice();
	void checkSliceNormals();


protected: //members

	ccMainAppInterface* m_app;


	ccPointCloud* m_cloudRefConvergence;
	int m_cloudRefIndex;
	ccPointCloud* m_cloudRead;

	ccPointCloud* m_corePointsCloud;
	std::vector< std::shared_ptr<PM::DataPointsFilter>> m_filtersRef;
	std::vector<bool> m_needNormalsRef;
	std::vector<bool> m_useExistingNormalsRef;
	std::vector< std::shared_ptr<PM::DataPointsFilter>> m_filtersRead;
	std::vector<bool> m_needNormalsRead;
	std::vector<bool> m_useExistingNormalsRead;
	std::shared_ptr<PM::DataPointsFilter> m_normalParams;
	std::vector< std::shared_ptr<PM::TransformationChecker>> m_checkers;
	bool m_refNeedsNormalICP;
	bool m_readNeedsNormalICP;
	
	QString m_currentFilterName;
	int m_filterItemRef;
	int m_filterItemRead;

	int m_currentReadEntityIndex;

	bool m_refFilterInit;
	bool m_readFilterInit;
	bool m_noFilter;

	std::vector<ccPointCloud*> m_sliceList;
	std::vector<int> m_sliceListIndex;
	std::shared_ptr<PM::Matcher> m_kdTree;
	std::shared_ptr<PM::OutlierFilter> m_outlierFilter;
	std::shared_ptr<PM::ErrorMinimizer> m_errorMinimizer;
	
};

#endif 
