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

#ifndef LIBPOINTMATCHER_DIALOG_HEADER
#define LIBPOINTMATCHER_DIALOG_HEADER

#include <ui_LibpointmatcherDialog.h>

//Local
#include <LibpointmatcherTools.h>

//Qt
#include <QSettings>

class ccMainAppInterface;
class ccPointCloud;
class ccHObject;

//! Libpointmatcher plugin's main dialog
class LibpointmatcherDialog : public QDialog, public Ui::LibpointmatcherDialog
{
	Q_OBJECT

public:

	//! Default constructor
	LibpointmatcherDialog(ccMainAppInterface* app);

	//! Returns cloud #1
	ccPointCloud* getCloud1() const { return m_cloud1; }
	//! Returns cloud #2
	ccPointCloud* getCloud2() const { return m_cloud2; }

	//! Do we use existing normals will tell to convert to DP with normals descriptors 
	std::vector<bool> useExistingNormals() const { return m_useExistingNormals; }

	//! Do we need normals to be calculated
	std::vector<bool> needNormals() const { return m_needNormals; }

	//! Do we need normals to be calculated at index
	bool getNeedNormals(int i) const { return m_needNormals[i]; }


	//! Verify is we need at least normals one time
	bool needAtLeastOneNormal() const { return std::all_of(m_needNormals.begin(), m_needNormals.end(), [](bool v) { return v; }); };

	//! Verify is we import normals one time
	bool useAtLeastOneNormal() const { return std::all_of(m_useExistingNormals.begin(), m_useExistingNormals.end(), [](bool v) { return v; }); };
	
	//! Returns vector of parameters
	std::vector< std::shared_ptr<PM::DataPointsFilter>> getFilters() const { return m_filters; }

	std::shared_ptr<PM::DataPointsFilter> getNormalParams() const {return m_normalParams;}


	//! Exportation options
	enum ExportOptions {	PROJECT_ON_CLOUD1,
							PROJECT_ON_CLOUD2,
							PROJECT_ON_CORE_POINTS,
	};

	//! Returns selected export option
	ExportOptions getExportOption() const;

	//! Returns whether the original cloud should be kept instead of creating a new output one
	/** Only valid if the export option is PROJECT_ON_CORE_POINTS.
	**/
	bool keepOriginalCloud() const;


	//! change the filter options
	void acceptFilterOptions();
	//! changing the selected item on the filters list disabling and enabling position change and deleting filter
	void selectingFilterItem();
	//! accept Normals Options
	void acceptNormalOptions();
	//! add to filter list
	void addToFilterList();
	//! changeFilterPositionUp
	void changeFilterPositionUp();
	//! changeFilterPositionDown
	void changeFilterPositionDown();
	//! remove a Filter to the filter List
	void removeFromFilterList();
	//! disable list filter buttons;
	void disableFilterListButtons();
	//! return current filter tab widget
	int getCurrentFilterTabWidget();

protected:

	void setCloud1Visibility(bool);
	void setCloud2Visibility(bool);


	


protected: //members

	ccMainAppInterface* m_app;


	ccPointCloud* m_cloud1;
	ccPointCloud* m_cloud2;
	ccPointCloud* m_corePointsCloud;
	std::vector< std::shared_ptr<PM::DataPointsFilter>> m_filters;
	std::vector<bool> m_needNormals;
	std::vector<bool> m_useExistingNormals;
	std::shared_ptr<PM::DataPointsFilter> m_normalParams;
	QString m_currentFilterName;
	int m_filterItem;

};

#endif 
