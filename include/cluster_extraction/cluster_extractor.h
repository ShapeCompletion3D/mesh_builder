#include <vector>


#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>


class ClusterExtractor
{
	protected:

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr sceneCloud;
        std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloudClusters;

	public:

		void computeClusters();

        void setCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_cloud)
		{
			 sceneCloud = scene_cloud;
		}

        std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> getCloudClusters()
		{
			return cloudClusters;
		}

}; 
