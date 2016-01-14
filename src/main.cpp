#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/shared_ptr.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/surface/organized_fast_mesh.h>
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl/common/projection_matrix.h>

#include <sstream>
#include "cluster_extractor.h"
#include <pcl/common/centroid.h>
#include "geometry_msgs/Vector3.h"

#include "shape_msgs/Mesh.h"
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "graspit_shape_completion/GetSegmentedMeshedSceneAction.h"
#include "actionlib/server/simple_action_server.h"

#include <pcl/surface/gp3.h>
#include <sys/time.h>

namespace mesh_builder_node
{

    class MeshBuilderNode
    {
        private:

            sensor_msgs::PointCloud2 pc_msg;

            ros::NodeHandle node_handle;

            ros::Subscriber pointCloudSubscriber;

            actionlib::SimpleActionServer<graspit_shape_completion::GetSegmentedMeshedSceneAction> as_;
            void executeCB(const graspit_shape_completion::GetSegmentedMeshedSceneGoalConstPtr &goal);
            void pointCloudCB(const sensor_msgs::PointCloud2::ConstPtr &msg);

        public:
            MeshBuilderNode();
    };


    MeshBuilderNode::MeshBuilderNode():
        node_handle("mesh_builder_node"),
        as_(node_handle, "/get_segmented_meshed_scene", boost::bind(&MeshBuilderNode::executeCB, this, _1), false)
    {
        as_.start();
        pointCloudSubscriber =  node_handle.subscribe("/camera/depth_registered/points/", 10,  &MeshBuilderNode::pointCloudCB, this);

        ROS_INFO("mesh_builder_node ready\n");
    }


    void MeshBuilderNode::pointCloudCB(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        pc_msg = *msg;
    }

    void MeshBuilderNode::executeCB(const graspit_shape_completion::GetSegmentedMeshedSceneGoalConstPtr &goal)
    {
        ROS_INFO("executing complete mesh callback\n");

        graspit_shape_completion::GetSegmentedMeshedSceneResult result_;

        struct timeval startTime;
        gettimeofday(&startTime, NULL);
        long int startTimeMS = startTime.tv_sec * 1000 + startTime.tv_usec / 1000;

        //First Get a pointcloud
        pcl::PCLPointCloud2::Ptr pcl_pc (new pcl::PCLPointCloud2());
        pcl_conversions::toPCL(pc_msg, *pcl_pc);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::fromPCLPointCloud2(*pcl_pc, *cloud);

        //scale the cloud for GraspIt!
        pcl::PointCloud<pcl::PointXYZRGB>::iterator p;
        for (p = cloud->points.begin(); p < cloud->points.end(); p++)
        {
            p->x *= 1000;
            p->y *= 1000;
            p->z *= 1000;
        }

        //NEED TO USE THIS, THEN RUN CLUSTER EXTRACTOR,
        //THEN GRAB TRIANGLES WHOSE VERTICES ARE ALL IN SAME CLUSTER
        //WILL BE MUCH FASTER.
//        // This will output cloud as one large mesh
//         pcl::OrganizedFastMesh<pcl::PointXYZRGB > orgMesh;
//         pcl::PolygonMesh triangles;

//         orgMesh.setTriangulationType(pcl::OrganizedFastMesh<pcl::PointXYZRGB>::TRIANGLE_ADAPTIVE_CUT  );
//         orgMesh.setInputCloud(cloud);
//         orgMesh.reconstruct(triangles);

        //extract clusters
        ClusterExtractor *clusterExtractor = new ClusterExtractor();
        clusterExtractor->setCloud(cloud);
        clusterExtractor->computeClusters();
        std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloudClusters = clusterExtractor->getCloudClusters();
        delete clusterExtractor;

        int mesh_index = 0;
        for(int i = 0; i < cloudClusters.size(); i++)
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudCluster (cloudClusters.at(i));


            if (cloudCluster->size() > 10)
            {
                std::ostringstream model_name;
                model_name << "model_" << mesh_index;

                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZRGB>());
                // [...]
                pcl::copyPointCloud(*cloudCluster,*cloud_xyz);

                // Normal estimation*
                 pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
                 pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
                 pcl::search::KdTree<pcl::PointXYZRGB>::Ptr normalEstimationTree (new pcl::search::KdTree<pcl::PointXYZRGB>);
                 normalEstimationTree->setInputCloud (cloud_xyz);
                 n.setInputCloud (cloud_xyz);
                 n.setSearchMethod (normalEstimationTree);
                 n.setKSearch (20);
                 n.compute (*normals);
                 n.setViewPoint(0,0,0);

                 //* normals should not contain the point normals + surface curvatures

                 // Concatenate the XYZ and normal fields*
                 pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
                 pcl::concatenateFields (*cloud_xyz, *normals, *cloud_with_normals);
                 //* cloud_with_normals = cloud + normals


                 Eigen::Vector4f centroid (0.f, 0.f, 0.f, 1.f);
                 pcl::compute3DCentroid (*cloudCluster, centroid); centroid.w () = 1.f;

                 pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr centered_cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

                 pcl::PointCloud<pcl::PointXYZRGBNormal>::iterator p;
                 for (p = cloud_with_normals->points.begin(); p < cloud_with_normals->points.end(); p++)
                 {
                     pcl::PointXYZRGBNormal *point = new pcl::PointXYZRGBNormal;

                     point->x = p->x - centroid.x();
                     point->y = p->y - centroid.y();
                     point->z = p->z - centroid.z();
                     point->r = 200;
                     point->g = 0;
                     point->b = 0;
                     point->a = p->a;
                     point->normal_x = p->normal_x;
                     point->normal_y = p->normal_y;
                     point->normal_z = p->normal_z;

                     centered_cloud_with_normals->points.push_back(*point);
                 }

                 // Create search tree*
                 pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr meshTree (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
                 meshTree->setInputCloud (centered_cloud_with_normals);

                 // Initialize objects
                 pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;
                 pcl::PolygonMesh triangles;

                 // Set the maximum distance between connected points (maximum edge length)
                 gp3.setSearchRadius (25);

                 // Set typical values for the parameters
                 gp3.setMu (2.5);
                 gp3.setMaximumNearestNeighbors (100);
                 gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
                 gp3.setMinimumAngle(M_PI/18); // 10 degrees
                 gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
                 gp3.setNormalConsistency(true);
                 gp3.setConsistentVertexOrdering(true);

                 // Get result
                 gp3.setInputCloud (centered_cloud_with_normals);
                 gp3.setSearchMethod (meshTree);
                 gp3.reconstruct (triangles);

                 geometry_msgs::Vector3 *offset = new geometry_msgs::Vector3;
                 offset->x = centroid.x();
                 offset->y = centroid.y();
                 offset->z = centroid.z();

                 result_.offsets.push_back(*offset);

                shape_msgs::Mesh mesh;
                
                for (int triangle_index = 0; triangle_index < triangles.polygons.size(); triangle_index++)
                {
                    pcl::Vertices p = triangles.polygons.at(triangle_index);

                    shape_msgs::MeshTriangle shape_msg_triangle;
                    shape_msg_triangle.vertex_indices[0] = p.vertices.at(0);
                    shape_msg_triangle.vertex_indices[1] = p.vertices.at(1);
                    shape_msg_triangle.vertex_indices[2] = p.vertices.at(2);

                    mesh.triangles.push_back(shape_msg_triangle);
                }


                for (p = centered_cloud_with_normals->points.begin(); p < centered_cloud_with_normals->points.end(); p++)
                {
                    geometry_msgs::Point msg_p;
                    msg_p.x = p->x ;
                    msg_p.y = p->y ;
                    msg_p.z = p->z ;

                    mesh.vertices.push_back(msg_p);
                }

                result_.meshes.push_back(mesh);

                mesh_index++;
            }

        }

        struct timeval endTime;
        gettimeofday(&endTime, NULL);
        long int endTimeMs = endTime.tv_sec * 1000 + endTime.tv_usec / 1000;
        long int totalTime = endTimeMs - startTimeMS;
        result_.completion_time = totalTime;
        std::cout << "Total Time in milliseconds is " << totalTime <<  std::endl;

        as_.setSucceeded(result_);


    }


}



int main(int argc, char **argv)
{

  ros::init(argc, argv, "mesh_builder_node");
  ros::NodeHandle nh;

  mesh_builder_node::MeshBuilderNode node;

  ros::spin();

  return 0;
}
