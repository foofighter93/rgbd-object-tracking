#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <boost/format.hpp>

#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/pcl_search.h>
#include <pcl/common/transforms.h>
#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/particle_filter_omp.h>
#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>
#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>

#include <modbus/modbus.h>


using namespace pcl::tracking;

typedef pcl::PointXYZ PointType;
typedef pcl::PointXYZ RefPointType;

typedef ParticleXYZRPY ParticleT;
typedef pcl::PointCloud<PointType> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;
typedef ParticleFilterTracker<RefPointType, ParticleT> ParticleFilter;


pcl::PointCloud<PointType>::Ptr received_cloud_ptr;
sensor_msgs::PointCloud2 model_cloud_msg;


boost::shared_ptr<pcl::PointCloud<PointType> > model_cloud;

 ros::Publisher pub_model_cloud;


CloudPtr cloud_pass_;
CloudPtr cloud_pass_downsampled_;
CloudPtr target_cloud;

boost::shared_ptr<ParticleFilter> tracker_;
bool new_cloud_;
double downsampling_grid_size_;
int counter;

/*MODBUS kapcsolathoz szukseges valtozok deklaralasa
int reg = 540;
modbus_t *ctx;
uint16_t tab_reg[64];

ctx = modbus_new_tcp("192.168.0.104",502);  //PLC IP-cime es a port szama*/


void filterPassThrough (const CloudConstPtr &cloud, Cloud &result)
{
    pcl::PassThrough<PointType> pass;
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 1.5);
    pass.setKeepOrganized (false);
    pass.setInputCloud (cloud);
    pass.filter (result);
}

void gridSampleApprox (const CloudConstPtr &cloud, Cloud &result, double leaf_size)
{
    pcl::ApproximateVoxelGrid<PointType> grid;
    grid.setLeafSize (static_cast<float> (leaf_size), static_cast<float> (leaf_size), static_cast<float> (leaf_size));
    grid.setInputCloud (cloud);
    grid.filter (result);
}

//Draw the current particles
bool drawParticles ()
{
    ParticleFilter::PointCloudStatePtr particles = tracker_->getParticles ();
    if (particles && new_cloud_)
    {
        //Set pointCloud with particle's points
        pcl::PointCloud<pcl::PointXYZ>::Ptr particle_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
        for (size_t i = 0; i < particles->points.size (); i++)
        {
            pcl::PointXYZ point;

            point.x = particles->points[i].x;
            point.y = particles->points[i].y;
            point.z = particles->points[i].z;
            particle_cloud->points.push_back (point);
        }

    /*    //Draw red particles
        {

            pcl::toROSMsg(*particle_cloud.get(),particle_cloud_msg );
            particle_cloud_msg.header.frame_id = "/camera_depth_optical_frame";
            pub_particles_cloud.publish(particle_cloud_msg );

        }*/
        return true;
    }
    else
    {
        return false;
    }
}



double x,y,z;
double qx,qy,qz,qw, d;
int m_to_mm = 1000;

void publishObjectPose(Eigen::Affine3f &transformation)
{
    Eigen::Matrix3f rotationMatrix=transformation.rotation();

    x=(int) ((transformation.translation().x()* m_to_mm));
    y=(int) ((transformation.translation().y()* m_to_mm));
    z=(int) ((transformation.translation().z()* m_to_mm));

    Eigen::Quaternionf quaternion(rotationMatrix);

    qx=(int)(quaternion.x() * 1000) / 1000.0f;
    qy=(int)(quaternion.y() * 1000) / 1000.0f;
    qz=(int)(quaternion.z() * 1000) / 1000.0f;
    qw=(int)(quaternion.w() * 1000) / 1000.0f;

/*
    std::cout << "Quaternion x: " << qx <<std::endl;
    std::cout << "Quaternion y: " <<  qy <<std::endl;
    std::cout << "Quaternion z: " << qz <<std::endl;
    std::cout << "Quaternion w: " <<  qw <<std::endl;
 */

    std::cout << " "       <<std::endl;
    std::cout <<"x: " << x <<std::endl;
    std::cout <<"y: " << y <<std::endl;
    std::cout <<"z: " << z <<std::endl;
    std::cout <<" "        <<std::endl;
/*
//    std::cout<<"roll is:" <<atan2( rotationMatrix(2,1),rotationMatrix(2,2) ) <<std::endl;
//    std::cout<<"pitch is:" <<atan2( -rotationMatrix(2,0), std::pow( rotationMatrix(2,1)*rotationMatrix(2,1) +rotationMatrix(2,2)*rotationMatrix(2,2) ,0.5  )  ) <<std::endl;
//    std::cout<<"yaw is:" <<atan2( rotationMatrix(1,0),rotationMatrix(0,0) ) <<std::endl;



    std::cout<<"quaternion_x is:" <<quaternion_x <<std::endl;
    std::cout<<"quaternion_y is:" <<quaternion_y <<std::endl;
    std::cout<<"quaternion_z is:" <<quaternion_z <<std::endl;
    std::cout<<"quaternion_w is:" <<quaternion_w <<std::endl;
*/


}

//Draw model reference point cloud
void drawResult ()
{
    ParticleXYZRPY result = tracker_->getResult ();
    Eigen::Affine3f transformation = tracker_->toEigenMatrix (result);

    publishObjectPose(transformation);


    //move close to camera a little for better visualization
    transformation.translation () += Eigen::Vector3f (0.0f, 0.0f, -0.005f);
//    CloudPtr result_cloud (new Cloud ());
    model_cloud.reset(new pcl::PointCloud<PointType>);
    pcl::transformPointCloud<RefPointType> (*(tracker_->getReferenceCloud ()), *model_cloud, transformation);

    //Draw model reference point cloud
    {
        pcl::toROSMsg(*model_cloud,model_cloud_msg );
        model_cloud_msg.header.frame_id = "/camera_depth_optical_frame";
        pub_model_cloud.publish(model_cloud_msg );
    }
}

void cloud_cb (const CloudConstPtr &cloud)
{
    cloud_pass_.reset (new Cloud);
    cloud_pass_downsampled_.reset (new Cloud);
    filterPassThrough (cloud, *cloud_pass_);
    gridSampleApprox (cloud_pass_, *cloud_pass_downsampled_, downsampling_grid_size_);


    if(counter < 10)
    {
        counter++;
    }else
    {
        //Track the object
        tracker_->setInputCloud (cloud_pass_downsampled_);
        tracker_->compute ();
        new_cloud_ = true;
    }
    drawParticles();
}


template <typename PointT>

void computeMomentOfInertia(boost::shared_ptr<pcl::PointCloud<PointT> > cloud_ptr, Eigen::Vector3f &mass_center)
{

    pcl::MomentOfInertiaEstimation <PointT> feature_extractor;
    feature_extractor.setInputCloud (cloud_ptr);
    feature_extractor.compute ();

    std::vector <float> moment_of_inertia;
    std::vector <float> eccentricity;
    pcl::PointXYZ min_point_AABB;
    pcl::PointXYZ max_point_AABB;
    pcl::PointXYZ min_point_OBB;
    pcl::PointXYZ max_point_OBB;
    pcl::PointXYZ position_OBB;

    Eigen::Matrix3f rotational_matrix_OBB;
    Eigen::Vector3f major_vector, middle_vector, minor_vector;
    float major_value, middle_value, minor_value;

    feature_extractor.getMomentOfInertia (moment_of_inertia);
    feature_extractor.getEccentricity (eccentricity);
    feature_extractor.getAABB (min_point_AABB, max_point_AABB);
    feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    feature_extractor.getEigenValues (major_value, middle_value, minor_value);
    feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
    feature_extractor.getMassCenter (mass_center);
}



void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& pointcloud_msg)
{

    received_cloud_ptr.reset(new pcl::PointCloud<PointType>);
    pcl::fromROSMsg(*pointcloud_msg.get(), *received_cloud_ptr.get());

    cloud_cb (received_cloud_ptr);
    drawParticles();
    drawResult();

    Eigen::Vector3f mass_center;
    computeMomentOfInertia<pcl::PointXYZ>(model_cloud,mass_center);
    x=mass_center(0,0) ;
    y=mass_center(1,0);
    z=mass_center(2,0);

}


int main(int argc, char **argv)
{

    ros::init(argc,argv,"tracking_object");

    ros::NodeHandle nh;

    ros::Subscriber sub_pcl;
    std::string pointcloud_topic="cloud";
    std::string filepath= "/home/zsombi/catkin_ws/src/objtrack/src/bogre.pcd";

   // nh.getParam("filepath",filepath);

  //  ROS_INFO("Got parameter = %s",filepath.c_str());

    sub_pcl= nh.subscribe<sensor_msgs::PointCloud2>(pointcloud_topic, 1, pointcloudCallback);

    pub_model_cloud=nh.advertise<sensor_msgs::PointCloud2>("model_cloud",1);




    target_cloud.reset(new Cloud());

    if(pcl::io::loadPCDFile (filepath, *target_cloud) == -1)
    {
        std::cout << "pcd file not found" << std::endl;
        exit(-1);
    }


    //modbus kapcsolathoz szükséges változók deklarálása
      int reg = 540;
      int wreg;

   //   uint16_t tab_reg[64];
      modbus_t *mb;

      mb = modbus_new_tcp("192.168.0.104",502); //192.168.0.104 PLC ip cime

      if (modbus_connect(mb) == -1) {
        fprintf(stderr, "Connection failed: %s\n", modbus_strerror(errno));
        modbus_free(mb);
      }

    //  modbus_read_registers(mb,1000,1,tab_reg); //olvas egy bitet az 1000-es memoriacimrol


    counter = 0;

    //Set parameters
    new_cloud_  = false;
    downsampling_grid_size_ =  0.002;

    //Covariance matrix
    std::vector<double> default_step_covariance = std::vector<double> (6, 0.015 * 0.015);
    default_step_covariance[3] *= 40.0;
    default_step_covariance[4] *= 40.0;
    default_step_covariance[5] *= 40.0;

    std::vector<double> initial_noise_covariance = std::vector<double> (6, 0.00001);
    std::vector<double> default_initial_mean = std::vector<double> (6, 0.0);

    boost::shared_ptr<KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT> > tracker
    (new KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT> (8));

    //Particles size
    ParticleT bin_size;
    bin_size.x = 0.1f;
    bin_size.y = 0.1f;
    bin_size.z = 0.1f;
    bin_size.roll = 0.1f;
    bin_size.pitch = 0.1f;
    bin_size.yaw = 0.1f;


    //Set all parameters for  KLDAdaptiveParticleFilterOMPTracker
    //tracker->setMaximumParticleNum (1000);
    tracker->setMaximumParticleNum (800);
    tracker->setDelta (0.99);
    tracker->setEpsilon (0.2);
    tracker->setBinSize (bin_size);

    //Set all parameters for  ParticleFilter
    tracker_ = tracker;
    tracker_->setTrans (Eigen::Affine3f::Identity ());
    tracker_->setStepNoiseCovariance (default_step_covariance);
    tracker_->setInitialNoiseCovariance (initial_noise_covariance);
    tracker_->setInitialNoiseMean (default_initial_mean);
    tracker_->setIterationNum (5);
    tracker_->setParticleNum (600);
    tracker_->setResampleLikelihoodThr(0.00);
    tracker_->setUseNormal (false);


    //Setup coherence object for tracking
    ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr coherence = ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr
    (new ApproxNearestPairPointCloudCoherence<RefPointType> ());

    boost::shared_ptr<DistanceCoherence<RefPointType> > distance_coherence
    = boost::shared_ptr<DistanceCoherence<RefPointType> > (new DistanceCoherence<RefPointType> ());
    coherence->addPointCoherence (distance_coherence);

    boost::shared_ptr<pcl::search::Octree<RefPointType> > search (new pcl::search::Octree<RefPointType> (0.01));
    coherence->setSearchMethod (search);
    coherence->setMaximumDistance (0.01);

    tracker_->setCloudCoherence (coherence);

    //prepare the model of tracker's target
    Eigen::Vector4f c;
    Eigen::Affine3f trans = Eigen::Affine3f::Identity ();
    CloudPtr transed_ref (new Cloud);
    CloudPtr transed_ref_downsampled (new Cloud);

    pcl::compute3DCentroid<RefPointType> (*target_cloud, c);
    trans.translation ().matrix () = Eigen::Vector3f (c[0], c[1], c[2]);
    pcl::transformPointCloud<RefPointType> (*target_cloud, *transed_ref, trans.inverse());
    gridSampleApprox (transed_ref, *transed_ref_downsampled, downsampling_grid_size_);

    //set reference model and trans
    tracker_->setReferenceCloud (transed_ref_downsampled);
    tracker_->setTrans (trans);

    tf::TransformBroadcaster br;
    tf::Transform transform;

     ros::Rate loop_rate(30); //30 Hz

    while(nh.ok())
    {
        ros::spinOnce();

        loop_rate.sleep();
        transform.setOrigin( tf::Vector3(x, y, z) );
        transform.setRotation( tf::Quaternion(qx,qy,qz,qw) );
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/camera_depth_optical_frame", pointcloud_topic));
	
        //Meter to mm conversion
        int xmod = x*1000;
        int ymod = y*1000;
        int zmod = z*1000;
 
        wreg = modbus_write_register(ctx, reg,   xmod);
       // if (wreg != 1) { std::cout<<"ERROR writing register " << std::endl; }
        std::cout<<"MODBUS x" << reg << "   "<<xmod<<std::endl;
        modbus_write_register(ctx, reg+1, ymod);
        std::cout<<"MODBUS y" << reg+1<< "   " <<ymod<<std::endl;
        modbus_write_register(ctx, reg+2, zmod);
        std::cout<<"MODBUS z" << reg+2<< "   " << zmod<<std::endl;

    }

    return 0;
}
