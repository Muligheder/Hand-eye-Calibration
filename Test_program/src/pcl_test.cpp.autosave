#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "/home/anders/librealsense/examples/example.hpp"        // Include short list of convenience functions for rendering

#include <algorithm>            // std::min, std::max

// Helper functions
void register_glfw_callbacks(window& app, glfw_state& app_state);

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "pcl/common/eigen.h"
#include "pcl/common/transforms.h"
#include "pcl/common/distances.h"

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>
#include <string>

#include <opencv2/opencv.hpp>

typedef pcl::PointXYZRGB P_pcl;
typedef pcl::PointCloud<P_pcl> point_cloud;
typedef point_cloud::Ptr ptr_cloud;

std::tuple<uint8_t, uint8_t, uint8_t> get_texcolor(rs2::video_frame texture, rs2::texture_coordinate texcoords)
{
    const int w = texture.get_width(), h = texture.get_height();

    // convert normals [u v] to basic coords [x y]
    int x = std::min(std::max(int(texcoords.u*w + .5f), 0), w - 1);
    int y = std::min(std::max(int(texcoords.v*h + .5f), 0), h - 1);

    int idx = x * texture.get_bytes_per_pixel() + y * texture.get_stride_in_bytes();
    const auto texture_data = reinterpret_cast<const uint8_t*>(texture.get_data());
    return std::tuple<uint8_t, uint8_t, uint8_t>(texture_data[idx], texture_data[idx+1], texture_data[idx+2]);
}


ptr_cloud points_to_pcl(const rs2::points& points, const rs2::video_frame& color){

    // OpenCV Mat for showing the rgb color image, just as part of processing
    cv::Mat colorr(cv::Size(1280, 720), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
    namedWindow("Display Image", cv::WINDOW_AUTOSIZE );
    imshow("Display Image", colorr);

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    ptr_cloud cloud(new point_cloud);

    // Config of PCL Cloud object
    cloud->width = static_cast<uint32_t>(sp.width());
    cloud->height = static_cast<uint32_t>(sp.height());
    cloud->is_dense = false;
    cloud->points.resize(points.size());

    auto tex_coords = points.get_texture_coordinates();
    auto vertices = points.get_vertices();

    // Iterating through all points and setting XYZ coordinates
    // and RGB values
    for (int i = 0; i < points.size(); ++i)
    {
        cloud->points[i].x = vertices[i].x;
        cloud->points[i].y = vertices[i].y;
        cloud->points[i].z = vertices[i].z;

        std::tuple<uint8_t, uint8_t, uint8_t> current_color;
        current_color =get_texcolor(color, tex_coords[i]);

        // Reversed order- 2-1-0 because of BGR model used in camera
        cloud->points[i].r = std::get<2>(current_color);
        cloud->points[i].g = std::get<1>(current_color);
        cloud->points[i].b = std::get<0>(current_color);

    }

   return cloud;
}

std::vector<pcl::PointXYZ> selectedPoints;

void pp_callback(const pcl::visualization::PointPickingEvent& event, void* viewer_void)
{
    //std::ofstream myfile;
   // myfile.open("test1.txt", std::ofstream::out | std::ofstream::trunc);
    std::ofstream myfile1("error_test_3Dcamera.txt", std::ios::app | std::ofstream::binary);

    std::cout << "Picking event active" << std::endl;
    std::vector<double> fromQ;
    if(event.getPointIndex() != -1)
    {
        float x, y, z;
        event.getPoint(x, y, z);
        fromQ={x,y,z,0,0,0};

        std::ofstream myfile("Camera_points.txt", std::ios::app | std::ofstream::binary);
        for(size_t i = 0; i < fromQ.size()-1; i++)

            myfile << fromQ[i] << " ";

        myfile << fromQ[fromQ.size()-1]<< std::endl;

        std::cout << x << ", " << y << ", " << z << std::endl;
        selectedPoints.push_back(pcl::PointXYZ(x, y, z));
        if (selectedPoints.size() > 1)
            {
                float paper_size = 0.23;
                float distance = pcl::euclideanDistance(selectedPoints[0], selectedPoints[1]);
                std::cout << "Distance is " << distance << std::endl;
                float error= abs(paper_size-distance);
                myfile1 << error << std::endl;
            }
    }
}

int main(int argc, char * argv[]) try
{
    // Create a simple OpenGL window for rendering:
    window app(1280, 720, "RealSense Pointcloud Example");
    // Construct an object to manage view state
    glfw_state app_state;
    // register callbacks to allow manipulation of the pointcloud
    register_glfw_callbacks(app, app_state);

    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc;
    // We want the points object to be persistent so we can display the last cloud when a frame drops
    rs2::points points;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

    // Start streaming with default recommended configuration
    pipe.start();

    while (app) // Application still alive?
    {

        // Wait for the next set of frames from the camera
        auto frames = pipe.wait_for_frames();

        auto color = frames.get_color_frame();

        // For cameras that don't have RGB sensor, we'll map the pointcloud to infrared instead of color
        if (!color)
            color = frames.get_infrared_frame();

        // Tell pointcloud object to map to this color frame
        pc.map_to(color);

        auto depth = frames.get_depth_frame();

        // Generate the pointcloud and texture mappings
        points = pc.calculate(depth);

        // Upload the color frame to OpenGL
        app_state.tex.upload(color);

        // Draw the pointcloud
        draw_pointcloud(app.width(), app.height(), app_state, points);
}

    for (int i = 0; i < 30; i++) {
        auto frames = pipe.wait_for_frames(); //Drop several frames for auto-exposure
    }
    // Wait for the next set of frames from the camera
    auto frames = pipe.wait_for_frames();

    auto color1 = frames.get_color_frame();




    // Save current point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    //std::cout <<"I did it" <<std::endl;
    ptr_cloud cloud = points_to_pcl(points, color1);

    // filter everthing out with a distance further than 1m.
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 0.7);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_filtered);

    //Saving the point cloud
    pcl::io::savePCDFileASCII("cloud_test.pcd", *cloud_filtered);


    // Vizualize point cloud and generate point clicking event

    //Given a point cloud:
    //ptr_cloud cloud = points_to_pcl(points, color1);

//    //A Kdtree is then generated to perform an efficient range search:
//    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
//    kdtree.setInputCloud (cloud_filtered);

//    //Then, given a point and a radius:
//    pcl::PointXYZRGB searchPoint(0,0,0.5);
//    float radius = 0.05;

//    // Get all points within the radius distance of the point given (searchPoint)
//    std::vector<int> pointIdxRadiusSearch; //to store index of surrounding points
//    std::vector<float> pointRadiusSquaredDistance; // to store distance to surrounding points

//    if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
//    {
//        for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
//            std::cout << "    "  <<   cloud_filtered->points[ pointIdxRadiusSearch[i] ].x
//                    << " " << cloud_filtered->points[ pointIdxRadiusSearch[i] ].y
//                    << " " << cloud_filtered->points[ pointIdxRadiusSearch[i] ].z
//                    << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
//    }

//    //Print all the surrounding points and their distance to the searchPoint use (SHIFT + pick)

//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
//    for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
//        cloud_cluster->points.push_back(cloud->points[ pointIdxRadiusSearch[i] ]);
//    cloud_cluster->width = cloud_cluster->points.size ();
//    cloud_cluster->height = 1;
//    cloud_cluster->is_dense = true;

//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
//    double rotx = M_PI;
//    double roty = M_PI;
//    double rotz = 0.0;
//    Eigen::Matrix4f rotMatrixX;
//    Eigen::Matrix4f rotMatrixY;
//    Eigen::Matrix4f rotMatrixZ;

//    rotMatrixX <<
//    1.0, 0.0, 0.0, 0.0,
//    0.0, cos(rotx), -sin(rotx), 0.0,
//    0.0, sin(rotx), cos(rotx), 0.0,
//    0.0, 0.0, 0.0, 1.0;

//    rotMatrixY <<
//    cos(roty), 0.0, sin(roty), 0.0,
//    0.0, 1.0, 0.0, 0.0,
//    -sin(roty), 0.0, cos(roty), 0.0,
//    0.0, 0.0, 0.0, 1.0;

//    rotMatrixZ <<
//    cos(rotz), -sin(rotz), 0.0, 0.0,
//    sin(rotz), cos(rotz), 0.0, 0.0,
//    0.0, 0.0, 1.0, 0.0,
//    0.0, 0.0, 0.0, 1.0;

//    pcl::transformPointCloud(*cloud_filtered,*transformed_cloud, rotMatrixX * rotMatrixY * rotMatrixZ);
//        pcl::io::savePCDFileASCII("cloud_test_trans.pcd", *transformed_cloud);
    pcl::visualization::PCLVisualizer visualizer("PCL visualizer");
    visualizer.setBackgroundColor (0.251, 0.251, 0.251);// Floral white 1, 0.98, 0.94 | Misty Rose 1, 0.912, 0.9 |
    //visualizer.addCoordinateSystem (1.0);
    visualizer.registerPointPickingCallback(pp_callback, (void*)&visualizer);
    visualizer.addPointCloud<pcl::PointXYZRGB> (cloud_filtered, "sample cloud",0);
    visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");
    visualizer.initCameraParameters();
    visualizer.spin();

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}

//ROBUST POSE ESTIMATION
//#include <Eigen/Core>
//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
//#include <pcl/common/time.h>
//#include <pcl/console/print.h>
//#include <pcl/features/normal_3d_omp.h>
//#include <pcl/features/fpfh_omp.h>
//#include <pcl/filters/filter.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/filters/statistical_outlier_removal.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/registration/icp.h>
//#include <pcl/registration/sample_consensus_prerejective.h>
//#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/visualization/pcl_visualizer.h>

//// Types
//typedef pcl::PointNormal PointNT;
//typedef pcl::PointCloud<PointNT> PointCloudT;
//typedef pcl::FPFHSignature33 FeatureT;
//typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
//typedef pcl::PointCloud<FeatureT> FeatureCloudT;
//typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;

//// Align a rigid object to a scene with clutter and occlusions
//int
//main (int argc, char **argv)
//{
//  // Point clouds
//  PointCloudT::Ptr object (new PointCloudT);
//  PointCloudT::Ptr object_aligned (new PointCloudT);
//  PointCloudT::Ptr scene (new PointCloudT);
//  FeatureCloudT::Ptr object_features (new FeatureCloudT);
//  FeatureCloudT::Ptr scene_features (new FeatureCloudT);

//  // Get input object and scene
////  if (argc != 3)
////  {
////    pcl::console::print_error ("Syntax is: %s object.pcd scene.pcd\n", argv[0]);
////    return (1);
////  }

////  // Load object and scene
////  pcl::console::print_highlight ("Loading point clouds...\n");
////  if (pcl::io::loadPCDFile<PointNT> (argv[1], *object) < 0 ||
////      pcl::io::loadPCDFile<PointNT> (argv[2], *scene) < 0)
////  {
////    pcl::console::print_error ("Error loading object/scene file!\n");
////    return (1);
////  }

//  //Load pcl file created with get25DImage()
//  if (pcl::io::loadPCDFile("/home/anders/Master/Hand-eye-Calibration/Robot_control/workcell/m2.pcd", *object) == -1)
//  {
//    PCL_ERROR ("Couldn't read object pointcloud \n");
//    return 0;
//  }
//  if (pcl::io::loadPCDFile(/*"/home/anders/Master/Hand-eye-Calibration/Robot_control/workcell/cloud_test.pcd"*/"cloud_test_trans.pcd", *scene) == -1)
//  {
//    PCL_ERROR ("Couldn't read scene pointcloud \n");
//    return 0;
//  }
//  for(size_t i = 0; i < object->points.size(); ++i)
//   {
//        object->points[i].x *= 0.1;
//        object->points[i].y *= 0.1;
//        object->points[i].z *= 0.1;
//  }

//  pcl::visualization::PCLVisualizer visu1("Original");
//  visu1.addPointCloud (scene, ColorHandlerT (scene, 0.0, 255.0, 0.0), "scene");
//  visu1.addPointCloud (object, ColorHandlerT (object, 0.0, 0.0, 255.0), "object_aligned");
//  visu1.spin ();

//  pcl::StatisticalOutlierRemoval<pcl::PointNormal> sor;
//     sor.setInputCloud (scene);
//     sor.setMeanK (25);
//     sor.setStddevMulThresh (1.0f);
//     sor.filter (*scene);

//     pcl::visualization::PCLVisualizer visu2("Outlier");
//     visu2.addPointCloud (scene, ColorHandlerT (scene, 0.0, 255.0, 0.0), "scene");
//     visu2.addPointCloud (object, ColorHandlerT (object, 0.0, 0.0, 255.0), "object_aligned");
//     visu2.spin ();

//  // Downsample
//  pcl::console::print_highlight ("Downsampling...\n");
//  pcl::VoxelGrid<PointNT> grid;
//  const float leaf = 0.005f;
//  grid.setLeafSize (leaf, leaf, leaf);
//  grid.setInputCloud (object);
//  grid.filter (*object);
//  grid.setInputCloud (scene);
//  grid.filter (*scene);

//  pcl::visualization::PCLVisualizer visu3("Downsample");
//  visu3.addPointCloud (scene, ColorHandlerT (scene, 0.0, 255.0, 0.0), "scene");
//  visu3.addPointCloud (object, ColorHandlerT (object, 0.0, 0.0, 255.0), "object_aligned");
//  visu3.spin ();

//  // Estimate normals for scene
//  pcl::console::print_highlight ("Estimating scene normals...\n");
//  pcl::NormalEstimationOMP<PointNT,PointNT> nest;
//  nest.setRadiusSearch (0.01);
//  nest.setInputCloud (scene);
//  nest.compute (*scene);

//  // Estimate features
//  pcl::console::print_highlight ("Estimating features...\n");
//  FeatureEstimationT fest;
//  fest.setRadiusSearch (0.025);
//  fest.setInputCloud (object);
//  fest.setInputNormals (object);
//  fest.compute (*object_features);
//  fest.setInputCloud (scene);
//  fest.setInputNormals (scene);
//  fest.compute (*scene_features);

//  // Perform alignment
//  pcl::console::print_highlight ("Starting alignment...\n");
//  pcl::SampleConsensusPrerejective<PointNT,PointNT,FeatureT> align;
//  align.setInputSource (object);
//  align.setSourceFeatures (object_features);
//  align.setInputTarget (scene);
//  align.setTargetFeatures (scene_features);
//  align.setMaximumIterations (50000); // Number of RANSAC iterations
//  align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
//  align.setCorrespondenceRandomness (2); // Number of nearest features to use
//  align.setSimilarityThreshold (0.6f); // Polygonal edge length similarity threshold
//  align.setMaxCorrespondenceDistance (0.01f * leaf); // Inlier threshold
//  align.setInlierFraction (0.25f); // Required inlier fraction for accepting a pose hypothesis
//  {
//    pcl::ScopeTime t("Alignment");
//    align.align (*object_aligned);
//  }

//  if (align.hasConverged ())
//  {
//    // Print results
//    printf ("\n");
//    Eigen::Matrix4f transformation = align.getFinalTransformation ();
//    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
//    pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
//    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
//    pcl::console::print_info ("\n");
//    pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
//    pcl::console::print_info ("\n");
//    pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers ().size (), object->size ());

//    // Show alignment
//    pcl::visualization::PCLVisualizer visu("Alignment");
//    visu.addPointCloud (scene, ColorHandlerT (scene, 0.0, 255.0, 0.0), "scene");
//    visu.addPointCloud (object_aligned, ColorHandlerT (object_aligned, 0.0, 0.0, 255.0), "object_aligned");
//    visu.spin ();
//  }
//  else
//  {
//    pcl::console::print_error ("Alignment failed!\n");
//    return (1);
//  }

//  return (0);
//}
