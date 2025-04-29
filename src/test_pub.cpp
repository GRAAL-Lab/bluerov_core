#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <chrono>

using namespace std;
using namespace pcl;
using namespace Eigen;
using namespace std::chrono;

// Function to compute the transformation (rotation and translation) between two point clouds
void computeTransformation(const PointCloud<PointXYZ>::Ptr& source, const PointCloud<PointXYZ>::Ptr& target,
                           Matrix4f& transformation)
{
    int num_points = source->points.size();
    if (num_points != target->points.size() || num_points < 3)
    {
        cerr << "Point clouds must have the same number of points and at least 3 points!" << endl;
        return;
    }

    // Compute the centroids of both point clouds
    PointXYZ source_centroid(0, 0, 0);
    PointXYZ target_centroid(0, 0, 0);
    
    for (int i = 0; i < num_points; ++i)
    {
        source_centroid.x += source->points[i].x;
        source_centroid.y += source->points[i].y;
        source_centroid.z += source->points[i].z;

        target_centroid.x += target->points[i].x;
        target_centroid.y += target->points[i].y;
        target_centroid.z += target->points[i].z;
    }
    source_centroid.x /= num_points;
    source_centroid.y /= num_points;
    source_centroid.z /= num_points;

    target_centroid.x /= num_points;
    target_centroid.y /= num_points;
    target_centroid.z /= num_points;

    // Center the point clouds
    PointCloud<PointXYZ>::Ptr centered_source(new PointCloud<PointXYZ>());
    PointCloud<PointXYZ>::Ptr centered_target(new PointCloud<PointXYZ>());

    for (int i = 0; i < num_points; ++i)
    {
        PointXYZ source_point = source->points[i];
        PointXYZ target_point = target->points[i];

        centered_source->points.push_back(PointXYZ(source_point.x - source_centroid.x, 
                                                   source_point.y - source_centroid.y, 
                                                   source_point.z - source_centroid.z));

        centered_target->points.push_back(PointXYZ(target_point.x - target_centroid.x, 
                                                   target_point.y - target_centroid.y, 
                                                   target_point.z - target_centroid.z));
    }

    // Compute the covariance matrix H
    Matrix3f H = Matrix3f::Zero();
    for (int i = 0; i < num_points; ++i)
    {
        Vector3f source_vec(centered_source->points[i].x, centered_source->points[i].y, centered_source->points[i].z);
        Vector3f target_vec(centered_target->points[i].x, centered_target->points[i].y, centered_target->points[i].z);
        
        H += source_vec * target_vec.transpose();
    }

    // Perform SVD decomposition of H
    JacobiSVD<MatrixXf> svd(H, ComputeFullU | ComputeFullV);
    Matrix3f rotation_matrix = svd.matrixU() * svd.matrixV().transpose();

    // Compute translation vector
    Vector3f translation = Vector3f(target_centroid.x, target_centroid.y, target_centroid.z) - 
                           rotation_matrix * Vector3f(source_centroid.x, source_centroid.y, source_centroid.z);

    // Construct the transformation matrix
    transformation.setIdentity();
    transformation.block<3, 3>(0, 0) = rotation_matrix;
    transformation.block<3, 1>(0, 3) = translation;
}

// Function to perform ICP registration
void icp(PointCloud<PointXYZ>::Ptr& source, PointCloud<PointXYZ>::Ptr& target, int max_iterations = 50, float tolerance = 1e-6)
{
    Matrix4f transformation = Matrix4f::Identity();

    // KD-Tree for nearest neighbor search
    KdTreeFLANN<PointXYZ> kdtree;
    kdtree.setInputCloud(target);

    PointCloud<PointXYZ>::Ptr transformed_source(new PointCloud<PointXYZ>());

    for (int iter = 0; iter < max_iterations; ++iter)
    {
        // Apply the current transformation to the source point cloud
        transformed_source->clear();
        for (size_t i = 0; i < source->points.size(); ++i)
        {
            PointXYZ p = source->points[i];
            Eigen::Vector4f p_homogeneous(p.x, p.y, p.z, 1);
            Eigen::Vector4f transformed_p = transformation * p_homogeneous;
            transformed_source->points.push_back(PointXYZ(transformed_p[0], transformed_p[1], transformed_p[2]));
        }

        // Find the closest points in the target for each point in the transformed source
        float total_error = 0.0f;
        vector<int> indices;
        vector<float> distances;

        for (size_t i = 0; i < transformed_source->points.size(); ++i)
        {
            PointXYZ& p = transformed_source->points[i];
            kdtree.nearestKSearch(p, 1, indices, distances);
            total_error += distances[0];
        }

        // If the total error is below the threshold, stop the algorithm
        if (total_error < tolerance)
        {
            cout << "Converged in " << iter << " iterations." << endl;
            break;
        }

        // Compute the new transformation using point-to-point association
        computeTransformation(transformed_source, target, transformation);
    }

    cout << "Final Transformation Matrix:" << endl;
    cout << transformation << endl;
}

// Main function to test ICP
int main()
{
    // Create two point clouds (source and target) for testing
    PointCloud<PointXYZ>::Ptr source(new PointCloud<PointXYZ>());
    PointCloud<PointXYZ>::Ptr target(new PointCloud<PointXYZ>());

    // Example: manually add some points to source and target (should ideally come from actual data)
    source->points.push_back(PointXYZ(0.0f, 0.0f, 0.0f));
    source->points.push_back(PointXYZ(1.0f, 0.0f, 0.0f));
    source->points.push_back(PointXYZ(0.0f, 1.0f, 0.0f));

    target->points.push_back(PointXYZ(1.0f, 1.0f, 0.0f));
    target->points.push_back(PointXYZ(2.0f, 1.0f, 0.0f));
    target->points.push_back(PointXYZ(1.0f, 2.0f, 0.0f));

    // Measure the runtime of ICP
    auto start = high_resolution_clock::now();

    // Call ICP to register the source to the target
    icp(source, target);

    auto end = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(end - start);

    cout << "ICP runtime: " << duration.count() << " microseconds" << endl;

    return 0;
}
