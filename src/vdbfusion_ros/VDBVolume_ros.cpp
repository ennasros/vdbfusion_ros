// MIT License
//
// # Copyright (c) 2022 Saurabh Gupta, Ignacio Vizzo, Cyrill Stachniss, University of Bonn
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "VDBVolume_ros.hpp"

#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <Eigen/Core>
#include <vector>

#include "igl/write_triangle_mesh.h"
#include "openvdb/openvdb.h"

#include <geometry_msgs/Point.h>
#include <mesh_msgs/MeshGeometryStamped.h>
#include <mesh_msgs/MeshGeometry.h>
#include <mesh_msgs/MeshTriangleIndices.h>

namespace {
std::vector<Eigen::Vector3d> pcl2SensorMsgToEigen(const sensor_msgs::PointCloud2& pcl2) {
    std::vector<Eigen::Vector3d> points;
    points.reserve(pcl2.width);
    sensor_msgs::PointCloud pcl;
    sensor_msgs::convertPointCloud2ToPointCloud(pcl2, pcl);
    std::for_each(pcl.points.begin(), pcl.points.end(), [&](const auto& point) {
        points.emplace_back(Eigen::Vector3d(point.x, point.y, point.z));
    });
    return points;
}

void PreProcessCloud(std::vector<Eigen::Vector3d>& points, float min_range, float max_range) {
    points.erase(
        std::remove_if(points.begin(), points.end(), [&](auto p) { return p.norm() > max_range; }),
        points.end());
    points.erase(
        std::remove_if(points.begin(), points.end(), [&](auto p) { return p.norm() < min_range; }),
        points.end());
}
}  // namespace

vdbfusion::VDBVolume vdbfusion::VDBVolumeNode::InitVDBVolume() {
    float voxel_size;
    float sdf_trunc;
    bool space_carving;

    nh_.getParam("/voxel_size", voxel_size);
    nh_.getParam("/sdf_trunc", sdf_trunc);
    nh_.getParam("/space_carving", space_carving);

    VDBVolume vdb_volume(voxel_size, sdf_trunc, space_carving);

    return vdb_volume;
}

vdbfusion::VDBVolumeNode::VDBVolumeNode() : vdb_volume_(InitVDBVolume()), tf_(nh_) {
    openvdb::initialize();

    std::string pcl_topic;
    nh_.getParam("/pcl_topic", pcl_topic);
    nh_.getParam("/preprocess", preprocess_);
    nh_.getParam("/apply_pose", apply_pose_);
    nh_.getParam("/min_range", min_range_);
    nh_.getParam("/max_range", max_range_);

    nh_.getParam("/fill_holes", fill_holes_);
    nh_.getParam("/min_weight", min_weight_);

    nh_.getParam("/save_publish_wait_time", save_publish_wait_time);
    nh_.getParam("/save_path", save_path_);

    int32_t tol;
    nh_.getParam("/timestamp_tolerance_ns", tol);
    timestamp_tolerance_ = ros::Duration(0, tol);

    const int queue_size = 500;

    sub_ = nh_.subscribe(pcl_topic, queue_size, &vdbfusion::VDBVolumeNode::Integrate, this);
    srv_ = nh_.advertiseService("/save_vdb_volume", &vdbfusion::VDBVolumeNode::saveVDBVolume, this);
    mesh_geometry_pub_ = nh_.advertise<mesh_msgs::MeshGeometryStamped>("mesh", 1, true);

    // check if we are supposed to auto save and publish
    if (save_publish_wait_time > 0)
    {
        ROS_INFO("Setting up timer callback every %f seconds to auto save and publish the integrated volume.", save_publish_wait_time);
        serviceTimer = nh_.createTimer(ros::Duration(save_publish_wait_time), &vdbfusion::VDBVolumeNode::TimerCallback, this);
    }
    else
    {
        ROS_INFO("save_publish_wait_time not set. Use '/save_vdb_volume' service to save the integrated volume.");
    }
}

void vdbfusion::VDBVolumeNode::Integrate(const sensor_msgs::PointCloud2& pcd) {
    geometry_msgs::TransformStamped transform;
    sensor_msgs::PointCloud2 pcd_out;

    if (tf_.lookUpTransform(pcd.header.stamp, timestamp_tolerance_, transform)) {
        ROS_INFO("Transform available");
        if (apply_pose_) {
            tf2::doTransform(pcd, pcd_out, transform);
        }
        auto scan = pcl2SensorMsgToEigen(pcd_out);

        if (preprocess_) {
            PreProcessCloud(scan, min_range_, max_range_);
        }
        const auto& x = transform.transform.translation.x;
        const auto& y = transform.transform.translation.y;
        const auto& z = transform.transform.translation.z;
        auto origin = Eigen::Vector3d(x, y, z);
        vdb_volume_.Integrate(scan, origin, [](float /*unused*/) { return 1.0; });
    }
}

bool vdbfusion::VDBVolumeNode::saveVDBVolume(vdbfusion_ros::save_vdb_volume::Request& path,
                                             vdbfusion_ros::save_vdb_volume::Response& response) {
    ROS_INFO("Saving the mesh and VDB grid files ...");
    std::string volume_name = path.path;
    openvdb::io::File(volume_name + "_grid.vdb").write({vdb_volume_.tsdf_});

    // Run marching cubes and save a .ply file
    auto [vertices, triangles] =
        this->vdb_volume_.ExtractTriangleMesh(this->fill_holes_, this->min_weight_);

    // prepare mesh geometry message
    mesh_msgs::MeshGeometryStamped mesh_msg;

    Eigen::MatrixXd V(vertices.size(), 3);
    for (size_t i = 0; i < vertices.size(); i++) {
        V.row(i) = Eigen::VectorXd::Map(&vertices[i][0], vertices[i].size());

        // populate mesh message vertix data
        geometry_msgs::Point p;
        p.x = vertices[i][0];
        p.y = vertices[i][1];
        p.z = vertices[i][2];
        mesh_msg.mesh_geometry.vertices.push_back(p);
    }

    Eigen::MatrixXi F(triangles.size(), 3);
    for (size_t i = 0; i < triangles.size(); i++) {
        F.row(i) = Eigen::VectorXi::Map(&triangles[i][0], triangles[i].size());
        // populate mesh message face data
        mesh_msgs::MeshTriangleIndices indices;
        indices.vertex_indices[0] = triangles[i][0];
        indices.vertex_indices[1] = triangles[i][1];
        indices.vertex_indices[2] = triangles[i][2];
        mesh_msg.mesh_geometry.faces.push_back(indices);
    }
    igl::write_triangle_mesh(volume_name + "_mesh.ply", V, F, igl::FileEncoding::Binary);
    ROS_INFO("Done saving the mesh and VDB grid files");

    // chicken out if there are no vertices
    if (vertices.size() > 0)
    {
        ROS_INFO("Publishing mesh geometry");
        mesh_msg.uuid = "uuid";
        mesh_msg.header.frame_id = "map";
        mesh_msg.header.stamp = ros::Time::now();
        mesh_geometry_pub_.publish(mesh_msg);
    }
    return true;
}

void vdbfusion::VDBVolumeNode::TimerCallback(const ros::TimerEvent&)
{
    ROS_INFO("Auto invoking service call with save path %s", this->save_path_);
    // call service to save and publish
    vdbfusion_ros::save_vdb_volume::Request req;
    vdbfusion_ros::save_vdb_volume::Response res;
    req.path = this->save_path_;
    this->saveVDBVolume(req, res);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "vdbfusion_rosnode");
    vdbfusion::VDBVolumeNode vdb_volume_node;
    ros::spin();
    return 0;
}
