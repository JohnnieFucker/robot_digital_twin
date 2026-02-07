#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>

// 定义点云类型
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// 输出 JSON 辅助函数
std::string vec3ToJson(const Eigen::Vector3f &v) {
  std::stringstream ss;
  ss << "{\"x\": " << v[0] << ", \"y\": " << v[1] << ", \"z\": " << v[2] << "}";
  return ss.str();
}

int main(int argc, char **argv) {
  // 1. 加载点云数据
  std::string filename = (argc > 1) ? argv[1] : "cloud.ply";
  PointCloudT::Ptr cloud(new PointCloudT);

  if (pcl::io::loadPLYFile<PointT>(filename, *cloud) == -1) {
    std::cerr << "[错误] 无法读取文件: " << filename << std::endl;
    return -1;
  }
  std::cerr << "[信息] 成功加载点云，点数: " << cloud->points.size() << std::endl;

  // 2. 计算法线 (圆柱体分割需要法线)
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(
      new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
  ne.setSearchMethod(tree);
  ne.setInputCloud(cloud);
  ne.setKSearch(50); // 设置 K 近邻搜索数量
  ne.compute(*cloud_normals);
  std::cerr << "[信息] 法线计算完成，法线数量: " << cloud_normals->size() << std::endl;

  // 3. 提取全局主轴方向 (所有管子平行)
  // 通过法线在球面的分布找到主轴：轴线方向与所有法线垂直
  std::cerr << "[信息] 正在提取全局主轴方向..." << std::endl;
  Eigen::Vector3f global_axis(0, 0, 0);
  {
      // 修正：pcl::Normal 没有 x,y,z 成员，只有 normal_x, normal_y, normal_z
      // 我们需要手动构建协方差矩阵，或者将法线转换为 PointXYZ 类型
      Eigen::Matrix3f covariance_matrix = Eigen::Matrix3f::Zero();
      Eigen::Vector3f centroid(0, 0, 0);
      
      for (const auto& n : *cloud_normals) {
          centroid += Eigen::Vector3f(n.normal_x, n.normal_y, n.normal_z);
      }
      centroid /= static_cast<float>(cloud_normals->size());

      for (const auto& n : *cloud_normals) {
          Eigen::Vector3f vec(n.normal_x, n.normal_y, n.normal_z);
          Eigen::Vector3f centered = vec - centroid;
          covariance_matrix += centered * centered.transpose();
      }

      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance_matrix, Eigen::ComputeEigenvectors);
      // 特征值最小的特征向量即为法线最不集中的方向，即轴线方向
      global_axis = eigen_solver.eigenvectors().col(0); 
      global_axis.normalize();
  }
  std::cerr << "[数据] 提取的全局主轴方向: (" << global_axis[0] << ", " << global_axis[1] << ", " << global_axis[2] << ")" << std::endl;

  // 4. 分割圆柱体 (Pipe) - 引入全局轴线约束
  std::vector<pcl::ModelCoefficients::Ptr> cyl_coeffs;
  std::vector<PointCloudT::Ptr> cyl_clouds;
  
  PointCloudT::Ptr cloud_iter(new PointCloudT);
  pcl::copyPointCloud(*cloud, *cloud_iter);
  pcl::PointCloud<pcl::Normal>::Ptr normals_iter(new pcl::PointCloud<pcl::Normal>);
  pcl::copyPointCloud(*cloud_normals, *normals_iter);

  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg_cyl;
  seg_cyl.setOptimizeCoefficients(true);
  seg_cyl.setModelType(pcl::SACMODEL_CYLINDER);
  seg_cyl.setMethodType(pcl::SAC_RANSAC);
  seg_cyl.setNormalDistanceWeight(0.1); 
  seg_cyl.setMaxIterations(10000);
  seg_cyl.setDistanceThreshold(3.0);      // 稍微放宽阈值以适应噪声
  seg_cyl.setRadiusLimits(10.0, 60.0);    // 假设管子半径在 10-60mm 之间
  
  // 关键：强制圆柱轴线接近全局主轴
  seg_cyl.setAxis(global_axis);
  seg_cyl.setEpsAngle(0.17); // 允许约 10 度的偏差

  pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;

  int cyl_count = 0;
  const int MAX_CYLINDERS = 15; 

  while (cyl_count < MAX_CYLINDERS) {
    pcl::ModelCoefficients::Ptr coeff_cyl(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_cyl(new pcl::PointIndices);

    seg_cyl.setInputCloud(cloud_iter);
    seg_cyl.setInputNormals(normals_iter);
    seg_cyl.segment(*inliers_cyl, *coeff_cyl);

    // 停止条件：内点过少
    if (inliers_cyl->indices.size() < 2000) { // 增加最小点数要求
      break;
    }

    // 验证拟合出的半径是否合理（例如 15-50mm）
    float r = coeff_cyl->values[6];
    if (r < 10.0 || r > 60.0) {
        // 剔除这个错误的拟合，继续尝试
        extract.setInputCloud(cloud_iter);
        extract.setIndices(inliers_cyl);
        extract.setNegative(true);
        PointCloudT::Ptr cloud_tmp(new PointCloudT);
        extract.filter(*cloud_tmp);
        cloud_iter = cloud_tmp;
        continue;
    }

    std::cerr << "[信息] 成功拟合第 " << cyl_count + 1 << " 根管子，半径: " << r << "mm" << std::endl;
    cyl_coeffs.push_back(coeff_cyl);
    
    PointCloudT::Ptr cloud_cylinder(new PointCloudT);
    extract.setInputCloud(cloud_iter);
    extract.setIndices(inliers_cyl);
    extract.setNegative(false);
    extract.filter(*cloud_cylinder);
    cyl_clouds.push_back(cloud_cylinder);

    PointCloudT::Ptr cloud_next(new PointCloudT);
    pcl::PointCloud<pcl::Normal>::Ptr normals_next(new pcl::PointCloud<pcl::Normal>);
    extract.setNegative(true);
    extract.filter(*cloud_next);
    extract_normals.setInputCloud(normals_iter);
    extract_normals.setIndices(inliers_cyl);
    extract_normals.setNegative(true);
    extract_normals.filter(*normals_next);

    cloud_iter = cloud_next;
    normals_iter = normals_next;
    cyl_count++;
    if (cloud_iter->points.size() < 5000) break;
  }

  if (cyl_coeffs.empty()) {
    std::cerr << "[错误] 未能分割出任何圆柱体。" << std::endl;
    return -1;
  }
  
  std::cerr << "[信息] 共分割出 " << cyl_coeffs.size() << " 根圆柱。" << std::endl;

  // 4. 分割平面 (Flat Steel) - 剩余的点云视为平面（或多个平面）
  // 假设剩余点云主要就是扁钢部分
  PointCloudT::Ptr cloud_remaining = cloud_iter;
  std::cerr << "[信息] 移除所有圆柱后剩余点数: " << cloud_remaining->points.size() << std::endl;

  // 分割平面
  pcl::SACSegmentation<PointT> seg_plane;
  pcl::ModelCoefficients::Ptr coeff_plane(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);

  seg_plane.setOptimizeCoefficients(true);
  seg_plane.setModelType(pcl::SACMODEL_PLANE);
  seg_plane.setMethodType(pcl::SAC_RANSAC);
  seg_plane.setDistanceThreshold(2.0); // 2mm 阈值
  seg_plane.setInputCloud(cloud_remaining);
  seg_plane.segment(*inliers_plane, *coeff_plane);

  std::cerr << "[信息] 平面分割完成，内点数量: " << inliers_plane->indices.size() << std::endl;

  if (inliers_plane->indices.empty()) {
    std::cerr << "[错误] 未能分割出平面。" << std::endl;
    // 如果没有平面，可能是纯管排结构？这里先报错返回，或者给一个默认平面
    return -1;
  }

  // 提取平面点云
  PointCloudT::Ptr cloud_plane(new PointCloudT);
  extract.setInputCloud(cloud_remaining);
  extract.setIndices(inliers_plane);
  extract.setNegative(false);
  extract.filter(*cloud_plane);

  // 平面参数: ax + by + cz + d = 0
  Eigen::Vector3f plane_normal(coeff_plane->values[0], coeff_plane->values[1],
                               coeff_plane->values[2]);
  float plane_d = coeff_plane->values[3];
  
  std::cerr << "[数据] 平面参数: 法线=(" << plane_normal[0] << ", " << plane_normal[1] 
            << ", " << plane_normal[2] << "), D=" << plane_d << "mm" << std::endl;

  // 准备存储所有圆柱的数据结构
  struct CylinderData {
      Eigen::Vector3f center;
      Eigen::Vector3f axis;
      float radius;
      float length;
  };
  std::vector<CylinderData> cylinders_data;
  std::vector<std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>>> all_weld_seams;
  int total_seams_count = 0;

  // 遍历所有找到的圆柱，进行处理
  for (size_t i = 0; i < cyl_coeffs.size(); ++i) {
      pcl::ModelCoefficients::Ptr coeff = cyl_coeffs[i];
      PointCloudT::Ptr cloud_cyl = cyl_clouds[i];

      Eigen::Vector3f cyl_pt(coeff->values[0], coeff->values[1], coeff->values[2]);
      Eigen::Vector3f cyl_axis(coeff->values[3], coeff->values[4], coeff->values[5]);
      float cyl_radius = coeff->values[6];
      cyl_axis.normalize();
      
      std::cerr << "\n[处理圆柱 " << i+1 << "] 原始半径: " << cyl_radius << "mm" << std::endl;

      // --- 几何约束优化 ---
      float abs_dot = std::abs(cyl_axis.dot(plane_normal));
      if (abs_dot < 0.17) { // < 10 degrees
        // 校正轴线
        Eigen::Vector3f corrected_axis = cyl_axis - plane_normal * cyl_axis.dot(plane_normal);
        corrected_axis.normalize();
        cyl_axis = corrected_axis;
        std::cerr << "[优化] 轴线已校正为平行于平面。" << std::endl;
      }

      // --- 计算长度 ---
      std::vector<float> projections;
      projections.reserve(cloud_cyl->points.size());
      for (const auto &pt : cloud_cyl->points) {
        Eigen::Vector3f p(pt.x, pt.y, pt.z);
        float proj = (p - cyl_pt).dot(cyl_axis);
        projections.push_back(proj);
      }

      float min_proj = 0.0f;
      float max_proj = 0.0f;
      float cyl_length = 0.0f;

      if (!projections.empty()) {
        std::sort(projections.begin(), projections.end());
        size_t idx_min = static_cast<size_t>(projections.size() * 0.01);
        size_t idx_max = static_cast<size_t>(projections.size() * 0.99);
        if (idx_min >= projections.size()) idx_min = 0;
        if (idx_max >= projections.size()) idx_max = projections.size() - 1;
        if (idx_min > idx_max) idx_min = idx_max;
        min_proj = projections[idx_min];
        max_proj = projections[idx_max];
        cyl_length = max_proj - min_proj;
      }
      
      Eigen::Vector3f cyl_center_point = cyl_pt + cyl_axis * ((max_proj + min_proj) / 2.0);
      
      CylinderData data;
      data.center = cyl_center_point;
      data.axis = cyl_axis;
      data.radius = cyl_radius;
      data.length = cyl_length;
      cylinders_data.push_back(data);

      std::cerr << "[数据] 长度: " << cyl_length << "mm" << std::endl;

      // --- 计算焊缝 ---
      std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> current_weld_seams;
      float axis_to_plane_dist = std::abs(plane_normal.dot(cyl_pt) + plane_d);
      
      // 侧向量
      Eigen::Vector3f side_vec = plane_normal.cross(cyl_axis).normalized();
      // 投影中心
      float signed_dist = plane_normal.dot(cyl_pt) + plane_d;
      Eigen::Vector3f projected_center = cyl_pt - plane_normal * signed_dist;
      // 焊缝基准线端点
      Eigen::Vector3f start_base = projected_center - cyl_axis * (cyl_length / 2.0);
      Eigen::Vector3f end_base = projected_center + cyl_axis * (cyl_length / 2.0);

      if (axis_to_plane_dist < cyl_radius + 15.0) { // 放宽到 15mm 误差
         // 左侧
         current_weld_seams.push_back({start_base + side_vec * cyl_radius, end_base + side_vec * cyl_radius});
         // 右侧
         current_weld_seams.push_back({start_base - side_vec * cyl_radius, end_base - side_vec * cyl_radius});
         std::cerr << "[优化] 生成 2 条焊缝。" << std::endl;
         total_seams_count += 2;
      } else {
         std::cerr << "[警告] 距离平面过远 (" << axis_to_plane_dist << "mm vs " << cyl_radius << "mm)，忽略焊缝。" << std::endl;
      }
      all_weld_seams.push_back(current_weld_seams);
  }

  // 计算平面尺寸 (使用 PCA 或者 OBB)
  pcl::MomentOfInertiaEstimation<PointT> feature_extractor;
  feature_extractor.setInputCloud(cloud_plane);
  feature_extractor.compute();

  PointT min_point_OBB, max_point_OBB;
  PointT position_OBB;
  Eigen::Matrix3f rotational_matrix_OBB;
  feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB,
                           rotational_matrix_OBB);

  float plate_width = max_point_OBB.y - min_point_OBB.y;
  float plate_length_obb = max_point_OBB.x - min_point_OBB.x;
  // 确保 width 是较短边，length 是较长边 (通常习惯)
  if (plate_width > plate_length_obb)
    std::swap(plate_width, plate_length_obb);
  
  std::cerr << "[数据] 平板尺寸: 宽=" << plate_width << "mm, 长=" << plate_length_obb << "mm" << std::endl;

  // 6. 此时可以计算工件整体包围盒 (AABB)
  PointT min_pt, max_pt;
  pcl::getMinMax3D(*cloud, min_pt, max_pt);
  float workpiece_L = max_pt.x - min_pt.x;
  float workpiece_W = max_pt.y - min_pt.y;
  float workpiece_H = max_pt.z - min_pt.z;
  
  std::cerr << "[数据] 工件整体包围盒(AABB): L=" << workpiece_L << "mm, W=" << workpiece_W 
            << "mm, H=" << workpiece_H << "mm" << std::endl;

  // 8. 输出 JSON
  std::cout << "{" << std::endl;
  std::cout << "  \"result\": \"success\"," << std::endl;
  std::cout << "  \"workpiece\": {" << std::endl;
  std::cout << "    \"length\": " << workpiece_L << "," << std::endl;
  std::cout << "    \"width\": " << workpiece_W << "," << std::endl;
  std::cout << "    \"height\": " << workpiece_H << std::endl;
  std::cout << "  }," << std::endl;

  std::cout << "  \"params\": {" << std::endl;
  // 取第一个圆柱的参数作为参考，或者输出平均值
  float avg_radius = 0;
  float avg_length = 0;
  if (!cylinders_data.empty()) {
      for(const auto& d : cylinders_data) { avg_radius += d.radius; avg_length += d.length; }
      avg_radius /= cylinders_data.size();
      avg_length /= cylinders_data.size();
  }
  std::cout << "    \"pipe_diameter\": " << avg_radius * 2.0 << ","
            << std::endl;
  std::cout << "    \"flat_steel_width\": " << plate_width << "," << std::endl;
  std::cout << "    \"pipe_length\": " << avg_length << "," << std::endl;
  std::cout << "    \"pipe_count\": " << cylinders_data.size() << "," << std::endl;
  std::cout << "    \"weld_seam_count\": " << total_seams_count << std::endl;
  std::cout << "  }," << std::endl;

  std::cout << "  \"weld_seams\": [" << std::endl;
  bool first_seam = true;
  for (const auto& seams : all_weld_seams) {
      for (const auto& seam : seams) {
        if (!first_seam) std::cout << "," << std::endl;
        std::cout << "    {" << std::endl;
        std::cout << "      \"start\": " << vec3ToJson(seam.first) << ","
                  << std::endl;
        std::cout << "      \"end\": " << vec3ToJson(seam.second) << ","
                  << std::endl;
        std::cout << "      \"length\": " << avg_length << std::endl; // 近似长度
        std::cout << "    }";
        first_seam = false;
      }
  }
  std::cout << std::endl << "  ]," << std::endl;

  // 简化的 3D 信息用于 Three.js
  std::cout << "  \"digital_twin\": {" << std::endl;
  std::cout << "    \"cylinders\": [" << std::endl;
  for (size_t i = 0; i < cylinders_data.size(); ++i) {
      std::cout << "      {" << std::endl;
      std::cout << "        \"center\": " << vec3ToJson(cylinders_data[i].center) << ","
                << std::endl;
      std::cout << "        \"axis\": " << vec3ToJson(cylinders_data[i].axis) << "," << std::endl;
      std::cout << "        \"radius\": " << cylinders_data[i].radius << "," << std::endl;
      std::cout << "        \"height\": " << cylinders_data[i].length << std::endl;
      std::cout << "      }" << (i < cylinders_data.size() - 1 ? "," : "") << std::endl;
  }
  std::cout << "    ]," << std::endl;

  std::cout << "    \"plane\": {" << std::endl;
  std::cout << "      \"center\": "
            << vec3ToJson(Eigen::Vector3f(position_OBB.x, position_OBB.y,
                                          position_OBB.z))
            << "," << std::endl;
  std::cout << "      \"normal\": " << vec3ToJson(plane_normal) << ","
            << std::endl;
  std::cout << "      \"size\": "
            << vec3ToJson(Eigen::Vector3f(plate_length_obb, plate_width, 0.01))
            << "," << std::endl; // 厚度设为小值
  std::cout << "      \"rotation_matrix\": [" << rotational_matrix_OBB(0, 0)
            << "," << rotational_matrix_OBB(0, 1) << ","
            << rotational_matrix_OBB(0, 2) << "," << rotational_matrix_OBB(1, 0)
            << "," << rotational_matrix_OBB(1, 1) << ","
            << rotational_matrix_OBB(1, 2) << "," << rotational_matrix_OBB(2, 0)
            << "," << rotational_matrix_OBB(2, 1) << ","
            << rotational_matrix_OBB(2, 2) << "]" << std::endl;
  std::cout << "    }" << std::endl;
  std::cout << "  }" << std::endl;

  std::cout << "}" << std::endl;

  return 0;
}
