#include "Hand.h"
#include "pugixml.hpp"
#include "yaml-cpp/yaml.h"
#include "optim.hpp"
#include "SDFchecker.h"

using namespace Eigen;


double objFuncPSO(const arma::vec& X, arma::vec* grad_out, optim::ArgPasser* args)//, PointCloudRGBNormal::Ptr testcloud1, PointCloudRGBNormal::Ptr testcloud2,boost::shared_ptr<visualization_msgs::MarkerArray> line_markers)
{
  // line_markers->markers.clear();
  const std::string name = args->name;
  const float dist_thres = args->dist_thres;
  float score = 0;
  // get current the model tf
  Eigen::Matrix4f tf_self;
  {
    tf_self.setIdentity();
    tf_self(1,3) = X[0];
  }

  Eigen::Matrix4f cur_model2handbase = args->model2handbase * tf_self;

  //We penalize when a pair of gripper too close to hold anything, NOTE: y is inner side. We consider both sides of the finger, tip1:tip side, tip2:palm side
  Eigen::Vector4f tip2;

  tip2 << args->finger_property._min_x, args->finger_property._max_y, args->finger_property._min_z, 1;

  tip2 = cur_model2handbase*tip2; // get the current position of corner of finger

  float gripper_dist2;
  if (name=="r_gripper_finger_link")   //Right side finger
  {
    gripper_dist2 = tip2(1)-args->pair_tip2(1);
  }
  else
  {
    gripper_dist2 = -tip2(1)+args->pair_tip2(1);
  }

  float penalty_gripper_dist = 0;
  const float GRIPPER_MIN_DIST = 0.001;
  // if the finger is too close to another, then it will be penalized 
  if (gripper_dist2<GRIPPER_MIN_DIST)
  {
    penalty_gripper_dist = 1e3 + 1e3*std::abs(GRIPPER_MIN_DIST-gripper_dist2);
    score -= penalty_gripper_dist;
    return -score;   //NOTE: we are minimizing the cost function
  }
 

  float num_match = 0;
  const float PLANAR_DIST_THRES = args->cfg.yml["hand_match"]["planar_dist_thres"].as<float>();
  PointCloudRGBNormal::Ptr model = args->model;
  //NOTE: we compare in handbase frame, so that kdtree only build once
  PointCloudRGBNormal::Ptr model_in_handbase(new PointCloudRGBNormal);

  pcl::transformPointCloudWithNormals(*model, *model_in_handbase, cur_model2handbase);
  // if (name == "r_gripper_finger_link"){
  //   pcl::copyPointCloud(*model_in_handbase,*testcloud1);
  //   pcl::copyPointCloud(*args->scene_hand_region_removed_noise,*testcloud2);
  // }else if(name == "l_gripper_finger_link"){
  //   pcl::copyPointCloud(*model_in_handbase,*testcloud1);
  //   pcl::copyPointCloud(*args->scene_hand_region_removed_noise,*testcloud2);
  // }
  float NORMAL_ANGLE_THRES = std::cos(args->cfg.yml["hand_match"]["finger_normal_angle"].as<float>()/180.0*M_PI);
  
  // Eigen::MatrixXf V(model_in_handbase->points.size(),3);
  const bool check_normal = args->cfg.yml["hand_match"]["check_normal"].as<bool>();

  for (int ii=0;ii<model_in_handbase->points.size();ii++) // model_in_handbase is the model of current finger
  {
    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1);
    auto pt = model_in_handbase->points[ii];
    if (args->kdtree_scene->nearestKSearch (pt, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
    {
      auto nei = args->scene_hand_region_removed_noise->points[pointIdxNKNSearch[0]];
      float sq_planar_dist = (pt.x-nei.x)*(pt.x-nei.x) + (pt.y-nei.y)*(pt.y-nei.y);  //In handbase's frame

      // not match with the point cloud on another finger
      if (name=="r_gripper_finger_link" && (nei.y < 0.0 || nei.x < -0.028))   //Right side finger
      {
        continue;
      }
      else if(name=="l_gripper_finger_link" && (nei.y > 0.0 || nei.x < -0.028))
      {
        continue;
      }

      if (pointNKNSquaredDistance[0]<=dist_thres*dist_thres /*|| (sq_planar_dist<=PLANAR_DIST_THRES*PLANAR_DIST_THRES && std::abs(pt.z-nei.z)<=0.03)*/)  // Squared dist!!
      {
        // if (!check_normal)
        // {
        //   // num_match+=std::exp(-sq_planar_dist/(PLANAR_DIST_THRES*PLANAR_DIST_THRES));
        //   num_match += 1 + std::abs(X[0]);
        //   continue;
        // }
        // if (nei.normal_x==0 && nei.normal_y==0 && nei.normal_z==0)
        // {
        //   num_match += 1 + std::abs(X[0]);
        //   continue;
        // }
        if (std::isfinite(nei.normal_x) && std::isfinite(nei.normal_y) && std::isfinite(nei.normal_z))
        {
          // V.block(num_match,0,1,3) << nei.x, nei.y, nei.z;
          Eigen::Vector3f n1(pt.normal_x, pt.normal_y, pt.normal_z);
          Eigen::Vector3f n2(nei.normal_x, nei.normal_y, nei.normal_z);
          if (n1.dot(n2)>=NORMAL_ANGLE_THRES)
          {

            // num_match += n1.dot(n2)*std::exp(-sq_planar_dist/(PLANAR_DIST_THRES*PLANAR_DIST_THRES));
            num_match += (1-std::sqrt(sq_planar_dist)/dist_thres) + std::abs(X[0]);
            // num_match += n1.dot(n2);
            // num_match += 1 + std::abs(X[0]);

            // add the matching line to the markers
            // visualization_msgs::Marker line;
            // line.header.stamp = ros::Time();
            // line.header.frame_id = "/gripper_link";
            // line.ns = "lines";
            // line.action = visualization_msgs::Marker::ADD;
            // line.pose.orientation.w = 1.0;
            // line.type = visualization_msgs::Marker::LINE_LIST;
            // line.scale.x = 0.0001;
            // line.color.r = 1.0;
            // line.color.a = 1.0;
            // line.id = ii;
            // geometry_msgs::Point p;
            // p.x = pt.x;
            // p.y = pt.y;
            // p.z = pt.z;
            // line.points.push_back(p);
            // p.x = nei.x;
            // p.y = nei.y;
            // p.z = nei.z;
            // line.points.push_back(p);
            // line_markers->markers.push_back(line);
          }
          continue;
        }
      }
    }
  }

  score += num_match;

  //We penalize points on outer side of finger, this only makes sense when it's somewhat matching. Otherwise scene in finger is not aligned with finger property along z axis
  float outer_dist_sum=0;
  int num_outer = 0;
  FingerProperty finger_property = args->finger_property;
  PointCloudRGBNormal::Ptr scene_no_swivel_in_finger(new PointCloudRGBNormal);
  pcl::transformPointCloudWithNormals(*args->scene_remove_swivel, *scene_no_swivel_in_finger, cur_model2handbase.inverse());
  
  if (num_match==0)    // encourage moving
  {
    score = -100 + std::abs(X[0]);
    return -score;
  }

  for (const auto &pt:scene_no_swivel_in_finger->points)
  {
    int cur_bin = finger_property.getBinAlongX(pt.x);

    if (name=="r_gripper_finger_link")   //Right side finger
    {
      if (pt.y<=finger_property._hist_alongx(4,cur_bin)){ // point should be less than max of finger in y
        continue;
      }
    }
    else
    {
      if(pt.y>=finger_property._hist_alongx(1,cur_bin)){ // point should be greater than min finger in y
        continue;
      }
    }

    // if (pt.y>=-0.01) continue;
    // outer_dist_sum += std::abs(pt.y+0.01);
    outer_dist_sum += std::abs(pt.y-finger_property._hist_alongx(1,cur_bin));
    num_outer++;
  }
  float penalty_outer=0;
  const int MAX_OUTER_PTS = args->cfg.yml["hand_match"]["max_outter_pts"].as<int>();
  const float outter_pt_dist = args->cfg.yml["hand_match"]["outter_pt_dist"].as<float>();
  const float outter_pt_dist_weight = args->cfg.yml["hand_match"]["outter_pt_dist_weight"].as<float>();
  // if ( (num_outer>=20 && outer_dist_sum/num_outer-outter_pt_dist>0) || num_outer>=MAX_OUTER_PTS )
  // {
  //   penalty_outer = 1e3 + 1e3*std::max(outer_dist_sum/num_outer-outter_pt_dist, 0.0f);
  //   score -= penalty_outer;
  // }
  float avg_outer_dist = outer_dist_sum/num_outer;
  if (num_outer>=MAX_OUTER_PTS || avg_outer_dist>=0.005)
  {
    penalty_outer = 1e3 + outter_pt_dist_weight*std::max(avg_outer_dist-outter_pt_dist, 0.0f);
    score -= penalty_outer;
  }
  else if ( (num_outer>=0 && avg_outer_dist-outter_pt_dist>0))
  {
    // penalty_outer = outter_pt_dist_weight*(avg_outer_dist-outter_pt_dist);
    penalty_outer = outter_pt_dist_weight*std::exp(avg_outer_dist*1000);
    score -= penalty_outer;
  }

  return -score;   //NOTE: we are minimizing the cost function
}

FingerProperty::FingerProperty(){};

FingerProperty::FingerProperty(PointCloudRGBNormal::Ptr model, int num_division):_num_division(num_division)
{
  if (model->size() == 0){
    ROS_ERROR("number of points in finger pointcloud should not be zero!!!");
    throw;
  }
  pcl::PointXYZRGBNormal min_pt, max_pt;
  pcl::getMinMax3D(*model, min_pt, max_pt);
  _min_x = min_pt.x;
  _min_y = min_pt.y;
  _min_z = min_pt.z;
  _max_x = max_pt.x;
  _max_y = max_pt.y;
  _max_z = max_pt.z;
  _stride_x = (_max_x-_min_x)/num_division;
  _hist_alongx.resize(6,num_division);
  _hist_alongx.block(0,0,3,num_division).setConstant(std::numeric_limits<float>::max());
  _hist_alongx.block(3,0,3,num_division).setConstant(-std::numeric_limits<float>::max());
  std::vector<bool> changed(num_division, false);
  for (int i=0;i<model->points.size();i++)
  {
    auto pt = model->points[i];
    int bin = getBinAlongX(pt.x);
    _hist_alongx(0,bin) = std::min(_hist_alongx(0,bin), pt.x);
    _hist_alongx(1,bin) = std::min(_hist_alongx(1,bin), pt.y);
    _hist_alongx(2,bin) = std::min(_hist_alongx(2,bin), pt.z);
    _hist_alongx(3,bin) = std::max(_hist_alongx(3,bin), pt.x);
    _hist_alongx(4,bin) = std::max(_hist_alongx(4,bin), pt.y);
    _hist_alongx(5,bin) = std::max(_hist_alongx(5,bin), pt.z);
    changed[bin]=true;
  }
  // it split the finger block into several sub blocks

  //When cur bin never touched, take nearest valid neighbor
  for (int i=0;i<num_division;i++)
  {
    if (changed[i]) continue;
    for (int j=i+1;j<num_division;j++)
    {
      if (changed[j])
      {
        _hist_alongx.col(i) = _hist_alongx.col(j);
        changed[i] = true;
        break;
      }
    }
  }
  if (changed[num_division-1]==false)
  {
    for (int i=num_division-2;i>=0;i--)
    {
      if (changed[i])
      {
        _hist_alongx.col(num_division-1) = _hist_alongx.col(i);
        changed[num_division-1] = true;
        break;
      }
    }
  }
}

FingerProperty::~FingerProperty()
{

}


int FingerProperty::getBinAlongX(float x)
{
  int bin = std::max(x-_min_x, 0.0f)/_stride_x;
  bin = std::max(bin,0);
  bin = std::min(bin, _num_division-1);
  return bin;
}


Hand::Hand()
{

}

Hand::Hand(ConfigParser *cfg1, const Eigen::Matrix3f &cam_K, int width=640, int height=480)
{
  cfg=cfg1;
  _hand_cloud = boost::make_shared<PointCloudRGBNormal>();
  test1 = boost::make_shared<PointCloudRGBNormal>();
  test2 = boost::make_shared<PointCloudRGBNormal>();
  _visible_set = boost::make_shared<PointCloudRGBNormal>();
  markerarray = boost::make_shared<visualization_msgs::MarkerArray>();
  _cam_K = cam_K;
  in_cam = false;
  cam_width = width;
  cam_height = height;

  parseURDF();
  for (auto h:_clouds)
  {
    std::string name = h.first;
    if (name.find("finger")==-1) continue;
    _finger_properties[name] = FingerProperty(h.second, 10);
  }
  initPSO();

}

Hand::~Hand()
{
}

void Hand::setCurScene(const cv::Mat &depth_meters, PointCloudRGBNormal::Ptr scene_organized, PointCloudRGBNormal::Ptr scene_hand_region, const Eigen::Matrix4f &handbase_in_cam)
{
  // current input pointcloud are in camera base
  _depth_meters=depth_meters;
  _scene_sampled=boost::make_shared<PointCloudRGBNormal>();
  _scene_hand_region=boost::make_shared<PointCloudRGBNormal>();
  Utils::downsamplePointCloud<pcl::PointXYZRGBNormal>(scene_organized, _scene_sampled, 0.002);
  Utils::downsamplePointCloud<pcl::PointXYZRGBNormal>(scene_hand_region, _scene_hand_region,0.002);

  _handbase_in_cam = handbase_in_cam;
  handbaseICP(scene_organized); // this function will update the _handbase_in_cam, so be careful!!

  if(!in_cam)
    return;

  //NOTE: we compare in handbase frame, so that kdtree only build once
  PointCloudRGBNormal::Ptr scene_in_handbase(new PointCloudRGBNormal);
  // use the updated handbase in camera to filter the pointcloud
  pcl::transformPointCloudWithNormals(*_scene_hand_region, *scene_in_handbase, _handbase_in_cam.inverse());

  // radiusoutlierremoval filter points in a cloud based on the number of neighbors they have.
  PointCloudRGBNormal::Ptr scene_hand_region_removed_noise(new PointCloudRGBNormal);
  {
    pcl::RadiusOutlierRemoval<pcl::PointXYZRGBNormal> outrem;
    outrem.setInputCloud(scene_in_handbase);
    outrem.setRadiusSearch(0.02);
    outrem.setMinNeighborsInRadius (30);
    outrem.filter (*scene_hand_region_removed_noise);
  }
  {
    pcl::RadiusOutlierRemoval<pcl::PointXYZRGBNormal> outrem;
    outrem.setInputCloud(scene_hand_region_removed_noise);
    outrem.setRadiusSearch(0.04);
    outrem.setMinNeighborsInRadius (100);
    outrem.filter (*scene_hand_region_removed_noise);
  }
  // staticaloutlierremoval uses point neighborhood statics to filter outlier data.
  {
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBNormal> sor;
    sor.setInputCloud (scene_hand_region_removed_noise);
    sor.setMeanK (20);
    sor.setStddevMulThresh (2);
    sor.filter (*scene_hand_region_removed_noise);   //! in hand base
  }
  // keep only fingers part
  PointCloudRGBNormal::Ptr scene_remove_swivel(new PointCloudRGBNormal);
  {
    pcl::PassThrough<pcl::PointXYZRGBNormal> pass;
    pass.setInputCloud (scene_hand_region_removed_noise);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (0.0, 0.08);
    pass.filter (*scene_remove_swivel);   // In handbase frame
  }


  assert(scene_remove_swivel->points.size()>0);

  // todo: why no use scene_noswivel_cam
  // PointCloudRGBNormal::Ptr scene_noswivel_cam(new PointCloudRGBNormal);
  // pcl::transformPointCloudWithNormals(*scene_remove_swivel, *scene_noswivel_cam, _handbase_in_cam); // scene_noswivel_cam is only finger parts in camera base
  
  // PointCloudRGBNormal::Ptr handregion_in_cam(new PointCloudRGBNormal);
  // pcl::transformPointCloudWithNormals(*scene_hand_region_removed_noise, *handregion_in_cam, _handbase_in_cam); // hardregion_in_cam is whole hand point cloud in camera base
  _pso_args.scene_hand_region_removed_noise = scene_hand_region_removed_noise;// scene_hand_region_removed_noise in hand base
  _pso_args.scene_hand_region = scene_in_handbase; // scene_in_handbase is in hand base
  boost::shared_ptr<pcl::KdTreeFLANN<pcl::PointXYZRGBNormal> > kdtree(new pcl::KdTreeFLANN<pcl::PointXYZRGBNormal>);

  kdtree->setInputCloud(scene_hand_region_removed_noise);

  _pso_args.kdtree_scene = kdtree;
  _pso_args.scene_remove_swivel = scene_remove_swivel; // scene_remove_swivel is in hand base
}

void Hand::reset()
{
  _kdtrees.clear();
  _matched_scene_indices.clear();
  for (auto &c:_component_status)
  {
    c.second = false;
  }
  for (auto &h:_finger_angles)
  {
    h.second = 0;
  }
  _handbase_in_cam.setIdentity();
  _hand_cloud->clear();
  test1->clear();
  test2->clear();
  _visible_set->clear();
  markerarray->markers.clear();
  _depth_meters.release();
  _scene_sampled->clear();
  _scene_hand_region->clear();
  _pso_args.reset();
  for (auto &t:_tf_self)
  {
    t.second.setIdentity();
  }

}

void Hand::printComponents()
{
  std::cout<<"\nprint hand info:\n";
  for (auto h:_clouds)
  {
    std::string name=h.first;
    std::string parent_name=_parent_names[name];
    std::cout<<"name="<<name<<", parent="<<parent_name<<"\n";
    std::cout<<"tf_self:\n"<<_tf_self[name]<<"\n";
    std::cout<<"tf_in_parent:\n"<<_tf_in_parent[name]<<"\n";
  }
  std::cout<<"\n";
}

void Hand::parseURDF()
{
  pugi::xml_document doc;
  pugi::xml_parse_result result = doc.load_file((cfg->yml["urdf_path"].as<std::string>()).c_str());
  assert(result);

  for (pugi::xml_node tool = doc.child("robot").child("link"); tool; tool = tool.next_sibling("link"))
  {
    auto visual=tool.child("visual");
    std::string name=tool.attribute("name").value();

    if (name.find("rail")!=-1) continue;

    std::vector<float> rpy(3);
    if (!visual.child("origin").attribute("rpy"))
    {
      rpy.resize(3,0);
    }
    else
    {
      std::string rpy_str=visual.child("origin").attribute("rpy").value();
      Utils::delimitString(rpy_str, ' ', rpy);
    }


    std::vector<float> xyz(3);
    if (!visual.child("origin").attribute("xyz"))
    {
      xyz.resize(3,0);
    }
    else
    {
      std::string xyz_str=visual.child("origin").attribute("xyz").value();
      Utils::delimitString(xyz_str,' ', xyz);
    }

    Eigen::Matrix3f R;
    R=AngleAxisf(rpy[2], Vector3f::UnitZ()) * AngleAxisf(rpy[1], Vector3f::UnitY()) * AngleAxisf(rpy[0], Vector3f::UnitX());

    Eigen::Matrix4f tf_init(Eigen::Matrix4f::Identity());
    tf_init.block(0,0,3,3)=R;
    tf_init.block(0,3,3,1)=Eigen::Vector3f(xyz[0],xyz[1],xyz[2]);

    Eigen::Matrix4f tf_in_parent(Eigen::Matrix4f::Identity());
    std::string parent_name;
    for (pugi::xml_node tool = doc.child("robot").child("joint"); tool; tool = tool.next_sibling("joint"))
    {
      if (tool.child("child").attribute("link").value()!=name) continue;
      parent_name=tool.child("parent").attribute("link").value();

      std::string rpy_str=tool.child("origin").attribute("rpy").value();
      std::vector<float> rpy(3,0);
      Utils::delimitString(rpy_str, ' ', rpy);
      std::string xyz_str=tool.child("origin").attribute("xyz").value();
      std::vector<float> xyz(3,0);
      Utils::delimitString(xyz_str, ' ', xyz);

      Eigen::Matrix3f R;
      R=AngleAxisf(rpy[2], Vector3f::UnitZ()) * AngleAxisf(rpy[1], Vector3f::UnitY()) * AngleAxisf(rpy[0], Vector3f::UnitX());

      tf_in_parent.block(0,0,3,3)=R;
      tf_in_parent.block(0,3,3,1)=Eigen::Vector3f(xyz[0],xyz[1],xyz[2]);
      break;
    }


    std::vector<float> scale(3,1);
    if (!visual.child("geometry").child("mesh").attribute("scale"))
    {
      scale.resize(3,1);
    }
    else
    {
      std::string scale_str=visual.child("geometry").child("mesh").attribute("scale").value();
      Utils::delimitString(scale_str, ' ', scale);
    }

    std::cout << "load component " << name << std::endl;

    PointCloudRGBNormal::Ptr cloud(new PointCloudRGBNormal);
    pcl::io::loadPLYFile(cfg->yml["Hand"][name]["cloud"].as<std::string>(), *cloud);
    assert(cloud->points.size()>0);
    for (auto &pt:cloud->points)
    {
      pt.x *= scale[0];
      pt.y *= scale[1];
      pt.z *= scale[2];
    }
    
    pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);

    pcl::io::loadOBJFile(cfg->yml["Hand"][name]["mesh"].as<std::string>(), *mesh);
    PointCloud::Ptr mesh_cloud(new PointCloud);
    pcl::fromPCLPointCloud2(mesh->cloud, *mesh_cloud);
    assert(mesh_cloud->points.size()>0);
    for (auto &pt:mesh_cloud->points)
    {
      pt.x *= scale[0];
      pt.y *= scale[1];
      pt.z *= scale[2];
    }
  
    pcl::toPCLPointCloud2(*mesh_cloud, mesh->cloud);

    // Component init pose must be applied at beginning according to URDF !!
    pcl::PolygonMesh::Ptr convex_mesh(new pcl::PolygonMesh);
    pcl::io::loadOBJFile(cfg->yml["Hand"][name]["convex_mesh"].as<std::string>(), *convex_mesh);
    PointCloud::Ptr convex_mesh_cloud(new PointCloud);
    pcl::fromPCLPointCloud2(convex_mesh->cloud, *convex_mesh_cloud);
    assert(convex_mesh_cloud->points.size()>0);
    for (auto &pt:convex_mesh_cloud->points)
    {
      pt.x *= scale[0];
      pt.y *= scale[1];
      pt.z *= scale[2];
    }
    pcl::toPCLPointCloud2(*convex_mesh_cloud, convex_mesh->cloud);

    // Component init pose must be applied at beginning according to URDF !!
    pcl::transformPointCloudWithNormals(*cloud,*cloud,tf_init);
    Utils::transformPolygonMesh(mesh,mesh,tf_init);
    Utils::transformPolygonMesh(convex_mesh,convex_mesh,tf_init);


    std::cout<<"adding component name:"+name<<std::endl;
    
    addComponent(name,parent_name,cloud,mesh,convex_mesh,tf_in_parent,Eigen::Matrix4f::Identity());
  }

}

//Component tf in handbase, accounting for self rotation
// todo: this function is updating the tf from end to root, which makes no sense. I may need to update it.
void Hand::getTFHandBase(std::string cur_name, Eigen::Matrix4f &tf_in_handbase)
{
  tf_in_handbase.setIdentity();
  while (1)   //Get pose in world
  {
    if (cur_name=="gripper_link")
    {
      break;
    }
    if (_tf_self.find(cur_name)==_tf_self.end())
    {
      std::cout<< cur_name << " does not exist in tf!!!\n";
      exit(1);
    }
    Matrix4f cur_tf = _tf_in_parent[cur_name];
    tf_in_handbase = cur_tf * _tf_self[cur_name] * tf_in_handbase;
    cur_name = _parent_names[cur_name];
  }
}

void Hand::addComponent(std::string name, std::string parent_name, PointCloudRGBNormal::Ptr cloud, pcl::PolygonMesh::Ptr mesh, pcl::PolygonMesh::Ptr convex_mesh, const Matrix4f &tf_in_parent, const Matrix4f &tf_self)
{
  Utils::downsamplePointCloud<pcl::PointXYZRGBNormal>(cloud, cloud, 0.004);

  _clouds[name] = cloud;
  _meshes[name] = mesh;
  _convex_meshes[name] = convex_mesh;
  _parent_names[name] = parent_name;
  _tf_in_parent[name] = tf_in_parent;
  _tf_self[name] = tf_self;
}

//Return cloud in handbase frame
// this function will build _kdtrees for each componment in hand baseframe
void Hand::makeHandCloud()
{
  _hand_cloud->clear();
  std::string component_status_info = "";
  for (auto h:_clouds)
  {
    std::string name = h.first;
    PointCloudRGBNormal::Ptr component_cloud = h.second;

    Eigen::Matrix4f model2handbase;
    getTFHandBase(name,model2handbase);
    PointCloudRGBNormal::Ptr tmp(new PointCloudRGBNormal);
    pcl::transformPointCloudWithNormals(*component_cloud, *tmp, model2handbase);

    
    component_status_info += (name + (_component_status[name] ? ": good | ": ": bad | "));
    if(_component_status[name])
      (*_hand_cloud) += (*tmp);
    else
      continue;

    // build the kd tree of part of model
    boost::shared_ptr<pcl::KdTreeFLANN<pcl::PointXYZRGBNormal> > kdtree(new pcl::KdTreeFLANN<pcl::PointXYZRGBNormal>);
    kdtree->setInputCloud(tmp);
    _kdtrees[name] = kdtree;
  }
  ROS_INFO_STREAM("tracking status: " << component_status_info);
}

//For visualization
void Hand::makeHandMesh()
{
  for (auto h:_meshes)
  {
    std::string name = h.first;
    std::cout<<"making hand mesh "+name<<std::endl;
    pcl::PolygonMesh::Ptr cur_mesh = h.second;
    Eigen::Matrix4f model2handbase;
    getTFHandBase(name,model2handbase);
    pcl::PolygonMesh::Ptr tmp(new pcl::PolygonMesh);
    Utils::transformPolygonMesh(cur_mesh, tmp, model2handbase);
    Utils::transformPolygonMesh(tmp, tmp, _handbase_in_cam);
  }
  for (auto h:_convex_meshes)
  {
    std::string name = h.first;
    pcl::PolygonMesh::Ptr cur_mesh = h.second;
    Eigen::Matrix4f model2handbase;
    getTFHandBase(name,model2handbase);
    pcl::PolygonMesh::Ptr tmp(new pcl::PolygonMesh);
    Utils::transformPolygonMesh(cur_mesh, tmp, model2handbase);
    pcl::io::saveOBJFile("/home/bowen/debug/handmesh/convex_"+name+".obj", *tmp);
    Utils::transformPolygonMesh(tmp, tmp, _handbase_in_cam);
    pcl::io::saveOBJFile("/home/bowen/debug/handmesh/convex_incam_"+name+".obj", *tmp);
  }
}

float Hand::visibleHandCloud(PointCloudRGBNormal::Ptr handbase_cloud, PointCloudRGBNormal::Ptr visible_set){
  visible_set->clear();

  pcl::VoxelGridOcclusionEstimation<pcl::PointXYZRGBNormal> voxelFilter;
  voxelFilter.setInputCloud(handbase_cloud);
  voxelFilter.setLeafSize(0.005, 0.005, 0.005);
  voxelFilter.initializeVoxelGrid();

  double visiblenum = 0;
  double inviewnum = 0;

  for (size_t i=0;i<handbase_cloud->size();i++) 
  { 

    pcl::PointXYZRGBNormal pt = handbase_cloud->points[i]; 

    Eigen::Vector3i grid_cordinates=voxelFilter.getGridCoordinates (pt.x, pt.y, pt.z);
    
    int grid_state; 

    int ret=voxelFilter.occlusionEstimation( grid_state, grid_cordinates );

    if (grid_state==1)
      continue;

    visiblenum++;

    if(pt.z < 0.01)
      continue;

    float p2x = (_cam_K(0,0) * pt.x + _cam_K(0,2) * pt.z) / pt.z;
    float p2y = (_cam_K(1,1) * pt.y + _cam_K(1,2) * pt.z) / pt.z;

    if(p2x < 0 || p2x >= cam_width || p2y < 0 || p2y >= cam_height)
      continue;

    inviewnum++;

    visible_set->push_back(handbase_cloud->points[i]); 
  }

  if (visiblenum == 0)
    return 0.0;
  else
    return inviewnum / visiblenum;
}


void Hand::initPSO()
{
  _pso_args.cfg = *cfg;
  _pso_settings.pso_n_pop = cfg->yml["hand_match"]["pso"]["n_pop"].as<int>();
  _pso_settings.pso_n_gen = cfg->yml["hand_match"]["pso"]["n_gen"].as<int>();
  _pso_settings.vals_bound = true;
  _pso_settings.pso_check_freq = cfg->yml["hand_match"]["pso"]["check_freq"].as<int>();
  _pso_settings.err_tol = 1e-5;
  _pso_settings.pso_par_c_cog = cfg->yml["hand_match"]["pso"]["pso_par_c_cog"].as<float>();
  _pso_settings.pso_par_c_soc = cfg->yml["hand_match"]["pso"]["pso_par_c_soc"].as<float>();
  _pso_settings.pso_par_initial_w = cfg->yml["hand_match"]["pso"]["pso_par_initial_w"].as<float>();


}


bool Hand::matchOneComponentPSO(std::string model_name, float min_angle, float max_angle, bool use_normal, float dist_thres, float normal_angle_thres, float least_match)
{
  _pso_args.dist_thres = dist_thres;

  _pso_settings.upper_bounds = arma::zeros(1) + max_angle;
  _pso_settings.lower_bounds = arma::zeros(1) + min_angle;
  _pso_settings.pso_initial_ub = arma::zeros(1) + max_angle;
  _pso_settings.pso_initial_lb = arma::zeros(1) + min_angle;
  

  std::map<std::string, std::string> pair_names;
  pair_names["r_gripper_finger_link"] = "l_gripper_finger_link";
  pair_names["l_gripper_finger_link"] = "r_gripper_finger_link";
  Eigen::Matrix4f pair_in_base;
  const std::string pair_name = pair_names[model_name];

  getTFHandBase(pair_name, pair_in_base);  // get the tf of pair finger
  Eigen::Vector4f pair_tip_pt2(_finger_properties[pair_name]._min_x, _finger_properties[pair_name]._max_y, _finger_properties[pair_name]._max_z, 1);
  pair_tip_pt2 = pair_in_base * pair_tip_pt2; // get the pair finger in higher corner in z axis 
  _pso_args.pair_tip2 = pair_tip_pt2; // pass it to pso later as argument


  arma::vec X = arma::zeros(1);
  _pso_args.name = model_name;
  _pso_args.model = _clouds[model_name]; // load the finger cloud
  Eigen::Matrix4f model2handbase;
  getTFHandBase(model_name, model2handbase); // load the tf of current finger
  _pso_args.model2handbase = model2handbase;

  Eigen::Matrix4f model_in_cam = _handbase_in_cam * model2handbase;
  _pso_args.model_in_cam = model_in_cam;

  _pso_args.finger_property = _finger_properties[model_name];
  // _pso_args.sdf.registerMesh(_meshes[model_name], model_name, Eigen::Matrix4f::Identity());

  bool success = optim::pso(X,objFuncPSO,&_pso_args,_pso_settings);
  // std::cout << "score: " << objFuncPSO(X, NULL, &_pso_args) << std::endl;//, test1, test2, markerarray);

  if (!success || -_pso_args.objval<=least_match)
  {
    std::cout<<model_name+" PSO matching failed";
    if(-_pso_args.objval<=least_match){
      std::cout << " because of objval " << -_pso_args.objval << std::endl;
    }else{
      std::cout << std::endl;
    }
    _tf_self[model_name].setIdentity();
    _component_status[model_name]=false;
    return false;
  }
  float angle = static_cast<float>(X[0]);
  _tf_self[model_name].setIdentity();
  _tf_self[model_name](1,3) = angle;
  _component_status[model_name]=true;
  ROS_INFO("%s PSO final angle=%f, match_score=%f", model_name.c_str(), angle, -_pso_args.objval);
  return true;
}



// handbase iterative closest point
void Hand::handbaseICP(PointCloudRGBNormal::Ptr scene_organized)
{
  PointCloudRGBNormal::Ptr scene_sampled(new PointCloudRGBNormal);
  Utils::downsamplePointCloud<pcl::PointXYZRGBNormal>(scene_organized, scene_sampled, 0.004);

  PointCloudRGBNormal::Ptr handbase(new PointCloudRGBNormal);
  pcl::copyPointCloud(*_clouds["gripper_link"],*handbase); // need to filter out point is not showing in the view!!!

  PointCloudRGBNormal::Ptr scene_handbase(new PointCloudRGBNormal);
  Eigen::Matrix4f cam_in_handbase = _handbase_in_cam.inverse();  // cam -> handbase^ -> handbase(real) -> finger
  assert(cam_in_handbase!=Eigen::Matrix4f::Identity());
  pcl::transformPointCloudWithNormals(*scene_sampled, *scene_handbase, cam_in_handbase);

  // filter out the point not related to the hand.
  pcl::PassThrough<pcl::PointXYZRGBNormal> pass;
  pass.setInputCloud (scene_handbase);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (-0.15, -0.0295);
  pass.filter (*scene_handbase);

  pass.setInputCloud (scene_handbase);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-0.05, 0.05);
  pass.filter (*scene_handbase);

  //Now all in handbase
  Eigen::Matrix4f cam2handbase_offset(Eigen::Matrix4f::Identity());

  float hand_base_icp_dist = cfg->yml["hand_match"]["hand_base_icp_dist"].as<float>();

  // filter out non visible part of the handbase
  PointCloudRGBNormal::Ptr hand_in_cam(new PointCloudRGBNormal);
  pcl::transformPointCloudWithNormals(*handbase, *hand_in_cam, _handbase_in_cam);

  pcl::copyPointCloud(*hand_in_cam,*test1);

  double inviewrate = visibleHandCloud(hand_in_cam, _visible_set);

  in_cam = false;
  if (inviewrate < 0.5){
    cam2handbase_offset.setIdentity();
    printf("hand is not in camera view\n");
  }else{
    
    pcl::transformPointCloudWithNormals(*_visible_set, *_visible_set, _handbase_in_cam.inverse());

    float score = Utils::runICP<pcl::PointXYZRGBNormal>(scene_handbase, _visible_set, cam2handbase_offset, 70, 20, hand_base_icp_dist, 1e-4);

    // float score = Utils::runICP<pcl::PointXYZRGBNormal>(scene_handbase, handbase, cam2handbase_offset, 70, 20, hand_base_icp_dist, 1e-4);

    pcl::transformPointCloudWithNormals(*scene_handbase,*scene_handbase,cam2handbase_offset);

    // need to test in simulation to make cam2handbase_offset to be identity matrix
    // std::cout<<"cam2handbase_offset:\n"<<cam2handbase_offset<<"\n\n";
    float translation = cam2handbase_offset.block(0,3,3,1).norm();
    ROS_INFO("hand palm translation=%f, match_score=%f", translation, score);
    if (translation >=0.005) // set this higher in real world
    {
      printf("cam2handbase_offset set to Identity, icp=%f, translation=%f, x=%f, y=%f\n",score, translation, std::abs(cam2handbase_offset(0,3)), std::abs(cam2handbase_offset(1,3)));
      cam2handbase_offset.setIdentity();
    }
    else{
      float rot_diff = Utils::rotationGeodesicDistance(Eigen::Matrix3f::Identity(), cam2handbase_offset.block(0,0,3,3)) / M_PI *180.0;
      Eigen::Matrix3f R = cam2handbase_offset.block(0,0,3,3);   // rotate rpy around static axis
      // because from pose matrix to rpy is not one to one, and there is singularity,
      // it must be handled carefully
      Eigen::Vector3f rpy = R.eulerAngles(2,1,0);
      float pitch = rpy(1); // Rotation along y axis
      
      float pitch1 = std::min(std::abs(pitch), std::abs(static_cast<float>(M_PI)-pitch));
      float pitch2 = std::min(std::abs(pitch), std::abs(static_cast<float>(M_PI)+pitch));
      pitch = std::min(pitch1,pitch2);
      if (rot_diff>=10 || std::abs(pitch)>=10/180.0*M_PI)
      {
        cam2handbase_offset.setIdentity();
        printf("cam2handbase_offset set to Identity\n");
      }
      else
      {
        in_cam = true;
        _component_status["gripper_link"] = true;
      }
    }
  }

  _handbase_in_cam = _handbase_in_cam*cam2handbase_offset.inverse();

}

HandT42::HandT42(ConfigParser *cfg1, const Eigen::Matrix3f &cam_K, int width, int height):Hand(cfg1, cam_K, width, height)
{
}

HandT42::HandT42()
{

}

HandT42::~HandT42()
{
}


//Operations in handbase frame. Hack: alternative to SDF with similar result while slightly faster.
//@dist_thres: distance square !!
template<class PointT, bool has_normal>
void HandT42::removeSurroundingPointsAndAssignProbability(boost::shared_ptr<pcl::PointCloud<PointT> > scene, boost::shared_ptr<pcl::PointCloud<PointT> > scene_out, float dist_thres)
{
  scene_out->clear();
  // scene_out->points.reserve(scene->points.size());
  pcl::transformPointCloudWithNormals(*scene, *scene, _handbase_in_cam.inverse());
  scene_out->points.reserve(scene->points.size());
  const float lambda = 231.04906018664843; // Exponential distribution: this makes about 0.003m the prob is 0.5
#pragma omp parallel
  {
    std::map<std::string, boost::shared_ptr<pcl::KdTreeFLANN<pcl::PointXYZRGBNormal>>> kdtrees_local;
    // reset the kdtrees_local with the original _kdtrees
    for (auto &h : _kdtrees) // _kdtree is the map from link name to its model kd tree
    {
      kdtrees_local[h.first].reset(new pcl::KdTreeFLANN<pcl::PointXYZRGBNormal>(*h.second));
    }

    #pragma omp for schedule(dynamic)
    for (int i = 0; i < scene->points.size(); i++)
    {
      auto pt = scene->points[i];
      pcl::PointXYZRGBNormal pt_tmp;
      // store point of scene to a temp point
      pt_tmp.x = pt.x;
      pt_tmp.y = pt.y;
      pt_tmp.z = pt.z;

      bool is_near = false;
      float min_dist = 1.0;
      for (auto h : kdtrees_local)
      {
        float local_dist_thres = dist_thres;
        std::string name = h.first;
        if (name == "r_gripper_finger_link" || name == "l_gripper_finger_link")
        {
          local_dist_thres = 0.01 * 0.01;
        }
        else if (name == "gripper_link")
        {
          local_dist_thres = 0.02 * 0.02;
        }

        // extract the kdtree of current link
        boost::shared_ptr<pcl::KdTreeFLANN<pcl::PointXYZRGBNormal>> kdtree = h.second;
        std::vector<int> pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);

        if (kdtree->nearestKSearch(pt_tmp, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
        {
          min_dist = std::min(min_dist, std::sqrt(pointNKNSquaredDistance[0]));
          if (pointNKNSquaredDistance[0] <= local_dist_thres)
          {
            is_near = true;
            break;
          }
          auto nei = kdtree->getInputCloud()->points[pointIdxNKNSearch[0]];
          float sq_planar_dist = (pt_tmp.x - nei.x) * (pt_tmp.x - nei.x) + (pt_tmp.y - nei.y) * (pt_tmp.y - nei.y);
          if (sq_planar_dist <= local_dist_thres && std::abs(pt_tmp.z - nei.z) <= 0.005)
          {
            is_near = true;
            break;
          }
        }
      }

      if (!is_near)
      {
        pt.confidence = 1 - std::exp(-lambda * min_dist);
        #pragma omp critical
        scene_out->points.push_back(pt);
      }
    }
  }

  pcl::transformPointCloudWithNormals(*scene_out, *scene_out, _handbase_in_cam);
}
template void HandT42::removeSurroundingPointsAndAssignProbability<pcl::PointSurfel,true>(boost::shared_ptr<pcl::PointCloud<pcl::PointSurfel> > scene, boost::shared_ptr<pcl::PointCloud<pcl::PointSurfel> > scene_out, float dist_thres);


void HandT42::fingerICP()
{

  int num_near_finger_2_1=0, num_near_finger_1_1=0;
  PointCloudRGBNormal::Ptr finger_2_1(new PointCloudRGBNormal);
  PointCloudRGBNormal::Ptr finger_1_1(new PointCloudRGBNormal);

  // we operate in hand base frame
  {
    Eigen::Matrix4f model2handbase(Eigen::Matrix4f::Identity());
    getTFHandBase("finger_2_1",model2handbase);
    pcl::transformPointCloudWithNormals(*_clouds["finger_2_1"], *finger_2_1, model2handbase);
  }
  {
    Eigen::Matrix4f model2handbase(Eigen::Matrix4f::Identity());
    getTFHandBase("finger_1_1",model2handbase);
    pcl::transformPointCloudWithNormals(*_clouds["finger_1_1"], *finger_1_1, model2handbase);
  }

  PointCloudRGBNormal::Ptr scene_handbase(new PointCloudRGBNormal);
  Eigen::Matrix4f cam_in_handbase = _handbase_in_cam.inverse();  // cam -> handbase^ -> handbase(real) -> finger
  pcl::transformPointCloudWithNormals(*_scene_sampled, *scene_handbase, cam_in_handbase);
  Utils::downsamplePointCloud<pcl::PointXYZRGBNormal>(scene_handbase, scene_handbase, 0.005);

  pcl::KdTreeFLANN<pcl::PointXYZRGBNormal> kdtree1, kdtree2;
  kdtree1.setInputCloud (finger_1_1);
  kdtree2.setInputCloud (finger_2_1);

  PointCloudRGBNormal::Ptr scene_icp(new PointCloudRGBNormal);
  for (const auto &pt:scene_handbase->points)
  {
    {
      std::vector<int> pointIdxNKNSearch(1);
      std::vector<float> pointNKNSquaredDistance(1);
      if (kdtree1.nearestKSearch (pt, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
      {
        if (pointNKNSquaredDistance[0]<=0.03*0.03)
        {
          auto nei = finger_1_1->points[pointIdxNKNSearch[0]];
          float sq_dist_planar = (nei.x-pt.x)*(nei.x-pt.x) + (nei.y-pt.y)*(nei.y-pt.y);
          if (sq_dist_planar<0.01*0.01)
          {
            num_near_finger_1_1++;
            scene_icp->points.push_back(pt);
            continue;
          }
        }
      }
    }

    {
      std::vector<int> pointIdxNKNSearch(1);
      std::vector<float> pointNKNSquaredDistance(1);
      if (kdtree2.nearestKSearch (pt, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
      {
        if (pointNKNSquaredDistance[0]<=0.03*0.03)
        {
          auto nei = finger_2_1->points[pointIdxNKNSearch[0]];
          float sq_dist_planar = (nei.x-pt.x)*(nei.x-pt.x) + (nei.y-pt.y)*(nei.y-pt.y);
          if (sq_dist_planar<0.01*0.01)
          {
            num_near_finger_2_1++;
            scene_icp->points.push_back(pt);
            continue;
          }
        }
      }
    }

  }

  pcl::PassThrough<pcl::PointXYZRGBNormal> pass;
  pass.setInputCloud (scene_icp);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (-1000.0, 0.0);
  pass.filter (*scene_icp);

  pcl::RadiusOutlierRemoval<pcl::PointXYZRGBNormal> outrem;
  outrem.setInputCloud(scene_icp);
  outrem.setRadiusSearch(0.01);
  outrem.setMinNeighborsInRadius (10);
  outrem.filter (*scene_icp);


  PointCloudRGBNormal::Ptr finger_icp(new PointCloudRGBNormal);
  if (num_near_finger_1_1>10)
  {
    *finger_icp += *finger_1_1;
  }
  if (num_near_finger_2_1>10)
  {
    *finger_icp += *finger_2_1;
  }


  //Now all in handbase
  Eigen::Matrix4f cam2handbase_offset(Eigen::Matrix4f::Identity());
  Utils::runICP<pcl::PointXYZRGBNormal>(scene_icp, finger_icp, cam2handbase_offset, 50, 15, 0.002);

  pcl::transformPointCloudWithNormals(*scene_icp, *scene_icp, cam2handbase_offset);

  _handbase_in_cam = _handbase_in_cam*cam2handbase_offset.inverse();
}



//NOTE: Only do this when handbase match fail
void HandT42::adjustHandHeight()
{
  makeHandCloud();
  if ( _component_status["gripper_link"]==true )
  {
    return;
  }
  pcl::KdTreeFLANN<pcl::PointXYZRGBNormal> kdtree;
  PointCloudRGBNormal::Ptr scene_handbase(new PointCloudRGBNormal);  //Compare in handbase frame
  pcl::transformPointCloudWithNormals(*_scene_hand_region, *scene_handbase, _handbase_in_cam.inverse());
  kdtree.setInputCloud(scene_handbase);
  std::vector<float> trial_heights = {-0.03, -0.025, -0.02, -0.015, -0.01, -0.005, 0, 0.005, 0.01, 0.015, 0.02, 0.025, 0.03};
  int max_match = 0;
  float best_height = 0;
  Eigen::Matrix4f best_offset(Eigen::Matrix4f::Identity());
  makeHandCloud();
  for (int i=0;i<trial_heights.size();i++)
  {
    /*
         T^handbase_cam                         offset
    cam------------------> handbase_hat <--------------------- true handbase
    */
    Eigen::Matrix4f offset(Eigen::Matrix4f::Identity());
    offset(2,3) = trial_heights[i];
    PointCloudRGBNormal::Ptr cur_hand(new PointCloudRGBNormal);
    pcl::transformPointCloudWithNormals(*_hand_cloud, *cur_hand, offset);
    int cur_match = 0;
    for  (const auto &pt:cur_hand->points)
    {
      std::vector<int> indices;
      std::vector<float> sq_dists;
      if (kdtree.nearestKSearch (pt, 1, indices, sq_dists) > 0)
      {
        if (sq_dists[0] > 0.005*0.005) continue;
        auto nei = scene_handbase->points[indices[0]];
        Eigen::Vector3f pt_normal(pt.normal[0], pt.normal[1], pt.normal[2]);
        Eigen::Vector3f nei_normal(nei.normal[0], nei.normal[1], nei.normal[2]);
        if (pt_normal.dot(nei_normal) >= std::cos(45 / 180.0 * M_PI))
        {
          cur_match++;
        }
      }
    }
    if (cur_match>max_match)
    {
      max_match = cur_match;
      best_height = trial_heights[i];
      best_offset = offset;
    }

  }
  _handbase_in_cam = _handbase_in_cam*best_offset;
}
