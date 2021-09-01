#include <Eigen/StdVector>
using namespace Eigen;
class ParticleFilter{
  public:
    ParticleFilter(int numberOfParticles_input){
      numberOfParticles = numberOfParticles_input;
      poses.reserve(numberOfParticles);
    }

    void resample(std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> &poses_of_particle, std::vector<float> &weights_of_particle){

      int maxWeightIndex = std::max_element(weights_of_particle.begin(), weights_of_particle.end()) - weights_of_particle.begin();

      std::random_device mch;
      std::default_random_engine generator(mch());
      std::discrete_distribution<int> distribution(weights_of_particle.begin(), weights_of_particle.end());

      poses.clear();

      poses.push_back(poses_of_particle[maxWeightIndex]);

      for(int i = 1; i < numberOfParticles; i++){
        poses.push_back(propagate(poses_of_particle[distribution(generator)], 0.04, 0.002));
      }
    }

    void sampling(const Eigen::Matrix4f& input){
      poses.clear();
      for(int i = 0; i < numberOfParticles; i++){
        poses.push_back(propagate(input, 0.5, 0.01));
      }
    }

    Eigen::Matrix4f get(int index){
      return poses[index];
    }

    size_t size(){
      return poses.size();
    }

    Eigen::Matrix4f propagate(const Eigen::Matrix4f& input, float deviationsOnRot, float deviationOnTrans){
      Eigen::Matrix4f result;
      result.setIdentity();
      std::random_device mch;
      std::default_random_engine generator(mch());
      std::normal_distribution<float> rot_distribution(0.0, deviationsOnRot);
      std::normal_distribution<float> trans_distribution(0.0, deviationOnTrans);
      std::vector<float> rpy(3);
      for(int r = 0; r < 3; r++)
        rpy[r] = rot_distribution(generator);
      
      Eigen::Matrix3f rot = (Eigen::AngleAxisf(rpy[0], Eigen::Vector3f::UnitX()) * 
                            Eigen::AngleAxisf(rpy[1], Eigen::Vector3f::UnitY()) * 
                            Eigen::AngleAxisf(rpy[2], Eigen::Vector3f::UnitZ())).toRotationMatrix();
      result.block<3,3>(0,0) = rot;
      for(int p = 0; p < 3; p++)
        result(p,3) = trans_distribution(generator);

      result = input * result;
      return result;
    }

    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> poses;
    int numberOfParticles;
};