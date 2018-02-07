/**
 * @file map_reader.h
 * @brief a yaml file reader
 */

#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>

/**
 * @brief yamp map reader
 */
template <class Ti, class Tf>
class MapReader {
  public:
    /**
     * @brief Simple constructor
     * @param file the yaml file name (with path)
     * @param verbose enable printing if true, default as false
     */
    MapReader(const std::string& file, bool verbose = false) {
      try {
        YAML::Node config = YAML::LoadFile(file);

        if(!config[0]["origin"] || !config[1]["dim"] || !config[2]["resolution"] || !config[3]["data"]) {
          printf("Check input format!\n" );
          return;
        }
  
        const std::vector<double>& origin_vec = config[0]["origin"].as<std::vector<double>>();
        for(unsigned int i = 0; i < origin_vec.size(); i++)
          origin_(i) = origin_vec[i];
        if(verbose)
          std::cout << "origin: " << origin_.transpose() << std::endl;

        const std::vector<int>& dim_vec = config[1]["dim"].as<std::vector<int>>();
        for(unsigned int i = 0; i < dim_vec.size(); i++)
          dim_(i) = dim_vec[i];
        if(verbose)
          std::cout << "dim: " << dim_.transpose() << std::endl;

        resolution_ = config[2]["resolution"].as<double>();
        if(verbose)
          std::cout << "resolution: " << resolution_ << std::endl;

        const std::vector<int>& data = config[3]["data"].as<std::vector<int>>();
        data_.resize(data.size());
        for(unsigned int i = 0; i < data.size(); i++)
          data_[i] = data[i] > 0 ? 1 : 0;
        
        exist_ = true;
      } catch (YAML::ParserException& e) {
        //std::cout << e.what() << "\n";
        exist_ = false;
      }
    }

    ///Check if a map is loaded successfully
    bool exist() { return exist_; }
    ///Get the map origin 
    Tf origin() { return origin_; }
    ///Get the map dimension
    Ti dim() { return dim_; }
    ///Get the map resolution
    double resolution() { return resolution_; }
    ///Get the map entity
    std::vector<signed char> data() { return data_; }
  private:
    ///Map origin, float type
    Tf origin_;
    ///Map dimension, int type
    Ti dim_;
    ///Map resolution, float type
    double resolution_;
    ///Map entity, vector of `signed char`
    std::vector<signed char> data_;
    ///Flag of the map exsistence
    bool exist_ = false;
};
