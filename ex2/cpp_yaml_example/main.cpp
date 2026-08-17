#include <iostream>
#include <fstream>
#include <filesystem>
#include <numbers>

#include <yaml-cpp/yaml.h>

namespace fs = std::filesystem;

void read_cpp_yaml(const fs::path& input_path)
{
    YAML::Node config = YAML::LoadFile(input_path);
    YAML::Node national_inner = config["national"];

    std::cout << "national tag as " << national_inner.size() << " elements." << std::endl;
    
    // for (auto it = national_inner.begin(); it != national_inner.end(); it++)
    for (auto team : national_inner)
    {
        std::cout << team << std::endl;
    }
}


void write_cpp_yaml(const fs::path& output_path)
{
    YAML::Node root;

    std::vector<std::string> sequence_values{ "first_v", "2nd_v", "v3", "v_fourth" };
    for (const auto& val : sequence_values)
    {
        root["sequence"].push_back(val);
    }

    root["float_val"] = std::numbers::pi;
    root["int_val"] = 89;

    {
        std::ofstream output(output_path, std::ios::trunc);
        output << root;
    }
}


int main() {
    fs::path yml_input_path{ "/workspaces/ex_2_skeleton/cpp_yaml_example/example.yml" }; // should be a CLI input in your project!
    fs::path yml_output_path{ "/workspaces/ex_2_skeleton/cpp_yaml_example/example.yml" };
    if (!fs::exists(yml_input_path))
    {
        std::cout << "file not found" << std::endl;
    }
    
    read_cpp_yaml(yml_input_path);
    write_cpp_yaml(yml_output_path);
    return 0;
}
