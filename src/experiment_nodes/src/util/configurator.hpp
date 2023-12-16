#ifndef CONFIGURATOR_H
#define CONFIGURATOR_H

#include <iostream>
#include <fstream>
#include "json.hpp"

#define CONFIG_PATH "src/experiment_nodes/src/configurations/configurations.json"

using json = nlohmann::json;

class Configurator{

    private:

        static json getConfig(std::string filePath){

            std::ifstream configFile(filePath);

            if (!configFile.is_open()) {
                std::cerr << "Unable to open " << filePath <<std::endl;
            }

            json configData;
            configFile >> configData;
            configFile.close();
            return configData;
        }

    public:

        static json getConfiguration(){
            return getConfig(CONFIG_PATH);
        }


};


#endif