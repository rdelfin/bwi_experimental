
#ifndef DOMAINPARSER_HPP
#define DOMAINPARSER_HPP

#include <string>
#include <vector>
#include <map>
#include "bwi_nav_reasoning/MdpModel.h"
#include "bwi_nav_reasoning/StateAction.h"

class DomainParser {

public:

    DomainParser() {}

    DomainParser(const std::string static_obs, const std::string dynamic_obs, 
        const std::string sunny_cells, const std::string plog_facts); 

    std::string file_static_obstacle;
    std::string file_dynamic_obstacle;
    std::string file_sunny_cells; 
    std::string file_plog_facts; 

    int col_num; 
    int row_num; 

    std::map<std::vector<int>, State> states_map; 

    void parseFile(const std::string file, std::vector<std::vector<int>>& vec); 
}

#endif
