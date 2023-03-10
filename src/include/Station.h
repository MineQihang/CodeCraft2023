//
// Created by Yipeng on 2023/3/10.
//

#ifndef CODECRAFTSDK_STATION_H
#define CODECRAFTSDK_STATION_H

#include <bitset>

template<size_t Nb>
class Station {
public:
    int id;
    int type;
    double x, y;
    int prod_time;
    std::bitset<Nb> material_state;
    bool product_state;
    int chosen;

    Station(int id, int type, double x, double y): id(id), type(type), x(x), y(y) {
        chosen = -1;
    }
};


#endif //CODECRAFTSDK_STATION_H
