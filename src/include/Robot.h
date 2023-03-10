//
// Created by Yipeng on 2023/3/10.
//

#ifndef CODECRAFTSDK_ROBOT_H
#define CODECRAFTSDK_ROBOT_H


class Robot {
public:
    int id;
    double x, y;
    int station_id;
    int object_type;
    double time_val, collision_val;
    double angular_speed;
    double line_speed_x, line_speed_y;
    double direction;

    int to_station;

    Robot(int id, double x, double y): id(id), x(x), y(y) {
        to_station = -1;
    }
};


#endif //CODECRAFTSDK_ROBOT_H
