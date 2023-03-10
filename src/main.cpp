#include <iostream>
#include <vector>
#include <cstring>
#include <cmath>
#include <array>
#include <algorithm>
#include "include/Robot.h"
#include "include/Station.h"
using namespace std;

// 常量
const int READ_MAP_SIZE = 200;
const int OBJECT_TYPES_NUM = 7;
const int STATION_TYPES_NUM = 9;
const int ROBOTS_NUM = 4;
const double PI = acos(-1);
const double SPEED_MAX = 6;

// 全局变量
vector<Robot> robots;
vector<Station<OBJECT_TYPES_NUM + 1>> stations;
vector<int> station_seq;
int curr_money;
int station_num;

// 规则
array<bitset<OBJECT_TYPES_NUM + 1>, STATION_TYPES_NUM + 1> need = {
        0, 0, 0, 0, 0b110, 0b1010, 0b1100, 0b1110000, 0b10000000, 0b10000000
};

// 读取地图
void readMap() {
    int y = READ_MAP_SIZE;  // 所在x的坐标
    int robot_count = 0;    // 机器人数量
    int station_count = 0;  // 工作站数量

    char s[1024];
    while (fgets(s, sizeof s, stdin)) {
        if (s[0] == 'O' && s[1] == 'K') return ;
        size_t len = strlen(s);
        y --;
        for(int x = 0; x < len; x ++) {
            if(s[x] == 'A') robots.emplace_back(robot_count ++, (double)x / 2 / READ_MAP_SIZE, (double)y / 2 / READ_MAP_SIZE);
            else if(s[x] >= '0' && s[x] <= '9') stations.emplace_back(station_count ++, s[x] - '0', (double)x / 2 / READ_MAP_SIZE, (double)y / 2 / READ_MAP_SIZE);
        }
    }
}

// 读取一帧的输入
void readFrameInput() {
    // 当前金钱数
    scanf("%d", &curr_money);
    // 场上工作台数量
    scanf("%d", &station_num);
    station_seq.clear();
    for(int i = 0; i < station_num; i ++) station_seq.push_back(i);
    // 每个工作台的状态
    for(int i = 0; i < station_num; i ++) {
        scanf("%d %lf %lf %d %d %d", &stations[i].type, &stations[i].x, &stations[i].y, &stations[i].prod_time, &stations[i].material_state, &stations[i].product_state);
    }
    for(int i = 0; i < ROBOTS_NUM; i ++) {
        scanf("%d %d %lf %lf %lf %lf %lf %lf %lf %lf", &robots[i].station_id, &robots[i].object_type, &robots[i].time_val, &robots[i].collision_val, &robots[i].angular_speed, &robots[i].line_speed_x, &robots[i].line_speed_y, &robots[i].direction, &robots[i].x, &robots[i].y);
    }
    // 读取OK
    char s[1024];
    fgets(s, sizeof s, stdin);
    fgets(s, sizeof s, stdin);
}

inline double get_theta(double x, double y) {
    if(fabs(x) < 1e-5) return 0;
    double k = y / x;
    if(x >= 0 && y >= 0) return atan(k);
    else if(x <= 0 && y >= 0) return PI - atan(-k);
    else if(x <= 0 && y <= 0) return atan(k) - PI;
    else return -atan(-k);
}

inline double get_theta_diff(double from, double to) {
    double diff = to - from;
    if(diff > PI) return PI - diff;
    else if(diff < -PI) return 2 * PI + diff;
    return diff;
}

inline double get_speed(double x1, double y1, double x2, double y2) {
    double d = pow((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2), 0.7);
    return (1 - exp(-d)) * SPEED_MAX / 2;
}

void generateStrategy(Robot& robot) {
    if(robot.object_type && stations[robot.to_station].material_state[robot.object_type]) {
        int temp = robot.to_station;
        int i;
        for(i = 0; i < station_num; i ++) {
            if(need[stations[i].type][robot.object_type] && !stations[i].material_state[robot.object_type]) {
                stations[i].chosen = robot.id;
                robot.to_station = i;
                break;
            }
        }
        if(robot.to_station == temp || i == station_num) {
//            printf("destroy %d\n", robot.id);
//            robot.to_station = -1;
//            stations[robot.station_id].chosen = -1;
//            return;
        }
    }
    if(robot.station_id != -1) {
        if(robot.object_type) {
            if(need[stations[robot.station_id].type][robot.object_type] && !stations[robot.station_id].material_state[robot.object_type]) {
                printf("sell %d\n", robot.id);
                robot.to_station = -1;
                stations[robot.station_id].chosen = -1;
                return;
            }
        }else if(stations[robot.station_id].product_state) {
            printf("buy %d\n", robot.id);
            robot.to_station = -1;
            stations[robot.station_id].chosen = -1;
            return;
        }
    }
    if(robot.to_station == -1) {
        if(robot.object_type) {
            for(int i = 0; i < station_num; i ++) {
                if(need[stations[i].type][robot.object_type] && !stations[i].material_state[robot.object_type]) {
                    stations[i].chosen = robot.id;
                    robot.to_station = i;
                    break;
                }
            }
        }else {
            // 选择station
            sort(station_seq.begin(), station_seq.end(), [&](const int& x, const int& y) {
               return pow(stations[x].x - robot.x, 2) + pow(stations[x].y - robot.y, 2) < pow(stations[y].x - robot.x, 2) + pow(stations[y].y - robot.y, 2);
            });
            for(int j = 0; j < station_num; j ++) {
                int i = j;//station_seq[j];
                if(stations[i].chosen == -1 && stations[i].product_state) {
                    stations[i].chosen = robot.id;
                    robot.to_station = i;
                    break;
                }
            }
        }
        if(robot.to_station == -1) return ;
    }
    auto& station = stations[robot.to_station];
    // 确定转向
    double to_direction = get_theta(station.x - robot.x, station.y - robot.y);
//    if(fabs(to_direction - robot.direction + PI) < 1e-1) return ;
//    cerr << get_theta_diff(robot.direction, to_direction) << endl;
    printf("rotate %d %lf\n", robot.id, get_theta_diff(robot.direction, to_direction));
    printf("forward %d %lf\n", robot.id, get_speed(station.x, station.y, robot.x, robot.y));
}

int main() {
    readMap();
    puts("OK");
    fflush(stdout);
    int frameID;
    while (scanf("%d", &frameID) != EOF) {
        readFrameInput();
        printf("%d\n", frameID);
        for(int robotId = 0; robotId < 4; robotId++){
            generateStrategy(robots[robotId]);
        }
        printf("OK\n");
        fflush(stdout);
    }
    return 0;
}
