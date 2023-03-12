#include <iostream>
#include <utility>
#include <vector>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <bitset>

using namespace std;

// 常量
const int READ_MAP_SIZE = 100;   // 地图的大小100 x 100，单位为0.5m
const int ROBOTS_NUM = 4;        // 机器人数量
const double PI = acos(-1);   // 圆周率
const double SPEED_POSITIVE_MAX = 6;  // 最大正向速度
const double SPEED_NEGATIVE_MAX = -2; // 最大负向速度
const int OBJECT_TYPES_NUM = 7;  // 物品种类
const int STATION_TYPES_NUM = 9; // 工作台种类
const int FRAMES_PER_SECOND = 50; // 一秒的总帧数
const int TIME_SCALE = 3 * 60 * FRAMES_PER_SECOND; // 总帧数
const int MAP_SIZE = 50; // 地图大小50 x 50，单位为米
const double NEAR_DISTANCE = 0.4; // 机器人-工作台判定距离
const double ROBOT_RADIUS = 0.45; // 机器人半径（常态）
const double ROBOT_RADIUS_BUSY = 0.53; // 机器人半径（持有物品）
const double ROBOT_DENSITY = 20; // 机器人密度，单位：kg/m2。质量=面积*密度
const double ROBOT_MAX_FORCE = 250; // 最大牵引力(N)，机器人的加速/减速/防侧滑均由牵引力驱动
const double ROBOT_TORQUE = 50; // 最大力矩(N*m)，机器人的旋转由力矩驱动

/***基础类***/
// 物品
class Object {
public:
    int type{};
    vector<int> material;
    bitset<OBJECT_TYPES_NUM + 1> material_bin{};
    int buy_money{};
    int sell_money{};
    Object() = default;
    Object(int type, vector<int> material, int material_bin, int buy_money, int sell_money): type(type), material(std::move(material)), material_bin(material_bin), buy_money(buy_money), sell_money(sell_money) {}
};
const vector<Object> objects = {
        {}, {1, {}, 0b00000000, 3000, 6000},
        {2, {}, 0b00000000, 4400, 7600},
        {3, {}, 0b00000000, 5800, 9200},
        {4, {1, 2}, 0b00000110, 15400, 22500},
        {5, {1, 3}, 0b00001010, 17200, 25000},
        {6, {2, 3}, 0b00001100, 19200, 27500},
        {7, {4, 5, 6}, 0b01110000, 76000, 105000},
};

// 工作台基本性质
class BasicStation {
public:
    int type{};
    vector<int> material;
    bitset<OBJECT_TYPES_NUM + 1> material_bin{};
    int period{};
    int product{};
    BasicStation() = default;
    BasicStation(int type, vector<int> material, int material_bin, int period, int product): type(type), material(std::move(material)), material_bin(material_bin), period(period), product(product) {}
};
const vector<BasicStation> basic_stations = {
        {}, {1, {}, 0b00000000, 50, 1},
        {2, {}, 0b00000000, 50, 2},
        {3, {}, 0b00000000, 50, 3},
        {4, {1, 2}, 0b00000110, 500, 4},
        {5, {1, 3}, 0b00001010, 500, 5},
        {6, {2, 3}, 0b00001100, 500, 6},
        {7, {4, 5, 6}, 0b01110000, 1000, 7},
        {8, {7}, 0b10000000, 1, -1},
        {9, {1, 2, 3, 4, 5, 6, 7}, 0b11111110, 1, -1},
};

// 工作台
class Station {
public:
    int id;
    int type;
    double x, y;
    int prod_time;
    std::bitset<OBJECT_TYPES_NUM + 1> material_state;
    int product_state;
    std::bitset<OBJECT_TYPES_NUM + 1> chosen_state;
    std::bitset<OBJECT_TYPES_NUM + 1> need_state;

    Station(int id, int type, double x, double y): id(id), type(type), x(x), y(y) {
        chosen_state = 0;
        product_state = false;
        prod_time = 0x3f3f3f3f;
        need_state = basic_stations[type].material_bin;
    }
};

// 机器人
class Robot {
public:
    int id;         // 机器人标识：[0,3]
    double x, y;    // 机器人坐标
    int station_id; // 当前所在station的id：-1, [0, station_num - 1]
    int object_type;// 携带的物品的类型：0, [1, 7]
    double time_val, collision_val; // 时间价值系数和碰撞价值系数：[0.8, 1]
    double angular_speed; // 角速度
    double line_speed_x, line_speed_y, line_speed{}; // 线速度
    double direction; // 朝向

    int to_station; // 想要去的station的id：-1, [0, station_num - 1]

    Robot(int id, double x, double y): id(id), x(x), y(y) {
        to_station = -1;
        station_id = -1;
        object_type = -1;
        time_val = 0;
        collision_val = 0;
        angular_speed = 0;
        line_speed_x = 0;
        line_speed_y = 0;
        direction = 0;
    }

    void forward(double speed) const {
        /* 机器人前进
         * @param speed: 前进速度，单位为米/秒。取值范围：[-2, 6]
         *      - 正数表示前进。
         *      - 负数表示后退。
         */
        printf("forward %d %lf\n", this->id, speed);
    }

    void rotate(double speed) const {
        /* 机器人转向
         * @param speed: 旋转速度，单位为弧度/秒。取值范围：[-pi, pi]
         *      - 负数表示顺时针旋转。
         *      - 正数表示逆时针旋转。
         */
        printf("rotate %d %lf\n", this->id, speed);
    }

    void buy(Station& station) {
        /* 购买当前工作台的物品，以输入数据的身处工作台ID为准。
         */
        printf("buy %d\n", this->id);
        this->to_station = -1;
        station.chosen_state[this->object_type] = false;
        this->object_type = 0;
    }

    void sell(Station& station) {
        /* 出售物品给当前工作台，以输入数据的身处工作台ID为准。
         */
        printf("sell %d\n", this->id);
        this->to_station = -1;
        station.chosen_state[this->object_type] = false;
        this->object_type = 0;
    }

    void destroy(Station& station) {
        /* 销毁物品。
         */
        printf("destroy %d\n", this->id);
        this->to_station = -1;
        station.chosen_state[this->object_type] = false;
        this->object_type = 0;
    }

    double calc_dis(Robot& other) const {
        return sqrt(pow(this->x - other.x, 2) + pow(this->y - other.y, 2));
    }

    double calc_dis(Station& station) const {
        return sqrt(pow(this->x - station.x, 2) + pow(this->y - station.y, 2));
    }
};

// 全局变量
vector<Robot> robots;
vector<Station> stations;
vector<int> station_seq;
int curr_money;
int station_num;

// 读取地图
void readMap() {
    /* 读取地图，进行初始化
     * 地图左下角坐标为原点(0,0)，右上角坐标为(50,50)。
     */
    int y = READ_MAP_SIZE;  // 所在x的坐标
    int robot_count = 0;    // 机器人数量

    char s[1024];
    while (fgets(s, sizeof s, stdin)) {
        if (s[0] == 'O' && s[1] == 'K') {
            for(int i = 0; i < station_num; i ++) station_seq.push_back(i);
            return ;
        }
        size_t len = strlen(s);
        y --;
        for(int x = 0; x < len; x ++) {
            if(s[x] == 'A') robots.emplace_back(robot_count ++, (double)x / 2 / READ_MAP_SIZE, (double)y / 2 / READ_MAP_SIZE);
            else if(s[x] >= '0' && s[x] <= '9') stations.emplace_back(station_num ++, s[x] - '0', (double)x / 2 / READ_MAP_SIZE, (double)y / 2 / READ_MAP_SIZE);
        }
    }
}

// 读取一帧的输入
void readFrameInput() {
    // 当前金钱数
    scanf("%d", &curr_money);
    // 场上工作台数量
    scanf("%d", &station_num);
    // 每个工作台的状态
    for(int i = 0; i < station_num; i ++) {
        scanf("%d %lf %lf %d %d %d", &stations[i].type, &stations[i].x, &stations[i].y, &stations[i].prod_time, &stations[i].material_state, &stations[i].product_state);
    }
    for(int i = 0; i < ROBOTS_NUM; i ++) {
        scanf("%d %d %lf %lf %lf %lf %lf %lf %lf %lf", &robots[i].station_id, &robots[i].object_type, &robots[i].time_val, &robots[i].collision_val, &robots[i].angular_speed, &robots[i].line_speed_x, &robots[i].line_speed_y, &robots[i].direction, &robots[i].x, &robots[i].y);
        robots[i].line_speed = sqrt(pow(robots[i].line_speed_x, 2) + pow(robots[i].line_speed_y, 2));
    }
    // 读取OK
    char s[1024];
    fgets(s, sizeof s, stdin);
    fgets(s, sizeof s, stdin);
}

inline double get_theta_diff(double from, double to) {
    double diff = to - from;
    if(diff > PI) return PI - diff;
    else if(diff < -PI) return 2 * PI + diff;
    return diff;
}

inline double get_speed(Robot& robot, Station& station) {
    double d = robot.calc_dis(station);
    if(d <= 2 * NEAR_DISTANCE && robot.line_speed > 1) return 0;
    return SPEED_POSITIVE_MAX; // (1 - exp(-d)) * SPEED_POSITIVE_MAX;
}

bool judge_product(int object_type) {
    if(object_type <= 0) return false;
    for(int i = 0; i < station_num; i ++) {
        if(stations[i].need_state[object_type] && !stations[i].material_state[object_type]) return true;
    }
    return false;
}

Station* chooseStation(Robot& robot) {
    if(robot.to_station != -1) {
        if(robot.object_type) {
            if(!stations[robot.to_station].material_state[robot.object_type])
                return &stations[robot.to_station];
        } else {
            if(judge_product(basic_stations[stations[robot.to_station].type].product))
                return &stations[robot.to_station];
        }
    }
    sort(station_seq.begin(), station_seq.end(), [&](const int& x, const int& y) {
        double dx = sqrt(pow(stations[x].x - robot.x, 2) + pow(stations[x].y - robot.y, 2));
        double dy = sqrt(pow(stations[y].x - robot.x, 2) + pow(stations[y].y - robot.y, 2));
//        if(fabs(dx - dy) < 25)
//            return stations[x].type > stations[y].type;
//        else
            return dx < dy;
    });
    for(int j = 0; j < station_num; j ++) {
        int i = station_seq[j];
        if(!stations[i].chosen_state[robot.object_type]) {
            if(robot.object_type) {
                if(stations[i].need_state[robot.object_type] && !stations[i].material_state[robot.object_type]) {
                    stations[i].chosen_state[robot.object_type] = true;
                    robot.to_station = i;
                    return &stations[i];
                }
            } else if(stations[i].product_state && judge_product(basic_stations[stations[i].type].product)) {
                stations[i].chosen_state[robot.object_type] = true;
                robot.to_station = i;
                return &stations[i];
            }
        }
    }
    return nullptr;
}

void adjustRobotStatus(Robot& robot, Station& station) {
    // 计算当前robot和其他robot的位置
    double min_dis = 0x3f3f3f3f;
    double angular_diff = 0;
    for(auto& other : robots) {
        if(robot.id != other.id) {
            double temp = robot.calc_dis(other);
            if(temp < min_dis) {
                min_dis = temp;
                angular_diff = fabs(robot.direction - other.direction);
            }
        }
    }
    // 确定转向
    double to_direction = atan2(station.y - robot.y, station.x - robot.x);
    double angular_speed = get_theta_diff(robot.direction, to_direction);
//    if(min_dis < 3 * ROBOT_RADIUS_BUSY) {
//        robot.rotate(angular_speed + PI/4);
//    }else
        robot.rotate(angular_speed * 2);
    // 确定速度
//    robot.forward(get_speed(robot, station));
    if(fabs(angular_speed) <= PI / 8) robot.forward(get_speed(robot, station));
    else robot.forward(0);
}

void generateStrategy(Robot& robot) {
    // 选择目的工作台
    auto station_ptr = chooseStation(robot);
    if(station_ptr == nullptr) {
//        if(robot.object_type)
//            robot.destroy(stations[robot.to_station]);
        return ;
    }
    // 到达工作台
    if(robot.station_id == robot.to_station) {
        if(robot.object_type) { // 出售物品
            if(station_ptr->need_state[robot.object_type] && !station_ptr->material_state[robot.object_type]) robot.sell(*station_ptr);
            else station_ptr = chooseStation(robot);
        } else { // 购买物品
            if(station_ptr->product_state) {
                robot.buy(*station_ptr);
            } else station_ptr = chooseStation(robot);
        }
    }
    if(station_ptr == nullptr) {
        robot.destroy(stations[robot.to_station]);
        return ;
    }
    // 根据目的地调整机器人状态
    adjustRobotStatus(robot, *station_ptr);
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
