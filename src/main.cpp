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
    int product;

    Station(int id, int type, double x, double y): id(id), type(type), x(x), y(y) {
        chosen_state = 0;
        product_state = false;
        prod_time = 0x3f3f3f3f;
        need_state = basic_stations[type].material_bin;
        product = basic_stations[type].product;
    }

    double calc_dis(Station& other) const {
        return sqrt(pow(this->x - other.x, 2) + pow(this->y - other.y, 2));
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
    double mass{}; // 质量
    double alpha{}; // 角加速度
    double radius{}; // 半径
    double a{}; // 加速度

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
        update_status(false);
    }

    void update_status(bool state) { // 0 -> NOT busy, 1 -> busy
        // 属性状态
        this->to_station = -1;
        // 物理状态
        this->radius = state ? ROBOT_RADIUS_BUSY : ROBOT_RADIUS;
        this->mass = PI * this->radius * this->radius * ROBOT_DENSITY;
        this->alpha = ROBOT_TORQUE * 2 / (this->mass * this->radius * this->radius);
        this->a = ROBOT_MAX_FORCE / this->mass;
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
         * @param station: 购买东西的工作台
         */
        printf("buy %d\n", this->id);
        station.chosen_state[station.product] = false;
        this->object_type = station.product;
        update_status(true);
    }

    void sell(Station& station) {
        /* 出售物品给当前工作台，以输入数据的身处工作台ID为准。
         * @param station: 出售东西的工作台
         */
        printf("sell %d\n", this->id);
        station.chosen_state[this->object_type] = false;
        this->object_type = 0;
        update_status(false);
    }

    void destroy(Station& station) {
        /* 销毁物品。
         * @param station: 销毁物品要运往的地方
         */
        printf("destroy %d\n", this->id);
        station.chosen_state[this->object_type] = false;
        this->object_type = 0;
        update_status(false);
    }

    double calc_dis(Robot& other) const {
        /* 计算当前机器人到另一个机器人的欧式距离
         * @param other: 其他机器人
         */
        return sqrt(pow(this->x - other.x, 2) + pow(this->y - other.y, 2));
    }

    double calc_dis(Station& station) const {
        /* 计算当前机器人到一个工作台的欧氏距离
         * @param station: 工作台
         */
        return sqrt(pow(this->x - station.x, 2) + pow(this->y - station.y, 2));
    }
};

// 全局变量
vector<Robot> robots;
vector<Station> stations;
vector<int> station_seq;
int curr_money;
int station_num;
vector<vector<double>> dis_matrix;

// 初始化
void init() {
    // station id序列，方便之后排序
    for(int i = 0; i < station_num; i ++) station_seq.push_back(i);
    // 计算工作台间的距离矩阵
    dis_matrix = vector<vector<double>>(station_num, vector<double>(station_num, 0x3f3f3f3f));
    for(int i = 0; i < station_num; i ++) {
        dis_matrix[i][i] = 0;
        for(int j = i + 1; j < station_num; j ++) {
            dis_matrix[i][j] = dis_matrix[j][i] = stations[i].calc_dis(stations[j]);
        }
    }
}

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
            init();
            return ;
        }
        size_t len = strlen(s);
        y --;
        for(int x = 0; x < len; x ++) {
            if(s[x] == 'A') robots.emplace_back(robot_count ++, (double)x / READ_MAP_SIZE * MAP_SIZE, (double)y / READ_MAP_SIZE * MAP_SIZE);
            else if(s[x] >= '0' && s[x] <= '9') stations.emplace_back(station_num ++, s[x] - '0', (double)x / READ_MAP_SIZE * MAP_SIZE, (double)y / READ_MAP_SIZE * MAP_SIZE);
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
//        double tempx = 0, tempy = 0;
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

// 给定两个theta，求差距（[-pi, pi]）
inline double get_theta_diff(double from, double to) {
    double diff = to - from;
    if(diff > PI) return PI - diff;
    else if(diff < -PI) return 2 * PI + diff;
    return diff;
}

// 根据始末坐标控制速度
inline double get_speed(Robot& robot, Station& station) {
    double d = robot.calc_dis(station);
//    if(robot.line_speed * robot.line_speed / 2 / robot.a >= d) return 0;
    if(d <= 2 * NEAR_DISTANCE && robot.line_speed > 1) return 0;
//    if(robot.line_speed > robot.angular_speed * d)
//        return (1 - exp(-d)) * SPEED_POSITIVE_MAX;
    return SPEED_POSITIVE_MAX; // (1 - exp(-d)) * SPEED_POSITIVE_MAX;
}

// 判断产品有没有被其他的地方所需要
bool judge_product(int object_type) {
    if(object_type <= 0) return false;
    for(int i = 0; i < station_num; i ++) {
        if(!stations[i].chosen_state[object_type] && stations[i].need_state[object_type] && !stations[i].material_state[object_type]) return true;
    }
    return false;
}

// 符号函数
inline int sgn(double x) {
    return x >= 0 ? 1 : -1;
}

// 获取需要当前工作台产物的所有工作台的最短距离
double get_min_buyer_dis(Station& station) {
    double ans = 0X3f3f3f3f;
    for(int i = 0; i < station_num; i ++) {
        if(stations[i].need_state[station.product]) ans = min(ans, dis_matrix[i][station.id]);
    }
    return ans;
}

// 获取当前机器人离碰撞的最短距离
double get_min_col_dis(Robot& robot) {
    double min_dis = 0x3f3f3f3f;
    // 距离机器人
    for(auto& other : robots) {
        if(robot.id != other.id) {
            min_dis = min(min_dis, robot.calc_dis(other));
        }
    }
    // 距离墙壁
    double d1 = min(robot.x, MAP_SIZE - robot.x);
    double d2 = min(robot.y, MAP_SIZE - robot.y);
    min_dis = min(min_dis, min(d1, d2));
    return min_dis;
}

// 选择要去哪个工作台
Station* chooseStation(Robot& robot) {
    if(robot.to_station != -1) {
        if(robot.object_type) {
            if(!stations[robot.to_station].material_state[robot.object_type]){
                return &stations[robot.to_station];
            } else {
                stations[robot.to_station].chosen_state[robot.object_type] = false;
                robot.to_station = -1;
            }
        } else {
            if(judge_product(stations[robot.to_station].product)) {
                auto& prv_station = stations[robot.to_station];
                robot.to_station = -1;
                auto* station_ptr = chooseStation(robot);
                if(station_ptr != nullptr && robot.calc_dis(*station_ptr) + 20 < robot.calc_dis(prv_station)) {
                    prv_station.chosen_state[prv_station.product] = false;
                    robot.to_station = station_ptr->id;
                    station_ptr->chosen_state[station_ptr->product] = true;
                    return station_ptr;
                } else if(station_ptr != nullptr) {
                    station_ptr->chosen_state[station_ptr->product] = false;
                    robot.to_station = prv_station.id;
                }
                return &prv_station;
            } else {
                stations[robot.to_station].chosen_state[stations[robot.to_station].product] = false;
                robot.to_station = -1;
            }
        }
    }
    sort(station_seq.begin(), station_seq.end(), [&](const int& x, const int& y) {
//        if(stations[x].type == stations[y].type)
            return robot.calc_dis(stations[x]) < robot.calc_dis(stations[y]);
//        else return stations[x].type > stations[y].type;
//        return robot.calc_dis(stations[x]) + get_min_buyer_dis(stations[x]) < robot.calc_dis(stations[y])+ get_min_buyer_dis(stations[y]);
    });
    for(int j = 0; j < station_num; j ++) {
        int i = station_seq[j];
        if(robot.object_type) { // 带着东西的
            if(!stations[i].chosen_state[robot.object_type] && stations[i].need_state[robot.object_type] && !stations[i].material_state[robot.object_type]) {
                stations[i].chosen_state[robot.object_type] = true;
                robot.to_station = i;
                return &stations[i];
            }
        } else { // 没带
            if(stations[i].product_state && !stations[i].chosen_state[stations[i].product] && judge_product(stations[i].product)) {
                stations[i].chosen_state[stations[i].product] = true;
                robot.to_station = i;
                return &stations[i];
            }
        }
    }
    return nullptr;
}

// 调整机器人姿态（预期角速度和线速度）
void adjustRobotStatus(Robot& robot, Station& station) {
    // 确定转向
    double to_direction = atan2(station.y - robot.y, station.x - robot.x);
    double theta_diff = get_theta_diff(robot.direction, to_direction); // robot的朝向和与station的相对角位置的差
    double delta = theta_diff; // 真正要偏转的角度
    double angular_speed = pow(robot.angular_speed, 2) / robot.alpha / 2 >= fabs(delta) ? 0 : sgn(delta) * PI;
    // 确定速度
    double line_speed = get_speed(robot, station);
    // 调整一下
//    double min_dis = get_min_col_dis(robot);
//    if(min_dis <= 2 * ROBOT_RADIUS_BUSY + NEAR_DISTANCE) line_speed *= 0.8;
    robot.rotate(angular_speed);
    robot.forward(line_speed);
}

// 单独为每个机器人分配策略
void generateStrategy(Robot& robot) {
    // 到达工作台
    if(robot.station_id != -1 && robot.station_id == robot.to_station) {
        auto& station = stations[robot.station_id];
        if(robot.object_type) { // 出售物品
            if(station.need_state[robot.object_type] && !station.material_state[robot.object_type]) robot.sell(station);
        } else { // 购买物品
            if(station.product_state) robot.buy(station);
        }
    }
    // 下次去哪里
    auto* station_ptr = chooseStation(robot);
    if(station_ptr == nullptr) {
//        robot.destroy(stations[robot.to_station]);
//        cerr << "Can NOT found!" << endl;
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
