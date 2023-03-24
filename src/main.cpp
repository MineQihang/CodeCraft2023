#include <iostream>
#include <utility>
#include <vector>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <bitset>
#include <queue>
#include <map>

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
const double EPS = 1e-1; // 最大偏差
const double MAX_UNIT_DIS = SPEED_POSITIVE_MAX / FRAMES_PER_SECOND; // 最小单位距离
const int PREDICT_TIME = FRAMES_PER_SECOND * 2; // 碰撞预测帧数

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
        {}, {1, {}, 0b00000000, 3000, 6000}, // 利润：3000
        {2, {}, 0b00000000, 4400, 7600}, // 利润：3200
        {3, {}, 0b00000000, 5800, 9200}, // 利润：3400
        {4, {1, 2}, 0b00000110, 15400, 22500}, // 利润：7100
        {5, {1, 3}, 0b00001010, 17200, 25000}, // 利润：7800
        {6, {2, 3}, 0b00001100, 19200, 27500}, // 利润：8300
        {7, {4, 5, 6}, 0b01110000, 76000, 105000}, // 利润：29000
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
        {9, {7}, 0b11111110, 1, -1}, // TODO: 1, 2, 3, 4, 5, 6, 7
};

// 计算距离 // TODO: 不使用欧式距离
inline double _calc_dis(double x1, double y1, double x2, double y2) {
    return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
//    return fabs(x1 - x2) + fabs(y1 - y2);
}

// 计算距离时间 // TODO: 优化-精准预测时间
inline double _calc_time(double dis) {
    return dis / SPEED_POSITIVE_MAX;
}

vector<vector<double>> dis_matrix;  // 工作台之间的距离矩阵

// 工作台
class Station {
public:
    int id{};       // 工作台id:[0, 49]
    int type{};     // 工作台类型:[1, 9]
    double x{}, y{};// 坐标
    int prod_time{};// 剩余生产时间: -1:没有生产, 0:生产因输出格满而阻塞, >=0:剩余生产帧数
    std::bitset<OBJECT_TYPES_NUM + 1> material_state;   // 原料格当前填充状态
    std::bitset<OBJECT_TYPES_NUM + 1> chosen_state;     // 原料/成品格是否被机器人选中
    std::bitset<OBJECT_TYPES_NUM + 1> need_state;       // 这个工作台需要的原料有哪些
    int product_state{};    // 产品的状态: 0:还没有成品, 1:已经有了成品
    int product{};  // 产品类型

    Station() = default;

    Station(int id, int type, double x, double y): id(id), type(type), x(x), y(y) {
        chosen_state = 0;
        product_state = false;
        prod_time = -1;
        need_state = basic_stations[type].material_bin;
        product = basic_stations[type].product;
    }

    double calc_dis(Station& other, bool mode=false) const {
        if(mode) return _calc_dis(x, y, other.x, other.y);
        else return dis_matrix[id][other.id];
    }

    int count_need() const {
        return need_state.count() - material_state.count();
    }

    bool can_sell(int object_type, double time=-1) {
        // TODO: 这个判断还可以考虑product_state可能在他过去的时候发生改变
        if(prod_time >= 0 && time != -1 && time * FRAMES_PER_SECOND >= prod_time && !product_state && count_need() == 0) return need_state[object_type];
        return need_state[object_type] && !material_state[object_type];
    }

    bool can_buy(double time=-1) const {
        if(prod_time >= 0 && time != -1 && time * FRAMES_PER_SECOND >= prod_time) return true;
        else return product_state;
    }

    bool check_chosen(int object_type) {
        return this->chosen_state[object_type];
    }

    bool check_valid(int obj) {
        return can_sell(obj) && !check_chosen(obj);
    }

    double calc_time(Station& other) const {
        return _calc_time(calc_dis(other));
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
    double angular_speed{}; // 角速度
    double line_speed_x{}, line_speed_y{}, line_speed{}; // 线速度
    double direction{}; // 朝向

    double mass{}; // 质量
    double alpha{}; // 角加速度
    double radius{}; // 半径
    double a{}; // 加速度
    double run_time{}; // 运输了多久了
    double expect_time{}; // 预期运输时间

    int to_station; // 想要去的station的id：-1, [0, station_num - 1]
    queue<int> path;

    Robot(int id, double x, double y): id(id), x(x), y(y) {
        to_station = -1;
        station_id = -1;
        object_type = -1;
        update_physical_status(false);
    }

    void update_physical_status(bool state) { // 0 -> NOT busy, 1 -> busy
        // 更新物理状态
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

    void add_path(Station& station, int obj) {
        /* 向规划队列中加入之后要去的地方
         */
        path.push(station.id);
        choose_station(station, obj);
    }

    void next_station() {
        /* 进入下一个地方
         */
        if(path.empty()) {
            to_station = -1;
//            cerr << "hi" << endl;
        }
        else {
            to_station = path.front();
            path.pop();
//            cerr << path.size() << endl;
        }
    }

    void buy(Station& station) {
        /* 购买当前工作台的物品，以输入数据的身处工作台ID为准。
         * @param station: 购买东西的工作台
         */
        printf("buy %d\n", this->id);
        cancel_choose_station(station, station.product);
        this->object_type = station.product;
        update_physical_status(true);
        next_station();
    }

    void sell(Station& station) {
        /* 出售物品给当前工作台，以输入数据的身处工作台ID为准。
         * @param station: 出售东西的工作台
         */
        printf("sell %d\n", this->id);
        cancel_choose_station(station, this->object_type);
        station.material_state[this->object_type] = true;
        this->object_type = 0;
        update_physical_status(false);
        next_station();
    }

    void destroy(Station& station) {
        /* 销毁物品。 // TODO:完善销毁物品的函数
         * @param station: 销毁物品要运往的地方
         */
        printf("destroy %d\n", this->id);
        cancel_choose_station(station, this->object_type);
        this->object_type = 0;
        update_physical_status(false);
        next_station();
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

    void update_time(bool state, Station* station = nullptr) {
        /* 更新时间
         * @param state: bool 0: 加一帧的时间, 1: 已运行时间归零、计算新的期望时间
         * @param station: ptr 根据station计算距离从而计算预期时间
         */
        if(state) {
//            cerr << "run time: " << this->run_time << endl;
            this->run_time = 0;
            if(station != nullptr)
                this->expect_time = this->calc_dis(*station) / SPEED_POSITIVE_MAX * 5;
        } else this->run_time += 1.0 / FRAMES_PER_SECOND;
    }

    void choose_station(Station& station, int obj) {
        /* 选择去某个station，需要占用哪个格子
         */
        this->to_station = station.id;
        station.chosen_state[obj] = true;
    }

    void cancel_choose_station(Station& station, int obj) {
        /* 取消选择去某个station，需要解除占用哪个格子
         */
        this->to_station = -1;
        station.chosen_state[obj] = false;
    }

    bool check_arrived() const {
        /* 检查是否已到达需要去的地方
         */
        return this->to_station != -1 && this->to_station == this->station_id;
    }

    double calc_time(Station& station) const {
        return _calc_time(calc_dis(station));
    }
};

// 全局变量
vector<Robot> robots;
vector<Station> stations;
vector<int> station_seq; // 排序用seq
vector<int> station_lru; // LRU算法确定station类型
int frameID;    // 当前帧数
int curr_money; // 当前还剩多少钱
int station_num;// 工作台的总数量
vector<vector<int>> obj_stations_buy(OBJECT_TYPES_NUM + 1); // [x, [y1, y2, ...]] 可以买x的工作台有[y1, y2, ...](工作台产品是x)
vector<vector<int>> obj_stations_sell(OBJECT_TYPES_NUM + 1);// [x, [y1, y2, ...]] 可以卖x的工作台有[y1, y2, ...](工作台以x为原料)
map<pair<int, int>, double> book; // 记录从x到y能获得的利润

// 任务
class Task {
public:
    int from{};  // 去哪买
    int to{};    // 去哪卖
    int obj{};
    int level{}; // 优先级
    int need_num{};
    Task() = default;
    Task(int _from, int _to, int _obj, int _level, int _need_num): from(_from), to(_to), obj(_obj), level(_level), need_num(_need_num) {};

    bool operator < (const Task other) const {
        if(level == other.level) {
//            if(need_num == other.need_num)
            return need_num > other.need_num;
        } else return level < other.level;
    }
};

// 控制器
class Controller{
public:
    priority_queue<Task> tasks;

    void init() {

    }

    void updateTasks() {
        while(!tasks.empty()) tasks.pop();
        for(int obj = OBJECT_TYPES_NUM; obj >= 1; obj --) {
            for(auto& buy_station_id : obj_stations_buy[obj]) {
                for(auto& sell_station_id : obj_stations_sell[obj]) {
                    auto& sell_station = stations[sell_station_id];
                    int level;
//                    if(sell_station.product == -1) level = 8;
//                    else if(sell_station.type == 8) level = -1;
//                    else
                        level = sell_station.product;
                    tasks.emplace(buy_station_id, sell_station_id, obj, level, sell_station.count_need());
                }
            }
        }
    }

    void requestStrategy(Robot& robot) {
        updateTasks();
        while(!tasks.empty()) {
            auto task = tasks.top();
            tasks.pop();
            auto [buy_station_id, sell_station_id, obj, _, __] = task;
            auto& buy_station = stations[buy_station_id];
            auto& sell_station = stations[sell_station_id];
            if(buy_station.can_buy() && sell_station.can_sell(obj) && !buy_station.check_chosen(obj) && !sell_station.check_chosen(obj)){
                robot.add_path(buy_station, obj);
                robot.add_path(sell_station, obj);
                break;
            }
        }
    }
} controller;

namespace RobotMove{
    // 判断两个浮点数相等
    inline bool double_equal(double x, double y) {
        return x - EPS <= y && y <= x + EPS;
    }

    // 符号函数
    inline int sgn(double x) {
        return x >= 0 ? 1 : -1;
    }

    // 获取当前机器人离碰撞的最短距离: mode: 0->机器人, 1->墙壁, 2->机器人和墙壁
    double get_min_col_dis(Robot& robot, int mode=0) {
        double min_dis = 0x3f3f3f3f;
        // 距离机器人
        if(mode == 0 || mode == 2) {
            for(auto& other : robots) {
                if(robot.id != other.id) {
                    min_dis = min(min_dis, robot.calc_dis(other));
                }
            }
        }
        // 距离墙壁
        if(mode == 1 || mode == 2) {
            if(robot.direction > 0 && robot.direction <= PI / 2) min_dis = min(min_dis, min(MAP_SIZE - robot.x, MAP_SIZE - robot.y));
            if(robot.direction > PI / 2 && robot.direction <= PI) min_dis = min(min_dis, min(robot.x, MAP_SIZE - robot.y));
            if(robot.direction >= -PI && robot.direction < -PI / 2) min_dis = min(min_dis, min(robot.x, robot.y));
            if(robot.direction >= -PI / 2 && robot.direction <= 0) min_dis = min(min_dis, min(MAP_SIZE - robot.x, robot.y));
        }
        return min_dis;
    }

    // 预测是否会发生碰撞
    bool predict_col(Robot& robot1, Robot& robot2) {
        double min_d = robot1.radius + robot2.radius;
        double x1 = robot1.x, x2 = robot2.x;
        double y1 = robot1.y, y2 = robot2.y;
//        double theta1 = robot1.direction, theta2 = robot2.direction;
        double theta1 = atan2(robot1.line_speed_y, robot1.line_speed_x), theta2 = atan2(robot2.line_speed_y, robot2.line_speed_x);
        double v1 = robot1.line_speed, v2 = robot2.line_speed;
        double w1 = robot1.angular_speed, w2 = robot2.angular_speed;
        auto calc_dis_xy = [](double _x1, double _y1, double _x2, double _y2){
            return sqrt(pow(_x1 - _x2, 2) + pow(_y1 - _y2, 2));
        };
        auto update_val = [](double& x, double& y, double& theta, double& v, double& w) {
            x += v * cos(theta) * 1 / FRAMES_PER_SECOND;
            y += v * sin(theta) * 1 / FRAMES_PER_SECOND;
            theta += w * 1 / FRAMES_PER_SECOND;
        };
        for(int i = 0; i < PREDICT_TIME; i ++) {
            if(calc_dis_xy(x1, y1, x2, y2) <= min_d) return true;
            update_val(x1, y1, theta1, v1, w1);
            update_val(x2, y2, theta2, v2, w2);
        }
        return false;
    }

    // 给定两个theta，求差距（[-pi, pi]）
    inline double get_theta_diff(double from, double to) {
        double diff = to - from;
        if(diff > PI) return PI - diff;
        else if(diff < -PI) return 2 * PI + diff;
        return diff;
    }

    // 根据始末坐标控制前进速度
    inline double get_forward_speed(Robot& robot, Station& station) {
        double d = robot.calc_dis(station);
//        if(robot.line_speed * robot.line_speed / 2 / robot.a >= d) return 0;
        if(d <= NEAR_DISTANCE + robot.radius + MAX_UNIT_DIS && robot.line_speed > 1) return 0;
        // 避免撞墙
        double min_dis = get_min_col_dis(robot, 1);
        if((min_dis - robot.radius - MAX_UNIT_DIS) * 2 * robot.a <= pow(robot.line_speed, 2) * fabs(sin(robot.direction))) return 0;
//        if((d - NEAR_DISTANCE - MAX_UNIT_DIS) * 2 * robot.a <= pow(robot.line_speed, 2) * fabs(sin(robot.direction)) && robot.line_speed > SPEED_POSITIVE_MAX / 2) return 0;
//        if(double_equal(robot.line_speed, robot.angular_speed * 1.9101)) return 0;
        return SPEED_POSITIVE_MAX; // (1 - exp(-d)) * SPEED_POSITIVE_MAX;
    }

    // 根据始末坐标控制角速度
    inline double get_angular_speed(Robot& robot, Station& station) {
        double to_direction = atan2(station.y - robot.y, station.x - robot.x);
        double theta_diff = get_theta_diff(robot.direction, to_direction); // robot的朝向和与station的相对角位置的差
        double delta = theta_diff; // 真正要偏转的角度
        double angular_speed = pow(robot.angular_speed, 2) / robot.alpha / 2 >= fabs(delta) ? 0 : sgn(delta) * PI;
//        double d = robot.calc_dis(station);
//        angular_speed *= (1 - exp(-3 * d));
        return angular_speed;
    }

    // 调整机器人姿态（预期角速度和线速度）
    void adjustRobotStatus(Robot& robot, Station& station) {
        // 确定转向
        double angular_speed = get_angular_speed(robot, station);
        // 确定速度
        double line_speed = get_forward_speed(robot, station);
        // 调整一下
        for(int i = 0; i < ROBOTS_NUM; i ++) {
            if(i != robot.id && ((robot.direction > 0) ^ (robots[i].direction > 0)) && predict_col(robot, robots[i])) {
//            line_speed = -2;
//            angular_speed *= 0.5;
//            cerr << robot.id << ": CHANGE!" << endl;
//                angular_speed = robot.angular_speed * 0.1 + PI / 2;
//            angular_speed += PI;
                break;
//            if(angular_speed <= 0) angular_speed += PI / 2;
//            else angular_speed -= PI / 2;
            }
        }
//    double min_dis = get_min_col_dis(robot);
//    if(min_dis <= 2 * ROBOT_RADIUS_BUSY + NEAR_DISTANCE) line_speed *= 0.8;
//    angular_speed *= (1 - exp(-100 * robot.calc_dis(station)));
//    if(robot.run_time > robot.expect_time) {
//        line_speed = line_speed * 0.9;
//    }
        // 执行
        robot.rotate(angular_speed);
        robot.forward(line_speed);
    }
};

namespace Strategy{
    // 判断产品有没有被其他的地方所需要
    bool judge_product(int object_type) {
        if(object_type <= 0) return false;
        for(int i = 0; i < station_num; i ++) {
            if(!stations[i].chosen_state[object_type] && stations[i].need_state[object_type] && !stations[i].material_state[object_type]) return true;
        }
        return false;
    }

    // 计算时间系数
    inline double calc_time_val(double t) {
        return (1 - sqrt(1 - pow(1 - t / TIME_SCALE, 2))) * (1 - 0.8) + 0.8;;
    }

    // 选择要去哪个工作台
    void chooseStation(Robot& robot) {
        for(int obj = OBJECT_TYPES_NUM; obj >= 1; obj --) {
            // 物品类型由大到小去遍历，利润会高一些
            // 选择第一个可行类型中的最优路径即可
            double min_dis = 0x3f3f3f3f;
            int bs = -1, ss = -1; // buy, sell | station
            for(int& sell_station_id : obj_stations_sell[obj]) {
                auto& sell_station = stations[sell_station_id];
                if(!sell_station.can_sell(obj/*, robot.calc_time(buy_station) + buy_station.calc_time(sell_station)*/) || sell_station.check_chosen(obj)) continue;
                for(int& buy_station_id : obj_stations_buy[obj]) {
                    auto& buy_station = stations[buy_station_id];
                    if(!buy_station.can_buy(/*robot.calc_time(buy_station)*/) || buy_station.check_chosen(obj)) continue;
                    double dis = robot.calc_dis(buy_station) + buy_station.calc_dis(sell_station);
                    if(dis < min_dis) {
                        min_dis = dis;
                        bs = buy_station_id;
                        ss = sell_station_id;
                    }
                }
            }
            if(min_dis != 0x3f3f3f3f) {
                robot.add_path(stations[bs], obj);
                robot.add_path(stations[ss], obj);
                break;
            }
        }
//        robot.to_station = 1;
//        return;
//        priority_queue<pair<double, array<int, 3>>> q;
//        // 要去买什么
//        for(int i = 1; i <= OBJECT_TYPES_NUM; i ++) {
//            // 到哪里去买
//            for(int buy_station : obj_stations_buy[i]) {
//                // 看看现在可以买吗
//                double buy_time = robot.calc_time(stations[buy_station]);
//                if(!stations[buy_station].can_buy(buy_time) || stations[buy_station].check_chosen(i)) continue;
//                // 要到哪里去卖
//                for(int sell_station : obj_stations_sell[i]) {
//                    // 时间优化！！！
//                    double sell_time = stations[buy_station].calc_time(stations[sell_station]);
//                    if(!stations[sell_station].can_sell(i, (buy_time + sell_time)) || stations[sell_station].check_chosen(i)) continue;
//                    q.push({objects[i].sell_money * calc_time_val(buy_time + sell_time) / (buy_time + sell_time), {i, buy_station, sell_station}});
//                }
//            }
//        }
//        if(q.empty()) return ;
//        auto [_, temp] = q.top();
//        auto [obj, bs, ss] = temp;
//        robot.add_path(stations[bs], obj);
//        robot.add_path(stations[ss], obj);
//        if(stations[ss].prod_time >= 0 || stations[ss].product_state) {
//            obj = stations[ss].product;
//            // 要到哪里去卖
//            int t = -1;
//            double r = 0x3f3f3f3f;
//            for(int sell_station : obj_stations_sell[obj]) {
//                // 时间优化！！！
//                double sell_time = stations[ss].calc_time(stations[sell_station]);
//                if(!stations[sell_station].can_sell(obj, sell_time) || stations[sell_station].check_chosen(obj)) continue;
//                if(sell_time < r) {
//                    r = sell_time;
//                    t = sell_station;
//                }
//            }
//            if(t != -1) {
//                robot.add_path(stations[ss], obj);
//                robot.add_path(stations[t], obj);
//            }
//        }
    }

    // 单独为每个机器人分配策略
    void generateStrategyForRobot(Robot& robot) {
        // 更新运送时间
        robot.update_time(false);
        // 还没有策略需要生成策略
        if(robot.to_station == -1) {
            controller.requestStrategy(robot);
            robot.next_station();
        } else { // 已经有了策略
            // 到达工作台
            if(robot.check_arrived()) {
                auto& station = stations[robot.station_id];
                if(robot.object_type) { // 出售物品
                    if(station.can_sell(robot.object_type)) {
                        robot.sell(station);
//                        robot.next_station();
                    }
                } else { // 购买物品
                    if(station.product_state){
                        // TODO: 优化最后的时间【其实就是优化距离时间】 这里固定为3秒
                        if(frameID < TIME_SCALE - 3 * FRAMES_PER_SECOND) {
                            robot.buy(station);
//                            robot.next_station();
                        }
                    }
                }
            }
        }
        // 没有成功生成策略就无需进行调整了/卖了
        if(robot.to_station == -1) return ;
    }
};

// 初始化
void init() {
    // station id序列，方便之后排序
    for(int i = 0; i < station_num; i ++) station_seq.push_back(i);
    // 计算工作台间的距离矩阵
    dis_matrix = vector<vector<double>>(station_num, vector<double>(station_num, 0x3f3f3f3f));
    for(int i = 0; i < station_num; i ++) {
        dis_matrix[i][i] = 0;
        for(int j = i + 1; j < station_num; j ++) {
            dis_matrix[i][j] = dis_matrix[j][i] = stations[i].calc_dis(stations[j], true);
        }
    }
    // 初始化LRU
    for(int i = 1; i <= STATION_TYPES_NUM; i ++) station_lru.push_back(i);
    // 初始化购买stations和出售stations
    for(int i = 0; i < station_num; i ++) {
        auto basic_station = basic_stations[stations[i].type];
        if(basic_station.product != -1) obj_stations_buy[basic_station.product].push_back(i);
        for(auto& material : basic_station.material) obj_stations_sell[material].push_back(i);
    }
    // 初始化controller
    controller.init();
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
            if(s[x] == 'A') { // 机器人
                robots.emplace_back(robot_count ++, (double)x / READ_MAP_SIZE * MAP_SIZE, (double)y / READ_MAP_SIZE * MAP_SIZE);
            } else if(s[x] >= '0' && s[x] <= '9') { // 工作台
                stations.emplace_back(station_num ++, s[x] - '0', (double)x / READ_MAP_SIZE * MAP_SIZE, (double)y / READ_MAP_SIZE * MAP_SIZE);
            }
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
//        if(robots[i].collision_val < 1 && robots[i].collision_val != 0) cerr << i << ": col!" << endl;
    }
    // 读取OK
    char s[1024];
    fgets(s, sizeof s, stdin);
    fgets(s, sizeof s, stdin);
}

// 生成策略
void generateStrategy() {
    for(int robotId = 0; robotId < 4; robotId++){
        Strategy::generateStrategyForRobot(robots[robotId]);
        // 根据目的地调整机器人状态
        if(robots[robotId].to_station == -1) continue;
        RobotMove::adjustRobotStatus(robots[robotId], stations[robots[robotId].to_station]);
    }
}

// 调整策略
void optimizeStrategy(){
    for(int i = 0; i < ROBOTS_NUM; i ++) {
        for(int j = i + 1; j < ROBOTS_NUM; j ++) {
            if(robots[i].to_station != -1 && robots[i].to_station != -1 && robots[i].object_type == robots[j].object_type) {
                // TODO: 优化判断距离的方式
                if((robots[i].calc_dis(stations[robots[i].to_station]) + robots[j].calc_dis(stations[robots[j].to_station])) >
                (robots[i].calc_dis(stations[robots[j].to_station]) + robots[j].calc_dis(stations[robots[i].to_station]))) {
                    swap(robots[i].to_station, robots[j].to_station);
                    swap(robots[i].path, robots[j].path);
                }
            }
        }
    }
}

int main() {
    readMap();
    puts("OK");
    fflush(stdout);
    while (scanf("%d", &frameID) != EOF) {
        readFrameInput();
        printf("%d\n", frameID);
        generateStrategy();
//        optimizeStrategy();
        printf("OK\n");
        fflush(stdout);
    }
    return 0;
}
