//Compile with: g++ solution.cpp -o app -std=c++11 -I/usr/include/python2.7 -lpython2.7
// #include "src/matplotlibcpp.h" //Graph Library
#include "matplotlib-cpp/matplotlibcpp.h"

#include <iostream>
#include <string>
#include <math.h>
#include <stdexcept> // throw errors
#include <random> //C++ 11 Random Numbers
#include <vector>

namespace plt = matplotlibcpp;
using namespace std;

// Landmarks
double landmarks[8][2] = { { 20.0, 20.0 }, { 20.0, 80.0 }, { 20.0, 50.0 },
    { 50.0, 20.0 }, { 50.0, 80.0 }, { 80.0, 80.0 },
    { 80.0, 20.0 }, { 80.0, 50.0 } };

// Map size in meters
double world_size = 100.0;

// Random Generators
random_device rd;
mt19937 gen(rd());

// Global Functions
double mod(double first_term, double second_term);
double gen_real_random();

// Robot 类是一个模拟移动机器人及其传感器的模型，用于粒子滤波或其他基于概率的定位算法。
class Robot {
public:
    // x, y, orient
    // 表示机器人的当前位置 (x, y) 和朝向 orient（弧度）。
    // 初始值是通过 gen_real_random() 生成的随机数，分布在世界大小范围内。
    double x, y, orient; //robot poses
    // forward_noise, turn_noise, sense_noise
    // 用于模拟机器人的运动、转向和传感器测量中的噪声。
    double forward_noise, turn_noise, sense_noise; //robot noises

    Robot()
    {
        // Constructor
        // 初始化机器人位置、朝向以及噪声为默认值（噪声为 0）。位置和朝向使用随机生成的数值，使每个机器人的初始状态不同。
        x = gen_real_random() * world_size; // robot's x coordinate
        y = gen_real_random() * world_size; // robot's y coordinate
        orient = gen_real_random() * 2.0 * M_PI; // robot's orientation

        forward_noise = 0.0; //noise of the forward movement
        turn_noise = 0.0; //noise of the turn
        sense_noise = 0.0; //noise of the sensing
    }

    // 手动设置机器人的位置和朝向。
    // 对输入值进行检查，确保位置在合法范围 [0, world_size] 内，朝向在 [0, 2π] 范围内。
    void set(double new_x, double new_y, double new_orient)
    {
        // Set robot new position and orientation
        if (new_x < 0 || new_x >= world_size)
            throw std::invalid_argument("X coordinate out of bound");
        if (new_y < 0 || new_y >= world_size)
            throw std::invalid_argument("Y coordinate out of bound");
        if (new_orient < 0 || new_orient >= 2 * M_PI)
            throw std::invalid_argument("Orientation must be in [0..2pi]");

        x = new_x;
        y = new_y;
        orient = new_orient;
    }

    // 设置运动和传感器的噪声，用于模拟现实中的不确定性。
    void set_noise(double new_forward_noise, double new_turn_noise, double new_sense_noise)
    {
        // Simulate noise, often useful in particle filters
        forward_noise = new_forward_noise;
        turn_noise = new_turn_noise;
        sense_noise = new_sense_noise;
    }

    // 模拟机器人对环境中固定地标（landmarks）的测量。
    // 返回一个向量 z，包含到每个地标的距离（实际距离加上测量噪声）。
    vector<double> sense()
    {
        // Measure the distances from the robot toward the landmarks
        vector<double> z(sizeof(landmarks) / sizeof(landmarks[0]));
        double dist;

        for (int i = 0; i < sizeof(landmarks) / sizeof(landmarks[0]); i++) {
            dist = sqrt(pow((x - landmarks[i][0]), 2) + pow((y - landmarks[i][1]), 2));
            dist += gen_gauss_random(0.0, sense_noise);
            z[i] = dist;
        }
        return z;
    }

    // 模拟机器人向前移动和转向。
    // 添加随机噪声以反映现实中的运动误差。
    // 保证机器人的位置和朝向在周期范围内（例如：世界是循环的，超出范围会回到另一边）。
    // 注意：
    // 如果前进距离是负值，会抛出异常。
    // 返回一个新的 Robot 对象，表示运动后的状态。
    Robot move(double turn, double forward)
    {
        if (forward < 0)
            throw std::invalid_argument("Robot cannot move backward");

        // turn, and add randomness to the turning command
        orient = orient + turn + gen_gauss_random(0.0, turn_noise);
        orient = mod(orient, 2 * M_PI);

        // move, and add randomness to the motion command
        double dist = forward + gen_gauss_random(0.0, forward_noise);
        x = x + (cos(orient) * dist);
        y = y + (sin(orient) * dist);

        // cyclic truncate
        x = mod(x, world_size);
        y = mod(y, world_size);

        // set particle
        Robot res;
        res.set(x, y, orient);
        res.set_noise(forward_noise, turn_noise, sense_noise);

        return res;
    }

    // 以字符串形式返回机器人的当前位置和朝向，便于调试和观察。
    string show_pose()
    {
        // Returns the robot current position and orientation in a string format
        return "[x=" + to_string(x) + " y=" + to_string(y) + " orient=" + to_string(orient) + "]";
    }

    // 返回机器人的传感器测量值，以字符串形式表示。
    string read_sensors()
    {
        // Returns all the distances from the robot toward the landmarks
        vector<double> z = sense();
        string readings = "[";
        for (int i = 0; i < z.size(); i++) {
            readings += to_string(z[i]) + " ";
        }
        readings[readings.size() - 1] = ']';

        return readings;
    }

    // 计算给定传感器测量值的概率，用于粒子滤波中的重要性权重。
    // 概率通过高斯分布计算，越接近测量值的实际位置，概率越高。
    double measurement_prob(vector<double> measurement)
    {
        // Calculates how likely a measurement should be
        double prob = 1.0;                  // 最终返回的总概率  初始化为 1.0，用于累计所有传感器测量的概率。
        double dist;                        // 计算粒子到某地标的实际距离 粒子当前位置到地标的欧几里得距离（真实距离）。

        for (int i = 0; i < sizeof(landmarks) / sizeof(landmarks[0]); i++) {
            // 真实距离: 使用欧几里得公式计算粒子当前位置 (x,y) 到第 i 个地标的真实距离
            dist = sqrt(pow((x - landmarks[i][0]), 2) + pow((y - landmarks[i][1]), 2));

            // 调用 gaussian 函数计算该地标测量值的概率. 将所有地标的概率连乘（假设测量值之间相互独立），累积到 prob
            // 高斯分布函数计算粒子的距离测量与真实测量值之间的相似度。
            //              真实距离  感知噪声     测量距离
            prob *= gaussian(dist, sense_noise, measurement[i]);
        }

        return prob;
    }

private:
    // 生成高斯分布的随机数，用于模拟测量或运动噪声。
    double gen_gauss_random(double mean, double variance)
    {
        // Gaussian random
        normal_distribution<double> gauss_dist(mean, variance);
        return gauss_dist(gen);
    }

    // 计算一维高斯分布的概率密度函数，用于评估某个值出现的概率。
    // 高斯分布函数计算粒子的距离测量与真实测量值之间的相似度。
    // 当粒子的实际距离 d 接近传感器测量值 μ 时，概率 P 较大。距离差越大，概率 P 指数级下降。
    //                理论距离（均值）   测量噪声标准差    传感器测量到某地标距离
    double gaussian(double mu, double sigma, double x)
    {
        // Probability of x for 1-dim Gaussian with mean mu and var. sigma
        return exp(-(pow((mu - x), 2)) / (pow(sigma, 2)) / 2.0) / sqrt(2.0 * M_PI * (pow(sigma, 2)));
    }
};

// Functions
double gen_real_random()
{
    // Generate real random between 0 and 1
    uniform_real_distribution<double> real_dist(0.0, 1.0); //Real
    return real_dist(gen);
}

double mod(double first_term, double second_term)
{
    // Compute the modulus
    return first_term - (second_term)*floor(first_term / (second_term));
}

double evaluation(Robot r, Robot p[], int n)
{
    //Calculate the mean error of the system
    double sum = 0.0;
    for (int i = 0; i < n; i++) {
        //the second part is because of world's cyclicity
        double dx = mod((p[i].x - r.x + (world_size / 2.0)), world_size) - (world_size / 2.0);
        double dy = mod((p[i].y - r.y + (world_size / 2.0)), world_size) - (world_size / 2.0);
        double err = sqrt(pow(dx, 2) + pow(dy, 2));
        sum += err;
    }
    return sum / n;
}

double max(double arr[], int n)
{
    // Identify the max element in an array
    double max = 0;
    for (int i = 0; i < n; i++) {
        if (arr[i] > max)
            max = arr[i];
    }
    return max;
}

void visualization(int n, Robot robot, int step, Robot p[], Robot pr[])
{
    //Draw the robot, landmarks, particles and resampled particles on a graph

    //Graph Format
    plt::title("MCL, step " + to_string(step));
    plt::xlim(0, 100);
    plt::ylim(0, 100);

    //Draw particles in green
    for (int i = 0; i < n; i++) {
        plt::plot({ p[i].x }, { p[i].y }, "go");
    }

    //Draw resampled particles in yellow
    for (int i = 0; i < n; i++) {
        plt::plot({ pr[i].x }, { pr[i].y }, "yo");
    }

    //Draw landmarks in red
    for (int i = 0; i < sizeof(landmarks) / sizeof(landmarks[0]); i++) {
        plt::plot({ landmarks[i][0] }, { landmarks[i][1] }, "ro");
    }

    //Draw robot position in blue
    plt::plot({ robot.x }, { robot.y }, "bo");

    //Save the image and close the plot
    plt::save("./myImages/Step" + to_string(step) + ".png");
    plt::clf();
}

void visualizationAndw(int n, Robot robot, int step, Robot p[], Robot pr[], const double w[])
{
    plt::clf();
    // plt::figure(); // NEW: 每次绘制一个新的图像
    // plt::figure_size(1600, 900); // NEW: 设置图像尺寸 (宽 x 高)

    // Draw the robot, landmarks, particles and resampled particles on a graph

    // Graph Format
    plt::title("MCL, step " + std::to_string(step));
    plt::xlim(0, 100);
    plt::ylim(0, 100);


    // 找到权重的最大值和最小值
    double w_max = *std::max_element(w, w + n);
    double w_min = *std::min_element(w, w + n);
    // 绘制粒子位置（绿色）
    for (int i = 0; i < n; i++) {
        // plt::plot({ p[i].x }, { p[i].y }, "go");
        // plt::plot({ p[i].x }, { p[i].y }, "g.");
        double normalized_size = (w[i] - w_min) / (w_max - w_min + 1e-6); // 归一化
        std::string color = normalized_size > 0.1 ? "gx" : "g.";          // 高权重用点，低权重用叉
        plt::plot({ p[i].x }, { p[i].y }, color);
    }

    // 绘制重采样粒子（黄色）
    for (int i = 0; i < n; i++) {
        // plt::plot({ pr[i].x }, { pr[i].y }, "yo");
        plt::plot({ pr[i].x }, { pr[i].y }, "y.");
    }

    // Draw landmarks in red
    for (int i = 0; i < sizeof(landmarks) / sizeof(landmarks[0]); i++) {
        plt::plot({ landmarks[i][0] }, { landmarks[i][1] }, "ro");
        // plt::plot({ landmarks[i][0] }, { landmarks[i][1] }, "r.");
    }

    // 绘制机器人当前位置（蓝色）
    plt::plot({ robot.x }, { robot.y }, "bo");

    // // 绘制机器人方向箭头（蓝色）
    // double arrow_length = 2.0; // NEW: 箭头长度
    // double arrow_x = robot.x + cos(robot.orient) * arrow_length;
    // double arrow_y = robot.y + sin(robot.orient) * arrow_length;
    // plt::arrow(robot.x, robot.y, arrow_x - robot.x, arrow_y - robot.y, 0.2);

    // Show the image and wait for a key press
    // plt::show();

    // 显示图像
    plt::show(false); // 交互式显示，非阻塞
    plt::pause(0.05); // 短暂停以允许显示更新

    // // 等待用户输入
    // std::cout << "Press [Space] to continue...";
    // std::cin.ignore(); // 等待用户按回车键
}

int main()
{
    //Practice Interfacing with Robot Class
    Robot myrobot;
    // 设置噪声
    // myrobot.set_noise(5.0, 0.1, 5.0);
    // 减缓收敛速度： 增加运动噪声和传感器噪声
    myrobot.set_noise(20.0, 1.0, 10.0); 

    // 设置初始位置和方向
    myrobot.set(30.0, 50.0, M_PI / 2.0);

    // 机器人两次移动：
    // 第一次左转 -π/2，向前移动 15.0。
    // 第二次再次左转 -π/2，向前移动 10.0。
    myrobot.move(-M_PI / 2.0, 15.0);
    // cout << myrobot.read_sensors() << endl;
    myrobot.move(-M_PI / 2.0, 10.0);
    // cout << myrobot.read_sensors() << endl;

    // Create a set of particles 创建粒子群
    int n = 1000; // 创建了 1000 个粒子，每个粒子是一个 Robot 对象。
    // 减缓收敛速度： 增加粒子数量
    // int n = 2000;
    Robot p[n];

    // 创建了 1000 个粒子，每个粒子是一个 Robot 对象。
    // 每个粒子设置了运动和感测噪声。
    // 粒子群用于模拟机器人可能的不同位置。
    for (int i = 0; i < n; i++) {
        // 减缓收敛速度： 初始化粒子位置更均匀 以及 适度的运动和感测噪声
        p[i].set(gen_real_random() * 100.0, gen_real_random() * 100.0, gen_real_random() * 2.0 * M_PI);
        p[i].set_noise(0.1, 0.1, 5.0);

        // p[i].set_noise(0.05, 0.05, 5.0);
        //cout << p[i].show_pose() << endl;
    }

    //Re-initialize myrobot object and Initialize a measurment vector
    myrobot = Robot();
    vector<double> z;

    //Iterating 50 times over the set of particles
    int steps = 50;
    for (int t = 0; t < steps; t++) {
        // step 1: 机器人移动并感测环境。
        //Move the robot and sense the environment afterwards
        myrobot = myrobot.move(0.1, 5.0);       // 移动：机器人每步以角度 0.1 弧度向前移动 5.0 单位。
        // 获取传感器读数
        z = myrobot.sense();                    // 感测：获取机器人与所有地标的距离（包括感测噪声）。

        // step 2: 更新每个粒子的权重, 更新粒子状态。
        // Simulate a robot motion for each of these particles
        Robot p2[n];
        for (int i = 0; i < n; i++) {
            p2[i] = p[i].move(0.1, 5.0); // 所有粒子按照与机器人相同的方式移动，加入运动噪声。
            p[i] = p2[i];                // 更新粒子群的位置。
        }

        // step 3: 根据机器人测量结果，计算每个粒子的权重。
        //Generate particle weights depending on robot's measurement
        double w[n];
        for (int i = 0; i < n; i++) {
            w[i] = p[i].measurement_prob(z);    // 权重：基于粒子的感测值与机器人真实感测值 z 的接近程度计算权重。  权重通过高斯分布计算，更接近的粒子权重更高。
            
            // 减缓收敛速度： 增加随机扰动
            w[i] += gen_real_random() * 6e-11; 
            // cout << w[i] << endl;
        }

        // step 4: 根据权重，重采样粒子。
        // Resample the particles with a sample probability proportional to the importance weight
        // 轮盘法（Resampling Wheel）：
        // 随机选取一个起始粒子索引 index。
        // 用 beta（累加的随机值）寻找合适粒子，将其复制到新粒子集合 p3 中。
        // 重复此过程直到新粒子集合填满。
        // 结果：粒子权重大的粒子更有可能被多次采样，权重小的粒子被淘汰。
        // 没有被选中的粒子会被丢弃。这是粒子滤波的一个关键步骤，叫做重采样（Resampling）。以下是为什么和如何丢弃未被选中的粒子，以及可能的后果和改进方法。
        Robot p3[n];
        int index = gen_real_random() * n;       // index 是粒子的初始位置，值为[0,n−1] 的随机整数。
        //cout << index << endl;

        double beta = 0.0;                       // beta 是累加的轮盘值。
        double mw = max(w, n);                   // mw 是所有权重中的最大值，用于确定轮盘的范围 [0,2×mw]。这是为了确保权重较大的粒子有足够的采样机会。
        //cout << mw;

        for (int i = 0; i < n; i++) {
            beta += gen_real_random() * 2.0 * mw;   // 增加随机轮盘值   beta的值代表轮盘的位置。
            while (beta > w[index]) {               // 消耗当前粒子的权重
                beta -= w[index];                   // 减去当前粒子的权重
                index = mod((index + 1), n);        // 移动到下一个粒子  实现旋转
            }
            p3[i] = p[index];                       // 选择当前粒子     粒子有可能被多次选中
        }
        for (int k = 0; k < n; k++) {
            p[k] = p3[k];                           // 更新粒子集合 用重采样后的粒子集合 p3 更新原粒子集合 p。
            //cout << p[k].show_pose() << endl;
        }

        //Evaluate the Error
        cout << "Step = " << t << ", Evaluation = " << evaluation(myrobot, p, n) << endl;

        //####   DON'T MODIFY ANYTHING ABOVE HERE! ENTER CODE BELOW ####

        // step 5: 可视化粒子和机器人位置。
        //Graph the position of the robot and the particles at each step
        // visualization(n, myrobot, t, p2, p3);
        visualizationAndw(n, myrobot, t, p2, p3, w);

    } //End of Steps loop

    return 0;
}
