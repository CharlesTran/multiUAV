#include <vector>
#include<unordered_map>
#include <cstdint>


// 单个位置点，包括时间戳、经度和纬度
struct Position {
    std::int64_t timestamp; // 时间戳，单位可自定义（如毫秒/秒）
    double latitude;        // 纬度
    double longitude;       // 经度
};

// 目标结构体
struct Target {
    std::int64_t timestamp;                // 当前时间戳
    double latitude;                       // 当前纬度
    double longitude;                      // 当前经度
    std::vector<Position, 5> history;
    std::vector<Position, 5> predictions;   // 之后5个时间戳的预测位置
};

// 定义区域视图，一个视图由ID和多个target组成
unordered_map<int, vector<Target>> uavs;

long computeDistance(const Position& p1, const Position& p2) {
    // 计算两点之间的距离，这里可以使用简单的欧几里得距离或更复杂的地理距离计算
    double latDiff = p1.latitude - p2.latitude;
    double lonDiff = p1.longitude - p2.longitude;
    return static_cast<long>(sqrt(latDiff * latDiff + lonDiff * lonDiff) * 100000); // 假设单位为米
}

// 更新目标的时间戳和位置，并更新历史位置点
void updateTarget(Target& target, std::int64_t timestamp, double latitude, double longitude) {
    // 更新当前时间戳和位置
    target.timestamp = timestamp;
    target.latitude = latitude;
    target.longitude = longitude;
    // 更新历史位置点
    for (size_t i = 4; i > 0; --i) {
        target.history[i] = target.history[i - 1];
    }
    target.history[0] = {timestamp, latitude, longitude};
}

// 若uav中target的predict为空，则进行预测
void predictTargets(Target& target) {
    return;
}
 //无人机获取目标
std::queue<Target> catchUavTargets(int uavId);

//