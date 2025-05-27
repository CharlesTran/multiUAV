#include <iostream>
#include <thread>
#include <vector>
#include <unordered_map>
#include <mutex>
#include <chrono>
#include <queue>

void updateTarget(Target& target, std::int64_t timestamp, double latitude, double longitude);

// 假设 catchUavTargets 是一个用于获取新目标的函数
std::queue<Target> catchUavTargets(int uavId);

void processUav(int uavId, std::unordered_map<int, std::vector<Target>>& uavs, std::mutex& mtx) {
    while (1) {
        // 获取无人机当前目标
        std::queue<Target> newTargets = catchUavTargets(uavId);
        // 记录无人机在当前时间段中捕获到的目标，由于时间戳相近，因此使用最短距离匹配是合理的
        // unmatchedTargets中的目标一定满足五个时间戳后才能进行轨迹匹配
        std::vector<Target> unmatchedTargets;
		// 没有获取到目标时，等待一段时间后重试
        if( newTargets.empty()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
		}
        //对于每个检测到的目标，将其与现有目标进行比较
        //找到最近的目标比较距离，如果小于阈值则更新，否则就添加到unmatch队列
        const double DIST_THRESHOLD = 0.0005; // 距离阈值
        const double VELO_COEFFICIENT = 1e-6; // 速度系数

        while (!newTargets.empty() {
            Target newTarget = newTargets.front();
            newTargets.pop();
            std::lock_guard<std::mutex> lock(mtx); // 保证线程安全
            double minDist = std::numeric_limits<double>::max();
            int minIdx = -1;
            int minType = -1; //距离最小的目标的类别：0-暂存组，1-全局数据

            
            for (size_t i = 0; i < unmatchedTargets.size(); ++i) {
                double dist = std::sqrt(
                    (unmatchedTargets[i].latitude - newTarget.latitude) * (unmatchedTargets[i].latitude - newTarget.latitude) +
                    (unmatchedTargets[i].longitude - newTarget.longitude) * (unmatchedTargets[i].longitude - newTarget.longitude)
                ) + VELO_COEFFICIENT * (newTarget.timestamp - unmatchedTargets[i].timestamp);
                if (dist < minDist) {
                    minDist = dist;
                    minIdx = static_cast<int>(i);
                    minType = 0;
                }
            }
            for (size_t i = 0; i < uavs[uavId].size(); ++i) {
                double dist = std::sqrt(
                    (uavs[uavId][i].latitude - newTarget.latitude) * (uavs[uavId][i].latitude - newTarget.latitude) +
                    (uavs[uavId][i].longitude - newTarget.longitude) * (uavs[uavId][i].longitude - newTarget.longitude)
                ) + VELO_COEFFICIENT * (newTarget.timestamp - uavs[uavId][i].timestamp);
                if (dist < minDist) {
                    minDist = dist;
                    minIdx = static_cast<int>(i);
                    minType = 1;
                }
            }


            if (minType == 0 && minDist < DIST_THRESHOLD) {
                // 距离小于阈值，更新目标
                updateTarget(unmatchedTargets[minIdx], newTarget.timestamp, newTarget.latitude, newTarget.longitude);
            }
            else if (minType == 1 && minDist < DIST_THRESHOLD) {
                // 距离小于阈值，更新目标
                updateTarget(uavs[uavId][minIdx], newTarget.timestamp, newTarget.latitude, newTarget.longitude);
            }
            else {
                // 没有匹配到，加入unmatchedTargets队列
                unmatchedTargets.push(newTarget);
            }
        }
        for (Target& target : unmatchedTargets) {
            if (target.history.size() == 5) {
				predictTargets(target);
            }
		}
       
    }
    
    std::lock_guard<std::mutex> lock(mtx); // 保证线程安全
    auto& targets = uavs[uavId];
    for (auto& t : newTargets) {
        targets.push_back(t);
    }
}

int main() {
    std::unordered_map<int, std::vector<Target>> uavs;
    std::vector<int> uavIds;
    for (const auto& kv : uavs) {
        uavIds.push_back(kv.first);
    }
    std::mutex mtx;
    std::vector<std::thread> threads;

    for (int uavId : uavIds) {
        threads.emplace_back(processUav, uavId, std::ref(uavs), std::ref(mtx));
    }

    for (auto& th : threads) {
        th.join();
    }

    return 0;
}