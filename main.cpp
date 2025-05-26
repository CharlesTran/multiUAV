#include <iostream>
#include <thread>
#include <vector>
#include <unordered_map>
#include <mutex>
#include <chrono>
#include <queue>

void updateTarget(Target& target, std::int64_t timestamp, double latitude, double longitude);

// ���� catchUavTargets ��һ�����ڻ�ȡ��Ŀ��ĺ���
std::queue<Target> catchUavTargets(int uavId);

void processUav(int uavId, std::unordered_map<int, std::vector<Target>>& uavs, std::mutex& mtx) {
    while (1) {
        // ��ȡ���˻���ǰĿ��
        std::queue<Target> newTargets = catchUavTargets(uavId);
        // �洢����ƥ��ʧ�ܵ�Ŀ�꣬׼������ƥ��
        // unmatchedTargets�е�Ŀ��һ���������ʱ�������ܽ��й켣ƥ��
        std::vector<Target> unmatchedTargets;
		// û�л�ȡ��Ŀ��ʱ���ȴ�һ��ʱ�������
        if( newTargets.empty()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
		}
        //����ÿ����⵽��Ŀ�꣬����������Ŀ����бȽ�
        //�ҵ������Ŀ��ȽϾ��룬���С����ֵ����£��������ӵ�unmatch����
        const double DIST_THRESHOLD = 0.0005; // ������ֵ���ɸ���ʵ���������
        //
        // ����ÿ����⵽��Ŀ�꣬����������Ŀ����бȽ�
        // Ŀǰֻ�ڵ�ǰ���˻�Ŀ����ƥ�䣬���ӽ��ݲ�����
        std::queue<Target> tmpUnmatch;
        while (!newTargets.empty()) {
            Target newTarget = newTargets.front();
            newTargets.pop();

            std::lock_guard<std::mutex> lock(mtx); // ��֤�̰߳�ȫ
            std::vector& targets = uavs[uavId];

            double minDist = std::numeric_limits<double>::max();
            int minIdx = -1;
            for (size_t i = 0; i < targets.size(); ++i) {
                double dist = std::sqrt(
                    (targets[i].latitude - newTarget.latitude) * (targets[i].latitude - newTarget.latitude) +
                    (targets[i].longitude - newTarget.longitude) * (targets[i].longitude - newTarget.longitude)
                );
                if (dist < minDist) {
                    minDist = dist;
                    minIdx = static_cast<int>(i);
                }
            }

            if (minIdx != -1 && minDist < DIST_THRESHOLD) {
                // ����С����ֵ������Ŀ��
                updateTarget(targets[minIdx], newTarget.timestamp, newTarget.latitude, newTarget.longitude);
            } else {
                // û��ƥ�䵽������unmatchedTargets����
                tmpUnmatch.push(newTarget);
            }
        }

        // ����δƥ���ϵ�Ŀ��
        // ��tmpUnmatch��unmatchedTargets�е�Ŀ��Ƚ�
        while (!tmpUnmatch.empty()) {
            Target tmp = tmpUnmatch.front();
            tmpUnmatch.pop();

            std::lock_guard<std::mutex> lock(mtx); // ��֤�̰߳�ȫ

            double minDist = std::numeric_limits<double>::max();
            int minIdx = -1;
            for (size_t i = 0; i < unmatchedTargets.size(); ++i) {
                double dist = std::sqrt(
                    (targets[i].latitude - newTarget.latitude) * (targets[i].latitude - newTarget.latitude) +
                    (targets[i].longitude - newTarget.longitude) * (targets[i].longitude - newTarget.longitude)
                );
                if (dist < minDist) {
                    minDist = dist;
                    minIdx = static_cast<int>(i);
                }
            }

            if (minIdx != -1 && minDist < DIST_THRESHOLD) {
                // ����С����ֵ������Ŀ��
                updateTarget(unmatchedTargets[minIdx], newTarget.timestamp, newTarget.latitude, newTarget.longitude);
            }
            else {
                // û��ƥ�䵽������unmatchedTargets����
                unmatchedTargets.push(tmp);
            }
        }
    }
    
    std::lock_guard<std::mutex> lock(mtx); // ��֤�̰߳�ȫ
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