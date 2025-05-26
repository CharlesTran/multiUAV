#include <vector>
#include<unordered_map>
#include <cstdint>


// ����λ�õ㣬����ʱ��������Ⱥ�γ��
struct Position {
    std::int64_t timestamp; // ʱ�������λ���Զ��壨�����/�룩
    double latitude;        // γ��
    double longitude;       // ����
};

// Ŀ��ṹ��
struct Target {
    std::int64_t timestamp;                // ��ǰʱ���
    double latitude;                       // ��ǰγ��
    double longitude;                      // ��ǰ����
    std::vector<Position, 5> history;
    std::vector<Position, 5> predictions;   // ֮��5��ʱ�����Ԥ��λ��
};

// ����������ͼ��һ����ͼ��ID�Ͷ��target���
unordered_map<int, vector<Target>> uavs;

long computeDistance(const Position& p1, const Position& p2) {
    // ��������֮��ľ��룬�������ʹ�ü򵥵�ŷ����þ��������ӵĵ���������
    double latDiff = p1.latitude - p2.latitude;
    double lonDiff = p1.longitude - p2.longitude;
    return static_cast<long>(sqrt(latDiff * latDiff + lonDiff * lonDiff) * 100000); // ���赥λΪ��
}

// ����Ŀ���ʱ�����λ�ã���������ʷλ�õ�
void updateTarget(Target& target, std::int64_t timestamp, double latitude, double longitude) {
    // ���µ�ǰʱ�����λ��
    target.timestamp = timestamp;
    target.latitude = latitude;
    target.longitude = longitude;
    // ������ʷλ�õ�
    for (size_t i = 4; i > 0; --i) {
        target.history[i] = target.history[i - 1];
    }
    target.history[0] = {timestamp, latitude, longitude};
}

// ��uav��target��predictΪ�գ������Ԥ��
void predictTargets(Target& target) {
    return;
}
 //���˻���ȡĿ��
std::queue<Target> catchUavTargets(int uavId);

//