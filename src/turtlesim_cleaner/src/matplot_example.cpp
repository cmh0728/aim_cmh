#include "matplotlibcpp.h"  // matplotlib-cpp 헤더 포함
#include <vector>

namespace plt = matplotlibcpp;

int main() {
    // x, y 데이터 생성
    std::vector<double> x = {1, 2, 3, 4, 5};
    std::vector<double> y = {1, 4, 9, 16, 25};

    // 그래프 그리기
    plt::plot(x, y);
    plt::title("Plot example");
    plt::xlabel("X-axis");
    plt::ylabel("Y-axis");

    // 그래프 표시
    plt::show();

    return 0;
}
