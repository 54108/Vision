#include <string>

#include <fmt/color.h>
#include <fmt/core.h>

#include <opencv2/core.hpp>

namespace fps
{

// auto idntifier = fmt::format(fg(fmt::color::green) | fmt::emphasis::bold, "fps");
static double last_time;
class FPS
{
  public:
    FPS();

    FPS(std::string _name);

    ~FPS() = default;

    void calculateFPS();

    void calculateFPSGlobal();

    void getTick();

    static double lastTime();

    inline float returnFps()
    {
        return time * 1000;
    }

  private:
    void displayFPS() const
    {
        fmt::print("[{}] {} FPS of current/min/max: {}, {}, {}, time of current/averge: {}, {}\n",
                   fmt::format(fg(fmt::color::green) | fmt::emphasis::bold, "fps"), name_, fps, min, max, time * 1000,
                   average * 1000);
    }

    int cnt;        //  计算次数
    double time1;   //  记录第一次时间点
    double time2;   //  记录第二次时间点
    double time;    //  记录时间段
    double fps;     //  帧率
    double max;     //  最大帧率
    double min;     //  最小帧率
    double average; //  平均帧率（去除前10帧）
    double total;   //  总帧率

    std::string name_;
};

} // namespace fps
