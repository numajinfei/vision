#include "seam_tracking/seam_tracking.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "opencv2/opencv.hpp"

namespace seam_tracking
{

std::vector<cv::Point2f> Execute(const cv::Mat& img, cv::Mat& dx, int ksize, int threshold, int widthMin, int widthMax)
{
    std::vector<cv::Point2f> ret;
    if(img.empty())
        return ret;

    ret.reserve(img.rows);

    double scale = 1.;
    switch(ksize)
    {
        case 1:
            scale = 1.;
            break;
        case 3:
            scale = 0.25;
            break;
        case 5:
            scale = 1. / 48;
            break;
        default:
            break;
    }

    cv::Sobel(img, dx, CV_16S, 1, 0, ksize, scale, 0, cv::BorderTypes::BORDER_REPLICATE);

    for (int r = 0; r < dx.rows; r++)
    {
        auto pRow = dx.ptr<short>(r);
        auto minmax = std::minmax_element(pRow, pRow + dx.cols);

        auto minEle = minmax.first;
        auto maxEle = minmax.second;

        auto minVal = *minEle;
        auto maxVal = *maxEle;

        auto minPos = minEle - pRow;
        auto maxPos = maxEle - pRow;
        auto width = minPos - maxPos;

        if(maxVal < threshold || minVal > -threshold)
            continue;

        if(width < widthMin || width > widthMax)
            continue;

        //filter by position
        if(maxPos == 0 || minPos == dx.cols - 1)
            continue;

        auto a1 = pRow[maxPos - 1] + pRow[maxPos + 1] - pRow[maxPos] * 2;
        auto b1 = pRow[maxPos + 1] - pRow[maxPos - 1];
        auto s1 = -0.5 * b1 / a1;

        auto a2 = pRow[minPos - 1] + pRow[minPos + 1] - pRow[minPos] * 2;
        auto b2 = pRow[minPos + 1] - pRow[minPos - 1];
        auto s2 = -0.5 * b2 / a2;

        ret.emplace_back((maxPos + minPos + s1 + s2) * 0.5, r);
    }

    return ret;
}

std::vector<cv::Point2f> CleanLines(const std::vector<cv::Point2f>& pnts, int dx, int dy, int dz)
{
    std::vector<cv::Point2f> ret;
    ret.reserve(pnts.size());

    for(size_t i = 0; i < pnts.size(); i++)
    {
        std::vector<cv::Point2f> temp;
        temp.emplace_back(pnts[i]);
        for(; i < pnts.size() - 1;)
        {
            if(abs(pnts[i].x - pnts[i + 1].x) < dx && abs(pnts[i].y - pnts[i + 1].y) < dy)
            {
                temp.emplace_back(pnts[i + 1]);
                i++;
            }
            else
                break;
        }

        if(temp.rbegin()->y - temp.begin()->y > dz)
            ret.insert(ret.end(), temp.begin(), temp.end());
    }

    return ret;
}

int FindDistinct(const std::vector<cv::Point2f>& pnts, int length, int aspect, int depth)
{
    if(pnts.size() < length)
        return -1;

    auto rr = cv::minAreaRect(pnts);

    cv::Point2f v[4];
    rr.points(v);

    float d[4];
    for(size_t i = 0; i < 4; ++i)
        d[i] = v[i].y - v[(i + 1) % 4].y;

    auto i = std::distance(d, std::max_element(d, d + 4));
    auto a = v[i].y - v[(i + 1) % 4].y;
    auto b = v[(i + 1) % 4].x - v[i].x;
    auto c = v[i].x * v[(i + 1) % 4].y - v[(i + 1) % 4].x * v[i].y;
    auto s = sqrt(a * a + b * b);
    a /= s; b /= s; c /= s;

    std::vector<float> vd;
    vd.reserve(pnts.size());
    for(const auto& p : pnts)
    {
        vd.push_back(p.x * a + p.y * b + c);
    }

    i = std::distance(vd.begin(), max_element(vd.begin(), vd.end()));

    if(rr.size.height * 1.0 / rr.size.width > aspect || rr.size.height * 1.0 / rr.size.width < 1.0 / aspect)
    {
        auto avg = std::accumulate(vd.begin(), vd.end(), 0.0) / vd.size();
        return vd[i] - avg > depth ? i : -1;
    }
    else
        return i;
}

class SeamTracking::_Impl
{
public:
    explicit _Impl(SeamTracking* ptr) : _node(ptr)
    {
        _node->get_parameter("ksize", _ksize);
        _node->get_parameter("threshold", _threshold);
        _node->get_parameter("width_min", _widthMin);
        _node->get_parameter("width_max", _widthMax);
        _node->get_parameter("binning", _binning);
        _node->get_parameter("ratio", _ratio);

        _node->get_parameter("dx", _dx);
        _node->get_parameter("dy", _dy);
        _node->get_parameter("dz", _dz);
        _node->get_parameter("length", _length);
        _node->get_parameter("aspect", _aspect);
        _node->get_parameter("depth", _depth);

        std::vector<double> c, d, h;
        _node->get_parameter("camera_matrix", c);
        _node->get_parameter("distort_coeffs", d);
        _node->get_parameter("homography_matrix", h);

        _coef = cv::Mat(3, 3, CV_64F, c.data()).clone();
        _dist = cv::Mat(1, 5, CV_64F, d.data()).clone();
        _H = cv::Mat(3, 3, CV_64F, h.data()).clone();

        _thread = std::thread(&SeamTracking::_Impl::_Worker, this);
    }

    ~_Impl()
    {
        _con.notify_all();
        _thread.join();
    }

    void PushBack(sensor_msgs::msg::Image::UniquePtr& ptr)
    {
        std::unique_lock<std::mutex> lk(_mutex);
        _deq.emplace_back(std::move(ptr));
        lk.unlock();
        _con.notify_all();
    }

private:
    visualization_msgs::msg::Marker::UniquePtr _MarkerPoint(float x = 0, float y = 0, float z = 0)
    {
        auto ptr = std::make_unique<visualization_msgs::msg::Marker>();
        ptr->header.frame_id = "map";
        ptr->header.stamp = _node->now();
        ptr->ns = "my_namespace";
        ptr->id = 0;
        ptr->type = visualization_msgs::msg::Marker::SPHERE;
        ptr->action = 0;
        ptr->pose.position.x = x;
        ptr->pose.position.y = y;
        ptr->pose.position.z = z;
        ptr->pose.orientation.x = 0.0;
        ptr->pose.orientation.y = 0.0;
        ptr->pose.orientation.z = 0.0;
        ptr->pose.orientation.w = 1.0;
        ptr->scale.x = 0.005;
        ptr->scale.y = 0.005;
        ptr->scale.z = 0.005;
        ptr->color.a = 1.0;
        ptr->color.r = 1.0;
        ptr->color.g = 0.0;
        ptr->color.b = 0.0;
        return ptr;
    }

    std::vector<cv::Point2f> _Reconstruct(std::vector<cv::Point2f>& pnts)
    {
        std::vector<cv::Point2f> ret;
        if(pnts.empty())
            return ret;

        for(auto& p : pnts)
            p *= _binning;

        cv::undistortPoints(pnts, ret, _coef, _dist, cv::noArray(), _coef);
        cv::perspectiveTransform(ret, ret, _H);

        return ret;
    }

    sensor_msgs::msg::PointCloud2::UniquePtr _ConstructPointCloud(const std::vector<cv::Point2f>& pnts)
    {
        if(pnts.empty())
            return nullptr;

        pcl::PointCloud<pcl::PointXYZ> cloud;
        cloud.height = 1;
        cloud.width = pnts.size();
        cloud.points.reserve(pnts.size());

        for(size_t i = 0; i < pnts.size(); ++i)
            cloud.points.emplace_back(0, pnts[i].x * _ratio, pnts[i].y * _ratio);

        auto ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(cloud, *ptr);
        ptr->header.stamp = _node->now();
        ptr->header.frame_id = "map";
        return ptr;
    }

    void _Worker()
    {
        cv::Mat dx;

        while(rclcpp::ok())
        {
            try
            {
                std::unique_lock<std::mutex> lk(_mutex);
                if(_deq.empty() == false)
                {
                    auto ptr = std::move(_deq.front());
                    _deq.pop_front();
                    lk.unlock();

                    cv::Mat img(ptr->height, ptr->width, CV_8UC1, ptr->data.data());
                    cv::rotate(img, img, cv::ROTATE_90_CLOCKWISE);
                    auto cet = Execute(img, dx, _ksize, _threshold, _widthMin, _widthMax);
                    auto ret = CleanLines(cet, _dx, _dy, _dz);
                    auto ind = FindDistinct(ret, _length, _aspect, _depth);
                    auto pnt = _Reconstruct(ret);
                    /*auto msg = _ConstructPointCloud(pnt);
                    if(msg)
                        _node->PublishLine(msg);*/
                    if(ind != -1)
                    {
                        auto msg = std::make_unique<shared_interfaces::msg::ModbusCoord>();
                        msg->a = 1.;
                        msg->b = 0.;
                        msg->c = pnt[ind].x * _ratio;
                        msg->d = pnt[ind].y * _ratio;
                        _node->PublishCoord(msg);

                        auto mark = _MarkerPoint(0, pnt[ind].x * _ratio, pnt[ind].y * _ratio);
                        _node->PublishMarker(mark);
                    }
                    else
                    {
                        auto msg = std::make_unique<shared_interfaces::msg::ModbusCoord>();
                        msg->a = 0.;
                        msg->b = 0.;
                        msg->c = 0.;
                        msg->d = 0.;
                        _node->PublishCoord(msg);

                        auto mark = _MarkerPoint();
                        _node->PublishMarker(mark);
                    }

                    /*pcl::PointCloud<pcl::PointXYZ> cloud;
                    cloud.height = 1;
                    cloud.width = ret.size();
                    cloud.points.reserve(ret.size());*/
                    /*cv::Mat img(ptr->height, ptr->width, CV_8UC1, ptr->data.data()), mid, dst;
                    cv::rotate(img, mid, cv::ROTATE_90_CLOCKWISE);

                    cv::resize(mid, dst, cv::Size(), 0.5, 0.5, cv::INTER_AREA);
                    cv::Rect r(100, 80, 100, 190);
                    cv::Mat roi(dst, r);

                    auto ret = Execute(roi, dx, _ksize, _threshold, _widthMin, _widthMax);
                    cv::Point2f p;
                    FindDistinct(ret, p);

                    p.x += 100; p.x *= 4.;
                    p.y += 80; p.y *= 4;
                    std::vector<cv::Point2f> vp;
                    vp.push_back(p);
                    cv::undistortPoints(vp, vp, _coef, _dist);
                    cv::perspectiveTransform(vp, vp, _H);

                    auto msg = std::make_unique<shared_interfaces::msg::ModbusCoord>();
                    msg->a = 1;
                    msg->b = 0;
                    msg->c = vp[0].x;
                    msg->d = vp[0].y;

                    _node->Publish(msg);*/
                }
                else
                    _con.wait(lk);
            }
            catch(const std::exception& e)
            {
                RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Exception in seam_tracking worker: %s", e.what());
            }
            catch(...)
            {
                RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Exception in seam_tracking worker: unknown");
            }
        }
    }

private:
    SeamTracking* _node;
    cv::Mat _coef, _dist, _H;
    int _ksize;
    int _threshold;
    int _widthMin;
    int _widthMax;
    int _binning;
    double _ratio;
    int _dx, _dy, _dz;
    int _length, _aspect, _depth;
    std::mutex _mutex;              ///< Mutex to protect shared storage
    std::condition_variable _con;   ///< Conditional variable rely on mutex
    std::deque<sensor_msgs::msg::Image::UniquePtr> _deq;
    std::thread _thread;
};

SeamTracking::SeamTracking(const rclcpp::NodeOptions& options) : Node("seam_tracking_node", options)
{
    _init = std::thread(&SeamTracking::_Init, this);
}

SeamTracking::~SeamTracking()
{
    _init.join();

    _sub.reset();
    _impl.reset();
    _pubMarker.reset();
    _pubCoord.reset();
    _pubLine.reset();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "seam_tracking destroyed successfully");
}

void SeamTracking::_Init() try
{
    _InitializeParameters();

    _UpdateParameters();

    _pubLine = this->create_publisher<sensor_msgs::msg::PointCloud2>(_pubNameLine, 10);

    _pubCoord = this->create_publisher<shared_interfaces::msg::ModbusCoord>(_pubNameCoord, 10);

    _pubMarker = this->create_publisher<visualization_msgs::msg::Marker>(_pubNameMarker, 10);

    _impl = std::make_unique<_Impl>(this);

    _sub = this->create_subscription<sensor_msgs::msg::Image>(_subName, 10, std::bind(&SeamTracking::_Sub, this, std::placeholders::_1));

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "seam_tracking initialized successfully");
}
catch(const std::exception& e)
{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in seam_tracking initializer: %s", e.what());
    rclcpp::shutdown();
}
catch(...)
{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in seam_tracking initializer: unknown");
    rclcpp::shutdown();
}

void SeamTracking::_Sub(sensor_msgs::msg::Image::UniquePtr ptr) try
{
    _impl->PushBack(ptr);
}
catch(const std::exception& e)
{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in seam_tracking subscription: %s", e.what());
}
catch(...)
{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in seam_tracking subscription: unknown");
}

void SeamTracking::_InitializeParameters()
{
    declare_parameter("ksize");
    declare_parameter("threshold");
    declare_parameter("width_min");
    declare_parameter("width_max");
    declare_parameter("binning");
    declare_parameter("ratio");
    declare_parameter("dx");
    declare_parameter("dy");
    declare_parameter("dz");
    declare_parameter("length");
    declare_parameter("aspect");
    declare_parameter("depth");
    declare_parameter("camera_matrix");
    declare_parameter("distort_coeffs");
    declare_parameter("homography_matrix");
}

void SeamTracking::_UpdateParameters()
{
    //this->get_parameter("", );
}

}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(seam_tracking::SeamTracking)

